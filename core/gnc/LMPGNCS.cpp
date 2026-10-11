// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "LMPGNCS.h"

#include "../sim/Spacecraft.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gnc {

using sim::Mat3;
using sim::Thruster;
using sim::Vec3;

namespace {

inline uint16_t
Bit(int n)
{
	return uint16_t(1u << (n - 1));
}

const double kFeet = 0.3048;
const double kPIPAPulseLM = 0.01;                 // m/s
const double kGyroPulse = 6.28318530717958647692 / 2097152.0;  // 2^-21 rev
const double kCDUCount = 6.28318530717958647692 / 32768.0;
const double kPulseRate = 3200.0;                 // drive pulses per second

// LR antenna geometry (Luminary: HBEAMANT and the SETPOS rotations).
const Vec3 kRangeBeamAntenna(-0.9374036, 0.0, -0.3482449);
const double kLRMaxRangeFt = 45000;
const double kLRVelocityAcquireFt = 30000;
const double kLRLowScaleFt = 2500;

}  // namespace


LMPGNCS::LMPGNCS(agc::Computer& computer, sim::Vehicle& vehicle)
	:
	fComputer(computer),
	fVehicle(&vehicle)
{
}


void
LMPGNCS::Attach(sim::Vehicle& vehicle, const sim::Quat& frame)
{
	fVehicle = &vehicle;
	fFrame = frame;
}


Thruster*
LMPGNCS::_DPS()
{
	return fVehicle->GetThruster("DPS");
}


Thruster*
LMPGNCS::_APS()
{
	return fVehicle->GetThruster("APS");
}


void
LMPGNCS::RateOfDescentClick(int direction)
{
	// Channel 16 bit 6 = descent+ (slower), bit 7 = descent- (faster).
	fComputer.NavKey(direction > 0 ? Bit(6) : Bit(7));
}


void
LMPGNCS::Mark(int which)
{
	fComputer.NavKey(which == 0 ? Bit(3) : (which == 1 ? Bit(4) : Bit(5)));
}


// ---------------------------------------------------------------------------
// Inputs (all bits in channels 30-33 are active low)


void
LMPGNCS::_Inputs()
{
	auto set = [&](int ch, int bit, bool active) {
		fComputer.SetInputBit(ch, bit, !active);
	};

	// Channel 30.
	set(030, 1, panel.abort);
	set(030, 3, panel.engineArm != LMPanel::ARM_OFF);
	set(030, 4, panel.abortStage);
	set(030, 5, panel.autoThrottle);
	set(030, 6, false);
	set(030, 7, false);
	set(030, 9, imu.powered && !imu.failed);
	set(030, 10, panel.pgnsControl);
	set(030, 11, panel.imuCage);
	set(030, 12, imu.cduFailed);
	set(030, 13, imu.failed);
	set(030, 14, false);
	set(030, 15, imu.powered);

	// Channel 31: hand controllers and mode control.
	const double detent = 0.12;
	bool outOfDetent = std::fabs(panel.aca.x) > detent
		|| std::fabs(panel.aca.y) > detent || std::fabs(panel.aca.z) > detent;
	set(031, 1, panel.acaPulse && panel.aca.y > detent);
	set(031, 2, panel.acaPulse && panel.aca.y < -detent);
	set(031, 3, panel.acaPulse && panel.aca.x > detent);
	set(031, 4, panel.acaPulse && panel.aca.x < -detent);
	set(031, 5, panel.acaPulse && panel.aca.z > detent);
	set(031, 6, panel.acaPulse && panel.aca.z < -detent);
	set(031, 7, panel.thc[0] > 0);
	set(031, 8, panel.thc[0] < 0);
	set(031, 9, panel.thc[1] > 0);
	set(031, 10, panel.thc[1] < 0);
	set(031, 11, panel.thc[2] > 0);
	set(031, 12, panel.thc[2] < 0);
	set(031, 13, panel.mode == LMPanel::MODE_ATT_HOLD);
	set(031, 14, panel.mode == LMPanel::MODE_AUTO);
	set(031, 15, outOfDetent && !panel.acaPulse);

	// Channel 32.
	for (int i = 0; i < 8; i++)
		set(032, i + 1, panel.thrusterPairOff[i]);
	set(032, 9, !panel.gimbalEnable);
	set(032, 10, false);

	// Channel 33: radar status (flip-flop bits 11-15 left to the CPU).
	set(033, 2, false);
	set(033, 3, false);
	set(033, 4, false);
	set(033, 5, fLRRangeGood);
	set(033, 6, fLRPosition == 1 && fLRSlew <= 0);
	set(033, 7, fLRPosition == 2 && fLRSlew <= 0);
	set(033, 8, fLRVelGood);
	set(033, 9, fLRAltitude > 0 && fLRAltitude / kFeet < kLRLowScaleFt);

	// Hand controller counters (read when channel 13 bits 8 and 9 set).
	uint16_t ch13 = fComputer.Channel(013);
	if ((ch13 & Bit(8)) && (ch13 & Bit(9))) {
		auto counts = [&](double d) {
			if (std::fabs(d) < detent)
				return uint16_t(0);
			int c = int(std::lround(42.0 * d));
			return agc::FromInt15(c);
		};
		fComputer.SetCell(agc::REG_RHCP, counts(panel.aca.y));
		fComputer.SetCell(agc::REG_RHCY, counts(panel.aca.x));
		fComputer.SetCell(agc::REG_RHCR, counts(panel.aca.z));
	}
}


// ---------------------------------------------------------------------------
// Step


void
LMPGNCS::Step(double dt, const RadarScene* scene)
{
	// IMU: gimbal lock monitoring, CDUs and accelerometers.
	fGimbalWarning = imu.Update(_Attitude(), dt);
	uint16_t ch12 = fComputer.Channel(012);
	Vec3 g = imu.Gimbals(_Attitude());
	if (ch12 & Bit(5)) {
		fComputer.SetCell(agc::REG_CDUX, 0);
		fComputer.SetCell(agc::REG_CDUY, 0);
		fComputer.SetCell(agc::REG_CDUZ, 0);
	} else if (imu.powered && !imu.cduFailed) {
		fComputer.SetCell(agc::REG_CDUX, AngleToCDU(g.x));
		fComputer.SetCell(agc::REG_CDUY, AngleToCDU(g.y));
		fComputer.SetCell(agc::REG_CDUZ, AngleToCDU(g.z));
	}
	int counts[3];
	imu.Sense(fVehicle->SensedDeltaV(), kPIPAPulseLM, counts);
	fComputer.AddCounts(agc::REG_PIPAX, counts[0]);
	fComputer.AddCounts(agc::REG_PIPAY, counts[1]);
	fComputer.AddCounts(agc::REG_PIPAZ, counts[2]);

	_Inputs();
	if (rrCDUHunting) {
		fHuntResidual += 2 * 6400.0 * huntScale * dt;
		int n = int(fHuntResidual);
		fHuntResidual -= n;
		fComputer.StealCycles(n);
	}
	fComputer.Run(dt);
	_Outputs(dt, scene);
}


void
LMPGNCS::_Outputs(double dt, const RadarScene* scene)
{
	uint16_t ch11 = fComputer.Channel(011);
	uint16_t ch12 = fComputer.Channel(012);
	uint16_t ch14 = fComputer.Channel(014);

	// --- Engine on/off latch (channel 11 bits 13 and 14) ----------------------
	bool armedDescent = panel.engineArm == LMPanel::ARM_DESCENT;
	bool armedAscent = panel.engineArm == LMPanel::ARM_ASCENT
		&& !ascentArmBreakerOpen;
	bool lgcOn = panel.pgnsControl && (ch11 & Bit(13));
	bool lgcOff = (ch11 & Bit(14)) != 0;
	if (lgcOn || panel.engineStart) {
		if (armedDescent)
			fEngineLatch = true;
		if (armedAscent)
			fAPSLatch = true;
	}
	if (lgcOff || panel.engineStop) {
		fEngineLatch = false;
		fAPSLatch = false;
	}
	if (!armedDescent)
		fEngineLatch = false;
	if (!armedAscent)
		fAPSLatch = false;
	fLastCh11 = ch11;

	// --- Descent engine throttle (THRUST counter, channel 14 bit 4) -----------
	if (ch14 & Bit(4)) {
		uint16_t raw = fComputer.Cell(agc::REG_THRUST);
		int pending = agc::ToInt15(raw);
		fThrottleDrain += kPulseRate * dt;
		int n = int(std::min(std::fabs(double(pending)), fThrottleDrain));
		fThrottleDrain -= n;
		if (n > 0) {
			int sign = pending > 0 ? 1 : -1;
			if (!throttleFailed)
				fDecaIntegrator += sign * n / sim::craft::kDPSPulsesPerNewton;
			int left = pending - sign * n;
			fComputer.SetCell(agc::REG_THRUST, agc::FromInt15(left));
		}
	} else {
		fThrottleDrain = 0;
	}
	double full = sim::craft::kDPSFullThrust;
	double minimum = sim::craft::kDPSMinThrust;
	double manual = minimum + panel.manualThrottle * (full - minimum);
	double command = panel.autoThrottle ? manual + fDecaIntegrator : manual;
	// The integrator itself stops at the hardware limits.
	fDecaIntegrator = sim::Clamp(fDecaIntegrator, -full, full);
	fThrustCommand = sim::Clamp(command, minimum, full);
	double lag = 1.0 - std::exp(-dt / 0.2);
	fThrustActual += (fThrustCommand - fThrustActual) * lag;

	if (Thruster* dps = _DPS()) {
		dps->on = fEngineLatch;
		dps->throttle = fThrustActual / full;
		// Gimbal trim (channel 12 bits 9-12, bell motion at 0.2 deg/s).
		double rate = sim::craft::kDPSGimbalRate * dt;
		if (panel.gimbalEnable && panel.pgnsControl) {
			if (ch12 & Bit(9))
				fGimbalPitch -= rate;
			if (ch12 & Bit(10))
				fGimbalPitch += rate;
			if (ch12 & Bit(11))
				fGimbalRoll -= rate;
			if (ch12 & Bit(12))
				fGimbalRoll += rate;
		}
		fGimbalPitch = sim::Clamp(fGimbalPitch, -dps->gimbalLimit,
			dps->gimbalLimit);
		fGimbalRoll = sim::Clamp(fGimbalRoll, -dps->gimbalLimit,
			dps->gimbalLimit);
		dps->gimbalPitch = fGimbalPitch;
		dps->gimbalYaw = fGimbalRoll;
	}
	if (Thruster* aps = _APS())
		aps->on = fAPSLatch;

	// --- IMU coarse alignment (channel 12 bit 4, channel 14 bits 13-15) -------
	if ((ch12 & Bit(4)) && imu.powered) {
		const uint16_t cells[3] = {agc::REG_CDUXCMD, agc::REG_CDUYCMD,
			agc::REG_CDUZCMD};
		const int driveBits[3] = {15, 14, 13};
		Vec3 gimbals = imu.Gimbals(_Attitude());
		double* axes[3] = {&gimbals.x, &gimbals.y, &gimbals.z};
		bool moved = false;
		for (int i = 0; i < 3; i++) {
			if (!(ch14 & Bit(driveBits[i])))
				continue;
			int pending = agc::ToInt15(fComputer.Cell(cells[i]));
			fCoarseResidual[i] += kPulseRate * dt;
			int n = int(std::min(std::fabs(double(pending)),
				fCoarseResidual[i]));
			fCoarseResidual[i] -= n;
			if (n == 0)
				continue;
			int sign = pending > 0 ? 1 : -1;
			*axes[i] += sign * n * 4 * kCDUCount;   // a coarse-align pulse is 4 CDU counts
			fComputer.SetCell(cells[i], agc::FromInt15(pending - sign * n));
			moved = true;
		}
		if (moved)
			imu.SetGimbals(_Attitude(), gimbals);
	}

	// --- IMU fine alignment: gyro torquing (channel 14 bits 6-10) -------------
	if ((ch14 & Bit(10)) && (ch14 & Bit(6))) {
		int pending = agc::ToInt15(fComputer.Cell(agc::REG_GYROCTR));
		fGyroResidual += kPulseRate * dt;
		int n = int(std::min(std::fabs(double(pending)), fGyroResidual));
		fGyroResidual -= n;
		if (n > 0) {
			bool a = ch14 & Bit(8), b = ch14 & Bit(7);
			int axis = a ? (b ? 2 : 1) : (b ? 0 : -1);
			double sign = (ch14 & Bit(9)) ? -1.0 : 1.0;
			if (axis >= 0)
				imu.Torque(axis, sign * n * kGyroPulse);
			int s = pending > 0 ? 1 : -1;
			fComputer.SetCell(agc::REG_GYROCTR,
				agc::FromInt15(pending - s * n));
		}
	} else {
		fGyroResidual = 0;
	}

	// --- Altitude / altitude-rate tape meters (R10 through ALTM) -------------
	if (ch14 & Bit(3)) {
		int value = agc::ToInt15(fComputer.Cell(agc::REG_ALTM));
		if (ch14 & Bit(2))
			fTapeAltitude = value * 2.345;
		else
			fTapeRate = value * 0.5;
	}

	_Radar(dt, scene);
	_Thrusters();
}


void
LMPGNCS::_Thrusters()
{
	fJets5 = fComputer.Channel(005) & 0377;
	fJets6 = fComputer.Channel(006) & 0377;
	bool lgcHasJets = panel.pgnsControl && panel.mode != LMPanel::MODE_OFF;
	for (int ch = 5; ch <= 6; ch++) {
		uint16_t bits = ch == 5 ? fJets5 : fJets6;
		for (int bit = 1; bit <= 8; bit++) {
			const char* name = sim::craft::LMJetName(ch, bit);
			Thruster* t = name ? fVehicle->GetThruster(name) : nullptr;
			if (t == nullptr)
				continue;
			int index = (ch - 5) * 8 + bit - 1;
			// Channel 32 isolation pairs disable jets in hardware.
			// Channel 32 bit (0-7) isolating each jet, from the pairs in
			// Luminary's channel description (2&4, 5&8, 1&3, 6&7, 14&16,
			// 13&15, 9&12, 10&11) with jets numbered by quad.
			static const int pairOf[16] = {
				2, 0, 1, 3, 6, 7, 5, 4,     // ch5: jets 1 2 5 6 9 10 13 14
				3, 2, 5, 7, 6, 1, 0, 4      // ch6: jets 7 3 15 11 12 8 4 16
			};
			bool disabled = panel.thrusterPairOff[pairOf[index]];
			t->on = lgcHasJets && (bits & Bit(bit)) && !disabled
				&& !jetDriverFailed[index];
		}
	}
}


void
LMPGNCS::_Radar(double dt, const RadarScene* scene)
{
	uint16_t ch12 = fComputer.Channel(012);
	uint16_t ch13 = fComputer.Channel(013);

	// Antenna position: the LGC commands position 2 with channel 12 bit 13
	// (removing the command once the antenna is there); the antenna stays
	// put until commanded again. LR AUTO off drives it back to position 1.
	int wanted = fLRPosition;
	if (!panel.lrAuto)
		wanted = 1;
	else if (ch12 & Bit(13))
		wanted = 2;
	if (wanted != fLRPosition) {
		fLRPosition = wanted;
		fLRSlew = 4.0;
	}
	if (fLRSlew > 0)
		fLRSlew -= dt;

	// Beam geometry in the LM frame: antenna -> NB = Rx(alpha) Ry(beta).
	double alpha = 6.0 * sim::kDeg;
	double beta = (fLRPosition == 1 ? 24.0 : 0.0) * sim::kDeg;
	Mat3 antenna = Mat3::RotX(alpha) * Mat3::RotY(beta);
	// Columns of antenna^T are... we need antenna axes expressed in NB:
	// v_nb = antenna * v_ant for the frame-rotation convention used by
	// the flight software's *SMNB*.
	auto toInertial = [&](const Vec3& vAnt) {
		return _Attitude().Rotate(antenna * vAnt);
	};

	fLRRangeGood = false;
	fLRVelGood = false;
	double range = 0;
	Vec3 relVel;
	if (scene != nullptr && !lrFailed && fLRSlew <= 0) {
		// Luminary's HBEAMANT is the outbound range-beam direction.
		Vec3 beam = toInertial(kRangeBeamAntenna).Unit();
		if (scene->SlantRange(fVehicle->position, beam,
				kLRMaxRangeFt * kFeet * 1.2, &range)) {
			fLRRangeGood = range / kFeet < kLRMaxRangeFt;
			fLRVelGood = range / kFeet < kLRVelocityAcquireFt;
		}
		relVel = fVehicle->velocity - scene->SurfaceVelocity(fVehicle->position);
	}
	fLRAltitude = fLRRangeGood ? range : 0;

	// A read starts when the activity bit (channel 13 bit 4) is raised; the
	// radar interface drops it again when the data word is delivered, so
	// each further sample is another rising edge.
	uint16_t select = ch13 & 017;
	if ((select & Bit(4)) && !(fRadarSelect & Bit(4)))
		fRadarTimer = 0.08;
	else if (!(select & Bit(4)))
		fRadarTimer = -1;      // read cancelled (e.g. by a restart)
	fRadarSelect = select;
	if (fRadarTimer >= 0) {
		fRadarTimer -= dt;
		if (fRadarTimer < 0) {
			uint16_t word = 0;
			switch (select & 07) {
				case 07: {
					// LR altitude: 1.079 ft/bit in low scale, 5.395 ft/bit in
					// high scale (the LGC multiplies high-scale counts by 5).
					double ft = range / kFeet;
					double bits = ft < kLRLowScaleFt ? ft / 1.079
						: ft / (1.079 * 5.0);
					word = uint16_t(sim::Clamp(bits, 0, 077777));
					break;
				}
				case 04: case 05: case 06: {
					// LR velocity along an antenna axis, 12288-count bias.
					Vec3 axisAnt = (select & 07) == 04 ? Vec3(1, 0, 0)
						: ((select & 07) == 05 ? Vec3(0, 1, 0) : Vec3(0, 0, 1));
					double scale = (select & 07) == 04 ? -0.644
						: ((select & 07) == 05 ? 1.212 : 0.8668);
					double v = relVel.Dot(toInertial(axisAnt)) / kFeet;
					double bits = 12288.0 + v / scale;
					word = uint16_t(sim::Clamp(bits, 0, 077777));
					break;
				}
				default:
					word = 0;   // rendezvous radar (not fitted yet)
			}
			fComputer.RadarData(word);
			fComputer.SetInputChannel(013, uint16_t(fComputer.Channel(013)
				& ~Bit(4)));
			fRadarSelect &= uint16_t(~Bit(4));
		}
	}
}

}  // namespace gnc
