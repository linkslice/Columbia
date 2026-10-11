// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "CMPGNCS.h"

#include "../sim/Spacecraft.h"

#include <algorithm>
#include <cmath>

namespace gnc {

using sim::Thruster;
using sim::Vec3;

namespace {

inline uint16_t
Bit(int n)
{
	return uint16_t(1u << (n - 1));
}

const double kPIPAPulseCM = 0.0585;               // m/s
const double kGyroPulse = 6.28318530717958647692 / 2097152.0;
const double kCDUCount = 6.28318530717958647692 / 32768.0;
const double kPulseRate = 3200.0;
const double kActuatorPulse = 85.41 / 3600.0 * 3.14159265358979323846 / 180.0;
const double kGimbalRate = 10.0 * 3.14159265358979323846 / 180.0;  // rad/s

}  // namespace


CMPGNCS::CMPGNCS(agc::Computer& computer, sim::Vehicle& vehicle)
	:
	fComputer(computer),
	fVehicle(&vehicle)
{
}


void
CMPGNCS::Attach(sim::Vehicle& vehicle, const sim::Quat& frame)
{
	fVehicle = &vehicle;
	fFrame = frame;
}


Thruster*
CMPGNCS::_SPS()
{
	return fVehicle->GetThruster("SPS");
}


void
CMPGNCS::Mark(bool reject)
{
	fComputer.NavKey(reject ? Bit(7) : Bit(6));
}


// ---------------------------------------------------------------------------
// Inputs (channels 30-32 are active low)


void
CMPGNCS::_Inputs()
{
	auto set = [&](int ch, int bit, bool active) {
		fComputer.SetInputBit(ch, bit, !active);
	};

	// Channel 30.
	set(030, 1, panel.ullage);
	set(030, 2, panel.cmSmSeparated);
	set(030, 3, panel.spsArmed);
	set(030, 4, panel.sivbSeparated);
	set(030, 5, panel.liftoff);
	set(030, 6, panel.guidanceRelease);
	set(030, 7, false);                       // optics CDU fail
	set(030, 9, imu.powered && !imu.failed);  // IMU operate
	set(030, 10, false);                      // S/C control of Saturn
	set(030, 11, panel.imuCage);
	set(030, 12, imu.cduFailed);
	set(030, 13, imu.failed);
	set(030, 14, false);                      // ISS turn-on requested
	set(030, 15, imu.powered);                // IMU temperature in limits

	// Channel 31: hand controllers and the CMC mode switch.
	const double detent = 0.15;
	bool impulse = panel.minimumImpulse;
	set(031, 1, !impulse && panel.rhc.y > detent);
	set(031, 2, !impulse && panel.rhc.y < -detent);
	set(031, 3, !impulse && panel.rhc.z > detent);
	set(031, 4, !impulse && panel.rhc.z < -detent);
	set(031, 5, !impulse && panel.rhc.x > detent);
	set(031, 6, !impulse && panel.rhc.x < -detent);
	set(031, 7, panel.thc[0] > 0);
	set(031, 8, panel.thc[0] < 0);
	set(031, 9, panel.thc[1] > 0);
	set(031, 10, panel.thc[1] < 0);
	set(031, 11, panel.thc[2] > 0);
	set(031, 12, panel.thc[2] < 0);
	set(031, 13, panel.mode == CMPanel::MODE_HOLD);
	set(031, 14, panel.mode == CMPanel::MODE_FREE);
	set(031, 15, panel.scContCMC && imu.powered);   // G&N autopilot control

	// Channel 32: minimum-impulse controller.
	set(032, 1, impulse && panel.rhc.y > detent);
	set(032, 2, impulse && panel.rhc.y < -detent);
	set(032, 3, impulse && panel.rhc.z > detent);
	set(032, 4, impulse && panel.rhc.z < -detent);
	set(032, 5, impulse && panel.rhc.x > detent);
	set(032, 6, impulse && panel.rhc.x < -detent);
}


// ---------------------------------------------------------------------------
// Step


void
CMPGNCS::Step(double dt)
{
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
	imu.Sense(fVehicle->SensedDeltaV(), kPIPAPulseCM, counts);
	fComputer.AddCounts(agc::REG_PIPAX, counts[0]);
	fComputer.AddCounts(agc::REG_PIPAY, counts[1]);
	fComputer.AddCounts(agc::REG_PIPAZ, counts[2]);

	_Inputs();
	fComputer.Run(dt);
	_Outputs(dt);
}


void
CMPGNCS::_Outputs(double dt)
{
	uint16_t ch11 = fComputer.Channel(011);
	uint16_t ch12 = fComputer.Channel(012);
	uint16_t ch14 = fComputer.Channel(014);

	// --- SPS: channel 11 bit 13 through the delta-V thrust switches ----------
	bool cmcOn = panel.scContCMC && (ch11 & Bit(13));
	fSPSOn = panel.spsArmed && !panel.spsStop && !spsFailed
		&& (cmcOn || panel.spsStart);

	// --- TVC: optics CDU error counters as actuator commands -----------------
	// Channel 12 bit 2 enables the error counters (clearing them when it
	// drops), bit 8 enables TVC; channel 14 bits 11 (pitch, CDUSCMD) and
	// 12 (yaw, CDUTCMD) release pulses at 3200 per second.
	bool countersOn = (ch12 & Bit(2)) != 0;
	bool tvc = (ch12 & Bit(8)) != 0;
	if (!countersOn) {
		fTVCCounts[0] = fTVCCounts[1] = 0;
		fTVCResidual[0] = fTVCResidual[1] = 0;
	} else {
		const uint16_t cells[2] = {agc::REG_CDUSCMD, agc::REG_CDUTCMD};
		const int bits[2] = {11, 12};
		for (int i = 0; i < 2; i++) {
			if (!(ch14 & Bit(bits[i])))
				continue;
			int pending = agc::ToInt15(fComputer.Cell(cells[i]));
			fTVCResidual[i] += kPulseRate * dt;
			int n = int(std::min(std::fabs(double(pending)), fTVCResidual[i]));
			fTVCResidual[i] -= n;
			if (n == 0)
				continue;
			int sign = pending > 0 ? 1 : -1;
			fTVCCounts[i] += sign * n;
			fComputer.SetCell(cells[i], agc::FromInt15(pending - sign * n));
		}
	}
	if (tvc && panel.tvcEnable) {
		fCmdPitch = fTVCCounts[0] * kActuatorPulse;
		fCmdYaw = fTVCCounts[1] * kActuatorPulse;
	}
	Thruster* sps = _SPS();
	double limit = sps ? sps->gimbalLimit : 6.0 * sim::kDeg;
	fCmdPitch = sim::Clamp(fCmdPitch, -limit, limit);
	fCmdYaw = sim::Clamp(fCmdYaw, -limit, limit);
	double step = kGimbalRate * dt;
	if (!gimbalFailedPitch)
		fGimbalPitch += sim::Clamp(fCmdPitch - fGimbalPitch, -step, step);
	if (!gimbalFailedYaw)
		fGimbalYaw += sim::Clamp(fCmdYaw - fGimbalYaw, -step, step);
	if (sps) {
		sps->on = fSPSOn;
		sps->gimbalPitch = fGimbalPitch;
		sps->gimbalYaw = fGimbalYaw;
	}

	// --- IMU coarse alignment (channel 12 bit 4, channel 14 bits 13-15) -------
	if (ch12 & Bit(4)) {
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
			int n = int(std::min(std::fabs(double(pending)), fCoarseResidual[i]));
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

	// --- Gyro torquing (channel 14 bits 6-10) ---------------------------------
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
			fComputer.SetCell(agc::REG_GYROCTR, agc::FromInt15(pending - s * n));
		}
	} else {
		fGyroResidual = 0;
	}

	_Thrusters();
}


void
CMPGNCS::_Thrusters()
{
	fJets5 = fComputer.Channel(005) & 0377;
	fJets6 = fComputer.Channel(006) & 0377;
	bool cmcHasJets = panel.scContCMC;
	// The same channel bits drive the SM quads until separation and the
	// CM's jets after it (the RCS transfer to CM happens at CM/SM SEP).
	for (int ch = 5; ch <= 6; ch++) {
		uint16_t bits = ch == 5 ? fJets5 : fJets6;
		for (int bit = 1; bit <= 8; bit++) {
			int index = (ch - 5) * 8 + bit - 1;
			bool on = cmcHasJets && (bits & Bit(bit)) && !jetDriverFailed[index];
			if (const char* name = sim::craft::CSMJetName(ch, bit))
				if (Thruster* t = fVehicle->GetThruster(name))
					t->on = on && !panel.cmSmSeparated;
			if (const char* name = sim::craft::CMJetName(ch, bit))
				if (Thruster* t = fVehicle->GetThruster(name))
					t->on = on && panel.cmSmSeparated;
		}
	}
}

}  // namespace gnc
