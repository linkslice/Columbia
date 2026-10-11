// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The LM Primary Guidance, Navigation and Control System as Luminary sees
// it: every input and output channel bit, counter and interrupt between
// the LGC and the spacecraft, following Luminary's own channel
// descriptions and scale factors (THRUST pulses 0.0798/N, PIPA 1 cm/s,
// landing radar 1.079 ft/bit and 0.644/1.212/0.8668 ft/s per bit).

#ifndef COLUMBIA_LMPGNCS_H
#define COLUMBIA_LMPGNCS_H

#include "../agc/AGC.h"
#include "../sim/Vehicle.h"
#include "IMU.h"

namespace gnc {

// What the landing radar sees.
class RadarScene {
public:
	virtual ~RadarScene() {}
	// Distance along an inertial ray to the surface; false if no return.
	virtual bool SlantRange(const sim::Vec3& from, const sim::Vec3& direction,
		double maxRange, double* range) const = 0;
	// Inertial velocity of the surface near a point.
	virtual sim::Vec3 SurfaceVelocity(const sim::Vec3& point) const = 0;
};

struct LMPanel {
	// Attitude controller assembly: rate command, -1..1 per axis
	// (x = yaw about LM X, y = pitch about Y, z = roll about Z).
	sim::Vec3 aca;
	bool acaPulse = false;         // minimum-impulse mode
	// Thrust/translation controller: -1, 0 or 1 per axis.
	int thc[3] = {0, 0, 0};
	double manualThrottle = 0;     // 0..1 above the 10% minimum
	bool autoThrottle = true;
	bool pgnsControl = true;       // guidance control switch: PGNS / AGS
	enum Mode { MODE_OFF, MODE_ATT_HOLD, MODE_AUTO } mode = MODE_AUTO;
	enum Arm { ARM_OFF, ARM_DESCENT, ARM_ASCENT } engineArm = ARM_OFF;
	bool abort = false;            // ABORT pushbutton (DPS abort)
	bool abortStage = false;       // ABORT STAGE pushbutton
	bool engineStart = false;      // manual START pushbutton
	bool engineStop = false;       // manual STOP pushbutton
	bool gimbalEnable = true;      // descent engine gimbal enable
	bool lrAuto = true;            // LR antenna slew switch in AUTO
	bool thrusterPairOff[8] = {};  // RCS isolation (channel 32 bits 1-8)
	bool ascentFeed = false;       // RCS ASC FEED A/B open: jets on APS propellant
	bool imuCage = false;
};

class LMPGNCS {
public:
	LMPGNCS(agc::Computer& computer, sim::Vehicle& vehicle);

	IMU imu;
	LMPanel panel;

	// Re-homes the hardware on another physical vehicle (docking and
	// undocking merge and split vehicles); `frame` rotates this
	// spacecraft's body axes into that vehicle's frame.
	void Attach(sim::Vehicle& vehicle, const sim::Quat& frame = sim::Quat());
	sim::Vehicle& Vehicle() { return *fVehicle; }

	// Advances the computer and the hardware by dt (keep dt <= 10 ms).
	// Call after the vehicle's physics step for the same interval.
	void Step(double dt, const RadarScene* scene);

	// Rate-of-descent switch (P66): +1 slows the descent, -1 speeds it.
	void RateOfDescentClick(int direction);
	// AOT mark buttons (X, Y, reject) via channel 16.
	void Mark(int which);

	// --- State for displays and the simulation ------------------------------
	bool DescentEngineOn() const { return fEngineLatch; }
	bool AscentEngineOn() const { return fAPSLatch; }
	double ThrustCommand() const { return fThrustCommand; }   // N
	double GimbalPitch() const { return fGimbalPitch; }
	double GimbalRoll() const { return fGimbalRoll; }
	bool LRRangeGood() const { return fLRRangeGood; }
	bool LRVelocityGood() const { return fLRVelGood; }
	double LRAltitude() const { return fLRAltitude; }         // m, slant
	int LRPosition() const { return fLRPosition; }
	bool GimbalLockWarning() const { return fGimbalWarning; }
	// Altitude and altitude rate the LGC sent to the tape meters (R10).
	double TapeAltitude() const { return fTapeAltitude; }      // ft
	double TapeAltitudeRate() const { return fTapeRate; }      // ft/s
	uint16_t JetBits(int channel) const
		{ return channel == 5 ? fJets5 : fJets6; }

	// Failures and hardware faults injected by the simulation.
	bool lrFailed = false;
	bool ascentArmBreakerOpen = false;   // Apollo 11's broken breaker
	// Apollo 11: with the rendezvous radar switch in SLEW/AUTO-TRACK and
	// the RR CDUs' 800 Hz excitation out of phase with the LGC's, the
	// shaft and trunnion CDUs hunt continuously, each stealing a counter
	// cycle at 6.4 kpps -- some 15% of the computer's time.
	bool rrCDUHunting = false;
	double huntScale = 1.0;         // test knob
	bool throttleFailed = false;    // DECA stuck
	bool jetDriverFailed[16] = {};

private:
	void _Inputs();
	void _Outputs(double dt, const RadarScene* scene);
	void _Radar(double dt, const RadarScene* scene);
	void _Thrusters();
	sim::Thruster* _DPS();
	sim::Thruster* _APS();

	agc::Computer& fComputer;
	sim::Vehicle* fVehicle;
	int fJetTank[16] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		-1, -1};
	sim::Quat fFrame;             // this spacecraft's body -> vehicle frame
	sim::Quat _Attitude() const { return fVehicle->attitude * fFrame; }

	// Engine and throttle.
	bool fEngineLatch = false;
	bool fAPSLatch = false;
	uint16_t fLastCh11 = 0;
	double fDecaIntegrator = 0;   // auto throttle, N
	double fThrustCommand = 0;
	double fThrustActual = 0;
	double fGimbalPitch = 0, fGimbalRoll = 0;

	// IMU drive.
	double fCoarseResidual[3] = {0, 0, 0};
	double fGyroResidual = 0;
	bool fGimbalWarning = false;
	double fHuntResidual = 0;

	// Landing radar.
	int fLRPosition = 1;
	double fLRSlew = 0;
	bool fLRRangeGood = false, fLRVelGood = false;
	double fLRAltitude = 0;
	uint16_t fRadarSelect = 0;
	double fRadarTimer = -1;

	// Displays.
	double fTapeAltitude = 0, fTapeRate = 0;

	uint16_t fJets5 = 0, fJets6 = 0;
	double fThrottleDrain = 0;
};

}  // namespace gnc

#endif
