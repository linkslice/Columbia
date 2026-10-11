// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The CSM Primary Guidance, Navigation and Control System as Colossus,
// Comanche and Artemis see it: IMU, CDUs, PIPAs (5.85 cm/s), the rotational
// and translational hand controllers, the SPS engine-on discrete, SPS
// thrust-vector control through the optics CDU error counters (85.41
// arcsec of actuator command per pulse), RCS jets (SM quads, then the CM's
// own jets after separation), and the discretes the boost and entry
// programs watch (liftoff, guidance reference release, CM/SM separation).

#ifndef COLUMBIA_CMPGNCS_H
#define COLUMBIA_CMPGNCS_H

#include "../agc/AGC.h"
#include "../sim/Vehicle.h"
#include "IMU.h"

namespace gnc {

struct CMPanel {
	// Rotational hand controller, -1..1 per axis (x roll, y pitch, z yaw:
	// CSM body axes). The CMC reads only its breakout switches.
	sim::Vec3 rhc;
	bool minimumImpulse = false;    // RHC used as the minimum-impulse controller
	// Translational hand controller: -1, 0 or 1 per body axis.
	int thc[3] = {0, 0, 0};
	// SC CONT switch: CMC (true) or SCS.
	bool scContCMC = true;
	// CMC MODE switch.
	enum Mode { MODE_FREE, MODE_HOLD, MODE_AUTO } mode = MODE_AUTO;
	bool spsArmed = false;          // delta-V thrust switches A/B
	bool spsBankA = true;           // ...with bank A (false: B only)
	bool spsStart = false;          // THRUST ON button (SCS)
	bool spsStop = false;           // delta-V thrust switches off
	bool tvcEnable = true;          // gimbal motors on
	bool imuCage = false;
	bool liftoff = false;           // channel 30 bit 5
	bool guidanceRelease = false;   // channel 30 bit 6 (GRR)
	bool cmSmSeparated = false;     // channel 30 bit 2
	bool sivbSeparated = false;     // channel 30 bit 4 (S-IVB sep/abort)
	bool ullage = false;            // channel 30 bit 1
};

class CMPGNCS {
public:
	CMPGNCS(agc::Computer& computer, sim::Vehicle& vehicle);

	IMU imu;
	CMPanel panel;

	// Re-homes the hardware on another physical vehicle (docking and
	// undocking merge and split vehicles); `frame` rotates this
	// spacecraft's body axes into that vehicle's frame.
	void Attach(sim::Vehicle& vehicle, const sim::Quat& frame = sim::Quat());
	sim::Vehicle& Vehicle() { return *fVehicle; }

	// Advances the computer and the hardware by dt (keep dt <= 10 ms),
	// after the vehicle's physics step for the same interval.
	void Step(double dt);

	// Optics mark / reject buttons (channel 16).
	void Mark(bool reject = false);

	bool SPSOn() const { return fSPSOn; }
	double GimbalPitch() const { return fGimbalPitch; }   // rad
	double GimbalYaw() const { return fGimbalYaw; }       // rad
	double CommandPitch() const { return fCmdPitch; }
	double CommandYaw() const { return fCmdYaw; }
	bool GimbalLockWarning() const { return fGimbalWarning; }
	uint16_t JetBits(int channel) const
		{ return channel == 5 ? fJets5 : fJets6; }

	// Failures injected by the simulation.
	bool spsFailed = false;         // won't ignite (or shuts down)
	bool spsThrustShortA = false;   // Apollo 15: bank A fires the SPS when armed
	bool gimbalFailedPitch = false; // actuator frozen
	bool gimbalFailedYaw = false;
	bool jetDriverFailed[16] = {};

private:
	void _Inputs();
	void _Outputs(double dt);
	void _Thrusters();
	sim::Thruster* _SPS();

	agc::Computer& fComputer;
	sim::Vehicle* fVehicle;
	sim::Quat fFrame;             // this spacecraft's body -> vehicle frame
	sim::Quat _Attitude() const { return fVehicle->attitude * fFrame; }

	bool fSPSOn = false;
	double fTVCResidual[2] = {0, 0};
	long fTVCCounts[2] = {0, 0};    // accumulated actuator command, pulses
	double fCmdPitch = 0, fCmdYaw = 0;
	double fGimbalPitch = 0, fGimbalYaw = 0;
	double fCoarseResidual[3] = {0, 0, 0};
	double fGyroResidual = 0;
	bool fGimbalWarning = false;
	uint16_t fJets5 = 0, fJets6 = 0;
};

}  // namespace gnc

#endif
