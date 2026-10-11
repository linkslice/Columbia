// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The Saturn Instrument Unit: the launch vehicle's own guidance computer
// (the LVDC), flying the Saturn V or IB from the pad to orbit and, on a
// lunar mission, the S-IVB's second burn out of Earth orbit.
//
// Boost follows the real sequence: hold-down release, a time-based tilt
// program through the S-IC (or S-IB) burn, center-engine cutoff, staging
// on propellant depletion, then iterative guidance through the S-II and
// S-IVB to the parking orbit. Steering goes through the engine gimbals,
// so an engine out leaves the others to trim the torque — or not.
//
// Translunar injection is targeted the way the RTCC fed the LVDC: a
// free-return pericynthion from the full force model, flown as a
// velocity-to-be-gained burn toward a fixed aim point.

#ifndef COLUMBIA_SATURN_H
#define COLUMBIA_SATURN_H

#include "Trajectory.h"
#include "../sim/Vehicle.h"

#include <string>

namespace flight {

class FlightSim;

class SaturnIU {
public:
	explicit SaturnIU(FlightSim& sim);

	enum class Mode {
		Prelaunch,       // engines start at T-8.9 s; hold-down until T-0
		FirstStage,      // S-IC (or S-IB) tilt program
		SecondStage,     // S-II, iterative guidance
		ThirdStage,      // S-IVB first burn to orbit
		Orbit,           // parking orbit, local-horizontal hold
		TLIPrep,         // time base 6: restart preparations
		TLI,             // S-IVB second burn
		Coast,           // after TLI: inertial hold, T&D, slingshot
		Safed            // propellants dumped, IU off
	};

	void Step(double dt);
	Mode CurrentMode() const { return fMode; }
	const char* ModeName() const;
	// The vehicle carrying the IU (the stack until separation).
	sim::Vehicle* Carrier();

	// Guidance quantities for the displays.
	double TimeToGo() const { return fTGo; }
	double TLIIgnitionGET() const { return fTIG; }
	sim::Vec3 VelocityToGain() const { return fVGo; }

	// --- Crew and ground ---------------------------------------------------
	bool tliInhibit = false;         // TLI INHIBIT switch on the MDC
	bool autoTowerJettison = true;   // crew presses TWR JETT on cue
	// "Go for TLI": the ground can retarget or scrub the burn before TB6.
	bool tliEnabled = true;

	// --- Failures ------------------------------------------------------------
	// Shuts an engine down (by name, e.g. "S-II J-2 5", "F-1 3").
	bool FailEngine(const std::string& name, bool logIt = true);
	bool guidanceFailed = false;     // platform failure: no steering

	// Insertion targets (altitudes, m).
	double targetPerigee = 185200;
	double targetApogee = 185200;

private:
	void _Prelaunch();
	void _FirstStage(double dt);
	void _UpperStages(double dt);
	void _Orbit(double dt);
	void _PrepareTLI();
	void _TLI(double dt);
	void _Steer(sim::Vehicle& v, const sim::Vec3& dir, const sim::Vec3& down,
		double maxRate, double dt);
	void _Ignite(sim::Vehicle& v, const std::string& module);
	void _Cutoff(sim::Vehicle& v, const std::string& module);
	bool _StageDepleted(sim::Vehicle& v, const std::string& module) const;
	double _StageThrust(const sim::Vehicle& v, const std::string& module) const;
	double _TimeForDV(sim::Vehicle& v, double dv) const;
	sim::Vec3 _IGM(sim::Vehicle& v);

	FlightSim& f;
	Mode fMode = Mode::Prelaunch;
	bool fSaturnV = true;
	bool fIgnited = false;
	double fLiftoffGET = 0;
	sim::Vec3 fLaunchUp, fLaunchDown;   // inertial reference at liftoff
	sim::Vec3 fPlane;                   // target plane normal
	double fEventGET = 0;               // last staging event
	int fSequence = 0;                  // step within the staging sequence
	bool fCenterOut = false;
	sim::Vec3 fFrozenDir;
	bool fFrozen = false;
	double fTGo = 0;
	// TLI.
	double fTIG = 0;
	State fAim;
	sim::Vec3 fBias;
	sim::Vec3 fVGo;
	double fBestVGo = 1e9;
	double fTLICutoffGET = 0;
	bool fSlingshot = false;
	double fExtractGET = 0;
	sim::Vec3 fHoldDir;
};

}  // namespace flight

#endif
