// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Mission Control and the flight plan. The director walks the mission's
// timeline: when a maneuver comes due, the RTCC computes it on the full
// force model from the tracked state and CAPCOM reads up the PAD. With
// the automatic crew engaged it also hands each step to the crew; flying
// yourself, the PADs and the go/no-go calls still come up from Houston.

#ifndef COLUMBIA_MISSIONDIRECTOR_H
#define COLUMBIA_MISSIONDIRECTOR_H

#include "AutoCrew.h"
#include "Trajectory.h"

#include <functional>
#include <string>
#include <vector>

namespace flight {

class FlightSim;

// A maneuver PAD as CAPCOM reads it up.
struct ManeuverPAD {
	std::string purpose;       // "LOI-1", "MCC-2", ...
	double tig = 0;            // GET
	sim::Vec3 dvLVLH;          // m/s (N81)
	double dvMagnitude = 0;    // m/s
	double burnTime = 0;       // s
	std::string engine;        // "SPS", "RCS", "DPS", "APS", "S-IVB"
	std::string Text() const;  // the PAD in Houston's units
};

class MissionDirector {
public:
	MissionDirector(FlightSim& sim, AutoCrew& crew);

	// Hand the flying to the automatic crew (or take it back).
	bool autopilot = true;

	// Drive the timeline (call once a second; Install() does it).
	void Install();
	void Tick();

	// Where the flight plan has got to, for the displays.
	std::string NextEvent() const;
	double NextEventGET() const;
	const std::vector<ManeuverPAD>& PADs() const { return fPADs; }
	std::function<void(const ManeuverPAD&)> onPAD;

	// The step list (public for tests).
	struct Step {
		std::string name;
		double get;                                   // earliest GET
		std::function<bool()> when;                   // extra condition
		std::function<void()> run;                    // do it
		bool done = false;
	};
	const std::vector<Step>& Steps() const { return fSteps; }
	// Where the recovery forces are waiting (deg; the plan's point until
	// the entry PAD moves them).
	double SplashLatitude() const { return fSplashLat; }
	double SplashLongitude() const { return fSplashLon; }

private:
	void _Build();
	void _Add(const std::string& name, double get, std::function<void()> run,
		std::function<bool()> when = nullptr);
	// RTCC helpers: an impulsive delta-V at `impulseGET` turned into a PAD
	// and (with the autopilot) a crew burn.
	void _Burn(const std::string& purpose, double impulseGET,
		const sim::Vec3& dvInertial, bool docked);
	State _Now() const;
	sim::Vec3 _LunarPlane() const;
	sim::Vec3 _SiteAtLanding() const;
	void _PlanDescent();
	void _AddSurface();
	void _AddReturn();

	FlightSim& f;
	AutoCrew& fCrew;
	std::vector<Step> fSteps;
	std::vector<ManeuverPAD> fPADs;
	bool fInstalled = false;
	double fTLICutoff = 0;
	double fLOI = 0;
	double fDOI = 0, fCirc = 0, fPDI = 0;
	double fLanded = 0, fDocked = 0, fEI = 0;
	double fSplashLat = 0, fSplashLon = 0;
};

}  // namespace flight

#endif
