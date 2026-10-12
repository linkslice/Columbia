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
#include <map>
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

// A Flight Director's call: the situation, the options with what each
// costs, and which one was taken on the real flight.
struct DecisionOption {
	std::string key;
	std::string label;
	std::string detail;
	bool historical = false;
};

struct Decision {
	std::string id;
	std::string title;
	std::string situation;
	std::vector<DecisionOption> options;
	double postedGET = 0;
	int chosen = -1;
	std::string chosenBy;      // "you", "Flight (as flown)", "preset"
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

	// Flight's calls. With waitForDecisions (the app sets it) a decision
	// holds the timeline until Decide() -- the session also stops the
	// clock. Otherwise a preset choice (presetChoices, or COLUMBIA_CHOICES
	// "id=key,id=key") or the historical one is taken at once.
	bool waitForDecisions = false;
	std::map<std::string, std::string> presetChoices;
	std::function<void(const Decision&)> onDecision;
	const Decision* PendingDecision() const;
	bool Decide(const std::string& id, const std::string& key,
		const std::string& by = "you");
	const std::vector<Decision>& Decisions() const { return fDecisions; }
	// The flight is now an abort (no landing, no lunar orbit).
	bool Aborted() const { return fAborted; }
	bool Lifeboat() const { return fLifeboat; }
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
	// The way home after TEI or an abort: midcourses, (in the lifeboat) SM
	// jettison, crew to the CM and LM jettison, then entry.
	void _AddEarthReturn();
	// A Flight Director decision; `then` runs with the chosen key.
	void _Ask(Decision d, std::function<void(const std::string&)> then);
	// Steps added while a step runs join the timeline on the next tick.
	void _AddLater(const std::string& name, double get, std::function<void()> run,
		std::function<bool()> when = nullptr);
	void _CheckEmergencies();
	void _SMDamageAbort(const std::string& choice);
	void _AddLifeboatHome(bool pc2Decision);
	void _EnterLifeboat();

	FlightSim& f;
	AutoCrew& fCrew;
	std::vector<Step> fSteps;
	std::vector<ManeuverPAD> fPADs;
	bool fInstalled = false;
	double fTLICutoff = 0;
	double fLOI = 0;
	double fTEIGiveUp = 0;     // stop re-trying TEI targeting after this GET
	double fDOI = 0, fCirc = 0, fPDI = 0;
	double fLanded = 0, fDocked = 0, fEI = 0;
	double fSplashLat = 0, fSplashLon = 0;
	std::vector<Decision> fDecisions;
	std::vector<std::function<void(const std::string&)>> fDecisionThen;
	std::vector<Step> fLater;
	bool fAborted = false, fLifeboat = false, fEmergencyAsked = false;
	bool fBuilt = false;            // steps added from now on wait a tick
	double fPericynthion = 0;       // GET of the predicted closest approach
};

}  // namespace flight

#endif
