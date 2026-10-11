// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Mission definitions: the flown Apollo missions (7-17) with their dates,
// crews, spacecraft, flight software, landing sites and flight-plan
// times, plus player-built missions that reuse the same structure.

#ifndef COLUMBIA_MISSIONSPEC_H
#define COLUMBIA_MISSIONSPEC_H

#include <string>
#include <vector>

namespace flight {

enum class MissionType {
	EarthOrbitCSM,      // C: Apollo 7
	EarthOrbitLM,       // D: Apollo 9
	LunarOrbit,         // C': Apollo 8
	LunarOrbitLM,       // F: Apollo 10 dress rehearsal
	Landing,            // G/H: Apollo 11-14
	LandingExtended     // J: Apollo 15-17 (rover, longer stay)
};

struct LandingSite {
	std::string name;
	double latitude = 0, longitude = 0;   // degrees
	double radius = 1737400;              // m, landing-site radius
	double sunElevation = 10;             // degrees at the planned landing
};

// The nominal flight plan, as ground elapsed times (s). Zero = not part
// of this mission.
struct FlightPlan {
	double tli = 0;
	double loi = 0;
	double loi2 = 0;          // circularization (DOI for J missions is CSM)
	double undock = 0;
	double doi = 0;
	double pdi = 0;
	double landing = 0;
	double liftoff = 0;       // lunar ascent
	double docking = 0;
	double tei = 0;
	double entry = 0;         // entry interface
	double splashdown = 0;
	int evas = 0;             // lunar surface EVAs
	double evaHours = 0;      // total EVA time
};

struct MissionSpec {
	int number = 0;                 // 7..17, or 0 for a custom mission
	std::string name;               // "Apollo 11"
	std::string motto;              // one line for the mission picker
	MissionType type = MissionType::Landing;
	std::string commander, cmPilot, lmPilot;
	std::string csmCallsign, lmCallsign;
	bool saturnV = true;            // false: Saturn IB (Apollo 7)
	// Launch: UTC date-time and pad (LC-39A/B or LC-34).
	int year = 1969, month = 7, day = 16, hour = 13, minute = 32;
	double second = 0;
	std::string pad = "LC-39A";
	double launchAzimuth = 72;      // degrees
	// Flight software (rope/ names).
	std::string cmcProgram = "Comanche055";
	std::string lgcProgram = "Luminary099";
	int padLoads = 11;              // which mission's pad loads to apply
	LandingSite site;
	FlightPlan plan;
	// Splashdown target.
	double splashLatitude = 0, splashLongitude = 0;
	// Masses (kg).
	double csmSpsPropellant = 18410;
	double lmDescentPropellant = 8248;
	double lmAscentPropellant = 2353;
	double lmDescentDry = 2034;
	double lmAscentDry = 2445;
	// What really happened: contingencies that occurred on this flight
	// (Contingency ids), offered as "fly it as flown".
	std::vector<std::string> historical;
	std::string summary;            // a paragraph for the briefing
};

// The flown missions, Apollo 7 to 17.
const std::vector<MissionSpec>& ApolloMissions();
const MissionSpec* FindMission(int number);

// A player mission based on the J-mission hardware.
MissionSpec CustomMission(const std::string& name, const LandingSite& site,
	int year, int month, int day);

// Notable landing sites (flown and studied) for the mission builder.
const std::vector<LandingSite>& CandidateSites();

}  // namespace flight

#endif
