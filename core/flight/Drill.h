// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Training drills: one piece of a mission, handed to you at the moment it
// gets interesting. A drill starts from an ordinary start point and lets the
// automatic crew fly, as fast as the simulation goes, until the drill's
// moment comes (the S-IVB coasting with the LM still in the SLA, 500 ft
// over the site, the LM just in orbit, ...). Then the crew lets go and the
// controls are yours, with the real flight software in the state it would
// really be in.

#ifndef COLUMBIA_FLIGHT_DRILL_H
#define COLUMBIA_FLIGHT_DRILL_H

#include "FlightSim.h"
#include "MissionSpec.h"

#include <string>
#include <vector>

namespace flight {

class AutoCrew;
class MissionDirector;

enum class DrillId {
	None,
	TranspositionDocking,   // S-IVB coasting after TLI, LM in the SLA
	LOI,                    // the LOI-1 PAD is up, 20 minutes to TIG
	LastFiveHundred,        // 500 ft over the site, P64 still flying
	RendezvousInsertion,    // the LM just in orbit after the ascent
	TPI,                    // ten minutes to terminal phase initiation
	Docking                 // 60 m out, the CSM to dock
};

struct DrillSpec {
	DrillId id;
	const char* key;        // short name for files and the command line
	const char* name;
	const char* brief;
	StartPoint from;
	double maxHours;        // give up setting it up after this much GET
};

const std::vector<DrillSpec>& Drills();
const DrillSpec* FindDrill(DrillId id);
const DrillSpec* FindDrill(const std::string& key);
// Does this mission have the moment the drill needs?
bool DrillAvailable(const DrillSpec& drill, const MissionSpec& mission);
// Has the moment come?
bool DrillReady(const DrillSpec& drill, FlightSim& f, AutoCrew& crew,
	MissionDirector& director);

}  // namespace flight

#endif
