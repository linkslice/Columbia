// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Per-mission pad loads: the constants each mission's CMC and LGC were
// loaded with before flight (see tools/gen_mission_pads.sh).

#ifndef COLUMBIA_MISSIONPADS_H
#define COLUMBIA_MISSIONPADS_H

#include <cstdint>

namespace gnc {

struct PadWord {
	const char* name;     // erasable symbol
	uint8_t index;        // word within it
	uint16_t value;       // 15-bit contents
};

// Terminated by a null name. Null when the mission has no such computer.
const PadWord* MissionPads(int mission, bool lm);

class PadLoader;
// Luminary 1E (Apollo 14 on) keeps the descent targets as X and Z pairs
// (RBRFGX, RBRFGZ, ...) where earlier programs had three-axis vectors
// (RBRFG ...): loads a table's vectors into that layout. Returns the
// words loaded (0 when the program uses the vectors).
int ApplyDescentTargetsXZ(PadLoader& pad, const PadWord* table);

}  // namespace gnc

#endif
