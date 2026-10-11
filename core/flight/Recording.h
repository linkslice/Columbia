// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// A flight recording: what a mission looked like from outside, sampled as
// it was flown, so it can be replayed without running the simulation (the
// screensaver, the post-flight report, "watch the replay"). It holds the
// spacecraft states, both DSKYs, the phases and every log line. This file
// depends only on the math library: a player doesn't need the AGC.

#ifndef COLUMBIA_RECORDING_H
#define COLUMBIA_RECORDING_H

#include "../sim/Math.h"

#include <cstdint>
#include <string>
#include <vector>

namespace flight {

// Which pieces make up a recorded vehicle (for drawing it).
enum RecModule : uint16_t {
	REC_SIC = 1 << 0, REC_SII = 1 << 1, REC_SIVB = 1 << 2, REC_SIB = 1 << 3,
	REC_SM = 1 << 4, REC_CM = 1 << 5, REC_LM_DESCENT = 1 << 6,
	REC_LM_ASCENT = 1 << 7, REC_TOWER = 1 << 8
};

enum RecFlag : uint8_t {
	REC_PRESENT = 1 << 0, REC_ENGINE = 1 << 1, REC_LANDED = 1 << 2,
	REC_DROGUES = 1 << 3, REC_MAINS = 1 << 4, REC_RCS = 1 << 5
};

enum RecVehicleSlot { REC_CSM = 0, REC_LM = 1, REC_BOOSTER = 2, REC_SLOTS = 3 };

struct RecVehicle {
	uint8_t flags = 0;
	uint16_t modules = 0;
	sim::Vec3 r;          // Earth-centred inertial (m)
	sim::Vec3 v;          // m/s
	sim::Quat q;          // body -> inertial
	bool Present() const { return flags & REC_PRESENT; }
};

struct RecSample {
	double get = 0;
	uint8_t phase = 0;            // flight::Phase
	sim::Vec3 moon;               // Moon centre (m, Earth-centred)
	sim::Vec3 moonV;
	double altitude = 0;          // of the crewed vehicle, above Earth or Moon
	double speed = 0;             // relative to that body (m/s)
	RecVehicle veh[REC_SLOTS];
};

// A DSKY as displayed.
struct RecDisplay {
	double get = 0;
	char prog[2] = {' ', ' '};
	char verb[2] = {' ', ' '};
	char noun[2] = {' ', ' '};
	char reg[3][5] = {{' ', ' ', ' ', ' ', ' '}, {' ', ' ', ' ', ' ', ' '},
		{' ', ' ', ' ', ' ', ' '}};
	char sign[3] = {' ', ' ', ' '};
	uint32_t lamps = 0;           // RecLamp bits
	bool SameAs(const RecDisplay& o) const;
};

enum RecLamp : uint32_t {
	LAMP_COMP_ACTY = 1 << 0, LAMP_UPLINK = 1 << 1, LAMP_TEMP = 1 << 2,
	LAMP_KEY_REL = 1 << 3, LAMP_FLASH = 1 << 4, LAMP_OPR_ERR = 1 << 5,
	LAMP_NO_ATT = 1 << 6, LAMP_GIMBAL_LOCK = 1 << 7, LAMP_PROG = 1 << 8,
	LAMP_TRACKER = 1 << 9, LAMP_RESTART = 1 << 10, LAMP_STBY = 1 << 11,
	LAMP_ALT = 1 << 12, LAMP_VEL = 1 << 13, LAMP_PRIO_DISP = 1 << 14,
	LAMP_NO_DAP = 1 << 15
};

struct RecEvent {
	double get = 0;
	uint8_t kind = 0;             // LogEntry::Kind
	std::string text;
};

struct RecHeader {
	int missionNumber = 0;
	std::string missionName;      // "Apollo 11"
	std::string csmCallsign, lmCallsign;
	std::string siteName;
	double siteLatitude = 0, siteLongitude = 0, siteRadius = 1737400;
	double splashLatitude = 0, splashLongitude = 0;
	double getZeroJD = 0;         // Julian date at GET 0 (liftoff)
	std::string difficulty;       // as flown
	std::string outcome;          // "Splashdown 4.2 nmi from the ship", ...
	bool success = false;
	std::vector<std::string> contingencies;   // what went wrong
};

class Recording {
public:
	RecHeader header;
	std::vector<RecSample> samples;
	std::vector<RecDisplay> cmc, lgc;
	std::vector<RecEvent> events;

	bool Save(const std::string& path) const;
	bool Load(const std::string& path, std::string* error = nullptr);

	double StartGET() const { return samples.empty() ? 0 : samples.front().get; }
	double EndGET() const { return samples.empty() ? 0 : samples.back().get; }

	// The state at `get`: positions by cubic Hermite on the recorded
	// velocities, attitudes by slerp, flags from the earlier sample.
	RecSample At(double get) const;
	// The DSKY as it was at `get` (null before the first record).
	const RecDisplay* DisplayAt(bool lgc, double get) const;
	// Indices of the events in (from, to].
	void EventsBetween(double from, double to, std::vector<size_t>* out) const;
	// The phase at `get`.
	int PhaseAt(double get) const;

private:
	size_t _SampleIndex(double get) const;
};

// Phase names without pulling in the simulation.
const char* RecPhaseName(int phase);

}  // namespace flight

#endif
