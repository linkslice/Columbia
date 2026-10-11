// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Contingencies: the failures Apollo actually met, and the ones Mission
// Control trained for. Each one is injected into the real hardware model
// (an engine, a fuel cell, a radar, a computer's input lines) so the
// spacecraft and its flight software respond the way they would have;
// the crew's way out is a switch, a procedure, a DSKY entry or a change
// of plan, as it was then.
//
// The difficulty selector decides which are scheduled: the ones that
// happened on the chosen flight, plus a draw from the library of
// contingencies the flight controllers' mission rules covered.

#ifndef COLUMBIA_CONTINGENCY_H
#define COLUMBIA_CONTINGENCY_H

#include <cstdint>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace flight {

class FlightSim;
struct MissionSpec;

enum class Difficulty {
	Nominal,       // a flight with nothing going wrong
	AsFlown,       // what happened on this mission, when it happened
	Challenging,   // as flown, plus a couple of minor contingencies
	Hard,          // as flown, plus several, some serious
	Nightmare      // a mission-threatening failure on top of the rest
};
const char* DifficultyName(Difficulty difficulty);
const char* DifficultyBlurb(Difficulty difficulty);

// Mission events that contingency windows are measured from.
enum class Event {
	Liftoff, Insertion, TLI, LOI, Undock, PDI, Landing, Liftoff2, TEI, Entry
};

struct ContingencySpec {
	std::string id;
	std::string title;
	int mission = 0;              // 0 = generic; else the flight it happened on
	int severity = 1;             // 1 nuisance .. 5 mission-threatening
	Event event = Event::Liftoff; // window start reference
	double earliest = 0, latest = 0;   // seconds after the event
	bool needsLM = false;
	bool lunarOnly = false;
	std::string history;          // what happened (or the mission rule)
	std::string cue;              // what the crew notices
	std::string remedy;           // what fixed it (shown after, or by CAPCOM)
	std::function<bool(FlightSim&)> applicable;   // may be null
	std::function<void(FlightSim&)> inject;
	// Called once a second while active; returns true when resolved.
	std::function<bool(FlightSim&)> resolved;
};

// The whole library.
const std::vector<ContingencySpec>& ContingencyLibrary();
const ContingencySpec* FindContingency(const std::string& id);

class ContingencyManager {
public:
	ContingencyManager(FlightSim& sim, Difficulty difficulty,
		uint32_t seed = 0);

	struct Scheduled {
		const ContingencySpec* spec;
		double get;               // when it fires
		bool fired = false;
		bool resolved = false;
		double firedGET = 0;
		double resolvedGET = 0;
	};

	// Draws the schedule for the mission (call after FlightSim::Start).
	void Plan();
	// Forces one now (the "surprise me" button, tests).
	bool Trigger(const std::string& id);
	// Hooked into the simulation's once-a-second callbacks.
	void Tick();

	Difficulty CurrentDifficulty() const { return fDifficulty; }
	const std::vector<Scheduled>& Schedule() const { return fSchedule; }
	int Fired() const;
	int Resolved() const;

private:
	double _EventGET(Event event) const;
	void _Fire(Scheduled& s);

	FlightSim& f;
	Difficulty fDifficulty;
	std::mt19937 fRandom;
	std::vector<Scheduled> fSchedule;
};

}  // namespace flight

#endif
