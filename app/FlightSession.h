// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// One flight in progress: the simulation with its crew, Mission Control,
// the contingencies and the flight recorder, run on a thread of its own.
// The windows lock it to look and to act. Plain C++ (std::thread), so it
// also runs headless in the tests.

#ifndef COLUMBIA_FLIGHT_SESSION_H
#define COLUMBIA_FLIGHT_SESSION_H

#include "flight/AutoCrew.h"
#include "flight/Contingency.h"
#include "flight/Drill.h"
#include "flight/FlightSim.h"
#include "flight/MissionDirector.h"
#include "flight/MissionSpec.h"
#include "flight/Recorder.h"

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace columbia {

struct FlightOptions {
	flight::MissionSpec mission;
	flight::StartPoint start = flight::StartPoint::Launch;
	flight::Difficulty difficulty = flight::Difficulty::AsFlown;
	bool autopilot = false;        // the automatic crew flies (you watch)
	bool realTimeOnly = false;     // no time warp
	uint32_t seed = 0;             // contingency draw (0: random)
	std::string ropes;             // directory with the flight software
	flight::DrillId drill = flight::DrillId::None;   // a training drill
	bool askDecisions = true;      // Flight's calls are yours (else as flown)
};

class FlightSession {
public:
	FlightSession();
	~FlightSession();

	bool Start(const FlightOptions& options, std::string* error);
	void Stop();
	bool Running() const { return fRunning; }

	// Hold this while looking at or changing anything below.
	std::recursive_mutex& Lock() { return fLock; }
	flight::FlightSim& Sim() { return *fSim; }
	flight::AutoCrew& Crew() { return *fCrew; }
	flight::MissionDirector& Director() { return *fDirector; }
	flight::ContingencyManager& Contingencies() { return *fContingencies; }
	flight::FlightRecorder& Recorder() { return *fRecorder; }
	const FlightOptions& Options() const { return fOptions; }

	// Time: the warp asked for (the situation may hold it lower).
	void SetWarp(double warp);
	double Warp() const { return fWarp; }
	void SetPaused(bool paused) { fPaused = paused; }
	bool Paused() const { return fPaused; }

	// The automatic crew: on, they fly; off, it's yours.
	void SetAutopilot(bool on);
	bool Autopilot() const { return fAutopilot; }

	// A drill being set up: the crew flies, flat out, to the drill's
	// moment, then hands over (and the warp drops to real time).
	bool DrillPending() const { return fDrillPending; }

	// Flight's calls: one waiting holds the clock (see MissionDirector).
	bool DecisionPending();
	bool Decide(const std::string& id, const std::string& key);
	bool DrillFailed() const { return fDrillFailed; }
	const flight::DrillSpec* Drill() const { return fDrill; }

	// Log lines since the last call (for the air-to-ground view).
	void TakeLog(std::deque<flight::LogEntry>* out);
	// The flight is over (splashdown or lost); the outcome line.
	bool Over() const { return fOver; }
	std::string Outcome() const;
	// Writes the recording (closing it with the outcome).
	bool SaveRecording(const std::string& path);

private:
	void _Run();
	bool _CheckDrill();

	FlightOptions fOptions;
	std::unique_ptr<flight::FlightSim> fSim;
	std::unique_ptr<flight::AutoCrew> fCrew;
	std::unique_ptr<flight::MissionDirector> fDirector;
	std::unique_ptr<flight::ContingencyManager> fContingencies;
	std::unique_ptr<flight::FlightRecorder> fRecorder;
	std::recursive_mutex fLock;
	std::thread fThread;
	std::atomic<bool> fRunning{false};
	std::atomic<bool> fPaused{false};
	std::atomic<bool> fOver{false};
	std::atomic<double> fWarp{1.0};
	bool fAutopilot = false;
	const flight::DrillSpec* fDrill = nullptr;
	std::atomic<bool> fDrillPending{false};
	std::atomic<bool> fDrillFailed{false};
	double fDrillStartGET = 0;
	std::mutex fLogLock;
	std::deque<flight::LogEntry> fLog;
	bool fRecordingClosed = false;
};

}  // namespace columbia

#endif
