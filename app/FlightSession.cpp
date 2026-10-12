// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "FlightSession.h"

#include <chrono>
#include <cmath>
#include <cstdio>

namespace columbia {

using namespace flight;

FlightSession::FlightSession()
{
}


FlightSession::~FlightSession()
{
	Stop();
}


bool
FlightSession::Start(const FlightOptions& options, std::string* error)
{
	Stop();
	fOptions = options;
	fDrill = fOptions.drill != DrillId::None ? FindDrill(fOptions.drill) : nullptr;
	if (fDrill && !DrillAvailable(*fDrill, fOptions.mission)) {
		if (error)
			*error = fOptions.mission.name + " has no " + fDrill->name + " to practise";
		return false;
	}
	if (fDrill)
		fOptions.start = fDrill->from;
	fSim.reset(new FlightSim(fOptions.mission, fOptions.ropes));
	if (!fSim->Start(fOptions.start, error)) {
		fSim.reset();
		return false;
	}
	fSim->logHooks.push_back([this](const LogEntry& e) {
		std::lock_guard<std::mutex> g(fLogLock);
		fLog.push_back(e);
		while (fLog.size() > 2000)
			fLog.pop_front();
	});
	fRecorder.reset(new FlightRecorder(*fSim));
	fRecorder->Data().header.difficulty = DifficultyName(fOptions.difficulty);
	fRecorder->Attach();
	fCrew.reset(new AutoCrew(*fSim));
	fCrew->Install();
	fContingencies.reset(new ContingencyManager(*fSim, fOptions.difficulty,
		fOptions.seed));
	fContingencies->Plan();
	fSim->secondHooks.push_back([this](FlightSim&, double) {
		fContingencies->Tick();
	});
	fDirector.reset(new MissionDirector(*fSim, *fCrew));
	fDirector->Install();
	fDirector->waitForDecisions = fOptions.askDecisions;
	fDrillPending = fDrill != nullptr;
	fDrillFailed = false;
	fDrillStartGET = fSim->GET();
	if (fDrill) {
		fSim->Log(LogEntry::Event, std::string("Drill: ") + fDrill->name
			+ ". Setting it up...");
		SetAutopilot(true);
		fWarp = 100000;
		fDirector->waitForDecisions = false;
		fSim->stopWhen = [this]() { return fDrillPending && _CheckDrill(); };
	} else {
		SetAutopilot(fOptions.autopilot);
		fWarp = 1;
	}
	fOver = false;
	fRecordingClosed = false;
	fRunning = true;
	fThread = std::thread([this]() { _Run(); });
	return true;
}


void
FlightSession::Stop()
{
	fRunning = false;
	if (fThread.joinable())
		fThread.join();
}


void
FlightSession::SetWarp(double warp)
{
	if (fDrillPending)
		return;     // flat out until the drill is ready
	if (fOptions.realTimeOnly)
		warp = std::min(warp, 1.0);
	fWarp = std::max(0.1, std::min(warp, 100000.0));
}


void
FlightSession::SetAutopilot(bool on)
{
	std::lock_guard<std::recursive_mutex> g(fLock);
	if (fDrillPending && !on)
		return;     // the crew sets the drill up
	fAutopilot = on;
	if (!fDirector)
		return;
	fDirector->autopilot = on;
	fCrew->troubleshoot = on;
	if (!on)
		fCrew->Abandon();
}


void
FlightSession::TakeLog(std::deque<LogEntry>* out)
{
	std::lock_guard<std::mutex> g(fLogLock);
	for (LogEntry& e : fLog)
		out->push_back(e);
	fLog.clear();
}


std::string
FlightSession::Outcome() const
{
	if (!fSim)
		return "";
	if (fSim->Lost())
		return fSim->LossReason();
	if (fSim->CurrentPhase() == Phase::Recovery) {
		double lat, lon, r;
		sim::Vec3 fixed = sim::world::Ephemeris::EarthFixedToInertial(
			fSim->JulianDate()).Transposed() * fSim->CSMVehicle().position;
		sim::world::VectorToLatLon(fixed, &lat, &lon, &r);
		sim::Vec3 tgt = sim::world::LatLonToVector(fDirector->SplashLatitude(),
			fDirector->SplashLongitude(), 1.0);
		double miss = std::acos(std::max(-1.0, std::min(1.0, fixed.Unit().Dot(tgt))))
			* sim::world::kREarth;
		char b[160];
		std::snprintf(b, sizeof(b), "Splashdown at %.1f%c %.1f%c, %.1f nmi from the"
			" recovery ship", std::fabs(lat), lat >= 0 ? 'N' : 'S', std::fabs(lon),
			lon >= 0 ? 'E' : 'W', miss / 1852);
		return b;
	}
	return std::string("In flight: ") + PhaseName(fSim->CurrentPhase());
}


bool
FlightSession::SaveRecording(const std::string& path)
{
	std::lock_guard<std::recursive_mutex> g(fLock);
	if (!fRecorder)
		return false;
	if (!fRecordingClosed && fOver) {
		for (const auto& s : fContingencies->Schedule())
			if (s.fired)
				fRecorder->Data().header.contingencies.push_back(s.spec->title);
		fRecorder->Data().header.splashLatitude = fDirector->SplashLatitude();
		fRecorder->Data().header.splashLongitude = fDirector->SplashLongitude();
		fRecorder->Finish(Outcome(), fSim->CurrentPhase() == Phase::Recovery);
		fRecordingClosed = true;
	}
	return fRecorder->Data().Save(path);
}


void
FlightSession::_Run()
{
	using clock = std::chrono::steady_clock;
	auto last = clock::now();
	while (fRunning) {
		auto now = clock::now();
		double dt = std::chrono::duration<double>(now - last).count();
		last = now;
		dt = std::min(dt, 0.1);
		bool waiting;
		{
			std::lock_guard<std::recursive_mutex> g(fLock);
			waiting = fDirector->PendingDecision() != nullptr;
		}
		if (!fPaused && !fOver && !waiting) {
			// Small slices with the lock let go in between, so the windows
			// stay responsive however much simulation a slice holds.
			double left = dt;
			while (left > 1e-6 && fRunning) {
				double slice = std::min(left, 0.02);
				{
					std::lock_guard<std::recursive_mutex> g(fLock);
					fSim->SetWarp(fWarp);
					fSim->Advance(slice);
					if (fSim->Lost() || fSim->CurrentPhase() == Phase::Recovery)
						fOver = true;
				}
				left -= slice;
				std::this_thread::yield();
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(8));
	}
}


bool
FlightSession::DecisionPending()
{
	std::lock_guard<std::recursive_mutex> g(fLock);
	return fDirector && fDirector->PendingDecision() != nullptr;
}


bool
FlightSession::Decide(const std::string& id, const std::string& key)
{
	std::lock_guard<std::recursive_mutex> g(fLock);
	return fDirector && fDirector->Decide(id, key);
}


bool
FlightSession::_CheckDrill()
{
	if (DrillReady(*fDrill, *fSim, *fCrew, *fDirector)) {
		fDrillPending = false;
		fWarp = 1;
		fSim->SetWarp(1);
		fDirector->waitForDecisions = fOptions.askDecisions;
		SetAutopilot(fOptions.autopilot);
		fSim->Log(LogEntry::Event, std::string("Drill: ") + fDrill->name
			+ (fOptions.autopilot ? ". Watch the crew." : ". You have the controls."));
		fSim->Log(LogEntry::Info, fDrill->brief);
		return true;
	}
	if (fSim->Lost() || fSim->CurrentPhase() == Phase::Recovery
			|| fSim->GET() > fDrillStartGET + fDrill->maxHours * 3600) {
		fDrillPending = false;
		fDrillFailed = true;
		fWarp = 1;
		fPaused = true;
		fSim->Log(LogEntry::Warning, std::string("Drill: ") + fDrill->name
			+ " could not be set up on this flight.");
		return true;
	}
	return false;
}

}  // namespace columbia
