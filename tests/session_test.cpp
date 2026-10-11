// Columbia — a flight session on its own thread: start, warp, watch it
// finish, save the recording.
// Usage: session_test <rope-dir>
#include "../app/FlightSession.h"
#include <chrono>
#include <cstdio>
#include <thread>
using namespace columbia;
using namespace flight;
int main(int argc, char** argv)
{
	FlightOptions o;
	o.mission = *FindMission(11);
	o.start = StartPoint::Entry;
	o.difficulty = Difficulty::Nominal;
	o.autopilot = true;
	o.ropes = argc > 1 ? argv[1] : "rope";
	FlightSession s;
	std::string err;
	if (!s.Start(o, &err)) {
		std::printf("start: %s\n", err.c_str());
		return 1;
	}
	s.SetWarp(100000);
	auto t0 = std::chrono::steady_clock::now();
	size_t lines = 0;
	while (!s.Over()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		std::deque<LogEntry> log;
		s.TakeLog(&log);
		lines += log.size();
		{
			// The UI's view: lock, look.
			std::lock_guard<std::recursive_mutex> g(s.Lock());
			(void)s.Sim().GET();
		}
		if (std::chrono::steady_clock::now() - t0 > std::chrono::minutes(8))
			break;
	}
	std::printf("%s (%zu log lines)\n", s.Outcome().c_str(), lines);
	bool ok = s.Over() && s.SaveRecording("/tmp/session_test.colrec");
	s.Stop();
	Recording r;
	ok = ok && r.Load("/tmp/session_test.colrec") && r.samples.size() > 100;
	std::printf("%s\n", ok ? "SESSION OK" : "SESSION FAILED");
	return ok ? 0 : 1;
}
