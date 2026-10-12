// Columbia — Flight's calls through a session: Apollo 13's abort comes up,
// the clock stops until it's decided, the choice changes the flight.
// Usage: decision_test <rope-dir>
#include "../app/FlightSession.h"
#include <chrono>
#include <cstdio>
#include <thread>
using namespace columbia;
using namespace flight;
int main(int argc, char** argv)
{
	std::setvbuf(stdout, nullptr, _IOLBF, 0);
	FlightOptions o;
	o.mission = *FindMission(13);
	o.start = StartPoint::TranslunarCoast;
	o.difficulty = Difficulty::AsFlown;
	o.autopilot = true;
	o.askDecisions = true;
	o.seed = 1;
	o.ropes = argc > 1 ? argv[1] : "rope";
	FlightSession s;
	std::string err;
	if (!s.Start(o, &err)) {
		std::printf("start: %s\n", err.c_str());
		return 1;
	}
	s.SetWarp(100000);
	auto waitFor = [&](auto cond, int seconds) {
		auto t0 = std::chrono::steady_clock::now();
		while (!cond()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(seconds))
				return false;
		}
		return true;
	};
	auto get = [&]() {
		std::lock_guard<std::recursive_mutex> g(s.Lock());
		return s.Sim().GET();
	};
	bool ok = waitFor([&]() { return s.DecisionPending(); }, 300);
	std::printf("abort decision posted: %d at GET %.2f h\n", ok, get() / 3600);
	double t1 = get();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	double t2 = get();
	bool held = ok && t2 - t1 < 1;
	std::printf("clock held while waiting: %d (%.1f s passed)\n", held, t2 - t1);
	ok = ok && s.Decide("sm-abort", "lifeboat");
	bool resumed = waitFor([&]() { return get() > t2 + 3600; }, 120);
	int inLM;
	{
		std::lock_guard<std::recursive_mutex> g(s.Lock());
		inLM = s.Sim().CrewInLM();
	}
	std::printf("decided, resumed %d, crew in the LM %d\n", resumed, inLM);
	bool pc2 = waitFor([&]() { return s.DecisionPending(); }, 300);
	std::printf("PC+2 decision posted: %d at GET %.2f h\n", pc2, get() / 3600);
	pc2 = pc2 && s.Decide("pc2", "pacific");
	bool pad = waitFor([&]() {
		std::lock_guard<std::recursive_mutex> g(s.Lock());
		for (const ManeuverPAD& p : s.Director().PADs())
			if (p.purpose == "PC+2")
				return true;
		return false;
	}, 300);
	std::printf("PC+2 PAD read up: %d\n", pad);
	s.Stop();
	bool all = ok && held && resumed && inLM == 3 && pc2 && pad;
	std::printf("%s\n", all ? "DECISIONS OK" : "DECISIONS FAILED");
	return all ? 0 : 1;
}
