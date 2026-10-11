// Columbia — training drills: each one set up on Apollo 11 and handed over.
// Usage: drill_test <rope-dir> [drill-key ...]
#include "../app/FlightSession.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
using namespace columbia;
using namespace flight;
int main(int argc, char** argv)
{
	std::setvbuf(stdout, nullptr, _IOLBF, 0);
	std::string ropes = argc > 1 ? argv[1] : "rope";
	int failures = 0;
	for (const DrillSpec& d : Drills()) {
		bool wanted = argc <= 2;
		for (int i = 2; i < argc; i++)
			wanted |= std::strcmp(argv[i], d.key) == 0;
		if (!wanted)
			continue;
		FlightOptions o;
		o.mission = *FindMission(11);
		o.difficulty = Difficulty::Nominal;
		o.drill = d.id;
		o.ropes = ropes;
		o.seed = 1;
		FlightSession s;
		std::string err;
		auto t0 = std::chrono::steady_clock::now();
		if (!s.Start(o, &err)) {
			std::printf("%-6s start: %s\n", d.key, err.c_str());
			failures++;
			continue;
		}
		while (s.DrillPending()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			if (std::chrono::steady_clock::now() - t0 > std::chrono::minutes(10))
				break;
		}
		double secs = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - t0).count();
		// A few seconds more in real time with the controls handed over.
		std::this_thread::sleep_for(std::chrono::seconds(2));
		std::lock_guard<std::recursive_mutex> g(s.Lock());
		FlightSim& f = s.Sim();
		double range = -1, rate, angle;
		f.DockingGeometry(&range, &rate, &angle);
		bool ok = !s.DrillPending() && !s.DrillFailed() && !f.Lost()
			&& !s.Autopilot();
		std::printf("%-6s %s  ready at GET %.3f h after %.0f s  phase %s  autopilot %d"
			"  range %.0f m\n", d.key, ok ? "OK  " : "FAIL",
			f.GET() / 3600, secs, PhaseName(f.CurrentPhase()),
			s.Autopilot(), range);
		if (!ok)
			failures++;
		s.Stop();
	}
	std::printf("%s\n", failures ? "DRILLS FAILED" : "DRILLS OK");
	return failures ? 1 : 0;
}
