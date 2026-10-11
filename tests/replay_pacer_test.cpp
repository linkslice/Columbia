// Columbia — replay pacing on a real recording.
// Usage: replay_pacer_test <file.colrec> [minutes]
#include "../ui/ReplayPacer.h"
#include <cstdio>
#include <cstdlib>
using namespace flight;
int main(int argc, char** argv)
{
	Recording r;
	std::string err;
	if (argc < 2 || !r.Load(argv[1], &err)) {
		std::printf("load: %s\n", err.c_str());
		return 1;
	}
	double minutes = argc > 2 ? std::atof(argv[2]) : 25;
	ui::ReplayPacer p;
	p.Build(r, minutes);
	std::printf("%s: %zu samples, replay %.1f min (wanted %.0f)\n",
		r.header.missionName.c_str(), r.samples.size(), p.TotalSeconds() / 60, minutes);
	for (const RecEvent& e : r.events) {
		if (e.kind == 0) continue;
		double t = p.SecondsAt(e.get);
		std::printf("  %6.1f min  x%-8.0f GET %8.0f  %s\n", t / 60, p.SpeedAt(t), e.get,
			e.text.substr(0, 60).c_str());
	}
	bool ok = p.TotalSeconds() > 60 && p.TotalSeconds() < minutes * 60 * 1.6;
	std::printf("%s\n", ok ? "PACER OK" : "PACER FAILED");
	return ok ? 0 : 1;
}
