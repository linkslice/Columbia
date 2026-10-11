// Columbia — the flight recorder: record a real flight segment, save,
// load, and check the replay matches what was flown.
//
// Usage: recording_test <rope-dir>

#include "../core/flight/AutoCrew.h"
#include "../core/flight/FlightSim.h"
#include "../core/flight/MissionSpec.h"
#include "../core/flight/Recorder.h"

#include <cmath>
#include <cstdio>
#include <cstring>

using namespace flight;
using namespace sim;

static int sFailures = 0;

static void
Check(bool ok, const char* what)
{
	std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok)
		sFailures++;
}

int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";
	const MissionSpec& m = *FindMission(11);
	FlightSim f(m, ropes);
	std::string err;
	if (!f.Start(StartPoint::Entry, &err)) {
		std::printf("start: %s\n", err.c_str());
		return 1;
	}
	FlightRecorder rec(f);
	rec.Attach();
	AutoCrew crew(f);
	crew.Install();
	crew.Add(AutoCrew::Entry(m.splashLatitude, m.splashLongitude));
	// Truth at a few moments, to compare with the replay.
	struct Truth { double get; Vec3 r; };
	std::vector<Truth> truth;
	f.secondHooks.push_back([&](FlightSim& sim, double) {
		if (std::fmod(sim.GET(), 97.0) < 1.0)
			truth.push_back({sim.GET(), sim.CSMVehicle().position});
	});
	f.SetWarp(10000);
	while (!f.Lost() && f.CurrentPhase() != Phase::Recovery
			&& f.GET() < m.plan.entry + 3000)
		f.Advance(0.05);
	rec.Finish("test", f.CurrentPhase() == Phase::Recovery);

	std::printf("Recorded %zu samples, %zu CMC displays, %zu events\n",
		rec.Data().samples.size(), rec.Data().cmc.size(), rec.Data().events.size());
	Check(rec.Data().samples.size() > 100, "samples recorded");
	Check(rec.Data().cmc.size() > 10, "DSKY changes recorded");
	Check(!rec.Data().events.empty(), "events recorded");

	const char* path = "/tmp/columbia_recording_test.colrec";
	Check(rec.Data().Save(path), "saved");
	Recording r;
	Check(r.Load(path, &err), ("loaded " + err).c_str());
	Check(r.samples.size() == rec.Data().samples.size()
		&& r.events.size() == rec.Data().events.size()
		&& r.cmc.size() == rec.Data().cmc.size(), "round trip sizes");
	Check(r.header.missionName == "Apollo 11", "header");

	// The replay against the truth (between samples, through entry).
	double worst = 0;
	for (const Truth& t : truth) {
		RecSample s = r.At(t.get);
		if (!s.veh[REC_CSM].Present())
			continue;
		worst = std::max(worst, (s.veh[REC_CSM].r - t.r).Length());
	}
	std::printf("  replay vs flown: worst %.1f m over %zu checks\n", worst,
		truth.size());
	Check(worst < 500, "replay positions within 500 m");
	const RecDisplay* d = r.DisplayAt(false, r.EndGET());
	Check(d != nullptr, "DSKY at the end");
	if (d)
		std::printf("  last CMC display: P%c%c V%c%c N%c%c\n", d->prog[0], d->prog[1],
			d->verb[0], d->verb[1], d->noun[0], d->noun[1]);
	std::printf("%s\n", sFailures ? "RECORDING FAILED" : "RECORDING OK");
	return sFailures ? 1 : 0;
}
