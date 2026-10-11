// Columbia — whole-flight smoke test: every start point of a mission comes
// up with the real computers running, flies in lockstep for a while, then
// warps through a coast where the situation allows.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: flight_test <rope-dir> [mission]

#include "../core/flight/FlightSim.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace flight;

namespace {

std::string
Disp(const agc::DSKYState& d)
{
	char buf[160];
	std::snprintf(buf, sizeof(buf), "P%c%c V%c%c N%c%c%s%s%s",
		d.prog[0], d.prog[1], d.verb[0], d.verb[1], d.noun[0], d.noun[1],
		d.progAlarm ? " PROG" : "", d.oprErr ? " OPR" : "",
		d.restart ? " RESTART" : "");
	return buf;
}

void
Report(FlightSim& f, const char* tag)
{
	Situation s = f.Situate(f.CSMVehicle());
	std::printf("  %-8s GET %9.1f s  %-20s %s alt %8.1f km  v %7.1f m/s"
		"  apo %9.1f peri %8.1f km  CMC %s",
		tag, f.GET(), PhaseName(f.CurrentPhase()), s.nearMoon ? "Moon " : "Earth",
		s.altitude / 1000, s.speed, s.apoapsis / 1000, s.periapsis / 1000,
		Disp(f.CMCDSKY().State()).c_str());
	std::printf("\n");
	if (f.LMVehicle()) {
		Situation l = f.Situate(*f.LMVehicle());
		std::printf("  %-8s LM %s alt %8.3f km  v %7.1f m/s  peri %8.1f km%s"
			"  LGC %s\n", "", l.nearMoon ? "Moon " : "Earth", l.altitude / 1000,
			l.speed, l.periapsis / 1000, f.LMLanded() ? " (landed)" : "",
			Disp(f.LGCDSKY().State()).c_str());
	}
}

}  // namespace

int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";
	int number = argc > 2 ? std::atoi(argv[2]) : 11;
	const MissionSpec* m = FindMission(number);
	if (m == nullptr) {
		std::printf("no mission %d\n", number);
		return 1;
	}
	int failures = 0;
	const StartPoint starts[] = {
		StartPoint::Launch, StartPoint::EarthOrbit, StartPoint::TranslunarCoast,
		StartPoint::LunarOrbit, StartPoint::PoweredDescent,
		StartPoint::LunarSurface, StartPoint::LunarAscent,
		StartPoint::TransearthCoast, StartPoint::Entry
	};
	for (StartPoint sp : starts) {
		FlightSim f(*m, ropes);
		std::string err;
		std::printf("%s — %s\n", m->name.c_str(), StartPointName(sp));
		if (!f.Start(sp, &err)) {
			std::printf("  not available: %s\n", err.c_str());
			continue;
		}
		Report(f, "start");
		f.Run(10.0);
		Report(f, "+10 s");
		bool bad = f.Lost() || f.CMCDSKY().State().restart;
		// Warp through a coast if the situation allows it.
		if (!f.Lost() && sp != StartPoint::Launch && sp != StartPoint::Entry
			&& sp != StartPoint::PoweredDescent) {
			f.SetWarp(10000);
			std::printf("  max warp %.0f\n", f.MaxWarp());
			for (int i = 0; i < 36 && !f.Lost(); i++)
				f.Advance(0.01);   // 100 s of sim time per call
			Report(f, "+1 h");
			f.SetWarp(1);
			f.Run(5.0);
			Report(f, "resume");
			bad = bad || f.Lost();
		}
		if (sp == StartPoint::Entry) {
			// Ride the entry out on the computer's guidance from P61 onward
			// isn't scripted here; just let it fall and check it splashes
			// or is lost honestly.
			f.SeparateCMFromSM();
			for (int i = 0; i < 300 && !f.Lost()
					&& f.CurrentPhase() != Phase::Recovery; i++) {
				f.Run(10);
				const sim::EntryAeroState& e = f.EntryState();
				if (e.dynamicPressure > 1)
					std::printf("    %6.0f alt %7.1f km  M %5.1f  g %5.2f  alpha %6.1f"
						" roll %6.1f\n", f.GET(),
						(f.CSMVehicle().position.Length() - sim::world::kREarth) / 1000,
						e.mach, e.gLoad, e.alpha / sim::kDeg, e.roll / sim::kDeg);
			}
			Report(f, "entry");
			std::printf("  max g %.1f\n", f.MaxGLoad());
		}
		for (const LogEntry& e : f.Entries())
			if (e.kind == LogEntry::Warning || e.kind == LogEntry::Caution)
				std::printf("  ! %.0f %s\n", e.get, e.text.c_str());
		if (bad) {
			std::printf("  FAILED%s\n", f.Lost() ? (": " + f.LossReason()).c_str()
				: "");
			failures++;
		}
	}
	std::printf(failures ? "FLIGHT SMOKE FAILED (%d)\n" : "FLIGHT SMOKE OK\n",
		failures);
	return failures ? 1 : 0;
}
