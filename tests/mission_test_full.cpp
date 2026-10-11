// Columbia — a whole mission, launch to splashdown, flown by the automatic
// crew through the real CMC and LGC with Mission Control's RTCC planning
// every maneuver. Headless; takes a while.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: mission_test_full <rope-dir> [mission] [start-point index]

#include "../core/flight/AutoCrew.h"
#include "../core/flight/FlightSim.h"
#include "../core/flight/MissionDirector.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>

using namespace flight;
using namespace sim;

int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";
	int number = argc > 2 ? std::atoi(argv[2]) : 11;
	int startIndex = argc > 3 ? std::atoi(argv[3]) : 0;
	const MissionSpec& m = *FindMission(number);
	FlightSim f(m, ropes);
	std::string err;
	if (!f.Start(StartPoint(startIndex), &err)) {
		std::printf("start: %s\n", err.c_str());
		return 1;
	}
	std::time_t wall0 = std::time(nullptr);
	auto stamp = [&](double get) {
		char b[32];
		long t = long(get);
		std::snprintf(b, sizeof(b), "%03ld:%02ld:%02ld", t / 3600, t / 60 % 60, t % 60);
		return std::string(b);
	};
	f.onLog = [&](const LogEntry& e) {
		if (e.kind != LogEntry::Info || e.text.rfind("Flight plan", 0) == 0)
			std::printf("%s [%4lds] %s\n", stamp(e.get).c_str(),
				long(std::time(nullptr) - wall0), e.text.c_str());
		std::fflush(stdout);
	};
	AutoCrew crew(f);
	crew.Install();
	crew.onReport = [&](const std::string& s) {
		std::printf("%s [%4lds]   crew: %s\n", stamp(f.GET()).c_str(),
			long(std::time(nullptr) - wall0), s.c_str());
		std::fflush(stdout);
	};
	crew.verbose = std::getenv("CREWDIAG") != nullptr;
	if (std::getenv("DIAGFROM")) {
		double from = std::atof(std::getenv("DIAGFROM"));
		f.stepHooks.push_back([from](FlightSim& sim, double) {
			static double last = 0;
			if (sim.GET() < from || sim.GET() > from + 1800 || sim.GET() - last < 20) return;
			last = sim.GET();
			Vehicle& v = sim.CSMVehicle();
			Vec3 g = sim.CMPGNCS().imu.Gimbals(v.attitude);
			const agc::DSKYState& d = sim.CMCDSKY().State();
			std::printf("    DIAG %.0f rates %.2f %.2f %.2f gimbals %.1f %.1f %.1f FAILREG %05o %05o %05o %c%c %c%c%c%c\n",
				sim.GET(), v.rates.x / kDeg, v.rates.y / kDeg, v.rates.z / kDeg,
				g.x / kDeg, g.y / kDeg, g.z / kDeg, sim.CMCPad().Word("FAILREG", 0),
				sim.CMCPad().Word("FAILREG", 1), sim.CMCPad().Word("FAILREG", 2),
				d.prog[0], d.prog[1], d.verb[0], d.verb[1], d.noun[0], d.noun[1]);
		});
	}
	MissionDirector director(f, crew);
	director.Install();
	if (std::getenv("TDDIAG"))
		f.stepHooks.push_back([](FlightSim& sim, double) {
			static double last = 0;
			if (sim.GET() < 10785 || sim.GET() > 10800 || sim.GET() - last < 0.25) return;
			last = sim.GET();
			Vehicle& v = sim.CSMVehicle();
			Vec3 g = sim.CMPGNCS().imu.Gimbals(v.attitude);
			const gnc::CMPanel& p = sim.CMPGNCS().panel;
			std::printf("    TD %.0f rates %.2f %.2f %.2f gimbals %.1f %.1f %.1f rhc %.0f %.0f %.0f mode %d jets5 %03o CMC %s\n",
				sim.GET(), v.rates.x / kDeg, v.rates.y / kDeg, v.rates.z / kDeg,
				g.x / kDeg, g.y / kDeg, g.z / kDeg, p.rhc.x, p.rhc.y, p.rhc.z, (int)p.mode,
				sim.CMC().Channel(05), (std::string(sim.CMCDSKY().State().prog, 2) + " " + std::string(sim.CMCDSKY().State().verb, 2) + std::string(sim.CMCDSKY().State().noun, 2) + (sim.CMCDSKY().State().progAlarm ? " PROG" : "") + (sim.CMCDSKY().State().verbNounFlash ? " FL" : "") + (sim.CMCDSKY().State().oprErr ? " OPR" : "") + (sim.CMCDSKY().State().keyRel ? " KR" : "")).c_str());
		});
	if (std::getenv("RCSDIAG"))
		f.secondHooks.push_back([](FlightSim& sim, double) {
			static double last = 0;
			if (!sim.LMVehicle() || sim.GET() - last < 300) return;
			last = sim.GET();
			const Module* a = sim.LMVehicle()->FindModule("LM ascent stage");
			if (a && a->tanks.size() > 2)
				std::printf("    RCS %.0f A %.1f B %.1f\n", sim.GET(), a->tanks[1].mass,
					a->tanks[2].mass);
		});
	double stopAt = std::getenv("STOPAT") ? std::atof(std::getenv("STOPAT")) : 1e30;
	f.SetWarp(100000);
	while (!f.Lost() && f.CurrentPhase() != Phase::Recovery
			&& f.GET() < m.plan.splashdown + 6 * 3600 && f.GET() < stopAt)
		f.Advance(0.05);
	std::printf("\nEnded at %s: %s%s\n", stamp(f.GET()).c_str(),
		PhaseName(f.CurrentPhase()), f.Lost() ? (" — " + f.LossReason()).c_str() : "");
	if (f.CurrentPhase() == Phase::Recovery) {
		double lat, lon, r;
		Vec3 fixed = world::Ephemeris::EarthFixedToInertial(f.JulianDate()).Transposed()
			* f.CSMVehicle().position;
		world::VectorToLatLon(fixed, &lat, &lon, &r);
		Vec3 tgt = world::LatLonToVector(director.SplashLatitude(),
			director.SplashLongitude(), 1.0);
		double miss = std::acos(Clamp(fixed.Unit().Dot(tgt), -1, 1)) * world::kREarth;
		std::printf("Splashdown %.2f N %.2f E, %.1f nmi from the recovery ship\n",
			lat, lon, miss / 1852);
	}
	std::printf("Next step was: %s\n", director.NextEvent().c_str());
	bool ok = f.CurrentPhase() == Phase::Recovery;
	std::printf(ok ? "MISSION OK\n" : "MISSION FAILED\n");
	return ok ? 0 : 1;
}
