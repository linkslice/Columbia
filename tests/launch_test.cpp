// Columbia — the Saturn's Instrument Unit flies from the pad to Earth
// orbit and, on a lunar mission, through TLI. Headless.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: launch_test <rope-dir> [mission] [tli]

#include "../core/flight/FlightSim.h"
#include "../core/flight/Saturn.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace flight;
using namespace sim;

int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";
	int number = argc > 2 ? std::atoi(argv[2]) : 11;
	bool tli = argc > 3 && std::atoi(argv[3]) != 0;
	const MissionSpec* m = FindMission(number);
	FlightSim f(*m, ropes);
	f.onLog = [](const LogEntry& e) {
		std::printf("  [%8.1f] %s\n", e.get, e.text.c_str());
	};
	std::string err;
	if (!f.Start(StartPoint::Launch, &err)) {
		std::printf("start failed: %s\n", err.c_str());
		return 1;
	}
	std::printf("%s: flight azimuth %.2f deg (planned %.2f)\n", m->name.c_str(),
		f.FlightAzimuth(), m->launchAzimuth);
	SaturnIU& iu = *f.IU();
	auto dsky = [&]() {
		const agc::DSKYState& d = f.CMCDSKY().State();
		std::printf("    CMC P%c%c V%c%c N%c%c %s %s %s%s%s%s%s%s\n", d.prog[0],
			d.prog[1], d.verb[0], d.verb[1], d.noun[0], d.noun[1],
			d.RegisterText(0).c_str(), d.RegisterText(1).c_str(),
			d.RegisterText(2).c_str(), d.progAlarm ? " PROG" : "",
			d.restart ? " RESTART" : "", d.oprErr ? " OPR" : "",
			d.verbNounFlash ? " FL" : "", d.compActy ? " ACTY" : "");
		std::printf("    truth v %.0f ft/s alt %.1f nmi\n",
			f.CSMVehicle().velocity.Length() / 0.3048,
			(f.CSMVehicle().position.Length() - world::kREarth) / 1852);
	};
	double next = -15;
	f.Run(599.9);
	if (std::getenv("CMCDIAG")) {
		Vehicle& v = f.CSMVehicle();
		Mat3 P = f.CMPGNCS().imu.platform;
		Vec3 up = v.position.Unit();
		Vec3 dr = v.ToInertial(Vec3(0, 0, 1));
		std::printf("platform at T-0: Z vs down %.3f deg, X vs downrange %.3f deg\n",
			std::acos(Clamp(P.Column(2).Dot(-up), -1, 1)) / kDeg,
			std::acos(Clamp(P.Column(0).Dot(dr), -1, 1)) / kDeg);
	}
	while (f.GET() < 900 && !f.Lost()
			&& iu.CurrentMode() != SaturnIU::Mode::Orbit) {
		f.Run(0.5);
		if (f.GET() >= next) {
			next += 10;
			if (std::getenv("CMCDIAG"))
				dsky();
			Vehicle& v = f.CSMVehicle();
			Situation s = f.Situate(v);
			Vec3 air = v.velocity - Vec3(0, 0, world::kEarthRate).Cross(v.position);
			double fpa = std::asin(v.velocity.Unit().Dot(v.position.Unit())) / kDeg;
			double pitch = std::acos(Clamp(v.ToInertial(Vec3(1, 0, 0))
				.Dot(v.position.Unit()), -1, 1)) / kDeg;
			std::printf("%6.0f %-6s alt %6.1f km  v %6.0f (air %6.0f) m/s  fpa %5.1f"
				"  tilt %5.1f  m %8.0f kg  a %4.2f g  tgo %5.0f  peri %7.1f\n",
				f.GET(), iu.ModeName(), s.altitude / 1000, v.velocity.Length(),
				air.Length(), fpa, pitch, v.Mass(),
				v.SensedAcceleration().Length() / 9.80665, iu.TimeToGo(),
				s.periapsis / 1000);
		}
	}
	f.Run(5.0);    // J-2 thrust decay
	Situation s = f.Situate(f.CSMVehicle());
	std::printf("orbit: %.1f x %.1f km, inclination %.2f deg at GET %.0f\n",
		s.apoapsis / 1000, s.periapsis / 1000, s.inclination, f.GET());
	bool ok = iu.CurrentMode() == SaturnIU::Mode::Orbit && s.periapsis > 150000;
	if (ok && tli) {
		f.SetWarp(1000);
		while (!f.Lost() && iu.CurrentMode() != SaturnIU::Mode::Coast
				&& f.GET() < m->plan.tli + 1200)
			f.Advance(1.0);
		Vehicle& v = f.CSMVehicle();
		State st{v.position, v.velocity, f.JulianDate()};
		Approach ap = ClosestMoonApproach(st, 5 * 86400);
		std::printf("after TLI: pericynthion %.1f km at GET %.2f h\n",
			(ap.radius - world::kRMoon) / 1000,
			(ap.jd - (f.JulianDate() - f.GET() / 86400)) * 24);
		ok = iu.CurrentMode() == SaturnIU::Mode::Coast && ap.found
			&& ap.radius - world::kRMoon > 0
			&& ap.radius - world::kRMoon < 2000e3;
	}
	std::printf(ok ? "LAUNCH OK\n" : "LAUNCH FAILED\n");
	return ok ? 0 : 1;
}
