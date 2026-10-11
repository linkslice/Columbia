// Columbia — RTCC targeting check: Apollo 11 TLI from the parking orbit,
// LOI, TEI to entry interface.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "../core/flight/MissionSpec.h"
#include "../core/flight/Trajectory.h"
#include "../core/sim/World.h"

#include <cmath>
#include <cstdio>

using namespace sim;
using namespace flight;

int
main()
{
	const MissionSpec& m = *FindMission(11);
	double jd0 = world::JulianDate(m.year, m.month, m.day, m.hour, m.minute, m.second);
	// Parking orbit plane from the pad and the launch azimuth.
	Vec3 pad = world::Ephemeris::EarthFixedToInertial(jd0)
		* world::LatLonToVector(28.608, -80.604, 1.0);
	Vec3 east = Vec3(0, 0, 1).Cross(pad).Unit();
	Vec3 north = pad.Cross(east);
	double az = m.launchAzimuth * kDeg;
	Vec3 dir = north * std::cos(az) + east * std::sin(az);
	Vec3 n = pad.Cross(dir).Unit();
	// The IU flies into the plane that holds the Moon's position at
	// arrival (the launch window makes the azimuth come out ~72 deg).
	Vec3 moonArrive = world::Ephemeris::MoonPosition(jd0 + 75.5 / 24.0).Unit();
	Vec3 n2 = pad.Cross(moonArrive).Unit();
	if (n2.Dot(n) < 0)
		n2 = -n2;
	std::printf("plane change from the 72 deg azimuth plane: %.2f deg\n",
		std::acos(n.Dot(n2)) / kDeg);
	n = n2;
	double r = world::kREarth + 185200;
	double vc = std::sqrt(world::kMuEarth / r);
	double tIns = 11 * 60 + 49;
	double thetaIns = 2100e3 / world::kREarth;
	double period = 2 * M_PI * std::sqrt(r * r * r / world::kMuEarth);
	double tTLI = m.plan.tli - 160;    // mid-burn, impulsive
	double theta = thetaIns + 2 * M_PI * (tTLI - tIns) / period;
	Vec3 b = n.Cross(pad);
	Vec3 pos = (pad * std::cos(theta) + b * std::sin(theta)) * r;
	Vec3 vel = (pad * -std::sin(theta) + b * std::cos(theta)) * vc;
	State s{pos, vel, jd0 + tTLI / 86400};

	Vec3 dv;
	double arrive;
	State ign;
	if (!TargetTLI(s, 111000, &dv, &arrive, &ign, 2700, jd0 + m.plan.loi / 86400)) {
		std::printf("TLI targeting failed\n");
		return 1;
	}
	std::printf("TLI dv %.1f m/s (Apollo 11: 3,182 m/s); arrival GET %.2f h"
		" (LOI 75.83 h)\n", dv.Length(), (arrive - jd0) * 24);
	std::printf("ignition moved %.0f s from the plan\n", (ign.jd - s.jd) * 86400);
	State post = ign;
	post.v += dv;
	Approach ap = ClosestMoonApproach(post, 5 * 86400);
	std::printf("pericynthion %.1f km at GET %.2f h\n",
		(ap.radius - world::kRMoon) / 1000, (ap.jd - jd0) * 24);
	Vec3 loi;
	TargetLOI(ap.state, 313000, 111000, &loi);
	if (!(loi.Length() > 0)) return 1;
	std::printf("LOI-1 dv %.1f m/s (Apollo 11: 889 m/s)\n", loi.Length());
	State orbit = ap.state;
	orbit.v += loi;
	// Two days later, TEI.
	orbit = Propagate(orbit, 59.5 * 3600);
	Vec3 tei;
	double ei;
	State teiAt;
	bool ok = TargetTEI(orbit, -6.5 * kDeg, &tei, &ei, &teiAt);
	std::printf("TEI %s at GET %.2f h: dv %.1f m/s (Apollo 11: 1,001 m/s), EI GET %.2f h"
		" (Apollo 11: TEI 135.40 h, EI 195.05 h)\n", ok ? "ok" : "FAILED",
		(teiAt.jd - jd0) * 24, tei.Length(), (ei - jd0) * 24);
	return ok ? 0 : 1;
}
