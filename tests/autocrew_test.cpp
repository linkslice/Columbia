// Columbia — the automatic crew flies mission phases inside the full
// simulation, through the real CMC and LGC: an SPS burn out of lunar
// orbit (TEI), the entry, the powered descent.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: autocrew_test <rope-dir> [tei|entry|landing|all] [mission]

#include "../core/flight/AutoCrew.h"
#include "../core/flight/FlightSim.h"
#include "../core/flight/Trajectory.h"
#include "../core/gnc/PadLoads.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>

using namespace flight;
using namespace sim;

namespace {

void
Echo(FlightSim& f, AutoCrew& crew)
{
	f.onLog = [](const LogEntry& e) {
		if (e.kind != LogEntry::Info)
			std::printf("    [%9.1f] %s\n", e.get, e.text.c_str());
	};
	crew.onReport = [&f](const std::string& s) {
		std::printf("    [%9.1f] crew: %s\n", f.GET(), s.c_str());
	};
}

bool
TEI(const std::string& ropes, int mission)
{
	std::printf("TEI from lunar orbit:\n");
	FlightSim f(*FindMission(mission), ropes);
	std::string err;
	if (!f.Start(StartPoint::LunarOrbit, &err)) {
		std::printf("  %s\n", err.c_str());
		return false;
	}
	AutoCrew crew(f);
	crew.Install();
	Echo(f, crew);
	f.JettisonLM();
	f.Run(30);
	// RTCC: TEI an orbit from now.
	Vehicle& v = f.CSMVehicle();
	State s{v.position, v.velocity, f.JulianDate()};
	Vec3 dv;
	double ei;
	State burn;
	if (!TargetTEI(s, -6.5 * kDeg, &dv, &ei, &burn)) {
		std::printf("  TEI targeting failed\n");
		return false;
	}
	double jd0 = f.JulianDate() - f.GET() / 86400;
	double impulse = (burn.jd - jd0) * 86400;
	double tig;
	Vec3 lvlh;
	AutoCrew::ExternalDeltaV(f, impulse, dv, &tig, &lvlh);
	double tb = 2 * (impulse - tig);
	State atTig = Propagate(s, tig - f.GET());
	Vec3 v0 = v.velocity;
	std::printf("  PAD: TIG %.0f (in %.0f min), dV LVLH %.1f %.1f %.1f m/s, burn %.0f s\n",
		tig, (tig - f.GET()) / 60, lvlh.x, lvlh.y, lvlh.z, tb);
	crew.verbose = std::getenv("CREWDIAG") != nullptr;
	crew.Add(AutoCrew::CSMBurn(tig, lvlh, "TEI"));
	{
		auto once = std::make_shared<bool>(false);
		State tigState = atTig;
		Vec3 dvTrue = dv;
		auto t40 = std::make_shared<double>(std::getenv("P40DIAG") ? -1 : 1e30);
		f.stepHooks.push_back([t40, dvTrue](FlightSim& sim, double) {
			const agc::DSKYState& ds = sim.CMCDSKY().State();
			bool in40 = ds.prog[0] == '4' && ds.prog[1] == '0';
			if (!in40) return;
			if (*t40 > 1e29) return;
			if (*t40 < 0) *t40 = sim.GET();
			double since = sim.GET() - *t40;
			if (since > 80 || std::fmod(since, 2.0) > 0.006) return;
			gnc::PadLoader& pad = sim.CMCPad();
			Vec3 ut, vg, dvin;
			pad.ReadVector("UT", -1, &ut);
			pad.ReadVector("VGTIG", -7, &vg);
			pad.ReadVector("DELVSIN", -7, &dvin);
			std::printf("      +%.1f UT-DV %.1f  VG-DV %.1f |VG| %.1f  %s\n", since,
				ut.Length() > 0 ? std::acos(Clamp(ut.Unit().Dot(dvin.Unit()), -1, 1)) / kDeg : -1,
				vg.Length() > 0 ? std::acos(Clamp(vg.Unit().Dot(dvin.Unit()), -1, 1)) / kDeg : -1,
				vg.Length() * 100, ds.verb[0] == ' ' ? "" : "disp");
			(void)dvTrue;
		});
		f.stepHooks.push_back([once, tigState, dvTrue](FlightSim& sim, double) {
			const agc::DSKYState& ds = sim.CMCDSKY().State();
			if (*once || ds.prog[1] != '0' || ds.prog[0] != '4'
					|| ds.noun[0] != '1' || ds.noun[1] != '8')
				return;
			*once = true;
			gnc::PadLoader& pad = sim.CMCPad();
			Vec3 rtig, vtig, dvin;
			pad.ReadVector("DELVSIN", -7, &dvin);
			pad.ReadVector("RTIG", -29, &rtig);
			pad.ReadVector("VTIG", -7, &vtig);
			double wg = 0, F = 0, dvab = 0;
			pad.ReadDouble("WEIGHT/G", -16, &wg);
			pad.ReadDouble("F", -7, &F);
			pad.ReadDouble("DELVSAB", -7, &dvab);
			Vec3 vgtig, ut;
			pad.ReadVector("VGTIG", -7, &vgtig);
			pad.ReadVector("UT", -1, &ut);
			std::printf("    CMC: WEIGHT/G %.0f kg, F %.0f, DELVSAB %.1f m/s, VGTIG %.1f m/s, "
				"VGTIG vs DELVSIN %.1f deg, UT vs DELVSIN %.1f deg\n", wg, F * 100, dvab * 100,
				vgtig.Length() * 100, std::acos(Clamp(vgtig.Unit().Dot(dvin.Unit()), -1, 1)) / kDeg,
				std::acos(Clamp(ut.Unit().Dot(dvin.Unit()), -1, 1)) / kDeg);
			{
				Vec3 rr = tigState.r - MoonPos(tigState.jd), vv = tigState.v - MoonVel(tigState.jd);
				Vec3 a = ToLVLH(rr, vv, ut.Unit()), b = ToLVLH(rr, vv, dvin.Unit()),
					g = ToLVLH(rr, vv, vgtig.Unit());
				std::printf("    LVLH@TIG: UT %.3f %.3f %.3f  DELVSIN %.3f %.3f %.3f  VGTIG %.3f %.3f %.3f\n",
					a.x, a.y, a.z, b.x, b.y, b.z, g.x, g.y, g.z);
			}
			pad.ReadVector("DELVSIN", -7, &dvin);
			vtig = vtig * 100;
			dvin = dvin * 100;
			Mat3 R = std::getenv("REFFRAME") ? Mat3() : sim.CMPGNCS().imu.REFSMMAT();
			Vec3 rT = R * (tigState.r - MoonPos(tigState.jd));
			Vec3 vT = R * (tigState.v - MoonVel(tigState.jd));
			Vec3 dT = R * dvTrue;
			std::printf("    CMC at P40: RTIG err %.1f km, VTIG err %.1f m/s, "
				"DELVSIN %.1f m/s vs %.1f, angle %.1f deg\n",
				(rtig - rT).Length() / 1000, (vtig - vT).Length(), dvin.Length(),
				dT.Length(), std::acos(Clamp(dvin.Unit().Dot(dT.Unit()), -1, 1)) / kDeg);
		});
		Vec3 dvI = dv.Unit();
		auto last = std::make_shared<double>(0);
		f.stepHooks.push_back([dvI, last](FlightSim& sim, double) {
			if (sim.GET() - *last < (sim.CMPGNCS().SPSOn() ? 20 : (sim.GET() > 412600 && sim.GET() < 414400 ? 20 : (sim.GET() > 360200 ? 1500 : 100)))
					|| sim.GET() < 358600)
				return;
			*last = sim.GET();
			Vec3 x = sim.CSMVehicle().ToInertial(Vec3(1, 0, 0));
			Vec3 g = sim.CMPGNCS().imu.Gimbals(sim.CSMVehicle().attitude);
			{
				gnc::PadLoader& pad = sim.CMCPad();
				Vec3 om, vgn;
				pad.ReadVector("OMEGAC", 0, &om);
				pad.ReadVector("VG", -7, &vgn);
				std::printf("      STEERSW %d OMEGAC %.4f %.4f %.4f VG %.1f DELVREF ok\n",
					gnc::GetFlag(pad, 34), om.x, om.y, om.z, vgn.Length() * 100);
			}
			std::printf("    %.0f X vs dv %.1f deg, rates %.3f %.3f %.3f, gimbals %.1f %.1f %.1f, gimbal %.2f %.2f\n",
				sim.GET(), std::acos(Clamp(x.Dot(dvI), -1, 1)) / kDeg,
				sim.CSMVehicle().rates.x / kDeg, sim.CSMVehicle().rates.y / kDeg,
				sim.CSMVehicle().rates.z / kDeg, g.x / kDeg, g.y / kDeg, g.z / kDeg,
				sim.CMPGNCS().GimbalPitch() / kDeg, sim.CMPGNCS().GimbalYaw() / kDeg);
		});
	}
	auto sensed = std::make_shared<Vec3>();
	f.stepHooks.push_back([sensed, tig](FlightSim& sim, double) {
		if (sim.GET() > tig - 40 && sim.GET() < tig + 600)
			*sensed += sim.CSMVehicle().SensedDeltaV();
		double k = sim.GET() - tig;
		if (k > -40 && k < 12 && std::fmod(k + 1000, 2.0) < 0.005) {
			Vec3 vgn;
			sim.CMCPad().ReadVector("VG", -7, &vgn);
			std::printf("      T%+.0f VG %.1f sensed %.2f SPS %d ch11 %05o\n", k,
				vgn.Length() * 100, sensed->Length(), sim.CMPGNCS().SPSOn(),
				sim.CMC().Channel(011));
		}
	});
	f.SetWarp(10000);
	while (crew.Busy() && !f.Lost() && f.GET() < tig + 3600)
		f.Advance(0.05);
	{
		double jdT = jd0 + tig / 86400;
		State ss = Propagate(s, tig - (s.jd - jd0) * 86400);
		Vec3 rr = ss.r - MoonPos(jdT), vv = ss.v - MoonVel(jdT);
		Vec3 a = ToLVLH(rr, vv, *sensed), b = ToLVLH(rr, vv, dv);
		std::printf("  sensed %.1f m/s LVLH@TIG %.1f %.1f %.1f; impulsive %.1f %.1f %.1f\n",
			sensed->Length(), a.x, a.y, a.z, b.x, b.y, b.z);
	}
	std::printf("  speed change %.1f m/s (impulsive %.1f)\n",
		(v.velocity - MoonVel(f.JulianDate())).Length()
		- (v0 - MoonVel(s.jd)).Length(), dv.Length());
	State after{v.position, v.velocity, f.JulianDate()};
	{
		State ideal = burn;
		ideal.v += dv;
		ideal = Propagate(ideal, (after.jd - ideal.jd) * 86400);
		Vec3 dvErr = after.v - ideal.v;
		Vec3 rr = after.r - MoonPos(after.jd), vv = after.v - MoonVel(after.jd);
		Vec3 e = ToLVLH(rr, vv, dvErr);
		std::printf("  vs impulsive: dr %.1f km, dv LVLH %.1f %.1f %.1f m/s\n",
			(after.r - ideal.r).Length() / 1000, e.x, e.y, e.z);
		State far = Propagate(after, 2 * 86400);
		std::printf("  in 2 days: %.0f km from Earth\n", far.r.Length() / 1000);
	}
	Interface in = EntryInterface(after, 4 * 86400);
	std::printf("  after: entry interface %s, gamma %.2f deg, GET %.1f h (planned %.1f h)\n",
		in.found ? "found" : "MISSED", in.gamma / kDeg,
		(in.jd - jd0) * 24, (ei - jd0) * 24);
	// MCC-5, fifteen hours out, as Mission Control would.
	if (in.found) {
		double mccGET = f.GET() + 15 * 3600;
		State at = Propagate(after, mccGET - f.GET());
		Vec3 mdv;
		if (TargetMCCEarth(at, -6.5 * kDeg, &mdv)) {
			double mtig;
			Vec3 mlvlh;
			AutoCrew::ExternalDeltaV(f, mccGET, mdv, &mtig, &mlvlh);
			State chk = at;
			chk.v += mdv;
			Interface ci = EntryInterface(chk, 4 * 86400);
			std::printf("  MCC-5: %.1f m/s at GET %.1f h (impulsive gives gamma %.2f, found %d); LVLH %.1f %.1f %.1f\n",
				mdv.Length(), mtig / 3600, ci.gamma / kDeg, ci.found, mlvlh.x, mlvlh.y, mlvlh.z);
			Vec3 vb = v.velocity;
			(void)vb;
			crew.Add(AutoCrew::CSMBurn(mtig, mlvlh, "MCC-5"));
			auto sens2 = std::make_shared<Vec3>();
			State baseline = at;
			auto base2 = std::make_shared<State>();
			f.secondHooks.push_back([baseline, base2](FlightSim& sim, double) {
				static double last = 0;
				if (base2->jd == 0)
					*base2 = State{sim.CSMVehicle().position, sim.CSMVehicle().velocity, sim.JulianDate()};
				if (sim.GET() - last < (sim.GET() < 364000 ? 300 : 3600)) return;
				last = sim.GET();
				State p = Propagate(*base2, (sim.JulianDate() - base2->jd) * 86400);
				(void)baseline;
				std::printf("      coast check GET %.0f jdlag %.3f s (lockstep-only %d, rates %.2f): dr %.2f km dv %.3f m/s\n", sim.GET(), (sim.JulianDate() - sim.CSMVehicle().jd) * 86400,
					!sim.MaxWarp() ? 0 : sim.MaxWarp() < 1000, sim.CSMVehicle().rates.Length() / kDeg,
					(sim.CSMVehicle().position - p.r).Length() / 1000,
					(sim.CSMVehicle().velocity - p.v).Length());
			});
			f.stepHooks.push_back([sens2, mtig](FlightSim& sim, double) {
				double k = sim.GET() - mtig;
				static Vec3 pre;
				if (k < -6 && k > -1600) {
					pre += sim.CSMVehicle().SensedDeltaV();
					if (std::fmod(k + 10000, 200) < 0.005)
						std::printf("      MCC T%+.0f pre-burn sensed %.3f m/s\n", k, pre.Length());
				}
				if (k < -6 || k > 8) return;
				*sens2 += sim.CSMVehicle().SensedDeltaV();
				if (std::fmod(k + 1000, 0.5) < 0.005) {
					Vec3 vgn;
					sim.CMCPad().ReadVector("VG", -7, &vgn);
					std::printf("      MCC T%+.1f VG %.2f sensed %.2f SPS %d mass %.0f\n", k,
						vgn.Length() * 100, sens2->Length(), sim.CMPGNCS().SPSOn(),
						sim.CSMVehicle().Mass());
				}
			});
			while (crew.Busy() && !f.Lost() && f.GET() < mtig + 3600)
				f.Advance(0.05);
			State a2{v.position, v.velocity, f.JulianDate()};
			State ideal = chk;
			ideal = Propagate(ideal, (a2.jd - ideal.jd) * 86400);
			std::printf("  vs ideal after MCC: dv %.2f m/s (%.2f %.2f %.2f)\n", (a2.v - ideal.v).Length(),
				(a2.v - ideal.v).x, (a2.v - ideal.v).y, (a2.v - ideal.v).z);
			Vec3 applied = a2.v - ideal.v + mdv;
			std::printf("  wanted %.2f %.2f %.2f, applied %.2f %.2f %.2f\n", mdv.x, mdv.y, mdv.z,
				applied.x, applied.y, applied.z);
			in = EntryInterface(a2, 4 * 86400);
			std::printf("  after MCC-5: gamma %.2f deg\n", in.gamma / kDeg);
		}
	}
	bool ok = in.found && std::fabs(in.gamma / kDeg + 6.5) < 1.0;
	std::printf("  %s\n", ok ? "TEI OK" : "TEI FAILED");
	return ok;
}

bool
EntryRun(const std::string& ropes, int mission)
{
	std::printf("Entry:\n");
	const MissionSpec& ms = *FindMission(mission);
	FlightSim f(ms, ropes);
	std::string err;
	if (!f.Start(StartPoint::Entry, &err)) {
		std::printf("  %s\n", err.c_str());
		return false;
	}
	AutoCrew crew(f);
	crew.Install();
	Echo(f, crew);
	crew.verbose = std::getenv("CREWDIAG") != nullptr;
	crew.Add(AutoCrew::Entry(ms.splashLatitude, ms.splashLongitude));
	f.SetWarp(10000);
	while (!f.Lost() && f.CurrentPhase() != Phase::Recovery && f.GET() < ms.plan.entry + 3000)
		f.Advance(0.05);
	Vehicle& v = f.CSMVehicle();
	double lat, lon, r;
	Vec3 fixed = world::Ephemeris::EarthFixedToInertial(f.JulianDate()).Transposed()
		* v.position;
	world::VectorToLatLon(fixed, &lat, &lon, &r);
	Vec3 tgt = world::LatLonToVector(ms.splashLatitude, ms.splashLongitude, 1.0);
	double miss = std::acos(Clamp(fixed.Unit().Dot(tgt), -1, 1)) * world::kREarth;
	std::printf("  splash %.2f N %.2f E, %.1f nmi from target, max %.1f g\n",
		lat, lon, miss / 1852, f.MaxGLoad());
	bool ok = f.CurrentPhase() == Phase::Recovery && miss < 30 * 1852
		&& f.MaxGLoad() < 10;
	std::printf("  %s\n", ok ? "ENTRY OK" : "ENTRY FAILED");
	return ok;
}

bool
LandingRun(const std::string& ropes, int mission)
{
	std::printf("Powered descent:\n");
	const MissionSpec& ms = *FindMission(mission);
	FlightSim f(ms, ropes);
	std::string err;
	if (!f.Start(StartPoint::PoweredDescent, &err)) {
		std::printf("  %s\n", err.c_str());
		return false;
	}
	AutoCrew crew(f);
	crew.Install();
	Echo(f, crew);
	crew.Add(AutoCrew::Landing(ms.plan.pdi));
	f.SetWarp(100);
	while (!f.Lost() && crew.Busy() && f.GET() < ms.plan.pdi + 1500)
		f.Advance(0.5);
	bool ok = f.LMLanded() && !f.Lost();
	if (f.LMVehicle()) {
		double lat, lon, rr;
		f.Moon().LatLon(f.LMVehicle()->position, &lat, &lon, &rr);
		double miss = std::hypot((lat - ms.site.latitude) * kDeg,
			(lon - ms.site.longitude) * kDeg * std::cos(ms.site.latitude * kDeg))
			* ms.site.radius;
		std::printf("  LM at %.4f N %.4f E, %.2f km from the site, DPS %.0f kg\n",
			lat, lon, miss / 1000,
			f.LMVehicle()->FindModule("LM descent stage")
				? f.LMVehicle()->FindModule("LM descent stage")->tanks[0].mass : 0);
	}
	std::printf("  %s\n", ok ? "LANDING OK" : "LANDING FAILED");
	return ok;
}

bool
DockRun(const std::string& ropes, int mission)
{
	std::printf("Transposition and docking (in Earth orbit):\n");
	FlightSim f(*FindMission(mission), ropes);
	std::string err;
	if (!f.Start(StartPoint::EarthOrbit, &err)) {
		std::printf("  %s\n", err.c_str());
		return false;
	}
	AutoCrew crew(f);
	crew.Install();
	Echo(f, crew);
	crew.Add(AutoCrew::TransposeAndDock(f.GET() + 30));
	double t0 = f.GET();
	auto last = std::make_shared<double>(0);
	f.stepHooks.push_back([last](FlightSim& sim, double) {
		if (sim.GET() - *last < 20) return;
		*last = sim.GET();
		double range, rate, angle;
		if (sim.DockingGeometry(&range, &rate, &angle)) {
			const gnc::CMPanel& p = sim.CMPGNCS().panel;
			std::printf("      %.0f range %.2f m rate %.3f m/s angle %.1f deg  rhc %.0f %.0f thc %d %d %d rates %.2f %.2f %.2f\n",
				sim.GET(), range, rate, angle, p.rhc.y, p.rhc.z, p.thc[0], p.thc[1], p.thc[2],
				sim.CSMVehicle().rates.x / kDeg, sim.CSMVehicle().rates.y / kDeg,
				sim.CSMVehicle().rates.z / kDeg);
		}
	});
	while (crew.Busy() && !f.Lost() && f.GET() < t0 + 4000)
		f.Run(1.0);
	double fuel = 0;
	for (const Module& m : f.CSMVehicle().Modules())
		if (m.name == "Service Module")
			for (size_t i = 1; i < m.tanks.size(); i++)
				fuel += m.tanks[i].mass;
	bool ok = f.LMDocked() && !f.Lost();
	std::printf("  docked %d after %.0f s, SM RCS left %.0f kg\n", f.LMDocked(),
		f.GET() - t0, fuel);
	std::printf("  %s\n", ok ? "DOCKING OK" : "DOCKING FAILED");
	return ok;
}

}  // namespace


int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";
	std::string which = argc > 2 ? argv[2] : "all";
	int mission = argc > 3 ? std::atoi(argv[3]) : 11;
	int failures = 0;
	if (which == "tei" || which == "all")
		failures += !TEI(ropes, mission);
	if (which == "entry" || which == "all")
		failures += !EntryRun(ropes, mission);
	if (which == "dock" || which == "all")
		failures += !DockRun(ropes, mission);
	if (which == "landing" || which == "all")
		failures += !LandingRun(ropes, mission);
	std::printf(failures ? "AUTOCREW FAILED (%d)\n" : "AUTOCREW OK\n", failures);
	return failures ? 1 : 0;
}
