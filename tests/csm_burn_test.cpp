// Columbia — Comanche flies an SPS burn (Apollo 11 TEI geometry), headless.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: csm_burn_test <rope-dir> [program] [delta-v fps] [mission]
//
// The CSM is alone in a 60 nmi lunar orbit. A scripted crew does what the
// checklist says: V48 DAP load, P30 with the uplinked TIG and LVLH delta-V
// (N33/N81), P40 with the R60 maneuver to burn attitude, gimbal test,
// V99 go for ignition. The CMC flies the burn with its TVC DAP and cuts
// the engine; the test checks the velocity actually gained.

#include "../core/agc/AGC.h"
#include "../core/agc/DSKY.h"
#include "../core/gnc/CMPGNCS.h"
#include "../core/gnc/MissionPads.h"
#include "../core/gnc/PadLoads.h"
#include "../core/gnc/Rope.h"
#include "../core/sim/MoonScene.h"
#include "../core/sim/Spacecraft.h"
#include "../core/sim/World.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace sim;
using sim::world::JulianDate;

namespace {

const double kGETZero = JulianDate(1969, 7, 16, 13, 32, 0);
const double kTIG = 135 * 3600 + 23 * 60 + 42;   // TEI 135:23:42

std::string
Disp(const agc::DSKYState& d)
{
	char buf[160];
	std::snprintf(buf, sizeof(buf), "P%c%c V%c%c N%c%c %s %s %s%s%s%s",
		d.prog[0], d.prog[1], d.verb[0], d.verb[1], d.noun[0], d.noun[1],
		d.RegisterText(0).c_str(), d.RegisterText(1).c_str(),
		d.RegisterText(2).c_str(), d.verbNounFlash ? " FL" : "",
		d.progAlarm ? " PROG" : "", d.oprErr ? " OPR" : "");
	return buf;
}

std::string
GET(double s)
{
	char b[32];
	long t = long(s);
	std::snprintf(b, sizeof(b), "%03ld:%02ld:%02ld", t / 3600, t / 60 % 60,
		t % 60);
	return b;
}

// Signed five-digit DSKY entry, e.g. Entry(135) = "+00135".
std::string
Entry(long v)
{
	char b[32];
	std::snprintf(b, sizeof(b), "%c%05ld", v < 0 ? '-' : '+', std::labs(v));
	return b;
}

}  // namespace


int
main(int argc, char** argv)
{
	std::string ropeDir = argc > 1 ? argv[1] : "rope";
	std::string program = argc > 2 ? argv[2] : "Comanche055";
	double dvFps = argc > 3 ? std::atof(argv[3]) : 3283.4;
	int mission = argc > 4 ? std::atoi(argv[4]) : 11;

	gnc::Rope rope;
	std::string err;
	if (!rope.Load(ropeDir, program, &err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 2;
	}
	agc::Computer agc;
	agc.LoadRope(rope.Image().data(), rope.Image().size());
	agc::DSKY dsky;
	agc.onChannelWrite = [&](int ch, uint16_t v) { dsky.ChannelWritten(ch, v); };

	// --- Orbit: 60 nmi circular, 10 minutes before TIG --------------------------
	world::Terrain terrain;
	MoonScene scene(terrain);
	double startGET = kTIG - 1500;
	double jd0 = kGETZero + startGET / 86400.0;
	scene.SetTime(jd0);
	Vec3 north = scene.FixedToInertial().Column(2);
	Vec3 r0 = scene.FixedToInertial() * world::LatLonToVector(0, 170, 1);
	double rc = world::kRMoon + 111120;
	Vec3 rhat = r0.Unit();
	Vec3 h = (-north - rhat * (-north).Dot(rhat)).Unit();   // retrograde orbit
	Vec3 vhat = h.Cross(rhat).Unit();
	double vc = std::sqrt(world::kMuMoon / rc);

	craft::CSMConfig cfg;
	cfg.spsPropellant = 6000;
	Vehicle csm("Columbia");
	csm.AddModule(craft::MakeCommandModule(cfg));
	csm.AddModule(craft::MakeServiceModule(cfg));
	csm.position = scene.MoonPosition() + rhat * rc;
	csm.velocity = scene.MoonVelocity() + vhat * vc;
	csm.jd = jd0;
	// Heads-up, +X along the velocity.
	Vec3 xb = vhat, zb = -rhat, yb = zb.Cross(xb);
	csm.attitude = Quat::FromMatrix(Mat3::Columns(xb, yb, zb));

	gnc::CMPGNCS pgncs(agc, csm);
	// Platform: inertial at the start (X along the velocity, Z down), like
	// a P52 "preferred" alignment for the burn.
	Mat3 refsmmat = Mat3::Rows(vhat, -h, -rhat);
	pgncs.imu.Align(refsmmat);

	agc.PowerOn();
	double t = startGET;
	auto run = [&](double seconds) {
		const double dt = 0.005;
		for (double s = 0; s < seconds; s += dt) {
			scene.SetTime(csm.jd);
			csm.Step(dt, [](const Vec3& rr, double jd) {
				return world::Gravity::Acceleration(rr, jd, false);
			});
			pgncs.Step(dt);
			t += dt;
		}
	};
	auto type = [&](const std::string& s) {
		for (char c : s) {
			if (c == 'P') {
				agc.SetProceed(true);
				run(0.3);
				agc.SetProceed(false);
				run(0.3);
				continue;
			}
			int code;
			switch (c) {
				case 'V': code = agc::KEY_VERB; break;
				case 'N': code = agc::KEY_NOUN; break;
				case 'E': code = agc::KEY_ENTR; break;
				case 'R': code = agc::KEY_RSET; break;
				case '+': code = agc::KEY_PLUS; break;
				case '-': code = agc::KEY_MINUS; break;
				default: code = c == '0' ? agc::KEY_0 : c - '0';
			}
			agc.PressKey(code);
			run(0.25);
		}
	};
	auto show = [&](const char* what) {
		dsky.Sync(agc);
		std::printf("%s  %-8s %s\n", GET(t).c_str(), what, Disp(dsky.State()).c_str());
	};

	run(1.0);
	type("V36E");
	run(3.0);
	gnc::PadLoader pad(agc, rope);
	std::printf("mission pads: %d words\n",
		pad.Apply(gnc::MissionPads(mission, false)));
	gnc::CMCPadLoad load;
	load.agcTime = t;
	load.tephem = (kGETZero - JulianDate(1969, 7, 1)) * 86400.0;
	load.refsmmat = refsmmat;
	scene.SetTime(csm.jd);
	load.csm.position = csm.position - scene.MoonPosition();
	load.csm.velocity = csm.velocity - scene.MoonVelocity();
	load.csm.agcTime = t;
	load.csmMass = csm.Mass();
	gnc::LoadCMCMission(pad, load);
	gnc::SetComputerClock(agc, t);
	if (!pad.Missing().empty()) {
		std::printf("pad-load names not in %s:", program.c_str());
		for (auto& n : pad.Missing())
			std::printf(" %s", n.c_str());
		std::printf("\n");
	}
	run(1.0);
	type("V37E00E");
	run(2.0);
	show("P00");

	// V48: DAP data load, PRO through N46, N47, N48.
	type("V48E");
	run(1.0);
	for (int i = 0; i < 3; i++) {
		show("V48");
		type("P");
		run(1.0);
	}

	type("V82E");     // orbit parameters as the CMC sees them
	run(4.0);
	show("V82");
	type("P");
	run(3.0);
	show("V82b");
	type("V34E");
	run(1.0);
	type("V46E");     // activate the DAP with the new configuration
	run(2.0);
	show("V46");

	// P30: TIG and LVLH delta-V from the "uplink" (PAD).
	type("V37E30E");
	run(2.0);
	long tig = long(std::lround(kTIG * 100));
	show("P30");
	type("V25E" + Entry(tig / 360000) + "E" + Entry(tig / 6000 % 60) + "E"
		+ Entry(tig % 6000) + "E");
	show("N33");
	type("P");
	run(1.0);
	show("N81?");
	type("V25E" + Entry(std::lround(dvFps * 10)) + "E+00000E+00000E");
	show("N81");
	type("P");
	for (int i = 0; i < 120; i++) {      // P30 integrates to TIG first
		run(0.5);
		dsky.Sync(agc);
		if (dsky.State().verbNounFlash)
			break;
	}
	show("N42");
	type("P");
	run(2.0);
	show("P30 end");
	{
		Vec3 rtig, vtig, dvin;
		pad.ReadVector("RTIG", -27, &rtig);
		pad.ReadVector("VTIG", -5, &vtig);
		pad.ReadVector("DELVSIN", -7, &dvin);
		scene.SetTime(csm.jd);
		Vec3 rNow = refsmmat * (csm.position - scene.MoonPosition());
		double ang = std::acos(Clamp(rtig.Unit().Dot(rNow.Unit()), -1, 1)) / kDeg;
		double expect = std::sqrt(world::kMuMoon / (rc * rc * rc)) * (kTIG - t) / kDeg;
		std::printf("P30: |RTIG| %.0f m, |VTIG| %.1f m/s, |DELVSIN| %.1f m/s,"
			" RTIG is %.1f deg from now (expected %.1f); dv along VTIG %.1f\n",
			rtig.Length(), vtig.Length() * 100, dvin.Length() * 100, ang, expect,
			dvin.Dot(vtig.Unit()) * 100);
	}

	// P40.
	pgncs.panel.spsArmed = false;
	type("V37E40E");
	run(2.0);
	bool armed = false, cutoff = false;
	double lastKey = -100, burnStart = -1, burnEnd = -1;
	std::string lastDisp;
	Vec3 v0 = csm.velocity - scene.MoonVelocity();
	double mass0 = csm.Mass();
	double lastPrint = 0;
	while (t < kTIG + 900) {
		run(0.1);
		dsky.Sync(agc);
		const agc::DSKYState& d = dsky.State();
		std::string disp = Disp(d);
		std::string vn = std::string(d.verb, 2) + std::string(d.noun, 2);
		std::string r1(d.reg[0], 5);
		if (!armed && t > kTIG - 120) {
			pgncs.panel.spsArmed = true;
			armed = true;
			std::printf("%s  crew: delta-V thrust A/B ON\n", GET(t).c_str());
		}
		if (d.verbNounFlash && t - lastKey > 2.0) {
			std::string key;
			if (vn == "5018") {
				// Accept once the CDUs match the maneuver's final angles.
				const char* fin[3] = {"CPHI", "CTHETA", "CPSI"};
				const int cdu[3] = {agc::REG_CDUX, agc::REG_CDUY, agc::REG_CDUZ};
				double worst = 0;
				for (int i = 0; i < 3; i++)
					worst = std::max(worst, std::fabs(WrapPi(
						gnc::CDUToAngle(pad.Word(fin[i]))
						- gnc::CDUToAngle(agc.Cell(cdu[i])))) / kDeg);
				static double lastManeuver = -1e9;
				if (worst < 2.0)
					key = "E";
				else if (t - lastManeuver > 120) {
					key = "P";
					lastManeuver = t;
				}
			} else if (vn == "5025" && r1 == "00204")
				key = "P";                    // do the gimbal test
			else if (vn.substr(0, 2) == "99")
				key = "P";                    // go for ignition
			else if (vn == "1685" || vn == "0685" || vn == "1640" || vn == "0640")
				key = "P";                    // residuals / end of burn
			else if (vn.substr(0, 2) == "50" || vn == "0644" || vn == "0640")
				key = "P";
			else if (vn == "37  " || vn.substr(0, 2) == "37") {
				if (cutoff)
					break;
			}
			if (!key.empty()) {
				std::printf("%s  crew: %s on %s\n", GET(t).c_str(),
					key == "P" ? "PRO" : "ENTR", disp.c_str());
				type(key);
				lastKey = t;
			}
		}
		if (pgncs.SPSOn() && burnStart < 0) {
			burnStart = t;
			std::printf("%s  SPS IGNITION\n", GET(t).c_str());
		}
		if (!pgncs.SPSOn() && burnStart >= 0 && burnEnd < 0) {
			burnEnd = t;
			cutoff = true;
			std::printf("%s  SPS CUTOFF after %.1f s\n", GET(t).c_str(),
				burnEnd - burnStart);
		}
		if (disp != lastDisp && (t - lastPrint > 1.0 || pgncs.SPSOn())) {
			if (!pgncs.SPSOn() || t - lastPrint > 5.0) {
				std::printf("%s  %-56s gimbal %5.2f %5.2f cmd %5.2f %5.2f rates"
					" %5.2f %5.2f %5.2f\n", GET(t).c_str(), disp.c_str(),
					pgncs.GimbalPitch() / kDeg, pgncs.GimbalYaw() / kDeg,
					pgncs.CommandPitch() / kDeg, pgncs.CommandYaw() / kDeg,
					csm.rates.x / kDeg, csm.rates.y / kDeg, csm.rates.z / kDeg);
				lastPrint = t;
			}
			lastDisp = disp;
		}
		if (cutoff && t > burnEnd + 30)
			break;
	}

	scene.SetTime(csm.jd);
	Vec3 v1 = csm.velocity - scene.MoonVelocity();
	// Compare with the gain a perfect impulsive prograde burn would give,
	// carried back along gravity: use the LVLH components at cutoff.
	Vec3 r = csm.position - scene.MoonPosition();
	double energy = v1.Dot(v1) / 2 - world::kMuMoon / r.Length();
	double vinf = energy > 0 ? std::sqrt(2 * energy) : 0;
	std::printf("\nburn %.1f s, mass %.0f -> %.0f kg, |v| %.1f -> %.1f m/s,"
		" C3 %.3f km2/s2, v_inf %.0f m/s\n", burnEnd - burnStart, mass0,
		csm.Mass(), v0.Length(), v1.Length(), 2 * energy / 1e6, vinf);
	double gained = v1.Length() - v0.Length();
	double want = dvFps * 0.3048;
	bool ok = burnStart > 0 && burnEnd > burnStart && std::fabs(gained - want) < 0.03 * want;
	std::printf("speed gained %.1f m/s, commanded %.1f m/s: %s\n", gained, want,
		ok ? "BURN OK" : "BURN FAILED");
	return ok ? 0 : 1;
}
