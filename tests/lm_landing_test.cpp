// Columbia — Luminary flies an Apollo 11 powered descent, headless.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: lm_landing_test <rope-dir> [program] [seconds-before-PDI]
//
// Sets up the real Apollo 11 geometry (dates, landing site, 60 x 8 nmi
// descent orbit), pad-loads the LGC, and lets a scripted crew answer the
// DSKY the way the checklist did. Luminary does the flying.

#include "../core/agc/AGC.h"
#include "../core/agc/DSKY.h"
#include "../core/gnc/LMPGNCS.h"
#include "../core/gnc/MissionPads.h"
#include "../core/gnc/PadLoads.h"
#include "../core/gnc/Rope.h"
#include "../core/sim/MoonScene.h"
#include "../core/sim/Spacecraft.h"
#include "../core/sim/World.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <string>

using namespace sim;
using sim::world::JulianDate;

namespace {

const double kGETZero = JulianDate(1969, 7, 16, 13, 32, 0);
const double kTLAND_GET = 102 * 3600 + 45 * 60 + 40;   // 102:45:40
const double kPDI_GET = 102 * 3600 + 33 * 60 + 5;      // 102:33:05
const double kSiteLat = 0.67408, kSiteLon = 23.47297;
const double kSiteRadius = 1735470.0;

std::string
Disp(const agc::DSKYState& d)
{
	char buf[160];
	std::snprintf(buf, sizeof(buf), "P%c%c V%c%c N%c%c %s %s %s%s%s%s%s",
		d.prog[0], d.prog[1], d.verb[0], d.verb[1], d.noun[0], d.noun[1],
		d.RegisterText(0).c_str(), d.RegisterText(1).c_str(),
		d.RegisterText(2).c_str(), d.verbNounFlash ? " FL" : "",
		d.progAlarm ? " PROG" : "", d.oprErr ? " OPR" : "",
		d.restart ? " RST" : "");
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

}  // namespace


int
main(int argc, char** argv)
{
	std::string ropeDir = argc > 1 ? argv[1] : "rope";
	std::string program = argc > 2 ? argv[2] : "Luminary099";
	double lead = argc > 3 ? std::atof(argv[3]) : 600;

	gnc::Rope rope;
	std::string err;
	if (!rope.Load(ropeDir, program, &err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 2;
	}
	agc::Computer agc;
	agc.LoadRope(rope.Image().data(), rope.Image().size());
	agc::DSKY dsky;
	static int traceBudget = 0;
	agc.onChannelWrite = [&](int ch, uint16_t v) {
		dsky.ChannelWritten(ch, v);
		if (traceBudget > 0 && (ch == 5 || ch == 6 || ch == 013)) {
			traceBudget--;
			std::printf("      ch%02o <- %05o  (T6 %05o)\n", ch, v,
				agc.Cell(agc::REG_TIME6));
		}
	};

	// --- World -----------------------------------------------------------------
	world::Terrain terrain;
	terrain.SetDatum(kSiteLat, kSiteLon, kSiteRadius);
	MoonScene scene(terrain);
	double jdLand = kGETZero + kTLAND_GET / 86400.0;
	scene.SetTime(jdLand);
	Vec3 siteFixed = world::LatLonToVector(kSiteLat, kSiteLon, kSiteRadius);
	Vec3 S = scene.FixedToInertial() * siteFixed;     // Moon-centred
	Vec3 north = scene.FixedToInertial().Column(2);
	Vec3 Shat = S.Unit();
	Vec3 h = (-north - Shat * (-north).Dot(Shat)).Unit();   // retrograde
	Vec3 downrange = h.Cross(Shat).Unit();

	// Descent orbit: perilune 50,000 ft above the site, apolune 60 nmi,
	// perilune ~16 degrees uprange (PDI), passed at the PDI time.
	double rp = kSiteRadius + 15240, ra = world::kRMoon + 111120;
	double a = 0.5 * (rp + ra), e = (ra - rp) / (ra + rp);
	double theta = -16.2 * kDeg;
	Vec3 P = Shat * std::cos(theta) + downrange * std::sin(theta);
	Vec3 Q = h.Cross(P);
	double n = std::sqrt(world::kMuMoon / (a * a * a));
	double startGET = kPDI_GET - lead;
	double M0 = -n * lead;
	double E = M0;
	for (int i = 0; i < 30; i++)
		E = E - (E - e * std::sin(E) - M0) / (1 - e * std::cos(E));
	double nu = 2 * std::atan2(std::sqrt(1 + e) * std::sin(E / 2),
		std::sqrt(1 - e) * std::cos(E / 2));
	double r = a * (1 - e * std::cos(E));
	double p = a * (1 - e * e);
	Vec3 relPos = (P * std::cos(nu) + Q * std::sin(nu)) * r;
	Vec3 relVel = (P * -std::sin(nu) + Q * (e + std::cos(nu)))
		* std::sqrt(world::kMuMoon / p);

	// CSM in a 60 nmi circular orbit a little ahead.
	double rc = world::kRMoon + 111120;
	double vc = std::sqrt(world::kMuMoon / rc);
	Vec3 csmPos = (P * std::cos(nu + 0.05) + Q * std::sin(nu + 0.05)) * rc;
	Vec3 csmVel = (P * -std::sin(nu + 0.05) + Q * std::cos(nu + 0.05)) * vc;

	// --- Vehicle -------------------------------------------------------------
	double jd0 = kGETZero + startGET / 86400.0;
	scene.SetTime(jd0);
	craft::LMConfig cfg;
	Vehicle lm("Eagle");
	lm.AddModule(craft::MakeLMDescentStage(cfg));
	lm.AddModule(craft::MakeLMAscentStage(cfg));
	craft::AddLMContacts(lm);
	lm.position = scene.MoonPosition() + relPos;
	lm.velocity = scene.MoonVelocity() + relVel;
	lm.jd = jd0;
	// Start retrograde, windows down: +X against the velocity.
	Vec3 xb = (-relVel).Unit();
	Vec3 zb = (-relPos).Unit();
	zb = (zb - xb * zb.Dot(xb)).Unit();
	Vec3 yb = zb.Cross(xb);
	lm.attitude = Quat::FromMatrix(Mat3::Columns(xb, yb, zb));

	gnc::LMPGNCS pgncs(agc, lm);
	Mat3 refsmmat = Mat3::Rows(Shat, downrange.Cross(Shat), downrange);
	pgncs.imu.Align(refsmmat);

	// --- Computer bring-up and pad loads ---------------------------------------
	agc.PowerOn();
	double t = startGET;
	auto run = [&](double seconds) {
		const double dt = 0.005;
		for (double s = 0; s < seconds; s += dt) {
			scene.SetTime(lm.jd);
			lm.Step(dt, [](const Vec3& rr, double jd) {
				return world::Gravity::Acceleration(rr, jd, false);
			});
			pgncs.Step(dt, &scene);
			t += dt;
		}
	};
	std::deque<int> keys;
	auto type = [&](const std::string& s) {
		for (char c : s) {
			int code = -1;
			switch (c) {
				case 'V': code = agc::KEY_VERB; break;
				case 'N': code = agc::KEY_NOUN; break;
				case 'E': code = agc::KEY_ENTR; break;
				case 'R': code = agc::KEY_RSET; break;
				case 'P': code = -1; break;
				default: code = c == '0' ? agc::KEY_0 : c - '0';
			}
			if (c == 'P') {
				agc.SetProceed(true);
				run(0.3);
				agc.SetProceed(false);
				run(0.3);
				continue;
			}
			agc.PressKey(code);
			run(0.25);
		}
	};
	run(1.0);
	type("V36E");
	run(3.0);
	gnc::PadLoader pad(agc, rope);
	gnc::LoadLuminaryInvariants(pad);
	if (!getenv("NOMISSIONPADS"))
		std::printf("mission pads: %d words\n", pad.Apply(gnc::MissionPads(11, true)));
	gnc::LMPadLoad load;
	load.agcTime = t;
	load.tland = kTLAND_GET;
	load.tephem = (kGETZero - JulianDate(1969, 7, 1)) * 86400.0;
	load.azo = world::Ephemeris::GMST(JulianDate(1969, 7, 1)) / (2 * M_PI);
	load.rls = siteFixed;
	load.refsmmat = refsmmat;
	scene.SetTime(lm.jd);
	load.lm.position = lm.position - scene.MoonPosition();
	load.lm.velocity = lm.velocity - scene.MoonVelocity();
	load.lm.agcTime = t;
	load.csm.position = csmPos;
	load.csm.velocity = csmVel;
	load.csm.agcTime = t;
	load.lmMass = lm.Mass();
	load.csmMass = 16500;
	gnc::LoadLuminaryMission(pad, load);
	gnc::SetComputerClock(agc, t);
	// DAP: 4-jet X translation, 0.3 deg deadband, 2 deg/s maneuvers.
	pad.Octal("DAPBOOLS", 002002);
	if (!pad.Missing().empty()) {
		std::printf("pad-load names not in %s:", program.c_str());
		for (auto& m : pad.Missing())
			std::printf(" %s", m.c_str());
		std::printf("\n");
	}

	std::printf("start %s  LM %.0f kg, alt %.1f km\n", GET(t).c_str(),
		lm.Mass(), scene.Altitude(lm.position) / 1000);
	type("V37E00E");
	run(3);
	std::printf("  P00: %s\n", Disp(dsky.State()).c_str());
	type("V48E");
	run(1);
	std::printf("  R03: %s\n", Disp(dsky.State()).c_str());
	for (int i = 0; i < 3; i++) {
		type("P");
		run(1);
		std::printf("  R03: %s\n", Disp(dsky.State()).c_str());
	}
	type("V37E63E");
	run(2);
	std::printf("  P63: %s  FAILREG %05o %05o %05o\n",
		Disp(dsky.State()).c_str(), pad.Word("FAILREG", 0),
		pad.Word("FAILREG", 1), pad.Word("FAILREG", 2));

	// Diagnostics: jets and rates while the DAP maneuvers.
	{
		int fired5 = 0, fired6 = 0;
		for (int i = 0; i < 200; i++) {
			run(0.05);
			if (pgncs.JetBits(5)) fired5++;
			if (pgncs.JetBits(6)) fired6++;
		}
		Vec3 g = pgncs.imu.Gimbals(lm.attitude);
		std::printf("  diag: jets ch5 active %d/200, ch6 %d/200, rates %.3f %.3f %.3f"
			" deg/s, gimbals %.2f %.2f %.2f, CDU %05o %05o %05o, FAILREG %05o %05o"
			" %05o, ch30 %05o ch31 %05o\n", fired5, fired6,
			lm.rates.x / kDeg, lm.rates.y / kDeg, lm.rates.z / kDeg,
			g.x / kDeg, g.y / kDeg, g.z / kDeg, agc.Cell(agc::REG_CDUX),
			agc.Cell(agc::REG_CDUY), agc.Cell(agc::REG_CDUZ),
			pad.Word("FAILREG", 0), pad.Word("FAILREG", 1),
			pad.Word("FAILREG", 2), agc.Channel(030), agc.Channel(031));
		std::printf("  diag: rupts T6 %u T5 %u T3 %u T4 %u KEY1 %u DOWN %u RADAR %u"
			"  TIME5 %05o TIME6 %05o\n", agc.InterruptCount(0),
			agc.InterruptCount(1), agc.InterruptCount(2), agc.InterruptCount(3),
			agc.InterruptCount(4), agc.InterruptCount(7), agc.InterruptCount(8),
			agc.Cell(agc::REG_TIME5), agc.Cell(agc::REG_TIME6));
		std::printf("  diag: CDUD %05o %05o %05o  ERR %05o %05o %05o  DELCDU %05o %05o"
			" %05o  OMEGA %05o %05o %05o  DB %05o\n", pad.Word("CDUXD"),
			pad.Word("CDUYD"), pad.Word("CDUZD"), pad.Word("ERRORX"),
			pad.Word("ERRORY"), pad.Word("ERRORZ"), pad.Word("DELCDUX"),
			pad.Word("DELCDUY"), pad.Word("DELCDUZ"), pad.Word("OMEGAP"),
			pad.Word("OMEGAQ"), pad.Word("OMEGAR"), pad.Word("DB"));
		dsky.Sync(agc);
		std::printf("  diag: NO ATT %d GIMBAL %d  IMODES30 %05o IMODES33 %05o"
			" DAPBOOLS %05o RCSFLAGS %05o ch12 %05o ch13 %05o ch14 %05o\n",
			dsky.State().noAtt, dsky.State().gimbalLock,
			pad.Word("IMODES30"), pad.Word("IMODES33"), pad.Word("DAPBOOLS"),
			pad.Word("RCSFLAGS"), agc.Channel(012), agc.Channel(013),
			agc.Channel(014));
	}

	// --- Fly ---------------------------------------------------------------------
	bool landed = false, v57 = false, armed = false;
	double lastPrint = 0, lastPRO = -10;
	std::string lastDisp;
	uint32_t restarts = agc.RestartCount();
	while (t < kTLAND_GET + 600 && !landed) {
		run(0.1);
		dsky.Sync(agc);
		const agc::DSKYState& d = dsky.State();
		std::string disp = Disp(d);
		std::string vn = std::string(d.verb, 2) + std::string(d.noun, 2);

		// Crew.
		if (!armed && t > kPDI_GET - 120) {
			pgncs.panel.engineArm = gnc::LMPanel::ARM_DESCENT;
			armed = true;
			std::printf("%s  crew: ENGINE ARM - DES\n", GET(t).c_str());
		}
		std::string r1(d.reg[0], 5);
		// Yaw-around to windows up (as Armstrong did ~4 minutes into the
		// burn) so the landing radar looks at the surface: X-axis override
		// with the hand controller while the LGC keeps steering thrust.
		{
			static int yaw = 0;   // 0 waiting, 1 rolling, 2 done
			Vec3 upv = scene.LocalVertical(lm.position);
			double xup = lm.ToInertial(Vec3(1, 0, 0)).Dot(upv);
			double zup = lm.ToInertial(Vec3(0, 0, 1)).Dot(upv)
				/ std::sqrt(std::max(1e-6, 1 - xup * xup));
			if (yaw == 0 && pgncs.DescentEngineOn() && t > kPDI_GET + 200
				&& zup < 0) {
				yaw = 1;
				std::printf("%s  crew: yaw-around to windows up\n", GET(t).c_str());
			}
			if (yaw == 1) {
				pgncs.panel.aca.x = 0.7;
				if (zup > 0.97) {
					pgncs.panel.aca.x = 0;
					yaw = 2;
					std::printf("%s  crew: windows up\n", GET(t).c_str());
				}
			}
		}
		// Crew procedures for the DSKY's requests.
		// R60: PRO starts the attitude maneuver; once the vehicle sits at
		// the requested gimbal angles (N18, XXX.XX deg) the crew ENTERs to
		// accept. Re-PRO only if the maneuver stalled well short.
		static double lastManeuverPRO = -1e9;
		if (d.verbNounFlash && t - lastPRO > 2.0) {
			std::string action;
			if (vn == "5025" && (r1 == "00014" || r1 == "00015"))
				action = "E";              // skip fine align: platform is good
			else if (vn == "5018") {
				// N18 shows FDAI ball angles; compare the CDUs with the
				// maneuver's final gimbal angles (CPHI, CTHETA, CPSI).
				const char* fin[3] = {"CPHI", "CTHETA", "CPSI"};
				const int cdu[3] = {agc::REG_CDUX, agc::REG_CDUY, agc::REG_CDUZ};
				double worst = 0;
				for (int i = 0; i < 3; i++) {
					double want = gnc::CDUToAngle(pad.Word(fin[i]));
					double have = gnc::CDUToAngle(agc.Cell(cdu[i]));
					worst = std::max(worst, std::fabs(WrapPi(want - have)) / kDeg);
				}
				if (worst < 2.0)
					action = "E";          // at attitude: accept
				else if (t - lastManeuverPRO > 60.0) {
					action = "P";          // start (or restart) the maneuver
					lastManeuverPRO = t;
				}
			} else if (vn.substr(0, 2) == "50" || vn.substr(0, 2) == "99"
				|| vn == "0661" || vn == "0662" || vn == "0663"
				|| vn == "0664")
				action = "P";
			if (!action.empty()) {
				std::printf("%s  crew: %s on %s\n", GET(t).c_str(),
					action == "P" ? "PRO" : "ENTR", disp.c_str());
				type(action);
				lastPRO = t;
			}
		}
		if (!v57 && pgncs.LRRangeGood() && pgncs.LRVelocityGood()) {
			std::printf("%s  crew: V57E (LR updates), LR alt %.0f ft\n",
				GET(t).c_str(), pgncs.LRAltitude() / 0.3048);
			if (!getenv("NOV57")) type("V57E");
			v57 = true;
			Vec3 up = scene.LocalVertical(lm.position);
			Vec3 vrel = lm.velocity - scene.SurfaceVelocity(lm.position);
			{
				double beta = (pgncs.LRPosition() == 1 ? 24.0 : 0.0) * kDeg;
				Mat3 ant = Mat3::RotX(6 * kDeg) * Mat3::RotY(beta);
				Vec3 beam = lm.ToInertial(ant * Vec3(-0.9374036, 0, -0.3482449)).Unit();
				Vec3 hdot;
				pad.ReadVector("HBEAMNB", -2, &hdot);
				Vec3 hnb = (ant * Vec3(-0.9374036, 0, -0.3482449)).Unit();
				std::printf("    [att] X.up %.3f Z.up %.3f\n",
					lm.ToInertial(Vec3(1, 0, 0)).Dot(up), lm.ToInertial(Vec3(0, 0, 1)).Dot(up));
				std::printf("    [beam] cos(beam,down) %.3f implied alt %.0f ft;"
					" AGC HBEAMNB %.3f %.3f %.3f vs ours %.3f %.3f %.3f\n",
					-beam.Dot(up), pgncs.LRAltitude() * -beam.Dot(up) / 0.3048,
					hdot.x, hdot.y, hdot.z, hnb.x, hnb.y, hnb.z);
			}
			Vec3 fwd = (vrel - up * vrel.Dot(up)).Unit();
			Vec3 bx = lm.ToInertial(Vec3(1, 0, 0)), bz = lm.ToInertial(Vec3(0, 0, 1));
			std::printf("    geometry: X.up %.3f X.fwd %.3f  Z.up %.3f Z.fwd %.3f"
				"  true alt %.0f ft\n", bx.Dot(up), bx.Dot(fwd), bz.Dot(up),
				bz.Dot(fwd), scene.Altitude(lm.position) / 0.3048);
		}
		if (disp != lastDisp && t - lastPrint > 0.5) {
			double alt = scene.Altitude(lm.position);
			Vec3 vrel = lm.velocity - scene.SurfaceVelocity(lm.position);
			double vz = vrel.Dot(scene.LocalVertical(lm.position));
			double vh = (vrel - scene.LocalVertical(lm.position) * vz).Length();
			std::printf("%s  %-60s alt %7.0f ft vz %6.1f vh %6.1f fps"
				" thr %4.0f%% DPS %.0f kg %s\n", GET(t).c_str(), disp.c_str(),
				alt / 0.3048, vz / 0.3048, vh / 0.3048,
				100 * pgncs.ThrustCommand() / 46700.0,
				lm.FindModule("LM descent stage")->tanks[0].mass,
				pgncs.DescentEngineOn() ? "ON" : "");
			lastDisp = disp;
			lastPrint = t;
		}
		static double lastGuid = 0;
		if (getenv("LMDIAG") && pgncs.DescentEngineOn() && t - lastGuid > 10) {
			lastGuid = t;
			auto sp = [&](const char* n, int i = 0) {
				return (int)agc::ToInt15(pad.Word(n, i)); };
			Vec3 up = scene.LocalVertical(lm.position);
			Vec3 vrel = lm.velocity - scene.SurfaceVelocity(lm.position);
			{
				double beta = (pgncs.LRPosition() == 1 ? 24.0 : 0.0) * kDeg;
				Mat3 ant = Mat3::RotX(6 * kDeg) * Mat3::RotY(beta);
				Vec3 beam = lm.ToInertial(ant * Vec3(-0.9374036, 0, -0.3482449)).Unit();
				Vec3 hdot;
				pad.ReadVector("HBEAMNB", -2, &hdot);
				Vec3 hnb = (ant * Vec3(-0.9374036, 0, -0.3482449)).Unit();
				std::printf("    [att] X.up %.3f Z.up %.3f\n",
					lm.ToInertial(Vec3(1, 0, 0)).Dot(up), lm.ToInertial(Vec3(0, 0, 1)).Dot(up));
				std::printf("    [beam] cos(beam,down) %.3f implied alt %.0f ft;"
					" AGC HBEAMNB %.3f %.3f %.3f vs ours %.3f %.3f %.3f\n",
					-beam.Dot(up), pgncs.LRAltitude() * -beam.Dot(up) / 0.3048,
					hdot.x, hdot.y, hdot.z, hnb.x, hnb.y, hnb.z);
			}
			std::printf("    [lr] pos %d rng %d vel %d slant %.0f ft true alt %.0f ft"
				" vz %.1f | ALT %d VEL %d  LRLCTR %d LRMCTR %d  FC %d FCOLD %d"
				" WCHPHASE %d\n", pgncs.LRPosition(), pgncs.LRRangeGood(),
				pgncs.LRVelocityGood(), pgncs.LRAltitude() / 0.3048,
				scene.Altitude(lm.position) / 0.3048, vrel.Dot(up) / 0.3048,
				d.alt, d.vel, sp("LRLCTR"), sp("LRMCTR"), sp("FC"), sp("FCOLD"),
				sp("WCHPHASE"));
		}
		static double lastNav = 0;
		if (getenv("NAVDIAG") && t - lastNav > 30 && t > kPDI_GET - 300) {
			lastNav = t;
			Vec3 rA, vA;
			pad.ReadVector("R", -24, &rA);
			pad.ReadVector("V", -7, &vA);   // m/cs
			vA = vA * 100.0;
			double pip = ((pad.Word("PIPTIME", 0) & 037777) * 16384.0
				+ (pad.Word("PIPTIME", 1) & 037777)) / 100.0;
			double age = t - pip;
			Vec3 rT = refsmmat * (lm.position - scene.MoonPosition());
			Vec3 vT = refsmmat * (lm.velocity - scene.MoonVelocity());
			rT = rT - vT * age;
			Vec3 up = rT.Unit(), fwd = (vT - up * vT.Dot(up)).Unit();
			Vec3 dr = rA - rT, dv = vA - vT;
			double agcT = ((agc.Cell(agc::REG_TIME2) & 037777) * 16384.0
				+ (agc.Cell(agc::REG_TIME1) & 037777)) / 100.0;
			std::printf("    [clock] t - AGC time = %.3f s\n", t - agcT);
			std::printf("    [nav] age %.2f dr up %.0f fwd %.0f m  dv up %.2f fwd %.2f m/s  |rA| %.0f\n",
				age, dr.Dot(up), dr.Dot(fwd), dv.Dot(up), dv.Dot(fwd), rA.Length());
		}
		static int diagCount = 0;
		static double lastDiag = 0;
		if (vn == "0618" && !d.verbNounFlash && diagCount < 30
			&& t - lastDiag > 1.0) {
			diagCount++;
			lastDiag = t;
			Vec3 g = pgncs.imu.Gimbals(lm.attitude);
			std::printf("    [maneuver] CDU %05o %05o %05o CDUD %05o %05o %05o"
				" ERR %05o %05o %05o OMEGA %05o %05o %05o rates %.3f %.3f %.3f"
				" T6 %u T5 %u T3 %u jets %03o %03o gimb %.1f %.1f %.1f\n",
				agc.Cell(agc::REG_CDUX), agc.Cell(agc::REG_CDUY),
				agc.Cell(agc::REG_CDUZ), pad.Word("CDUXD"), pad.Word("CDUYD"),
				pad.Word("CDUZD"), pad.Word("ERRORX"), pad.Word("ERRORY"),
				pad.Word("ERRORZ"), pad.Word("OMEGAP"), pad.Word("OMEGAQ"),
				pad.Word("OMEGAR"), lm.rates.x / kDeg, lm.rates.y / kDeg,
				lm.rates.z / kDeg, agc.InterruptCount(0), agc.InterruptCount(1),
				agc.InterruptCount(2), pgncs.JetBits(5), pgncs.JetBits(6),
				g.x / kDeg, g.y / kDeg, g.z / kDeg);
		}
		// Touchdown: the lowest footpad reaches the surface.
		for (const ContactPoint& c : lm.contacts) {
			Vec3 pt = lm.PointPosition(c.position);
			if (scene.Altitude(pt) <= 0) {
				landed = true;
				break;
			}
		}
		if (agc.RestartCount() != restarts) {
			std::printf("%s  !!! AGC hardware restart\n", GET(t).c_str());
			restarts = agc.RestartCount();
		}
		if (scene.Altitude(lm.position) < -5)
			break;
	}

	double alt = scene.Altitude(lm.position);
	Vec3 vrel = lm.velocity - scene.SurfaceVelocity(lm.position);
	Vec3 up = scene.LocalVertical(lm.position);
	double vz = vrel.Dot(up);
	double vh = (vrel - up * vz).Length();
	double tilt = std::acos(Clamp(lm.ToInertial(Vec3(1, 0, 0)).Dot(up), -1, 1))
		/ kDeg;
	double lat, lon, rr;
	scene.LatLon(lm.position, &lat, &lon, &rr);
	double miss = std::hypot((lat - kSiteLat) * kDeg,
		(lon - kSiteLon) * kDeg * std::cos(kSiteLat * kDeg)) * kSiteRadius;
	std::printf("\n%s at %s: alt %.1f m, vz %.2f m/s, vh %.2f m/s, tilt %.1f deg,"
		" %.0f kg DPS left, %.2f km from target (%.4f, %.4f)\n",
		landed ? "TOUCHDOWN" : "ENDED", GET(t).c_str(), alt, vz, vh, tilt,
		lm.FindModule("LM descent stage")->tanks[0].mass, miss / 1000, lat, lon);
	{
		Vec3 rAgc, land;
		pad.ReadVector("R", -24, &rAgc);
		pad.ReadVector("LAND", -24, &land);
		Vec3 rTrue = refsmmat * (lm.position - scene.MoonPosition());
		Vec3 sTrue = refsmmat * (scene.FixedToInertial() * siteFixed);
		std::printf("nav: AGC thinks %.2f km from LAND, truth %.2f km from site;"
			" AGC position error %.2f km; LAND vs site %.2f km\n",
			(rAgc - land).Length() / 1000, (rTrue - sTrue).Length() / 1000,
			(rAgc - rTrue).Length() / 1000, (land - sTrue).Length() / 1000);
	}
	bool ok = landed && std::fabs(vz) < 3.0 && vh < 1.5 && tilt < 12;
	std::printf("%s\n", ok ? "LANDING OK" : "LANDING FAILED");
	return ok ? 0 : 1;
}
