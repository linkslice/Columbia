// Columbia — Comanche flies an Apollo 11 lunar-return entry, headless.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: cm_entry_test <rope-dir> [program] [mission]
//
// The CSM is 25 minutes from entry interface on Apollo 11's return
// (36,200 ft/s at 400,000 ft, -6.5 degrees). A scripted crew loads the
// splash point in P61, separates the CM in P62 and lets P63/P64/P67 fly
// the entry; the drogues and mains come out at 24,000 and 10,000 ft.

#include "../core/agc/AGC.h"
#include "../core/agc/DSKY.h"
#include "../core/gnc/CMPGNCS.h"
#include "../core/gnc/MissionPads.h"
#include "../core/gnc/PadLoads.h"
#include "../core/gnc/Rope.h"
#include "../core/sim/Aero.h"
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
const double kEI = 195 * 3600 + 3 * 60 + 6;        // 195:03:06
const double kSplashLat = 13.30, kSplashLon = -169.15;
const double kEIAltitude = 121920;                  // 400,000 ft
const double kEIVelocity = 11032;                   // 36,194 ft/s
const double kEIGamma = -6.48 * kDeg;
const double kEIRange = 2250e3;                     // EI to splash, m

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

std::string
Entry(long v)
{
	char b[32];
	std::snprintf(b, sizeof(b), "%c%05ld", v < 0 ? '-' : '+', std::labs(v));
	return b;
}

Vec3
EarthGravity(const Vec3& r, double jd)
{
	(void)jd;
	double rl = r.Length();
	return r * (-world::kMuEarth / (rl * rl * rl));
}

}  // namespace


int
main(int argc, char** argv)
{
	std::string ropeDir = argc > 1 ? argv[1] : "rope";
	std::string program = argc > 2 ? argv[2] : "Comanche055";
	int mission = argc > 3 ? std::atoi(argv[3]) : 11;

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

	// --- Entry interface state, then back up 25 minutes ------------------------
	double jdEI = kGETZero + kEI / 86400.0;
	// Splash point in inertial space allowing for ~14 minutes of Earth
	// rotation between EI and splashdown.
	double jdSplash = jdEI + 840 / 86400.0;
	Vec3 target = world::Ephemeris::EarthFixedToInertial(jdSplash)
		* world::LatLonToVector(kSplashLat, kSplashLon, 1.0);
	target = target.Unit();
	// Arrive heading ~60 degrees (east-north-east), as Apollo 11 did.
	Vec3 eastT = Vec3(0, 0, 1).Cross(target).Unit();
	Vec3 northT = target.Cross(eastT);
	double heading = 60 * kDeg;
	Vec3 dirT = northT * std::cos(heading) + eastT * std::sin(heading);
	Vec3 axis = target.Cross(dirT).Unit();          // plane normal
	double arc = kEIRange / world::kREarth;
	Vec3 eiHat = target * std::cos(arc) - dirT * std::sin(arc);
	Vec3 along = axis.Cross(eiHat).Unit();
	double rEI = world::kREarth + kEIAltitude;
	Vec3 rState = eiHat * rEI;
	Vec3 vState = (along * std::cos(kEIGamma) + eiHat * std::sin(kEIGamma))
		* kEIVelocity;
	{
		double la, lo, rr;
		Vec3 f = world::Ephemeris::EarthFixedToInertial(jdEI).Transposed() * eiHat;
		world::VectorToLatLon(f, &la, &lo, &rr);
		Vec3 tf = world::LatLonToVector(kSplashLat, kSplashLon, 1.0);
		std::printf("EI at %.2f N %.2f E, %.0f nmi from the splash point\n", la, lo,
			std::acos(Clamp(f.Unit().Dot(tf), -1, 1)) * world::kREarth / 1852);
	}
	double lead = 1500;
	{
		const double h = -1.0;
		for (double s = 0; s < lead; s += 1.0) {
			Vec3 r = rState, v = vState;
			Vec3 k1v = EarthGravity(r, 0), k1r = v;
			Vec3 k2v = EarthGravity(r + k1r * (h / 2), 0), k2r = v + k1v * (h / 2);
			Vec3 k3v = EarthGravity(r + k2r * (h / 2), 0), k3r = v + k2v * (h / 2);
			Vec3 k4v = EarthGravity(r + k3r * h, 0), k4r = v + k3v * h;
			rState = r + (k1r + k2r * 2 + k3r * 2 + k4r) * (h / 6);
			vState = v + (k1v + k2v * 2 + k3v * 2 + k4v) * (h / 6);
		}
	}
	double startGET = kEI - lead;

	craft::CSMConfig cfg;
	cfg.spsPropellant = 300;
	Vehicle csm("Columbia");
	csm.AddModule(craft::MakeCommandModule(cfg));
	csm.AddModule(craft::MakeServiceModule(cfg));
	csm.position = rState;
	csm.velocity = vState;
	csm.jd = kGETZero + startGET / 86400.0;
	// Start in a heads-down, SPS-forward attitude: +X along -velocity.
	Vec3 xb = (-vState).Unit();
	Vec3 zb = (-rState).Unit();
	zb = (zb - xb * zb.Dot(xb)).Unit();
	Vec3 yb = zb.Cross(xb);
	csm.attitude = Quat::FromMatrix(Mat3::Columns(xb, yb, zb));

	gnc::CMPGNCS pgncs(agc, csm);
	// Entry REFSMMAT: X downrange at EI, Z down, Y = Z x X.
	Vec3 smX = along, smZ = -eiHat, smY = smZ.Cross(smX);
	Mat3 refsmmat = Mat3::Rows(smX, smY, smZ);
	pgncs.imu.Align(refsmmat);

	agc.PowerOn();
	double t = startGET;
	bool separated = false, drogues = false, mains = false, splash = false;
	double maxG = 0, maxHeat = 0;
	sim::EntryAeroState aero;
	auto run = [&](double seconds) {
		const double dt = 0.005;
		for (double s = 0; s < seconds && !splash; s += dt) {
			Vec3 rr = csm.position;
			double alt = rr.Length() - world::kREarth;
			Vec3 wind = Vec3(0, 0, world::kEarthRate).Cross(rr);
			aero = ApplyAerodynamics(csm, csm.velocity - wind, rr.Unit(), alt,
				separated);
			if (separated && (drogues || mains)) {
				// Parachutes: extra drag, and they hold the CM apex-up.
				world::Atmosphere air = world::Atmosphere::At(alt);
				Vec3 va = csm.velocity - wind;
				double cdA = mains ? 3 * 0.8 * 0.25 * 3.14159 * 25.5 * 25.5
					: 2 * 0.6 * 0.25 * 3.14159 * 5 * 5;
				csm.AddExternalForce(va * (-0.5 * air.density * va.Length() * cdA),
					csm.rates * -2000.0);
			}
			csm.Step(dt, [](const Vec3& r, double jd) {
				return world::Gravity::Acceleration(r, jd, false);
			});
			pgncs.Step(dt);
			t += dt;
			if (separated) {
				maxG = std::max(maxG, aero.gLoad);
				maxHeat = std::max(maxHeat, aero.heatRate);
			}
			if (alt < 0)
				splash = true;
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

	run(1.0);
	type("V36E");
	run(3.0);
	gnc::PadLoader pad(agc, rope);
	std::printf("mission pads: %d words\n",
		pad.Apply(gnc::MissionPads(mission, false)));
	gnc::CMCPadLoad load;
	load.agcTime = t;
	load.tephem = (kGETZero - JulianDate(1969, 7, 1)) * 86400.0;
	load.azo = world::Ephemeris::GMST(JulianDate(1969, 7, 1)) / (2 * M_PI);
	load.refsmmat = refsmmat;
	load.csm.position = csm.position;
	load.csm.velocity = csm.velocity;
	load.csm.agcTime = t;
	load.csm.moon = false;
	load.csmMass = csm.Mass();
	gnc::LoadCMCMission(pad, load);
	gnc::SetComputerClock(agc, t);
	{
		Vec3 rr, vv;
		pad.ReadVector("RRECTCSM", -29, &rr);
		pad.ReadVector("VRECTCSM", -7, &vv);
		std::printf("loaded: |R| %.0f km |V| %.1f m/s  CMOONFLG %d MOONFLAG %d\n",
			rr.Length() / 1000, vv.Length() * 100,
			gnc::GetFlag(pad, gnc::FLAG_CMOONFLG), gnc::GetFlag(pad, gnc::FLAG_MOONFLAG));
	}
	run(1.0);
	type("V37E00E");
	run(2.0);
	type("V48E");
	run(1.0);
	for (int i = 0; i < 3; i++) {
		type("P");
		run(1.0);
	}
	type("V46E");
	run(2.0);

	if (getenv("P21CHECK")) {
		// P21: where does the CMC think we are over the Earth?
		type("V37E21E");
		run(2.0);
		dsky.Sync(agc);
		std::printf("%s  %s\n", GET(t).c_str(), Disp(dsky.State()).c_str());
		type("P");                           // N06: this vehicle
		run(2.0);
		long now = std::lround((t + 30) * 100);
		type("V25E" + Entry(now / 360000) + "E" + Entry(now / 6000 % 60) + "E"
			+ Entry(now % 6000) + "E");
		run(1.0);
		double tq = now / 100.0;
		type("P");
		for (int i = 0; i < 40; i++) {
			run(0.5);
			dsky.Sync(agc);
			std::string vnn = std::string(dsky.State().noun, 2);
			if (dsky.State().verbNounFlash && vnn == "43") break;
		}
		run(1.0);
		dsky.Sync(agc);
		double la, lo, rr;
		Vec3 rq = csm.position + csm.velocity * (tq - t);   // rough
		double jq = kGETZero + tq / 86400.0;
		Vec3 f = world::Ephemeris::EarthFixedToInertial(jq).Transposed() * rq;
		world::VectorToLatLon(f, &la, &lo, &rr);
		std::printf("%s  %s   (truth %.2f N %.2f E)\n", GET(t).c_str(),
			Disp(dsky.State()).c_str(), la, lo);
		return 0;
	}
	// P61: splash point and heads-up/down.
	type("V37E61E");
	double lastKey = -100;
	std::string lastDisp;
	double lastPrint = 0;
	bool p61Loaded = false;
	while (t < kEI + 1500 && !splash) {
		run(0.1);
		dsky.Sync(agc);
		const agc::DSKYState& d = dsky.State();
		std::string disp = Disp(d);
		std::string vn = std::string(d.verb, 2) + std::string(d.noun, 2);
		std::string prog(d.prog, 2);
		std::string r1(d.reg[0], 5);
		double alt = csm.position.Length() - world::kREarth;

		static double lastDump = 0;
		if (getenv("NAVDUMP") && t - lastDump > 5) {
			lastDump = t;
			Vec3 rn, vn;
			pad.ReadVector("RN", -29, &rn);
			pad.ReadVector("VN", -7, &vn);
			vn = vn * 100;
			Vec3 smRN = refsmmat * csm.position, cross = vn.Cross(rn).Unit();
			std::printf("    RN %.0f km (true %.0f), VN %.0f m/s (true %.0f), RN.true %.4f,"
				" ySM.(VxR) %.3f\n", rn.Length() / 1000, csm.position.Length() / 1000,
				vn.Length(), csm.velocity.Length(), rn.Unit().Dot(csm.position.Unit()),
				refsmmat.Row(1).Dot(cross));
			(void)smRN;
		}
		if (d.verbNounFlash && t - lastKey > 2.0) {
			std::string key;
			if (vn == "0661" && !p61Loaded) {
				// Splash latitude, longitude (degrees, XXX.XX), heads up (+1).
				key = "V25E" + Entry(std::lround(kSplashLat * 100)) + "E"
					+ Entry(std::lround(kSplashLon * 100)) + "E-00001E";   // lift up
				p61Loaded = true;
			} else if (vn == "5025" && r1 == "00041") {
				if (!separated) {
					// CM/SM separation.
					csm.RemoveModule("Service Module");
					for (Module& m : csm.Modules())
						for (Thruster& th : m.thrusters)
							th.failedOff = false;
					pgncs.panel.cmSmSeparated = true;
					separated = true;
					std::printf("%s  crew: CM/SM SEP, alt %.0f km\n", GET(t).c_str(),
						alt / 1000);
				}
				key = "P";
			} else if (vn == "5018") {
				const char* fin[3] = {"CPHI", "CTHETA", "CPSI"};
				const int cdu[3] = {agc::REG_CDUX, agc::REG_CDUY, agc::REG_CDUZ};
				double worst = 0;
				for (int i = 0; i < 3; i++)
					worst = std::max(worst, std::fabs(WrapPi(
						gnc::CDUToAngle(pad.Word(fin[i]))
						- gnc::CDUToAngle(agc.Cell(cdu[i])))) / kDeg);
				static double lastManeuver = -1e9;
				if (worst < 3.0)
					key = "E";
				else if (t - lastManeuver > 150) {
					key = "P";
					lastManeuver = t;
				}
			} else if (vn.substr(0, 2) == "50" || vn.substr(0, 2) == "06"
				|| vn.substr(0, 2) == "16") {
				key = "P";
			}
			if (!key.empty()) {
				std::printf("%s  crew: %s on %s\n", GET(t).c_str(),
					key.size() > 1 ? key.c_str() : (key == "P" ? "PRO" : "ENTR"),
					disp.c_str());
				type(key);
				lastKey = t;
			}
		}
		if (separated && !drogues && alt < 7315 && aero.mach < 1.0) {
			drogues = true;
			std::printf("%s  DROGUES at %.0f ft\n", GET(t).c_str(), alt / 0.3048);
		}
		if (drogues && !mains && alt < 3048) {
			mains = true;
			std::printf("%s  MAIN CHUTES at %.0f ft\n", GET(t).c_str(), alt / 0.3048);
		}
		if (disp != lastDisp && t - lastPrint > 1.0) {
			std::printf("%s  %-52s alt %6.1f km v %6.0f m/s g %4.1f roll %6.1f"
				" alpha %6.1f\n", GET(t).c_str(), disp.c_str(), alt / 1000,
				csm.velocity.Length(), aero.gLoad, aero.roll / kDeg,
				aero.alpha / kDeg);
			lastDisp = disp;
			lastPrint = t;
		}
	}

	// Splash point versus target.
	double lat, lon, r;
	Vec3 fixed = world::Ephemeris::EarthFixedToInertial(csm.jd).Transposed()
		* csm.position;
	world::VectorToLatLon(fixed, &lat, &lon, &r);
	Vec3 tgt = world::LatLonToVector(kSplashLat, kSplashLon, 1.0);
	double miss = std::acos(Clamp(fixed.Unit().Dot(tgt), -1, 1)) * world::kREarth;
	Vec3 v = csm.velocity - Vec3(0, 0, world::kEarthRate).Cross(csm.position);
	std::printf("\n%s at %s: %.3f N %.3f E, %.1f nmi from target; max %.1f g,"
		" peak heating %.0f W/cm2, impact %.1f m/s\n", splash ? "SPLASHDOWN"
		: "ENDED", GET(t).c_str(), lat, lon, miss / 1852, maxG, maxHeat / 1e4,
		v.Length());
	bool ok = splash && miss < 20 * 1852 && maxG < 10 && v.Length() < 12;
	std::printf("%s\n", ok ? "ENTRY OK" : "ENTRY FAILED");
	return ok ? 0 : 1;
}
