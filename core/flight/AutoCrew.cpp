// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "AutoCrew.h"

#include "FlightSim.h"
#include "Trajectory.h"
#include "../gnc/PadLoads.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace flight {

using namespace sim;

namespace {

double
WrapPi(double a)
{
	while (a > M_PI)
		a -= 2 * M_PI;
	while (a < -M_PI)
		a += 2 * M_PI;
	return a;
}

// Worst difference (deg) between the CDUs and the maneuver's final gimbal
// angles, which the attitude maneuver routine leaves in CPHI/CTHETA/CPSI
// (N18 itself shows FDAI ball angles).
double
ManeuverError(gnc::PadLoader& pad, agc::Computer& c)
{
	const char* fin[3] = {"CPHI", "CTHETA", "CPSI"};
	const int cdu[3] = {agc::REG_CDUX, agc::REG_CDUY, agc::REG_CDUZ};
	double worst = 0;
	for (int i = 0; i < 3; i++)
		worst = std::max(worst, std::fabs(WrapPi(gnc::CDUToAngle(pad.Word(fin[i]))
			- gnc::CDUToAngle(c.Cell(cdu[i])))) / kDeg);
	return worst;
}

// P52 by uplink: a new REFSMMAT in the CMC and the platform torqued to it.
void
AlignCMC(FlightSim& f, const Mat3& refsmmat)
{
	gnc::PadLoader& pad = f.CMCPad();
	pad.Vector("REFSMMAT", refsmmat.Row(0), -1);
	Vec3 r1 = refsmmat.Row(1), r2 = refsmmat.Row(2);
	double comps[6] = {r1.x, r1.y, r1.z, r2.x, r2.y, r2.z};
	for (int i = 0; i < 6; i++) {
		uint16_t hi, lo;
		gnc::EncodeDP(std::ldexp(comps[i], -1), &hi, &lo);
		pad.Octal("REFSMMAT", hi, 6 + 2 * i);
		pad.Octal("REFSMMAT", lo, 7 + 2 * i);
	}
	gnc::SetFlag(pad, gnc::FLAG_REFSMFLG, true);
	f.CMPGNCS().imu.Align(refsmmat);
	// The RCS DAP holds the CDU angles it was given; after the platform
	// moves, hold the attitude the spacecraft actually has.
	Vec3 g = f.CMPGNCS().imu.Gimbals(f.CSMVehicle().attitude);
	pad.Octal("THETADX", gnc::AngleToCDU(g.x));
	pad.Octal("THETADY", gnc::AngleToCDU(g.y));
	pad.Octal("THETADZ", gnc::AngleToCDU(g.z));
}

// P57 on the lunar surface, by its result: the platform realigned with X up
// the local vertical and Z downrange in the CSM's orbit plane; then the
// surface V48 (ascent stage, ascent masses) and a fresh CSM state vector
// from the MSFN.
void
PrepareLGCForAscent(FlightSim& f)
{
	gnc::PadLoader& pad = *f.LGCPad();
	Vehicle& lm = *f.LMVehicle();
	const MoonScene& moon = f.Moon();
	Vec3 mp = moon.MoonPosition(), mv = moon.MoonVelocity();
	Vec3 up = moon.LocalVertical(lm.position);
	Vec3 h = (f.CSMVehicle().position - mp).Cross(f.CSMVehicle().velocity - mv).Unit();
	Vec3 downrange = h.Cross(up).Unit();
	Mat3 refsmmat = Mat3::Rows(up, downrange.Cross(up), downrange);
	pad.Vector("REFSMMAT", refsmmat.Row(0), -1);
	Vec3 r1 = refsmmat.Row(1), r2 = refsmmat.Row(2);
	double comps[6] = {r1.x, r1.y, r1.z, r2.x, r2.y, r2.z};
	for (int i = 0; i < 6; i++) {
		uint16_t hi, lo;
		gnc::EncodeDP(std::ldexp(comps[i], -1), &hi, &lo);
		pad.Octal("REFSMMAT", hi, 6 + 2 * i);
		pad.Octal("REFSMMAT", lo, 7 + 2 * i);
	}
	gnc::SetFlag(pad, gnc::FLAG_REFSMFLG, true);
	f.LMPGNCS()->imu.Align(refsmmat);

	const Module* a = lm.FindModule("LM ascent stage");
	double mass = a ? a->Mass() : lm.Mass();
	pad.Double("MASS", mass, -16);
	pad.Single("LEMMASS", mass, -16);
	gnc::SetFlag(pad, gnc::FLAG_APSFLAG, true);
	gnc::SetFlag(pad, gnc::FLAG_SURFFLAG, true);
	// V48 R1: LM alone (CSMDOCKED clear).
	pad.Octal("DAPBOOLS", pad.Word("DAPBOOLS") & ~010000);   // CSMDOCKD

	gnc::StateVector csm;
	csm.position = f.CSMVehicle().position - mp;
	csm.velocity = f.CSMVehicle().velocity - mv;
	csm.agcTime = f.GET();
	gnc::LoadStateVector(pad, csm, false);
}

// A MSFN state-vector uplink to the CMC as of now, about the body the CMC
// would use. Returns false (and loads nothing) if not needed.
gnc::StateVector
UplinkCSMState(FlightSim& f)
{
	Vehicle& v = f.CSMVehicle();
	bool moon = f.Situate(v).nearMoon;
	gnc::StateVector sv;
	sv.position = v.position - (moon ? f.Moon().MoonPosition() : Vec3());
	sv.velocity = v.velocity - (moon ? f.Moon().MoonVelocity() : Vec3());
	sv.agcTime = f.GET();
	sv.moon = moon;
	gnc::PadLoader& pad = f.CMCPad();
	gnc::LoadStateVector(pad, sv, false);
	gnc::SetFlag(pad, gnc::FLAG_MOONFLAG, moon);
	return sv;
}

// Did the uplink take? The CMC's periodic integration, if it was under way,
// stores its own (old) state over ours when it finishes; real uplinks were
// checked the same way and sent again.
bool
UplinkTook(gnc::PadLoader& pad, const gnc::StateVector& sv, bool lm)
{
	if (gnc::GetFlag(pad, lm ? gnc::FLAG_LMOONFLG : gnc::FLAG_CMOONFLG) != sv.moon)
		return false;
	Vec3 r;
	pad.ReadVector(lm ? "RRECTLEM" : "RRECTCSM", sv.moon ? -27 : -29, &r);
	return (r - sv.position).Length() < 50e3;
}

bool
UplinkTook(FlightSim& f, const gnc::StateVector& sv)
{
	return UplinkTook(f.CMCPad(), sv, false);
}

}  // namespace


// ---------------------------------------------------------------------------
// DSKYDriver


DSKYDriver::DSKYDriver(std::function<void(int)> press,
	std::function<void(bool)> proceed,
	std::function<const agc::DSKYState&()> state)
	:
	fPress(press),
	fProceed(proceed),
	fState(state)
{
}


void
DSKYDriver::Type(const std::string& keys)
{
	for (char c : keys)
		fQueue.push_back(c);
}


void
DSKYDriver::SelectProgram(int program)
{
	char buf[16];
	std::snprintf(buf, sizeof(buf), "V37E%02dE", program);
	Type(buf);
	fWantProgram = program;
	fSinceSelect = 0;
	fTries = 0;
}


void
DSKYDriver::Step(double dt)
{
	{
		std::string disp = Display();
		if (disp == fLastDisplay)
			fStable += dt;
		else {
			fLastDisplay = disp;
			fStable = 0;
		}
	}
	if (State().oprErr && fQueue.empty() && fWait <= 0 && fStable > 1.5)
		Type("R");      // clear an operator error before going on
	if (fWantProgram >= 0 && fQueue.empty()) {
		fSinceSelect += dt;
		char want[16];
		std::snprintf(want, sizeof(want), "%02d", fWantProgram % 100);
		if (Program() == want)
			fWantProgram = -1;
		else if (fSinceSelect > 6 && fTries < 20) {
			char buf[16];
			std::snprintf(buf, sizeof(buf), "RV37E%02dE", fWantProgram);
			Type(buf);
			fSinceSelect = 0;
			fTries++;
		}
	}
	if (fWait > 0) {
		fWait -= dt;
		if (fWait > 0)
			return;
		if (fProceedDown) {
			fProceed(false);
			fProceedDown = false;
			fWait = 0.3;
			return;
		}
	}
	if (fQueue.empty())
		return;
	char c = fQueue.front();
	fQueue.pop_front();
	if (c == 'P') {
		fProceed(true);
		fProceedDown = true;
		fWait = 0.3;
		return;
	}
	int code;
	switch (c) {
		case 'V': code = agc::KEY_VERB; break;
		case 'N': code = agc::KEY_NOUN; break;
		case 'E': code = agc::KEY_ENTR; break;
		case 'R': code = agc::KEY_RSET; break;
		case 'K': code = agc::KEY_KEYREL; break;
		case 'C': code = agc::KEY_CLR; break;
		case '+': code = agc::KEY_PLUS; break;
		case '-': code = agc::KEY_MINUS; break;
		default: code = c == '0' ? agc::KEY_0 : c - '0';
	}
	fPress(code);
	fWait = 0.25;
}


std::string
DSKYDriver::Program() const
{
	return std::string(State().prog, 2);
}


std::string
DSKYDriver::VerbNoun() const
{
	return std::string(State().verb, 2) + std::string(State().noun, 2);
}


std::string
DSKYDriver::Register(int r) const
{
	return std::string(State().reg[r], 5);
}


std::string
DSKYDriver::Display() const
{
	const agc::DSKYState& d = State();
	char buf[120];
	std::snprintf(buf, sizeof(buf), "P%c%c V%c%c N%c%c %s %s %s%s%s",
		d.prog[0], d.prog[1], d.verb[0], d.verb[1], d.noun[0], d.noun[1],
		d.RegisterText(0).c_str(), d.RegisterText(1).c_str(),
		d.RegisterText(2).c_str(), d.verbNounFlash ? " FL" : "",
		d.progAlarm ? " PROG" : "");
	return buf;
}


std::string
DSKYDriver::Signed(long v)
{
	char b[32];
	std::snprintf(b, sizeof(b), "%c%05ld", v < 0 ? '-' : '+',
		std::min(std::labs(v), 99999L));
	return b;
}


std::string
DSKYDriver::TimeEntry(double get)
{
	long cs = std::lround(get * 100);
	return "V25E" + Signed(cs / 360000) + "E" + Signed(cs / 6000 % 60) + "E"
		+ Signed(cs % 6000) + "E";
}


// ---------------------------------------------------------------------------
// AutoCrew


double
Procedure::WakeGET(FlightSim& f) const
{
	return f.GET();
}


AutoCrew::AutoCrew(FlightSim& sim)
	:
	cmc([&sim](int k) { sim.PressCMC(k); },
		[&sim](bool d) { sim.ProceedCMC(d); },
		[&sim]() -> const agc::DSKYState& { return sim.CMCDSKY().State(); }),
	lgc([&sim](int k) { sim.PressLGC(k); },
		[&sim](bool d) { sim.ProceedLGC(d); },
		[&sim]() -> const agc::DSKYState& { return sim.LGCDSKY().State(); }),
	f(sim)
{
}


AutoCrew::~AutoCrew()
{
}


void
AutoCrew::Install()
{
	if (fInstalled)
		return;
	fInstalled = true;
	f.stepHooks.push_back([this](FlightSim&, double dt) { _Step(dt); });
	// Between lockstep steps (coasting), keep the wake time current.
	f.secondHooks.push_back([this](FlightSim& sim, double) {
		if (troubleshoot)
			_Troubleshoot();
		if (fQueue.empty())
			sim.lockstepFrom = 1e30;
		else if (fQueue.front()->started)
			sim.lockstepFrom = sim.GET();
	});
}


// The crew's answers to the failures they notice, the way the real crews
// (and Houston) worked them out, after a little time to diagnose.
void
AutoCrew::_Troubleshoot()
{
	double get = f.GET();
	if (std::getenv("RCSDIAG") && f.LMVehicle()) {
		static double last = 0;
		if (get - last > 600) {
			last = get;
			const Module* asc = f.LMVehicle()->FindModule("LM ascent stage");
			gnc::LMPGNCS* pg = f.LMPGNCS();
			if (asc && asc->tanks.size() > 2 && pg)
				std::printf("RCSDIAG %.0f landed %d RCS A %.1f B %.1f mode %d jets %03o %03o P%s\n",
					get, (int)f.LMLanded(), asc->tanks[1].mass, asc->tanks[2].mass,
					(int)pg->panel.mode, pg->JetBits(5), pg->JetBits(6), lgc.Program().c_str());
		}
	}
	auto noticed = [&](const std::string& what, double delay) {
		auto it = fNoticed.find(what);
		if (it == fNoticed.end()) {
			fNoticed[what] = get;
			return false;
		}
		return get - it->second >= delay;
	};
	auto forget = [&](const std::string& what) { fNoticed.erase(what); };

	// Apollo 12: the fuel cells drop off the buses and the SCE is lost
	// ("SCE to AUX"), then the fuel cells reset.
	if (f.faults.sceFailed) {
		if (noticed("sce", 25)) {
			f.SCEToAux();
			Report("SCE to AUX");
			forget("sce");
		}
	}
	if (f.csmSystems.FuelCellsOnline() < 3 && f.CurrentPhase() == Phase::Boost
			&& !f.faults.sceFailed) {
		if (noticed("fuelcells", 15)) {
			f.ResetFuelCells();
			Report("Fuel cells reset to the main buses");
			forget("fuelcells");
		}
	}
	// Apollo 11: the broken engine-arm breaker, found on the floor.
	if (gnc::LMPGNCS* pg = f.LMPGNCS()) {
		if (pg->ascentArmBreakerOpen && f.LMLanded()) {
			if (noticed("breaker", 600)) {
				f.PenInBreaker();     // (logs it)
				forget("breaker");
			}
		}
		// Apollo 11: rendezvous radar left in SLEW after the landing.
		if (pg->rrCDUHunting && f.LMLanded()) {
			if (noticed("rr", 60)) {
				f.RRModeToLGC();
				Report("Rendezvous radar mode switch to LGC");
				forget("rr");
			}
		}
		// Apollo 14: the landing radar breaker cycled.
		if (pg->lrFailed && f.CurrentPhase() == Phase::PoweredDescent
				&& f.Mission().number == 14) {
			if (noticed("lr", 45)) {
				pg->lrFailed = false;
				Report("Landing radar breaker cycled. Radar data good.");
				forget("lr");
			}
		}
		// Apollo 14: the solder ball in the ABORT switch. The uplinked
		// procedure keyed at ignition leaves the LGC ignoring the discrete.
		if (pg->panel.abort && !pg->abortIgnored && f.LMVehicle()
				&& !f.LMLanded()) {
			pg->abortIgnored = true;
			Report("ABORT light with the switch out. Keying the uplinked"
				" procedure: the LGC will ignore the switch");
		}
		// Apollo 10: the AGS mode switch, and the rates nulled by hand.
		if (f.faults.stagingGyration) {
			if (noticed("gyration", 6)) {
				f.faults.stagingGyration = false;
				pg->panel.mode = gnc::LMPanel::MODE_ATT_HOLD;
				Report("Manual control. Rates nulled, ATTITUDE HOLD.");
				forget("gyration");
			}
		}
	}
	// A stuck RCS jet: find the quad and close its isolation valves.
	for (Module& m : f.CSMVehicle().Modules())
		for (Thruster& t : m.thrusters)
			if (t.failedOn && !t.failedOff) {
				if (noticed("jet" + t.name, 40)) {
					t.failedOff = true;
					t.on = false;
					t.level = 0;
					Report("Quad isolation valves closed on jet " + t.name);
					forget("jet" + t.name);
				}
			}
	// Apollo 15: the SPS thrust switch short on bank A. Bank B only.
	if (f.SMAttached() && f.CMPGNCS().spsThrustShortA
			&& f.CMPGNCS().panel.spsBankA) {
		if (noticed("spsA", 20)) {
			f.CMPGNCS().panel.spsBankA = false;
			Report("SPS bank A off; burns on bank B");
			forget("spsA");
		}
	}
	// A slow cabin leak: suits on, then find it.
	if (f.cmCabin.leakRate > 0) {
		if (noticed("leak", 1800)) {
			f.cmCabin.leakRate = 0;
			Report("Found the leak: a dump valve not fully seated");
			forget("leak");
		}
	}
}


void
AutoCrew::Add(std::unique_ptr<Procedure> p)
{
	fQueue.push_back(std::move(p));
	f.lockstepFrom = fQueue.front()->WakeGET(f);
}


void
AutoCrew::Abandon()
{
	fQueue.clear();
	cmc.Clear();
	lgc.Clear();
	f.lockstepFrom = 1e30;
}


void
AutoCrew::Report(const std::string& text)
{
	// On the air-to-ground loop (the log, the replays).
	if (logToFlight)
		f.Log(LogEntry::Crew, text);
	if (onReport)
		onReport(text);
	if (verbose)
		std::printf("[%9.1f] crew: %s\n", f.GET(), text.c_str());
}


void
AutoCrew::_Step(double dt)
{
	cmc.Step(dt);
	lgc.Step(dt);
	if (fQueue.empty())
		return;
	Procedure* p = fQueue.front().get();
	if (!p->started) {
		double wake = p->WakeGET(f);
		if (f.GET() < wake - 1.0) {
			f.lockstepFrom = wake;
			return;
		}
		p->started = true;
	}
	if (p->Step(*this, f, dt)) {
		if (p->failed)
			Report(p->Name() + " FAILED: " + p->status);
		else
			Report(p->Name() + " complete");
		fQueue.pop_front();
		f.lockstepFrom = fQueue.empty() ? 1e30 : fQueue.front()->WakeGET(f);
	}
}


// ---------------------------------------------------------------------------
// Generic procedures


namespace {

class FunctionProcedure : public Procedure {
public:
	FunctionProcedure(const std::string& name,
		std::function<bool(AutoCrew&, FlightSim&, double)> step, double wake)
		: fName(name), fStep(step), fWake(wake) {}
	std::string Name() const override { return fName; }
	bool Step(AutoCrew& c, FlightSim& f, double dt) override
		{ return fStep(c, f, dt); }
	double WakeGET(FlightSim& f) const override
		{ return std::max(fWake, f.GET() < fWake ? fWake : f.GET()); }

private:
	std::string fName;
	std::function<bool(AutoCrew&, FlightSim&, double)> fStep;
	double fWake;
};

}  // namespace


std::unique_ptr<Procedure>
AutoCrew::Do(const std::string& name,
	std::function<bool(AutoCrew&, FlightSim&, double)> step, double wake)
{
	return std::unique_ptr<Procedure>(new FunctionProcedure(name, step, wake));
}


std::unique_ptr<Procedure>
AutoCrew::Keys(bool lgc, const std::string& keys, double atGET)
{
	auto typed = std::make_shared<bool>(false);
	return Do("Keys " + keys, [lgc, keys, typed](AutoCrew& c, FlightSim&, double) {
		DSKYDriver& d = lgc ? c.lgc : c.cmc;
		if (!*typed) {
			d.Type(keys);
			*typed = true;
			return false;
		}
		return d.Idle();
	}, atGET);
}


// ---------------------------------------------------------------------------
// CSM burn: P30 / P40


namespace {

class CSMBurnProcedure : public Procedure {
public:
	CSMBurnProcedure(double tig, const Vec3& dv, const std::string& label)
		: fTIG(tig), fDV(dv), fLabel(label) {}
	std::string Name() const override { return fLabel; }
	double WakeGET(FlightSim&) const override { return fTIG - 1500; }
	bool Step(AutoCrew& c, FlightSim& f, double dt) override;

private:
	double fTIG;
	Vec3 fDV;               // LVLH m/s
	std::string fLabel;
	int fState = 0;
	gnc::StateVector fSV;
	double fTimer = 0;
	double fLastKey = -100;
	double fLastManeuver = -1e9;
	bool fArmed = false;
	double fIgnition = -1, fCutoff = -1;
};


bool
CSMBurnProcedure::Step(AutoCrew& c, FlightSim& f, double dt)
{
	DSKYDriver& d = c.cmc;
	fTimer += dt;
	double get = f.GET();
	gnc::CMPGNCS& pg = f.CMPGNCS();
	switch (fState) {
		case 0: {
			// Mission Control uplinks a fresh state vector; the crew aligns
			// the platform to the burn's "preferred" orientation (P52).
			Vehicle& v = f.CSMVehicle();
			bool moon = f.Situate(v).nearMoon;
			State s{v.position, v.velocity, f.JulianDate()};
			State atTig = Propagate(s, fTIG - get);
			Vec3 rT = atTig.r - (moon ? MoonPos(atTig.jd) : Vec3());
			Vec3 vT = atTig.v - (moon ? MoonVel(atTig.jd) : Vec3());
			Vec3 dvI = FromLVLH(rT, vT, fDV);
			// X along the burn, Y along the axis of the maneuver from the
			// present attitude, so the swing to the burn attitude keeps the
			// middle gimbal near zero (no gimbal lock on the way).
			Vec3 x = dvI.Length() > 1e-6 ? dvI.Unit() : vT.Unit();
			Vec3 now = v.ToInertial(Vec3(1, 0, 0));
			Vec3 y = now.Cross(x);
			if (y.Length() < 0.05) {
				Vec3 z = -rT.Unit();
				y = z.Cross(x);
			}
			y = (y - x * y.Dot(x)).Unit();
			Mat3 refsmmat = Mat3::Rows(x, y, x.Cross(y));
			gnc::PadLoader& pad = f.CMCPad();
			fSV = UplinkCSMState(f);
			AlignCMC(f, refsmmat);
			// DAP configuration and masses.
			bool docked = f.LMDocked();
			double lmMass = 0;
			if (docked) {
				for (const Module& m : v.Modules())
					if (m.name == "LM descent stage" || m.name == "LM ascent stage")
						lmMass += m.Mass();
			}
			pad.Single("CSMMASS", v.Mass() - lmMass, -16);
			pad.Single("LEMMASS", lmMass, -16);
			pad.Octal("DAPDATR1", docked ? 021102 : 011102);
			pg.panel.scContCMC = true;
			pg.panel.mode = gnc::CMPanel::MODE_AUTO;
			c.Report(fLabel + ": state vector and REFSMMAT uplinked");
			d.SelectProgram(0);
			fState = 1;
			fTimer = 0;
			break;
		}
		case 1:   // V48: DAP data load
			if (!d.Ready() || fTimer < 2)
				break;
			if (!UplinkTook(f, fSV)) {
				fSV = UplinkCSMState(f);
				c.Report(fLabel + ": state vector uplink sent again");
				fTimer = 0;
				break;
			}
			d.Type("V48E");
			fState = 2;
			fTimer = 0;
			break;
		case 2:
			if (!d.Idle() || fTimer < 1)
				break;
			if (d.Flashing() && (d.VerbNoun() == "0446" || d.VerbNoun() == "0647"
					|| d.VerbNoun() == "0648" || d.VerbNoun().substr(2) == "46"
					|| d.VerbNoun().substr(2) == "47" || d.VerbNoun().substr(2) == "48")) {
				d.Type("P");
				fTimer = 0;
				break;
			}
			if (fTimer > 2 && d.Ready()) {
				d.Type("V46E");
				fState = 3;
				fTimer = 0;
			}
			break;
		case 3:   // P30
			if (!d.Idle() || fTimer < 2)
				break;
			d.SelectProgram(30);
			fState = 4;
			fTimer = 0;
			break;
		case 4:
			if (!d.Idle() || fTimer < 2)
				break;
			if (d.Flashing() && d.VerbNoun().substr(2) == "33") {
				d.Type(DSKYDriver::TimeEntry(fTIG) + "P");
				fState = 5;
				fTimer = 0;
			}
			break;
		case 5:
			if (!d.Idle() || fTimer < 1)
				break;
			if (d.Flashing() && d.VerbNoun().substr(2) == "81") {
				auto e = [](double ms) {
					return DSKYDriver::Signed(std::lround(ms / 0.3048 * 10)); };
				d.Type("V25E" + e(fDV.x) + "E" + e(fDV.y) + "E" + e(fDV.z) + "EP");
				fState = 6;
				fTimer = 0;
			}
			break;
		case 6:   // P30 integrates to TIG, then N42 (apogee, perigee, DV)
			if (!d.Idle() || fTimer < 1)
				break;
			if (d.Flashing() && d.VerbNoun().substr(2) == "42") {
				c.Report(fLabel + ": P30 " + d.Display());
				if (std::getenv("P30DIAG")) {
					Vec3 dvl;
					double tig = 0;
					f.CMCPad().ReadVector("DELVSLV", -7, &dvl);
					f.CMCPad().ReadDouble("TIG", -28, &tig);
					char b[160];
					std::snprintf(b, sizeof(b), "P30DIAG TIG %.2f (want %.2f) DELVSLV %.2f %.2f %.2f"
						" (want %.2f %.2f %.2f)", tig / 100, fTIG, dvl.x * 100, dvl.y * 100,
						dvl.z * 100, fDV.x, fDV.y, fDV.z);
					c.Report(b);
					bool mf = gnc::GetFlag(f.CMCPad(), gnc::FLAG_MOONFLAG);
					Vec3 rc;
					f.CMCPad().ReadVector("RRECTCSM", mf ? -27 : -29, &rc);
					Vec3 rm = f.CSMVehicle().position - f.Moon().MoonPosition();
					std::snprintf(b, sizeof(b), "P30DIAG moonflag %d |RRECT| %.0f km; truth"
						" from Moon %.0f km, from Earth %.0f km", mf, rc.Length() / 1000,
						rm.Length() / 1000, f.CSMVehicle().position.Length() / 1000);
					c.Report(b);
				}
				d.Type("P");
				fState = 7;
				fTimer = 0;
			} else if (fTimer > 120) {
				status = "P30 never showed N42 (" + d.Display() + ")";
				failed = true;
				return true;
			}
			break;
		case 7:
			// P30 shows the middle gimbal angle (N45) next; PRO through it
			// until P30 finishes (it sets XDELVFLG on the way to P00).
			if (!d.Idle() || fTimer < 1.5)
				break;
			if (d.Program() != "00"
					&& !(d.Flashing() && d.VerbNoun().substr(0, 2) == "37")) {
				if (c.verbose)
					c.Report("P30 end: " + d.Display());
				if (d.Flashing()) {
					d.Type("P");
					fTimer = 0;
				} else if (fTimer > 30) {
					status = "P30 didn't finish (" + d.Display() + ")";
					failed = true;
					return true;
				}
				break;
			}
			pg.panel.spsArmed = false;
			d.SelectProgram(40);
			fState = 8;
			fTimer = 0;
			break;
		case 8: {   // P40 through the burn
			if (!fArmed && get > fTIG - 120) {
				pg.panel.spsArmed = true;
				fArmed = true;
				c.Report(fLabel + ": delta-V thrust A/B on");
			}
			if (pg.SPSOn() && fIgnition < 0) {
				fIgnition = get;
				c.Report(fLabel + ": ignition");
			}
			if (!pg.SPSOn() && fIgnition >= 0 && fCutoff < 0) {
				fCutoff = get;
				char buf[80];
				std::snprintf(buf, sizeof(buf), ": cutoff after %.1f s",
					fCutoff - fIgnition);
				c.Report(fLabel + buf);
				pg.panel.spsArmed = false;
			}
			if (!d.Idle() || !d.Flashing() || get - fLastKey < 2
					|| d.StableFor() < 0.8)
				break;
			std::string vn = d.VerbNoun();
			std::string key;
			if (vn == "5018") {
				if (ManeuverError(f.CMCPad(), f.CMC()) < 2.0)
					key = "E";
				else if (get - fLastManeuver > 120) {
					key = "P";
					fLastManeuver = get;
				}
			} else if (vn == "5025" && d.Register(0) == "00204")
				key = "P";                 // gimbal drive test
			else if (vn.substr(0, 2) == "99")
				key = "P";                 // go for ignition
			else if (vn.substr(2) == "85" || vn.substr(2) == "40"
				|| vn.substr(0, 2) == "50" || vn == "0644")
				key = "P";
			else if (vn.substr(0, 2) == "37" && fCutoff > 0) {
				pg.panel.spsArmed = false;
				return true;
			}
			if (!key.empty()) {
				if (c.verbose) {
					char buf[160];
					std::snprintf(buf, sizeof(buf), "%s on %s (maneuver error %.1f)",
						key.c_str(), d.Display().c_str(),
						ManeuverError(f.CMCPad(), f.CMC()));
					c.Report(buf);
				}
				d.Type(key);
				fLastKey = get;
			}
			if (get > fTIG + 1200) {
				status = "no cutoff (" + d.Display() + ")";
				failed = true;
				pg.panel.spsArmed = false;
				return true;
			}
			break;
		}
	}
	return false;
}

}  // namespace


void
AutoCrew::ExternalDeltaV(FlightSim& f, double impulseGET, const Vec3& dv,
	double* tigOut, Vec3* lvlhOut, double thrust, double isp)
{
	Vehicle& v = f.CSMVehicle();
	double m = v.Mass();
	State s{v.position, v.velocity, f.JulianDate()};
	double jd0 = f.JulianDate() - f.GET() / 86400;
	double tigJD;
	Vec3 dvF = dv;
	if (!FiniteEquivalent(s, jd0 + impulseGET / 86400, dv, thrust, m, isp, &tigJD,
			&dvF)) {
		double ve = isp * 9.80665;
		double tb = m * ve / thrust * (1 - std::exp(-dv.Length() / ve));
		tigJD = jd0 + (impulseGET - tb / 2) / 86400;
		dvF = dv;
	}
	double tig = (tigJD - jd0) * 86400;
	State at = Propagate(s, tig - f.GET());
	bool moon = (at.r - MoonPos(at.jd)).Length() < world::kMoonSOI;
	Vec3 r = at.r - (moon ? MoonPos(at.jd) : Vec3());
	Vec3 vel = at.v - (moon ? MoonVel(at.jd) : Vec3());
	// P40's S40.1 turns an external delta-V forward in the orbit plane by
	// THETAT = THETACON |R x V| / R^2 * |DV| * WEIGHT/G / F (revolutions;
	// half the arc flown during the burn) before steering to it. Rotate the
	// PAD back by the same angle so the CMC lands on the burn we want.
	double omega = r.Cross(vel).Length() / r.Dot(r);
	double tbCMC = dvF.Length() * m / 91200.0;
	double theta = 0.5 * omega * tbCMC;
	Vec3 up = vel.Cross(r).Unit();
	Vec3 along = up * dvF.Dot(up);
	Vec3 inPlane = dvF - along;
	Vec3 pre = inPlane * std::cos(theta) + up.Cross(inPlane) * std::sin(theta) + along;
	*tigOut = tig;
	*lvlhOut = ToLVLH(r, vel, pre);
}


double
AutoCrew::PhaseAngle(FlightSim& f, double get)
{
	Vehicle& csm = f.CSMVehicle();
	State s{csm.position, csm.velocity, f.JulianDate()};
	State c = Propagate(s, get - f.GET());
	double jd = c.jd;
	Vec3 rc = c.r - MoonPos(jd), vc = c.v - MoonVel(jd);
	Vec3 rl;
	if (f.LMLanded()) {
		// The LM rides the Moon round: its position now, turned with the
		// Moon to `get`.
		Vec3 now = f.LMVehicle()->position - f.Moon().MoonPosition();
		const double moonRate = 2 * M_PI / (27.321661 * 86400);
		double turn = moonRate * (get - f.GET());
		Vec3 axis = f.Moon().FixedToInertial().Column(2);
		rl = now * std::cos(turn) + axis.Cross(now) * std::sin(turn)
			+ axis * axis.Dot(now) * (1 - std::cos(turn));
	} else {
		State l{f.LMVehicle()->position, f.LMVehicle()->velocity, f.JulianDate()};
		l = Propagate(l, get - f.GET());
		rl = l.r - MoonPos(l.jd);
	}
	Vec3 h = rc.Cross(vc).Unit();
	return std::atan2(h.Dot(rl.Cross(rc)), rl.Dot(rc)) / kDeg;
}


double
AutoCrew::LiftoffTime(FlightSim& f, double earliest, double phase)
{
	Vehicle& csm = f.CSMVehicle();
	State s{csm.position, csm.velocity, f.JulianDate()};
	s = Propagate(s, earliest - f.GET());
	Vec3 lm0 = f.LMVehicle()->position - f.Moon().MoonPosition();
	Vec3 axis = f.Moon().FixedToInertial().Column(2);
	const double moonRate = 2 * M_PI / (27.321661 * 86400);
	const double step = 10;
	double prev = 1e9;
	for (double t = earliest; t < earliest + 3 * 7200; t += step) {
		Vec3 rc = s.r - MoonPos(s.jd), vc = s.v - MoonVel(s.jd);
		double turn = moonRate * (t - f.GET());
		Vec3 rl = lm0 * std::cos(turn) + axis.Cross(lm0) * std::sin(turn)
			+ axis * axis.Dot(lm0) * (1 - std::cos(turn));
		Vec3 h = rc.Cross(vc).Unit();
		double a = std::atan2(h.Dot(rl.Cross(rc)), rl.Dot(rc)) / kDeg;
		if (prev < phase && a >= phase && a - prev < 10)
			return t - step * (a - phase) / (a - prev);
		prev = a;
		s = Propagate(s, step);
	}
	return earliest;
}


std::unique_ptr<Procedure>
AutoCrew::CSMBurn(double tig, const Vec3& dv, const std::string& label)
{
	return std::unique_ptr<Procedure>(new CSMBurnProcedure(tig, dv, label));
}


// ---------------------------------------------------------------------------
// Entry: P61 - P67


namespace {

class EntryProcedure : public Procedure {
public:
	EntryProcedure(double lat, double lon) : fLat(lat), fLon(lon) {}
	std::string Name() const override { return "Entry"; }
	double WakeGET(FlightSim& f) const override;
	bool Step(AutoCrew& c, FlightSim& f, double dt) override;

private:
	double fLat, fLon;
	mutable double fWake = -1;
	int fState = 0;
	gnc::StateVector fSV;
	double fTimer = 0;
	double fLastKey = -100, fLastManeuver = -1e9;
	bool fP61Loaded = false;
};


double
EntryProcedure::WakeGET(FlightSim& f) const
{
	// Forty minutes before the entry interface.
	if (fWake >= 0)
		return fWake;
	// Still far out: look again in ten minutes (not cached).
	if (f.CSMVehicle().position.Length() > world::kREarth + 2.0e7)
		return f.GET() + 600;
	fWake = f.GET();
	State s{f.CSMVehicle().position, f.CSMVehicle().velocity, f.JulianDate()};
	Interface ei = EntryInterface(s, 4 * 3600);
	if (ei.found)
		fWake = f.GET() + (ei.jd - s.jd) * 86400 - 2400;
	return fWake;
}


bool
EntryProcedure::Step(AutoCrew& c, FlightSim& f, double dt)
{
	DSKYDriver& d = c.cmc;
	fTimer += dt;
	double get = f.GET();
	gnc::CMPGNCS& pg = f.CMPGNCS();
	{
		static int lastState = -1;
		if (c.verbose && fState != lastState) {
			lastState = fState;
			char fr[64];
			std::snprintf(fr, sizeof(fr), " FAILREG %05o %05o %05o",
				f.CMCPad().Word("FAILREG", 0), f.CMCPad().Word("FAILREG", 1),
				f.CMCPad().Word("FAILREG", 2));
			c.Report("Entry state " + std::to_string(fState) + " " + d.Display()
				+ (d.Idle() ? " idle" : " typing") + fr);
		}
	}
	switch (fState) {
		case 0: {
			// Entry REFSMMAT (P52) and a state vector uplink.
			Vehicle& v = f.CSMVehicle();
			State s{v.position, v.velocity, f.JulianDate()};
			Interface ei = EntryInterface(s, 4 * 3600);
			if (!ei.found) {
				status = "no entry interface ahead";
				failed = true;
				return true;
			}
			Vec3 eiHat = ei.state.r.Unit();
			Vec3 h = ei.state.r.Cross(ei.state.v).Unit();
			Vec3 along = h.Cross(eiHat);
			Vec3 smX = along, smZ = -eiHat, smY = smZ.Cross(smX);
			Mat3 refsmmat = Mat3::Rows(smX, smY, smZ);
			gnc::PadLoader& pad = f.CMCPad();
			AlignCMC(f, refsmmat);
			fSV = UplinkCSMState(f);
			pad.Single("CSMMASS", v.Mass(), -16);
			pad.Single("LEMMASS", 0, -16);
			pad.Octal("DAPDATR1", 011102);
			pg.panel.scContCMC = true;
			pg.panel.mode = gnc::CMPanel::MODE_AUTO;
			c.Report("Entry: REFSMMAT and state vector uplinked");
			d.SelectProgram(0);
			fState = 1;
			fTimer = 0;
			break;
		}
		case 1:
			if (!d.Ready() || fTimer < 2)
				break;
			if (!UplinkTook(f, fSV)) {
				fSV = UplinkCSMState(f);
				c.Report("Entry: state vector uplink sent again");
				fTimer = 0;
				break;
			}
			d.Type("V48E");
			fState = 2;
			fTimer = 0;
			break;
		case 2:
			if (!d.Idle() || fTimer < 1)
				break;
			if (d.Flashing() && (d.VerbNoun().substr(2) == "46"
					|| d.VerbNoun().substr(2) == "47"
					|| d.VerbNoun().substr(2) == "48")) {
				d.Type("P");
				fTimer = 0;
				break;
			}
			if (fTimer > 2 && d.Ready()) {
				d.Type("V46E");
				fState = 3;
				fTimer = 0;
			}
			break;
		case 3:
			if (!d.Idle() || fTimer < 2)
				break;
			d.SelectProgram(61);
			fState = 4;
			fTimer = 0;
			break;
		case 4: {
			if (f.CurrentPhase() == Phase::Recovery || f.Lost())
				return true;
			if (!d.Idle() || !d.Flashing() || get - fLastKey < 2
					|| d.StableFor() < 0.8)
				break;
			std::string vn = d.VerbNoun();
			std::string key;
			if (vn == "0661" && !fP61Loaded) {
				// Splash latitude, longitude (XXX.XX deg), lift up (-1).
				key = "V25E" + DSKYDriver::Signed(std::lround(fLat * 100)) + "E"
					+ DSKYDriver::Signed(std::lround(fLon * 100)) + "E-00001E";
				fP61Loaded = true;
			} else if (vn == "5025" && d.Register(0) == "00041") {
				// CM/SM separation about fifteen minutes before entry.
				if (fWake >= 0 && get < fWake + 2400 - 900)
					break;
				if (f.SMAttached()) {
					f.SeparateCMFromSM();
					c.Report("CM/SM SEP");
				}
				key = "P";
			} else if (vn == "5018") {
				if (ManeuverError(f.CMCPad(), f.CMC()) < 3.0)
					key = "E";
				else if (get - fLastManeuver > 150) {
					key = "P";
					fLastManeuver = get;
				}
			} else if (vn.substr(0, 2) == "50" || vn.substr(0, 2) == "06"
				|| vn.substr(0, 2) == "16") {
				key = "P";
			}
			if (!key.empty()) {
				if (c.verbose)
					c.Report("Entry: " + key + " on " + d.Display());
				d.Type(key);
				fLastKey = get;
			}
			break;
		}
	}
	return false;
}

}  // namespace


std::unique_ptr<Procedure>
AutoCrew::Entry(double lat, double lon)
{
	return std::unique_ptr<Procedure>(new EntryProcedure(lat, lon));
}


// ---------------------------------------------------------------------------
// Landing: P63 - P65


namespace {

class LandingProcedure : public Procedure {
public:
	explicit LandingProcedure(double pdi) : fPDI(pdi) {}
	std::string Name() const override { return "Powered descent"; }
	double WakeGET(FlightSim&) const override { return fPDI - 900; }
	bool Step(AutoCrew& c, FlightSim& f, double dt) override;

private:
	void _Redesignate(AutoCrew& c, FlightSim& f, gnc::LMPGNCS& pg, Vehicle& lm);
	gnc::StateVector fLMSV, fCSMSV;
	Vec3 _PredictedSite(FlightSim& f, Vehicle& lm) const;

	double fPDI;
	int fState = 0;
	double fTimer = 0;
	double fLastKey = -100, fLastManeuver = -1e9;
	bool fArmed = false, fV57 = false;
	int fYaw = 0;
	// P64: the commander looks through the LPD and redesignates away from
	// craters and slopes.
	bool fLooked = false, fSpotChosen = false;
	double fLastROD = 0;     // GET of the last ROD switch click (P66)
	double fSpotLat = 0, fSpotLon = 0;
	double fClickTimer = 0;
	int fClickAxis = -1, fClickDir = 0;
	Vec3 fBeforeClick;
	int fSign[2] = {-1, 1};   // ACA forward brings the site short
	double fStep[2] = {60, 60};   // m per click, as last seen
	int fClicks = 0;
};


// How rough a spot is: the worst slope under the LM's footprint (deg), with
// boulders counted as steep.
double
SpotRoughness(const world::Terrain& t, double lat, double lon)
{
	const double m = 180 / (M_PI * 1737400.0);
	double coslat = std::max(0.05, std::cos(lat * M_PI / 180));
	double worst = 0;
	const double off[9][2] = {{0, 0}, {5, 0}, {-5, 0}, {0, 5}, {0, -5},
		{4, 4}, {-4, 4}, {4, -4}, {-4, -4}};
	for (const auto& o : off) {
		double la = lat + o[0] * m, lo = lon + o[1] * m / coslat;
		worst = std::max(worst, t.Slope(la, lo));
		if (t.BoulderHeight(la, lo) > 0.6)
			worst = std::max(worst, 20.0);
	}
	return worst;
}


// Where the LGC is taking us, in truth: our position plus the LGC's
// LAND - R (both in stable-member coordinates, B24 m).
Vec3
LandingProcedure::_PredictedSite(FlightSim& f, Vehicle& lm) const
{
	gnc::PadLoader& pad = *f.LGCPad();
	Vec3 r, land;
	pad.ReadVector("R", -24, &r);
	pad.ReadVector("LAND", -24, &land);
	Vec3 rows[3];
	double* p[9] = {&rows[0].x, &rows[0].y, &rows[0].z, &rows[1].x, &rows[1].y,
		&rows[1].z, &rows[2].x, &rows[2].y, &rows[2].z};
	for (int i = 0; i < 9; i++)
		pad.ReadDouble("REFSMMAT", -1, p[i], 2 * i);
	Mat3 refsmmat = Mat3::Rows(rows[0], rows[1], rows[2]);
	// R is from the last SERVICER pass (every 2 s), at PIPTIME.
	double pip = 0;
	pad.ReadDouble("PIPTIME", -28, &pip);
	double age = Clamp(f.GET() - pip / 100.0, 0.0, 4.0);
	if (std::getenv("LPDDIAG") && std::getenv("LPDDIAG")[0] == '2') {
		Vec3 rt = refsmmat * (lm.position - (lm.velocity - f.Moon().MoonVelocity()) * age - f.Moon().MoonPosition());
		std::printf("LPDDIAG %.2f pip %.2f age %.2f R-true %.0f %.0f %.0f LAND-R %.0f %.0f %.0f\n",
			f.GET(), pip / 100, age, r.x - rt.x, r.y - rt.y, r.z - rt.z,
			land.x - r.x, land.y - r.y, land.z - r.z);
	}
	return lm.position - (lm.velocity - f.Moon().MoonVelocity()) * age + refsmmat.Transposed() * (land - r);
}


void
LandingProcedure::_Redesignate(AutoCrew& c, FlightSim& f, gnc::LMPGNCS& pg,
	Vehicle& lm)
{
	const MoonScene& moon = f.Moon();
	const world::Terrain& terrain = f.Terrain();
	bool diag = c.verbose || std::getenv("LPDDIAG");
	Vec3 up = moon.LocalVertical(lm.position);
	// The LPD's two directions on the ground: elevation clicks move the
	// site along the LM's forward (Z) axis, azimuth clicks across it.
	Vec3 axes[2] = {lm.ToInertial(Vec3(0, 0, 1)), lm.ToInertial(Vec3(0, 1, 0))};
	for (Vec3& a : axes)
		a = (a - up * a.Dot(up)).Unit();

	// A click in progress: hold the controller over briefly, let go, and
	// wait for the next guidance pass to see where it put the site.
	if (fClickAxis >= 0) {
		if (fClickTimer > 0.3 && pg.panel.acaPulse) {
			pg.panel.aca = Vec3();
			pg.panel.acaPulse = false;
		}
		if (fClickTimer < 2.6)
			return;
		Vec3 now = _PredictedSite(f, lm) - moon.MoonPosition();
		double moved = (now - fBeforeClick).Dot(axes[fClickAxis]);
		if (std::fabs(moved) > 0.3 * fStep[fClickAxis]) {
			if ((moved > 0) != (fClickDir > 0))
				fSign[fClickAxis] = -fSign[fClickAxis];
			fStep[fClickAxis] = std::fabs(moved);
		}
		if (diag) {
			char buf[120];
			std::snprintf(buf, sizeof(buf), "LPD click %s %+d: site moved %.1f m"
				" (alt %.0f m)", fClickAxis == 0 ? "elevation" : "azimuth",
				fClickDir, moved, moon.Altitude(lm.position));
			c.Report(buf);
		}
		fClickAxis = -1;
		fClickTimer = 0;
	}
	double alt = moon.Altitude(lm.position);
	if (alt < 80 || alt > 2500 || fClickTimer < 1.0 || fClicks > 40)
		return;
	fClickTimer = 0;

	Vec3 site = _PredictedSite(f, lm);
	double lat, lon, rr;
	moon.LatLon(site, &lat, &lon, &rr);
	double rough = SpotRoughness(terrain, lat, lon);
	char buf[140];
	if (rough < 7) {
		if (!fLooked) {
			std::snprintf(buf, sizeof(buf), "LPD: target looks good (%.0f deg)", rough);
			c.Report(buf);
		}
		fLooked = true;
		return;
	}
	auto roughAt = [&](const Vec3& p) {
		double la, lo, r;
		moon.LatLon(p, &la, &lo, &r);
		return SpotRoughness(terrain, la, lo);
	};
	// Where could the clicks put us? Each click moves the site about a
	// step; pick the fewest clicks to smooth ground, going long rather than
	// short (less fuel than coming back).
	int bi = 0, bj = 0;
	double best = 1e9;
	for (int i = -4; i <= 4; i++)
		for (int j = -4; j <= 4; j++) {
			if (i == 0 && j == 0)
				continue;
			Vec3 p = site + axes[0] * (i * fStep[0]) + axes[1] * (j * fStep[1]);
			if (roughAt(p) > 5)
				continue;
			double cost = std::abs(i) * fStep[0] * (i < 0 ? 2.0 : 1.0)
				+ std::abs(j) * fStep[1];
			if (cost < best) {
				best = cost;
				bi = i;
				bj = j;
			}
		}
	if (best > 1e8) {
		// Nothing on the click grid: head for the nearest smooth patch.
		double reach = std::min(400.0, std::max(40.0, alt * 0.8));
		for (double dx = -reach; dx <= reach; dx += 6)
			for (double dy = -reach; dy <= reach; dy += 6) {
				Vec3 p = site + axes[0] * dx + axes[1] * dy;
				if (std::hypot(dx, dy) > reach || roughAt(p) > 5)
					continue;
				double cost = std::fabs(dx) * (dx < 0 ? 2.0 : 1.0) + std::fabs(dy);
				if (cost < best) {
					best = cost;
					bi = std::fabs(dx) > 0.5 * fStep[0] ? (dx > 0 ? 1 : -1) : 0;
					bj = std::fabs(dy) > 0.5 * fStep[1] ? (dy > 0 ? 1 : -1) : 0;
				}
			}
		if (best > 1e8 || (bi == 0 && bj == 0)) {
			if (!fLooked) {
				std::snprintf(buf, sizeof(buf), "LPD: %.0f deg slopes all round;"
					" taking what we've got", rough);
				c.Report(buf);
			}
			fLooked = true;
			return;
		}
	}
	if (!fLooked || diag) {
		std::snprintf(buf, sizeof(buf), "LPD: target is rough (%.0f deg);"
			" redesignating %+d long, %+d right", rough, bi, bj);
		c.Report(buf);
	}
	fLooked = true;
	// One click at a time, the bigger correction first.
	int axis = std::abs(bi) * fStep[0] >= std::abs(bj) * fStep[1] ? 0 : 1;
	fClickDir = axis == 0 ? (bi > 0 ? 1 : -1) : (bj > 0 ? 1 : -1);
	double deflect = fClickDir * fSign[axis];
	pg.panel.acaPulse = true;
	// Pitch clicks move the site in elevation, roll clicks in azimuth.
	pg.panel.aca = axis == 0 ? Vec3(0, deflect, 0) : Vec3(0, 0, deflect);
	fBeforeClick = site - moon.MoonPosition();
	fClickAxis = axis;
	fClickTimer = 0;
	fClicks++;
}


bool
LandingProcedure::Step(AutoCrew& c, FlightSim& f, double dt)
{
	DSKYDriver& d = c.lgc;
	fTimer += dt;
	double get = f.GET();
	gnc::LMPGNCS* pg = f.LMPGNCS();
	Vehicle* lm = f.LMVehicle();
	if (!pg || !lm || !f.LGC()) {
		status = "no LM";
		failed = true;
		return true;
	}
	if (f.LMLanded()) {
		// Post-landing: engine arm off, and the mode control to OFF so the
		// DAP doesn't fire jets against the ground for the whole stay.
		pg->panel.engineArm = gnc::LMPanel::ARM_OFF;
		pg->panel.mode = gnc::LMPanel::MODE_OFF;
		return true;
	}
	if (f.Lost()) {
		failed = true;
		status = f.LossReason();
		return true;
	}
	switch (fState) {
		case 0: {
			// MSFN tracking: fresh LM and CSM state vectors for the LGC.
			gnc::PadLoader& pad = *f.LGCPad();
			Vec3 mp = f.Moon().MoonPosition(), mv = f.Moon().MoonVelocity();
			gnc::StateVector lmSV, csmSV;
			lmSV.position = lm->position - mp;
			lmSV.velocity = lm->velocity - mv;
			lmSV.agcTime = get;
			csmSV.position = f.CSMVehicle().position - mp;
			csmSV.velocity = f.CSMVehicle().velocity - mv;
			csmSV.agcTime = get;
			gnc::LoadStateVector(pad, lmSV, true);
			gnc::LoadStateVector(pad, csmSV, false);
			fLMSV = lmSV;
			fCSMSV = csmSV;
			pad.Double("MASS", lm->Mass(), -16);
			pad.Single("LEMMASS", lm->Mass(), -16);
			// The crew's V48 after undocking: LM alone.
			pad.Octal("DAPBOOLS", pad.Word("DAPBOOLS") & ~010000);   // CSMDOCKD
			c.Report("State vectors uplinked to the LGC");
			if (std::getenv("PDIDIAG")) {
				State s{lm->position, lm->velocity, f.JulianDate()};
				Approach ap = ClosestMoonApproach(s, 3 * 3600, true);
				Vec3 rp = ap.state.r - MoonPos(ap.jd);
				Vec3 site = f.Moon().FixedToInertial() * world::LatLonToVector(
					f.Mission().site.latitude, f.Mission().site.longitude, 1.0);
				double tl = 0;
				pad.ReadDouble("TLAND", -28, &tl);
				char b[200];
				std::snprintf(b, sizeof(b), "PDIDIAG perilune %.0f m above site radius"
					" at GET %.0f (PDI planned %.0f), %.2f deg from site; TLAND %.0f",
					ap.radius - f.Mission().site.radius, f.GET() + (ap.jd - s.jd) * 86400,
					fPDI, std::acos(Clamp(rp.Unit().Dot(site), -1, 1)) / kDeg, tl / 100);
				c.Report(b);
			}
			d.SelectProgram(0);
			fState = 1;
			fTimer = 0;
			break;
		}
		case 1:
			if (!d.Ready() || fTimer < 2)
				break;
			if (!UplinkTook(*f.LGCPad(), fLMSV, true)
					|| !UplinkTook(*f.LGCPad(), fCSMSV, false)) {
				c.Report("LGC state vector uplink sent again");
				fState = 0;
				break;
			}
			d.Type("V48E");
			fState = 2;
			fTimer = 0;
			break;
		case 2:
			if (!d.Idle() || fTimer < 1)
				break;
			if (d.Flashing() && (d.VerbNoun().substr(2) == "46"
					|| d.VerbNoun().substr(2) == "47"
					|| d.VerbNoun().substr(2) == "48")) {
				d.Type("P");
				fTimer = 0;
				break;
			}
			if (fTimer > 2) {
				fState = 3;
				fTimer = 0;
			}
			break;
		case 3:
			if (!d.Idle() || fTimer < 1 || get < fPDI - 720)
				break;
			d.SelectProgram(63);
			c.Report("P63");
			fState = 4;
			fTimer = 0;
			break;
		case 4: {
			if (!fArmed && get > fPDI - 120) {
				pg->panel.engineArm = gnc::LMPanel::ARM_DESCENT;
				pg->panel.autoThrottle = true;
				fArmed = true;
				c.Report("Engine arm: descent");
			}
			// Yaw-around to windows up a few minutes into the burn, so the
			// landing radar sees the surface.
			Vec3 up = f.Moon().LocalVertical(lm->position);
			double xup = lm->ToInertial(Vec3(1, 0, 0)).Dot(up);
			double zup = lm->ToInertial(Vec3(0, 0, 1)).Dot(up)
				/ std::sqrt(std::max(1e-6, 1 - xup * xup));
			if (fYaw == 0 && pg->DescentEngineOn() && get > fPDI + 200 && zup < 0) {
				fYaw = 1;
				c.Report("Yaw-around to windows up");
			}
			if (fYaw == 1) {
				pg->panel.aca.x = 0.7;
				if (zup > 0.97) {
					pg->panel.aca.x = 0;
					fYaw = 2;
				}
			}
			if (!fV57 && pg->LRRangeGood() && pg->LRVelocityGood() && d.Idle()) {
				d.Type("V57E");
				fV57 = true;
				c.Report("Radar data good: V57");
			}
			if (std::getenv("DESCDIAG") && std::fmod(get, 10.0) < dt) {
				const MoonScene& ms = f.Moon();
				Vec3 fx = ms.ToFixed(lm->position);
				double la, lo, rr;
				world::VectorToLatLon(fx, &la, &lo, &rr);
				Vec3 rel = lm->velocity - ms.SurfaceVelocity(lm->position);
				double hdot = rel.Dot(up);
				double hc = 0, dh = 0;
				f.LGCPad()->ReadDouble("HCALC", -24, &hc);
				f.LGCPad()->ReadDouble("DELTAH", -24, &dh);
				if (std::fmod(get, 120.0) < dt)
					for (const Module& m : lm->Modules())
						for (size_t ti = 0; ti < m.tanks.size(); ti++)
							std::printf("TANK %s %zu %.1f\n", m.name.c_str(), ti, m.tanks[ti].mass);
				std::printf("DESCDIAG HCALC %.0f DELTAH %.0f merr %.1f rates %.2f %.2f %.2f ", hc, dh,
					ManeuverError(*f.LGCPad(), *f.LGC()), lm->rates.x / kDeg, lm->rates.y / kDeg, lm->rates.z / kDeg);
				std::printf("DESCDIAG %.0f P%s %s h %.0f hdot %.1f vh %.1f thr %.2f"
					" | %s R1 %s R2 %s R3 %s\n", get - fPDI, d.Program().c_str(),
					d.VerbNoun().c_str(), rr - f.Terrain().SurfaceRadius(la, lo), hdot,
					(rel - up * hdot).Length(), [&]() { const sim::Thruster* t = lm->GetThruster("DPS");
						return t ? t->level * t->throttle : -1.0; }() * (lm->Mass() > 0 ? 1 : 0),
					d.Program().c_str(), d.Register(0).c_str(), d.Register(1).c_str(),
					d.Register(2).c_str());
			}
			// Luminary 1D and later: V57 puts up N68 and waits for PRO
			// before the radar data are let in.
			if (fV57 && !gnc::GetFlag(*f.LGCPad(), gnc::FLAG_LRINH) && d.Idle()
					&& d.VerbNoun() == "0668" && get - fLastKey > 2) {
				d.Type("P");
				fLastKey = get;
			}
			// P66 (rate of descent): a click of the ROD switch every couple
			// of seconds to hold about 3 ft/s down.
			if (d.Program() == "66" && get - fLastROD > 2.0) {
				Vec3 rel = lm->velocity - f.Moon().SurfaceVelocity(lm->position);
				double sink = rel.Dot(up);
				if (sink > -0.6)
					pg->RateOfDescentClick(-1);
				else if (sink < -1.5)
					pg->RateOfDescentClick(1);
				fLastROD = get;
			}
			fClickTimer += dt;
			if (d.Program() == "64")
				_Redesignate(c, f, *pg, *lm);
			if (!d.Idle() || !d.Flashing() || get - fLastKey < 2
					|| d.StableFor() < 0.8)
				break;
			std::string vn = d.VerbNoun();
			std::string key;
			if (vn == "5025" && (d.Register(0) == "00014" || d.Register(0) == "00015"))
				key = "E";
			else if (vn == "5018") {
				if (ManeuverError(*f.LGCPad(), *f.LGC()) < 2.0)
					key = "E";
				else if (get - fLastManeuver > 60) {
					key = "P";
					fLastManeuver = get;
				}
			} else if (vn.substr(0, 2) == "50" || vn.substr(0, 2) == "99"
				|| vn == "0661" || vn == "0662" || vn == "0663" || vn == "0664")
				key = "P";
			if (!key.empty()) {
				d.Type(key);
				fLastKey = get;
			}
			if (get > fPDI + 1200) {
				char fr[48];
				std::snprintf(fr, sizeof(fr), " FAILREG %05o %05o %05o",
					f.LGCPad()->Word("FAILREG", 0), f.LGCPad()->Word("FAILREG", 1),
					f.LGCPad()->Word("FAILREG", 2));
				status = "still flying " + d.Display() + fr;
				failed = true;
				return true;
			}
			break;
		}
	}
	return false;
}

}  // namespace


std::unique_ptr<Procedure>
AutoCrew::Landing(double pdi)
{
	return std::unique_ptr<Procedure>(new LandingProcedure(pdi));
}


// ---------------------------------------------------------------------------
// Transposition and docking


namespace {

class DockingProcedure : public Procedure {
public:
	DockingProcedure(double start, bool transposition)
		: fStart(start), fTransposition(transposition) {}
	std::string Name() const override
		{ return fTransposition ? "Transposition and docking" : "Docking"; }
	double WakeGET(FlightSim&) const override { return fStart; }
	bool Step(AutoCrew& c, FlightSim& f, double dt) override;

private:
	// RHC/THC commands to point +X along `want` and to fly the probe to
	// the drogue at `closing` m/s.
	void _Point(FlightSim& f, const Vec3& want, double deadband);
	void _Translate(FlightSim& f, const FlightSim::DockingTarget& t,
		double closing, double standoff);

	double fStart;
	bool fTransposition;
	int fState = 0;
	double fTimer = 0;
};


void
DockingProcedure::_Point(FlightSim& f, const Vec3& want, double deadband)
{
	gnc::CMPGNCS& pg = f.CMPGNCS();
	Vehicle& v = f.CSMVehicle();
	Vec3 xd = v.ToBody(want.Unit());
	Vec3 axis = Vec3(1, 0, 0).Cross(xd);       // body rotation wanted
	double s = axis.Length();
	double angle = std::atan2(s, xd.x);
	Vec3 err = s > 1e-9 ? axis * (angle / s) : Vec3();
	auto cmd = [&](double e, double rate) {
		// Drive at the DAP's rate; let go as the error closes, early
		// enough for the rate to die out.
		double lead = rate * 6.0;
		if (e > deadband && !(rate > 0 && e < lead))
			return 1.0;
		if (e < -deadband && !(rate < 0 && -e < -lead))
			return -1.0;
		return 0.0;
	};
	pg.panel.rhc = Vec3(0, cmd(err.y, v.rates.y), cmd(err.z, v.rates.z));
}


void
DockingProcedure::_Translate(FlightSim& f, const FlightSim::DockingTarget& t,
	double closing, double standoff)
{
	// Fly the probe down the target's docking axis: null the offset from
	// the axis line, then close along it.
	gnc::CMPGNCS& pg = f.CMPGNCS();
	Vehicle& v = f.CSMVehicle();
	Vec3 a = t.axis.Unit();
	Vec3 d = t.probe - t.drogue;
	double along = d.Dot(a);
	Vec3 perp = d - a * along;
	double lateral = perp.Length();
	Vec3 want;
	if (along < standoff) {
		// Behind the drogue's plane, or too close in front of it to line
		// up: get out in front first, keeping clear of the LM sideways.
		bool lined = along > 0 && lateral < 0.15 + 0.03 * along;
		if (lined)
			want = a * -closing + perp * -0.06;
		else {
			want = a * 0.15;
			if (lateral < 6.0 && along < 0.5) {
				Vec3 out = lateral > 0.01 ? perp.Unit()
					: (std::fabs(a.x) < 0.9 ? Vec3(1, 0, 0) : Vec3(0, 1, 0)).Cross(a).Unit();
				want += out * 0.1;
			}
		}
	} else {
		want = perp * -0.06;
		if (want.Length() > 0.15)
			want = want.Unit() * 0.15;
		if (lateral < 0.15 + 0.03 * along)
			want += a * -closing;
	}
	Vec3 rel = v.velocity - t.drogueVelocity;
	Vec3 e = v.ToBody(want - rel);
	const double band = 0.01;
	auto thc = [&](double ev) { return ev > band ? 1 : ev < -band ? -1 : 0; };
	pg.panel.thc[0] = thc(e.x);
	pg.panel.thc[1] = thc(e.y);
	pg.panel.thc[2] = thc(e.z);
	static double lastDiag = 0;
	if (std::getenv("DOCKDIAG") && f.GET() - lastDiag > 20) {
		lastDiag = f.GET();
		std::printf("      dock: along %.2f lateral %.2f  rel %.3f %.3f %.3f  want %.3f %.3f %.3f\n",
			along, lateral, rel.x, rel.y, rel.z, want.x, want.y, want.z);
		if (f.LMVehicle()) {
			Vehicle& lm = *f.LMVehicle();
			std::printf("      LM rates %.3f %.3f %.3f deg/s, CSM X vs axis %.1f deg, mode %d\n",
				lm.rates.x / kDeg, lm.rates.y / kDeg, lm.rates.z / kDeg,
				std::acos(Clamp(v.ToInertial(Vec3(1, 0, 0)).Dot(-a), -1, 1)) / kDeg,
				(int)f.LMPGNCS()->panel.mode);
		}
	}
}


bool
DockingProcedure::Step(AutoCrew& c, FlightSim& f, double dt)
{
	DSKYDriver& d = c.cmc;
	gnc::CMPGNCS& pg = f.CMPGNCS();
	fTimer += dt;
	FlightSim::DockingTarget t;
	switch (fState) {
		case 0: {
			// DAP for the CSM alone, platform and state vector as they are.
			gnc::PadLoader& pad = f.CMCPad();
			double sm = 0;
			for (const Module& m : f.CSMVehicle().Modules())
				if (m.name == "Command Module" || m.name == "Service Module")
					sm += m.Mass();
			pad.Single("CSMMASS", sm, -16);
			pad.Single("LEMMASS", 0, -16);
			pad.Octal("DAPDATR1", 011103);   // CSM alone, 2 deg/s maneuvers
			// Platform to the present attitude: the turnaround is then a
			// pitch about the stable member's Y axis, far from gimbal lock.
			{
				Vehicle& v = f.CSMVehicle();
				AlignCMC(f, Mat3::Rows(v.ToInertial(Vec3(1, 0, 0)),
					v.ToInertial(Vec3(0, 1, 0)), v.ToInertial(Vec3(0, 0, 1))));
			}
			pg.panel.scContCMC = true;
			pg.panel.mode = gnc::CMPanel::MODE_AUTO;
			d.SelectProgram(0);
			fState = 1;
			fTimer = 0;
			break;
		}
		case 1:
			if (!d.Ready() || fTimer < 2)
				break;
			d.Type("V48E");
			fState = 2;
			fTimer = 0;
			break;
		case 2:
			if (!d.Idle() || fTimer < 1)
				break;
			if (d.Flashing() && d.StableFor() > 0.8
					&& (d.VerbNoun().substr(2) == "46"
						|| d.VerbNoun().substr(2) == "47"
						|| d.VerbNoun().substr(2) == "48")) {
				d.Type("P");
				fTimer = 0;
				break;
			}
			if (fTimer > 2 && d.Ready()) {
				d.Type("V46E");
				fState = 3;
				fTimer = 0;
			}
			break;
		case 3:
			if (!d.Idle() || fTimer < 3)
				break;
			if (!fTransposition) {
				fState = 5;
				fTimer = 0;
				break;
			}
			if (f.BoosterAttached()) {
				f.SeparateCSMFromBooster();
				pg.panel.sivbSeparated = true;
				c.Report("CSM/S-IVB separation");
			}
			pg.panel.thc[0] = 1;   // a couple of seconds of +X: about 0.3 m/s
			fState = 4;
			fTimer = 0;
			break;
		case 4:   // back away a few seconds of +X
			if (fTimer > 2) {
				pg.panel.thc[0] = 0;
				c.Report("Turning around");
				fState = 5;
				fTimer = 0;
			}
			break;
		case 5: {   // pitch around to face the LM
			if (!f.Docking(&t)) {
				status = "no docking target";
				failed = true;
				return true;
			}
			Vec3 los = t.drogue - t.probe;
			_Point(f, -t.axis, 1.5 * kDeg);
			Vec3 x = f.CSMVehicle().ToInertial(Vec3(1, 0, 0));
			(void)los;
			if (x.Dot(-t.axis.Unit()) > std::cos(3 * kDeg)
					&& f.CSMVehicle().rates.Length() < 0.2 * kDeg) {
				c.Report("Lined up on the LM");
				fState = 6;
				fTimer = 0;
			}
			if (fTimer > 900) {
				status = "never turned around";
				failed = true;
				pg.panel.rhc = Vec3();
				return true;
			}
			break;
		}
		case 6: {   // close on the drogue
			if (!f.Docking(&t)) {
				status = "lost the target";
				failed = true;
				return true;
			}
			double range = (t.drogue - t.probe).Length();
			_Point(f, -t.axis, 0.7 * kDeg);
			double closing = range > 10 ? 0.25 : range > 3 ? 0.12 : 0.08;
			_Translate(f, t, closing, 5.0);
			if (range < 0.33) {
				if (f.DockToLM()) {
					pg.panel.thc[0] = pg.panel.thc[1] = pg.panel.thc[2] = 0;
					pg.panel.rhc = Vec3();
					c.Report("Capture, hard dock");
					fState = 7;
					fTimer = 0;
				}
			}
			if (fTimer > 3600) {
				status = "no capture";
				failed = true;
				pg.panel.thc[0] = pg.panel.thc[1] = pg.panel.thc[2] = 0;
				pg.panel.rhc = Vec3();
				return true;
			}
			break;
		}
		case 7:   // extraction: back the LM out of the SLA
			if (!fTransposition)
				return true;
			pg.panel.thc[0] = fTimer < 4 ? -1 : 0;
			if (fTimer > 30) {
				pg.panel.thc[0] = 0;
				return true;
			}
			break;
	}
	return false;
}

}  // namespace


std::unique_ptr<Procedure>
AutoCrew::TransposeAndDock(double start, bool transposition)
{
	return std::unique_ptr<Procedure>(new DockingProcedure(start, transposition));
}


// ---------------------------------------------------------------------------
// Lunar ascent: P12


namespace {

class AscentProcedure : public Procedure {
public:
	explicit AscentProcedure(double tig) : fTIG(tig) {}
	std::string Name() const override { return "Lunar ascent"; }
	double WakeGET(FlightSim&) const override { return fTIG - 900; }
	bool Step(AutoCrew& c, FlightSim& f, double dt) override;

private:
	double fTIG;
	int fState = 0;
	double fTimer = 0, fLastKey = -100;
	bool fStaged = false, fLit = false;
};


bool
AscentProcedure::Step(AutoCrew& c, FlightSim& f, double dt)
{
	DSKYDriver& d = c.lgc;
	fTimer += dt;
	double get = f.GET();
	gnc::LMPGNCS* pg = f.LMPGNCS();
	if (!pg || !f.LMVehicle() || !f.LGC()) {
		status = "no LM";
		failed = true;
		return true;
	}
	if (std::getenv("ASCDIAG") && std::fmod(get, 10.0) < dt && get > fTIG - 20) {
		Vehicle* lm = f.LMVehicle();
		const MoonScene& ms = f.Moon();
		Vec3 up = ms.LocalVertical(lm->position);
		Vec3 rel = lm->velocity - ms.SurfaceVelocity(lm->position);
		Vec3 fx = ms.ToFixed(lm->position);
		double la, lo, rr;
		world::VectorToLatLon(fx, &la, &lo, &rr);
		const sim::Thruster* aps = lm->GetThruster("APS");
		Vec3 x = lm->ToInertial(Vec3(1, 0, 0));
		{
			gnc::PadLoader& pad = *f.LGCPad();
			Vec3 rn, vn, r0, r1, r2;
			pad.ReadVector("RN", -29, &rn);
			pad.ReadVector("VN", -7, &vn);
			pad.ReadVector("REFSMMAT", -1, &r0);
			double c[6];
			for (int i = 0; i < 6; i++) pad.ReadDouble("REFSMMAT", -1, &c[i], 6 + 2 * i);
			r1 = Vec3(c[0], c[1], c[2]); r2 = Vec3(c[3], c[4], c[5]);
			Vec3 tr = lm->position - ms.MoonPosition(), tv = lm->velocity - ms.MoonVelocity();
			Vec3 trs(r0.Dot(tr), r1.Dot(tr), r2.Dot(tr)), tvs(r0.Dot(tv), r1.Dot(tv), r2.Dot(tv));
			std::printf("ASCDIAG LGC dr %.0f m dv %.1f m/s |rn| %.0f |vn| %.1f true |v| %.1f\n",
				(rn - tr).Length(), (vn * 100.0 - tv).Length(), rn.Length(), vn.Length() * 100, tv.Length());
		}
		{
			const Module* asc = lm->FindModule("LM ascent stage");
			if (asc && asc->tanks.size() > 2)
				std::printf("ASCDIAG RCS A %.1f B %.1f APS %.1f feed %d\n", asc->tanks[1].mass,
					asc->tanks[2].mass, asc->tanks[0].mass, (int)pg->panel.ascentFeed);
		}
		std::printf("ASCDIAG %.0f h %.0f hdot %.1f vh %.1f aps %.2f pitchX-up %.1f mass %.0f | %s\n",
			get - fTIG, rr - f.Terrain().SurfaceRadius(la, lo), rel.Dot(up),
			(rel - up * rel.Dot(up)).Length(), aps ? aps->level : -1,
			std::acos(Clamp(x.Dot(up), -1.0, 1.0)) / kDeg, lm->Mass(), d.Display().c_str());
	}
	if (f.Lost()) {
		status = f.LossReason();
		failed = true;
		return true;
	}
	switch (fState) {
		case 0:
			d.SelectProgram(0);
			fState = 1;
			fTimer = 0;
			break;
		case 1:
			if (!d.Idle() || fTimer < 2)
				break;
			if (f.LMLanded()) {
				f.LMPGNCS()->panel.mode = gnc::LMPanel::MODE_AUTO;
				PrepareLGCForAscent(f);
				c.Report("P57 realign; surface V48; CSM state vector uplinked");
			}
			d.SelectProgram(12);
			fState = 2;
			fTimer = 0;
			break;
		case 2: {
			bool aps = pg->AscentEngineOn();
			if (!fLit && aps) {
				fLit = true;
				c.Report("Ascent engine on");
				if (std::getenv("PHASEDIAG")) {
					char b[80];
					std::snprintf(b, sizeof(b), "phase: CSM leads by %.1f deg",
						AutoCrew::PhaseAngle(f, get));
					c.Report(b);
				}
			}
			if (fLit && !aps && get > fTIG + 60) {
				c.Report("Insertion");
				pg->panel.engineArm = gnc::LMPanel::ARM_OFF;
				pg->panel.ascentFeed = false;
				fState = 3;
				fTimer = 0;
				break;
			}
			// "Abort stage, engine arm ascent, proceed."
			if (!fStaged && get > fTIG - 8) {
				f.StageLM();
				pg->panel.engineArm = gnc::LMPanel::ARM_ASCENT;
				pg->panel.ascentFeed = true;     // ASC FEED A and B open
				fStaged = true;
				c.Report("Abort stage; engine arm ascent");
			}
			if (!d.Idle() || !d.Flashing() || get - fLastKey < 2
					|| d.StableFor() < 0.8)
				break;
			std::string vn = d.VerbNoun();
			std::string key;
			if (vn.substr(2) == "33" && fTimer < 120)
				key = DSKYDriver::TimeEntry(fTIG) + "P";
			else if (vn == "5018") {
				if (ManeuverError(*f.LGCPad(), *f.LGC()) < 2.0)
					key = "E";
				else
					key = "P";
			} else if (vn.substr(0, 2) == "99") {
				if (get > fTIG - 6)
					key = "P";
			} else if (vn.substr(0, 2) == "06" || vn.substr(0, 2) == "16"
				|| vn.substr(0, 2) == "50")
				key = "P";
			if (!key.empty()) {
				if (c.verbose)
					c.Report("Ascent: " + key + " on " + d.Display());
				d.Type(key);
				fLastKey = get;
			}
			if (get > fTIG + 1200) {
				status = "no insertion " + d.Display();
				failed = true;
				return true;
			}
			break;
		}
		case 3:   // residuals (N85): PRO through, then P00 and the DAP
			if (!d.Idle() || fTimer < 3)
				break;
			if (d.Flashing() && d.StableFor() > 0.8 && d.VerbNoun().substr(0, 2) != "37") {
				d.Type("P");
				fTimer = 0;
				break;
			}
			if (fTimer > 4) {
				d.SelectProgram(0);
				fState = 4;
				fTimer = 0;
			}
			break;
		case 4:
			if (!d.Idle() || fTimer < 3)
				break;
			d.Type("V48E");
			fState = 5;
			fTimer = 0;
			break;
		case 5:
			if (!d.Idle() || fTimer < 1)
				break;
			if (d.Flashing() && d.StableFor() > 0.8
					&& (d.VerbNoun().substr(2) == "46" || d.VerbNoun().substr(2) == "47"
						|| d.VerbNoun().substr(2) == "48")) {
				d.Type("P");
				fTimer = 0;
				break;
			}
			if (fTimer > 3)
				return true;
			break;
	}
	return false;
}

}  // namespace


std::unique_ptr<Procedure>
AutoCrew::Ascent(double tig)
{
	return std::unique_ptr<Procedure>(new AscentProcedure(tig));
}


// ---------------------------------------------------------------------------
// Rendezvous (LM active)


namespace {

class RendezvousProcedure : public Procedure {
public:
	std::string Name() const override { return "Rendezvous"; }
	bool Step(AutoCrew& c, FlightSim& f, double dt) override;
	double WakeGET(FlightSim& f) const override;

private:
	bool _Plan(FlightSim& f);
	bool _Burn(FlightSim& f, double dt);       // null fDVLeft with the THC
	// A burn still going after five minutes: report why and move on.
	bool _BurnStalled(AutoCrew& c, FlightSim& f);
	bool _Lambert(FlightSim& f, Vec3* dv);
	void _PointLM(FlightSim& f, const Vec3& want);

	int fState = 0;
	double fTPI = 0, fTPF = 0;
	Vec3 fDVLeft;
	int fMidcourse = 0;
	double fTimer = 0;
};


double
RendezvousProcedure::WakeGET(FlightSim& f) const
{
	if (fState >= 1 && fState <= 2 && fTPI > 0)
		return fTPI - 120;
	return f.GET();
}


// Moon-relative states now and at a later time.
static void
Relative(FlightSim& f, const Vehicle& v, double get, Vec3* r, Vec3* vel)
{
	State s{v.position, v.velocity, f.JulianDate()};
	State at = get > f.GET() + 0.01 ? Propagate(s, get - f.GET()) : s;
	*r = at.r - MoonPos(at.jd);
	*vel = at.v - MoonVel(at.jd);
}


bool
RendezvousProcedure::_Plan(FlightSim& f)
{
	// TPI: the CSM 26.6 degrees above the LM's local horizontal, ahead.
	Vehicle& lm = *f.LMVehicle();
	Vehicle& csm = f.CSMVehicle();
	double best = -1, prevE = 0;
	for (double t = f.GET() + 300; t < f.GET() + 4 * 7200; t += 20) {
		Vec3 rl, vl, rc, vc;
		Relative(f, lm, t, &rl, &vl);
		Relative(f, csm, t, &rc, &vc);
		Vec3 los = (rc - rl).Unit();
		Vec3 up = rl.Unit();
		double e = std::asin(Clamp(los.Dot(up), -1, 1)) / kDeg;
		bool ahead = los.Dot(vl) > 0;
		if (ahead && t > f.GET() + 320 && prevE < 26.6 && e >= 26.6) {
			best = t;
			break;
		}
		prevE = ahead ? e : -90;
	}
	if (best < 0)
		return false;
	fTPI = best;
	Vec3 rc, vc;
	Relative(f, csm, fTPI, &rc, &vc);
	double period = 2 * M_PI * std::sqrt(std::pow(rc.Length(), 3) / world::kMuMoon);
	fTPF = fTPI + period * 130.0 / 360.0;
	return true;
}


bool
RendezvousProcedure::_Lambert(FlightSim& f, Vec3* dv)
{
	Vehicle& lm = *f.LMVehicle();
	Vec3 rl, vl, rc, vc;
	Relative(f, lm, f.GET(), &rl, &vl);
	Relative(f, f.CSMVehicle(), fTPF, &rc, &vc);
	Vec3 v1;
	if (!Lambert(rl, rc, fTPF - f.GET(), world::kMuMoon, rl.Cross(vl), &v1))
		return false;
	*dv = v1 - vl;
	return true;
}


bool
RendezvousProcedure::_Burn(FlightSim& f, double)
{
	// Translation burn on the RCS: fire each body axis until its share of
	// the delta-V is in (attitude held by the DAP).
	gnc::LMPGNCS* pg = f.LMPGNCS();
	Vehicle& lm = *f.LMVehicle();
	fDVLeft -= lm.SensedDeltaV();
	Vec3 b = lm.ToBody(fDVLeft);
	const double band = 0.03;
	pg->panel.thc[0] = b.x > band ? 1 : b.x < -band ? -1 : 0;
	pg->panel.thc[1] = b.y > band ? 1 : b.y < -band ? -1 : 0;
	pg->panel.thc[2] = b.z > band ? 1 : b.z < -band ? -1 : 0;
	static double lastDiag = 0;
	if (std::getenv("RNDZDIAG") && f.GET() - lastDiag > 5) {
		lastDiag = f.GET();
		std::printf("      burn: P%c%c jets %03o %03o rates %.2f %.2f %.2f left %.2f %.2f %.2f thc %d %d %d sensed %.4f\n", f.LGCDSKY().State().prog[0], f.LGCDSKY().State().prog[1], pg->JetBits(5), pg->JetBits(6), lm.rates.x / kDeg, lm.rates.y / kDeg, lm.rates.z / kDeg, b.x, b.y, b.z,
			pg->panel.thc[0], pg->panel.thc[1], pg->panel.thc[2], lm.SensedDeltaV().Length());
	}
	return pg->panel.thc[0] == 0 && pg->panel.thc[1] == 0 && pg->panel.thc[2] == 0;
}


bool
RendezvousProcedure::_BurnStalled(AutoCrew& c, FlightSim& f)
{
	if (fTimer < 300)
		return false;
	gnc::LMPGNCS* pg = f.LMPGNCS();
	const Module* asc = f.LMVehicle()->FindModule("LM ascent stage");
	char buf[200];
	std::snprintf(buf, sizeof(buf), "RCS burn stalled with %.1f m/s to go "
		"(RCS A %.0f kg, B %.0f kg, APS %.0f kg, ASC FEED %s, jets %03o %03o)",
		fDVLeft.Length(), asc && asc->tanks.size() > 2 ? asc->tanks[1].mass : -1.0,
		asc && asc->tanks.size() > 2 ? asc->tanks[2].mass : -1.0,
		asc ? asc->tanks[0].mass : -1.0, pg->panel.ascentFeed ? "open" : "closed",
		pg->JetBits(5), pg->JetBits(6));
	c.Report(buf);
	pg->panel.thc[0] = pg->panel.thc[1] = pg->panel.thc[2] = 0;
	fTimer = 0;
	return true;
}


void
RendezvousProcedure::_PointLM(FlightSim& f, const Vec3& want)
{
	// LM +X (the docking hatch) toward the CSM: pitch and roll with the
	// ACA, letting go early enough for the DAP to stop the rate.
	gnc::LMPGNCS* pg = f.LMPGNCS();
	Vehicle& lm = *f.LMVehicle();
	Vec3 xd = lm.ToBody(want.Unit());
	Vec3 axis = Vec3(1, 0, 0).Cross(xd);
	double s = axis.Length();
	double angle = std::atan2(s, xd.x);
	Vec3 err = s > 1e-9 ? axis * (angle / s) : Vec3();
	auto cmd = [&](double e, double rate) {
		double lead = rate * 3.0;
		if (e > 3 * kDeg && !(rate > 0 && e < lead))
			return 0.2;
		if (e < -3 * kDeg && !(rate < 0 && -e < -lead))
			return -0.2;
		return 0.0;
	};
	pg->panel.aca = Vec3(0, cmd(err.y, lm.rates.y), cmd(err.z, lm.rates.z));
}


bool
RendezvousProcedure::Step(AutoCrew& c, FlightSim& f, double dt)
{
	gnc::LMPGNCS* pg = f.LMPGNCS();
	if (!pg || !f.LMVehicle() || f.Lost()) {
		status = f.Lost() ? f.LossReason() : "no LM";
		failed = true;
		return true;
	}
	Vehicle& lm = *f.LMVehicle();
	Vehicle& csm = f.CSMVehicle();
	fTimer += dt;
	double get = f.GET();
	switch (fState) {
		case 0:
			pg->panel.mode = gnc::LMPanel::MODE_AUTO;
			if (!_Plan(f)) {
				status = "no TPI opportunity";
				failed = true;
				return true;
			}
			{
				char buf[80];
				std::snprintf(buf, sizeof(buf), "TPI at %.0f, intercept at %.0f",
					fTPI, fTPF);
				c.Report(buf);
			}
			fState = 1;
			break;
		case 1:   // wait for TPI
			if (get < fTPI)
				break;
			if (!_Lambert(f, &fDVLeft)) {
				status = "TPI Lambert failed";
				failed = true;
				return true;
			}
			{
				char buf[64];
				std::snprintf(buf, sizeof(buf), "TPI: %.1f m/s", fDVLeft.Length());
				c.Report(buf);
			}
			fState = 2;
			fTimer = 0;
			break;
		case 2:
			if (_Burn(f, dt) || _BurnStalled(c, f)) {
				fState = 3;
				fTimer = 0;
			}
			break;
		case 3: {   // coast, midcourses at +12 and +24 minutes
			double since = get - fTPI;
			if ((fMidcourse == 0 && since > 720) || (fMidcourse == 1 && since > 1440)) {
				fMidcourse++;
				if (_Lambert(f, &fDVLeft)) {
					char buf[64];
					std::snprintf(buf, sizeof(buf), "Midcourse %d: %.2f m/s", fMidcourse,
						fDVLeft.Length());
					c.Report(buf);
					fState = 4;
					fTimer = 0;
				}
			}
			double range = (csm.position - lm.position).Length();
			if (range < 1500 || get > fTPF - 240) {
				c.Report("Braking");
				fState = 5;
			}
			break;
		}
		case 4:
			if (_Burn(f, dt) || _BurnStalled(c, f)) {
				fState = 3;
				fTimer = 0;
			}
			break;
		case 5: {   // braking down the line of sight to station keeping
			Vec3 r = csm.position - lm.position;
			Vec3 v = csm.velocity - lm.velocity;
			double range = r.Length();
			Vec3 u = r / range;
			double closing = -v.Dot(u);
			double want = Clamp(range * 0.004, 0.1, 9.0);
			if (range < 40)
				want = 0;
			// Desired relative velocity of the CSM as seen from the LM:
			// coming in along the line of sight, nothing across it.
			Vec3 desired = u * -want;
			Vec3 needed = v - desired;     // change in LM velocity
			Vec3 b = lm.ToBody(needed);
			const double band = range < 200 ? 0.03 : 0.15;
			pg->panel.thc[0] = b.x > band ? 1 : b.x < -band ? -1 : 0;
			pg->panel.thc[1] = b.y > band ? 1 : b.y < -band ? -1 : 0;
			pg->panel.thc[2] = b.z > band ? 1 : b.z < -band ? -1 : 0;
			if (range < 300)
				_PointLM(f, r);
			if (range < 45 && v.Length() < 0.06) {
				pg->panel.thc[0] = pg->panel.thc[1] = pg->panel.thc[2] = 0;
				pg->panel.aca = Vec3();
				char buf[64];
				std::snprintf(buf, sizeof(buf), "Station keeping at %.0f m", range);
				c.Report(buf);
				return true;
			}
			if (std::getenv("RNDZDIAG") && std::fmod(get, 60.0) < dt) {
				double rcs = 0;
				for (const Module& m : lm.Modules())
					for (size_t i = 0; i < m.tanks.size(); i++)
						if (m.name == "LM ascent stage" && i > 0)
							rcs += m.tanks[i].mass;
				std::printf("      braking: range %.0f m closing %.2f |v| %.2f m/s"
					" need %.2f %.2f %.2f thc %d %d %d RCS %.0f kg rates %.2f %.2f %.2f aca %.1f %.1f %.1f mode %d P%s\n", range, closing,
					v.Length(), b.x, b.y, b.z, pg->panel.thc[0], pg->panel.thc[1],
					pg->panel.thc[2], rcs, lm.rates.x / kDeg, lm.rates.y / kDeg, lm.rates.z / kDeg,
					pg->panel.aca.x, pg->panel.aca.y, pg->panel.aca.z, (int)pg->panel.mode,
					c.lgc.Program().c_str());
				double mass = 0;
				f.LGCPad()->ReadDouble("MASS", -16, &mass);
				const Module* asc = lm.FindModule("LM ascent stage");
				std::printf("        DAPBOOLS %05o MASS %.0f (truth %.0f) jets %03o %03o RCS A %.1f B %.1f\n",
					f.LGCPad()->Word("DAPBOOLS"), mass, lm.Mass(), pg->JetBits(5), pg->JetBits(6),
					asc && asc->tanks.size() > 2 ? asc->tanks[1].mass : -1,
					asc && asc->tanks.size() > 2 ? asc->tanks[2].mass : -1);
			}
			if (get > fTPF + 3600) {
				status = "never closed";
				failed = true;
				pg->panel.thc[0] = pg->panel.thc[1] = pg->panel.thc[2] = 0;
				return true;
			}
			break;
		}
	}
	return false;
}

}  // namespace


std::unique_ptr<Procedure>
AutoCrew::Rendezvous()
{
	return std::unique_ptr<Procedure>(new RendezvousProcedure());
}


// ---------------------------------------------------------------------------
// CMC DAP configuration


std::unique_ptr<Procedure>
AutoCrew::ConfigureCMCDAP()
{
	auto state = std::make_shared<int>(0);
	auto timer = std::make_shared<double>(0);
	return Do("DAP configuration", [state, timer](AutoCrew& c, FlightSim& f, double dt) {
		DSKYDriver& d = c.cmc;
		*timer += dt;
		switch (*state) {
			case 0: {
				Vehicle& v = f.CSMVehicle();
				double lm = 0;
				for (const Module& m : v.Modules())
					if (m.name == "LM descent stage" || m.name == "LM ascent stage")
						lm += m.Mass();
				gnc::PadLoader& pad = f.CMCPad();
				pad.Single("CSMMASS", v.Mass() - lm, -16);
				pad.Single("LEMMASS", lm, -16);
				pad.Octal("DAPDATR1", lm > 0 ? 021102 : 011102);
				d.SelectProgram(0);
				*state = 1;
				*timer = 0;
				return false;
			}
			case 1:
				if (!d.Ready() || *timer < 2)
					return false;
				d.Type("V48E");
				*state = 2;
				*timer = 0;
				return false;
			case 2:
				if (!d.Idle() || *timer < 1)
					return false;
				if (d.Flashing() && d.StableFor() > 0.8
						&& (d.VerbNoun().substr(2) == "46" || d.VerbNoun().substr(2) == "47"
							|| d.VerbNoun().substr(2) == "48")) {
					d.Type("P");
					*timer = 0;
					return false;
				}
				if (*timer > 2 && d.Ready()) {
					d.Type("V46E");
					*state = 3;
					*timer = 0;
				}
				return false;
			default:
				return d.Idle() && *timer > 2;
		}
	});
}

}  // namespace flight
