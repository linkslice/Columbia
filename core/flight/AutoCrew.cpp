// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "AutoCrew.h"

#include "FlightSim.h"
#include "Trajectory.h"
#include "../gnc/PadLoads.h"

#include <cmath>
#include <cstdio>

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
		if (fQueue.empty())
			sim.lockstepFrom = 1e30;
		else if (fQueue.front()->started)
			sim.lockstepFrom = sim.GET();
	});
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
			Vec3 org = moon ? f.Moon().MoonPosition() : Vec3();
			Vec3 orgV = moon ? f.Moon().MoonVelocity() : Vec3();
			Vec3 r = v.position - org, vel = v.velocity - orgV;
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
			gnc::StateVector sv;
			sv.position = r;
			sv.velocity = vel;
			sv.agcTime = get;
			sv.moon = moon;
			gnc::LoadStateVector(pad, sv, false);
			gnc::SetFlag(pad, gnc::FLAG_MOONFLAG, moon);
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
			pg.imu.Align(refsmmat);
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
			d.Type("V37E00E");
			fState = 1;
			fTimer = 0;
			break;
		}
		case 1:   // V48: DAP data load
			if (!d.Idle() || fTimer < 2)
				break;
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
			if (fTimer > 2) {
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
	fWake = f.GET();
	State s{f.CSMVehicle().position, f.CSMVehicle().velocity, f.JulianDate()};
	if (f.CSMVehicle().position.Length() < world::kREarth + 2.0e7) {
		Interface ei = EntryInterface(s, 4 * 3600);
		if (ei.found)
			fWake = f.GET() + (ei.jd - s.jd) * 86400 - 2400;
		return fWake;
	}
	fWake = f.GET() + 600;
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
			pg.imu.Align(refsmmat);
			gnc::StateVector sv;
			sv.position = v.position;
			sv.velocity = v.velocity;
			sv.agcTime = get;
			sv.moon = false;
			gnc::LoadStateVector(pad, sv, false);
			gnc::SetFlag(pad, gnc::FLAG_MOONFLAG, false);
			pad.Single("CSMMASS", v.Mass(), -16);
			pad.Single("LEMMASS", 0, -16);
			pad.Octal("DAPDATR1", 011102);
			pg.panel.scContCMC = true;
			pg.panel.mode = gnc::CMPanel::MODE_AUTO;
			c.Report("Entry: REFSMMAT and state vector uplinked");
			d.Type("V37E00E");
			fState = 1;
			fTimer = 0;
			break;
		}
		case 1:
			if (!d.Idle() || fTimer < 2)
				break;
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
	double fPDI;
	int fState = 0;
	double fTimer = 0;
	double fLastKey = -100, fLastManeuver = -1e9;
	bool fArmed = false, fV57 = false;
	int fYaw = 0;
};


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
		pg->panel.engineArm = gnc::LMPanel::ARM_OFF;
		return true;
	}
	if (f.Lost()) {
		failed = true;
		status = f.LossReason();
		return true;
	}
	switch (fState) {
		case 0:
			d.Type("V37E00E");
			fState = 1;
			fTimer = 0;
			break;
		case 1:
			if (!d.Idle() || fTimer < 2)
				break;
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
				status = "still flying " + d.Display();
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

}  // namespace flight
