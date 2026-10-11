// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Saturn.h"

#include "FlightSim.h"
#include "../sim/World.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace flight {

using namespace sim;

namespace {

const double kG0 = 9.80665;
const double kDay = 86400.0;

// Tilt program: pitch from the vertical (deg) against time from liftoff,
// close to the Saturn V's polynomial (the IB's is stretched to its
// shorter first-stage burn).
struct TiltPoint { double t, deg; };
const TiltPoint kTiltV[] = {
	{0, 0}, {13, 0}, {25, 3.5}, {40, 9.5}, {55, 17}, {70, 25}, {85, 33},
	{100, 40.5}, {115, 47.5}, {130, 54}, {145, 60}, {160, 65}, {175, 68}
};
const TiltPoint kTiltIB[] = {
	{0, 0}, {10, 0}, {25, 4}, {40, 10}, {55, 17}, {70, 25}, {85, 33},
	{100, 41}, {115, 48}, {130, 54}, {145, 59}
};

template <size_t N>
double
Tilt(const TiltPoint (&table)[N], double t)
{
	if (t <= table[0].t)
		return table[0].deg;
	for (size_t i = 1; i < N; i++)
		if (t < table[i].t) {
			double u = (t - table[i - 1].t) / (table[i].t - table[i - 1].t);
			return table[i - 1].deg + u * (table[i].deg - table[i - 1].deg);
		}
	return table[N - 1].deg;
}

bool
IsMain(const Thruster& t)
{
	return t.kind == THRUSTER_MAIN;
}

}  // namespace


SaturnIU::SaturnIU(FlightSim& sim)
	:
	f(sim)
{
	fSaturnV = f.Mission().saturnV;
	if (!fSaturnV) {
		// Apollo 7: 123 x 153 nmi (227 x 283 km).
		targetPerigee = 227000;
		targetApogee = 283000;
	}
	fPlane = f.ParkingOrbitNormal();
	Vehicle& v = f.CSMVehicle();
	if (f.OnPad())
		fMode = Mode::Prelaunch;
	else if (v.FindModule("S-IVB"))
		fMode = Mode::Orbit;
	else
		fMode = Mode::Coast;
	if (fMode == Mode::Orbit)
		fHoldDir = v.velocity.Unit();
}


const char*
SaturnIU::ModeName() const
{
	switch (fMode) {
		case Mode::Prelaunch: return "Prelaunch";
		case Mode::FirstStage: return fSaturnV ? "S-IC" : "S-IB";
		case Mode::SecondStage: return "S-II";
		case Mode::ThirdStage: return "S-IVB";
		case Mode::Orbit: return "Orbit";
		case Mode::TLIPrep: return "TB6";
		case Mode::TLI: return "TLI";
		case Mode::Coast: return "Coast";
		case Mode::Safed: return "Safed";
	}
	return "";
}


Vehicle*
SaturnIU::Carrier()
{
	if (f.BoosterAttached())
		return &f.CSMVehicle();
	Vehicle* b = f.BoosterVehicle();
	if (b && b->FindModule("S-IVB"))
		return b;
	return nullptr;
}


// ---------------------------------------------------------------------------
// Engines


void
SaturnIU::_Ignite(Vehicle& v, const std::string& module)
{
	Module* m = v.FindModule(module);
	if (!m)
		return;
	for (Thruster& t : m->thrusters)
		if (IsMain(t) && !t.failedOff)
			t.on = true;
}


void
SaturnIU::_Cutoff(Vehicle& v, const std::string& module)
{
	Module* m = v.FindModule(module);
	if (!m)
		return;
	for (Thruster& t : m->thrusters)
		if (IsMain(t))
			t.on = false;
}


bool
SaturnIU::_StageDepleted(Vehicle& v, const std::string& module) const
{
	const Module* m = v.FindModule(module);
	return m == nullptr || m->tanks.empty() || m->tanks[0].mass <= 1.0;
}


double
SaturnIU::_StageThrust(const Vehicle& v, const std::string& module) const
{
	const Module* m = v.FindModule(module);
	double sum = 0;
	if (m)
		for (const Thruster& t : m->thrusters)
			if (IsMain(t) && t.on && !t.failedOff)
				sum += t.maxThrust * t.health;
	return sum;
}


bool
SaturnIU::FailEngine(const std::string& name, bool logIt)
{
	for (Vehicle* v : {&f.CSMVehicle(), f.BoosterVehicle()}) {
		if (!v)
			continue;
		Thruster* t = v->GetThruster(name);
		if (t) {
			t->failedOff = true;
			t->on = false;
			if (logIt)
				f.Log(LogEntry::Caution, name + " shut down");
			return true;
		}
	}
	return false;
}


// ---------------------------------------------------------------------------
// Attitude control: pitch and yaw through the gimballed engines, roll and
// coasting attitude with the auxiliary propulsion (and the outboard
// engines' differential gimbal, lumped as a roll torque).


void
SaturnIU::_Steer(Vehicle& v, const Vec3& dirInertial, const Vec3& downInertial,
	double maxRate, double dt)
{
	if (guidanceFailed)
		return;
	if (dt > 0.5) {
		// Long coast steps: the APS holds the attitude.
		Vec3 x = dirInertial.Unit();
		Vec3 z = (downInertial - x * downInertial.Dot(x)).Unit();
		v.attitude = Quat::FromMatrix(Mat3::Columns(x, z.Cross(x), z));
		v.rates = Vec3();
		return;
	}
	Vec3 xd = v.ToBody(dirInertial.Unit());
	Vec3 e = Vec3(1, 0, 0).Cross(xd);
	double s = e.Length();
	double angle = std::atan2(s, xd.x);
	Vec3 err = s > 1e-9 ? e * (angle / s) : Vec3();
	Vec3 zd = v.ToBody(downInertial);
	zd = Vec3(0, zd.y, zd.z);
	if (zd.Length() > 1e-6 && angle < 30 * kDeg) {
		zd = zd.Unit();
		err.x = std::atan2(-zd.y, zd.z);
	}
	// Rate command, then angular acceleration to reach it.
	const double kAtt = 0.6, kRate = 1.5;
	Vec3 rateCmd = err * kAtt;
	double rc = rateCmd.Length();
	if (rc > maxRate)
		rateCmd = rateCmd * (maxRate / rc);
	Vec3 alpha = (rateCmd - v.rates) * kRate;
	Vec3 torque = v.Inertia() * alpha;

	// Engine gimbal effectiveness and the untrimmed torque.
	Vec3 cg = v.CenterOfMass();
	Vec3 bp, by, t0;
	std::vector<Thruster*> gimballed;
	for (Module& m : v.Modules()) {
		for (Thruster& t : m.thrusters) {
			if (!IsMain(t) || t.level <= 0 || t.failedOff)
				continue;
			Vec3 pos = m.offset + m.rotation * t.position;
			Vec3 dir = m.rotation * t.direction;
			double F = t.maxThrust * t.level * t.health;
			Vec3 arm = pos - cg;
			t0 += arm.Cross(dir * F);
			if (t.gimbalLimit > 0) {
				Vec3 ap = m.rotation * t.gimbalPitchAxis.Unit();
				Vec3 ay = m.rotation * t.gimbalYawAxis.Unit();
				bp += arm.Cross(ap.Cross(dir) * F);
				by += arm.Cross(ay.Cross(dir) * F);
				gimballed.push_back(&t);
			}
		}
	}
	Vec3 need = torque - t0;
	if (!gimballed.empty()) {
		// Least squares on pitch and yaw (Y and Z body torques).
		double a11 = bp.y, a12 = by.y, a21 = bp.z, a22 = by.z;
		double det = a11 * a22 - a12 * a21;
		double gp = 0, gy = 0;
		if (std::fabs(det) > 1e-6) {
			gp = (need.y * a22 - a12 * need.z) / det;
			gy = (a11 * need.z - a21 * need.y) / det;
		}
		for (Thruster* t : gimballed) {
			double lim = t->gimbalLimit;
			// Gimbal actuators slew at about 5 deg/s.
			double step = 5 * kDeg * dt;
			t->gimbalPitch += Clamp(Clamp(gp, -lim, lim) - t->gimbalPitch,
				-step, step);
			t->gimbalYaw += Clamp(Clamp(gy, -lim, lim) - t->gimbalYaw,
				-step, step);
		}
		// Roll by differential gimbal (lumped).
		double rollLimit = 3.0e6;
		v.AddExternalForce(Vec3(), Vec3(Clamp(torque.x, -rollLimit, rollLimit),
			0, 0));
	} else {
		// APS: a few kN m on each axis.
		double lim = 8000;
		v.AddExternalForce(Vec3(), Vec3(Clamp(torque.x, -lim, lim),
			Clamp(torque.y, -lim, lim), Clamp(torque.z, -lim, lim)));
	}
}


// ---------------------------------------------------------------------------
// Sequencing


void
SaturnIU::Step(double dt)
{
	switch (fMode) {
		case Mode::Prelaunch: _Prelaunch(); break;
		case Mode::FirstStage: _FirstStage(dt); break;
		case Mode::SecondStage:
		case Mode::ThirdStage: _UpperStages(dt); break;
		case Mode::Orbit:
		case Mode::TLIPrep: _Orbit(dt); break;
		case Mode::TLI: _TLI(dt); break;
		case Mode::Coast: _Orbit(dt); break;
		case Mode::Safed: break;
	}
}


void
SaturnIU::_Prelaunch()
{
	Vehicle& v = f.CSMVehicle();
	const std::string first = fSaturnV ? "S-IC" : "S-IB";
	double get = f.GET();
	// The F-1s start at T-8.9 s; the H-1s at T-3.
	if (!fIgnited && get >= (fSaturnV ? -8.9 : -3.0)) {
		_Ignite(v, first);
		fIgnited = true;
		f.Log(LogEntry::Event, "Ignition sequence start");
	}
	if (get >= 0 && fIgnited) {
		// Hold-down release once the engines are at full thrust.
		const Module* m = v.FindModule(first);
		double level = 0;
		int n = 0;
		for (const Thruster& t : m->thrusters)
			if (IsMain(t)) {
				level += t.level;
				n++;
			}
		if (n > 0 && level / n > 0.9) {
			f.holdDown = false;
			fLiftoffGET = get;
			fLaunchUp = v.position.Unit();
			fLaunchDown = fPlane.Cross(fLaunchUp).Unit();
			f.CMPGNCS().panel.liftoff = true;
			f.SetPhase(Phase::Boost);
			f.Log(LogEntry::Event, "Liftoff");
			f.Log(LogEntry::Capcom, "Liftoff. The clock is running.");
			fMode = Mode::FirstStage;
		}
	}
}


void
SaturnIU::_FirstStage(double dt)
{
	Vehicle& v = f.CSMVehicle();
	double t = f.GET() - fLiftoffGET;
	const std::string first = fSaturnV ? "S-IC" : "S-IB";
	double deg = fSaturnV ? Tilt(kTiltV, t) : Tilt(kTiltIB, t);
	Vec3 dir = fLaunchUp * std::cos(deg * kDeg) + fLaunchDown * std::sin(deg * kDeg);
	Vec3 down = -fLaunchUp * std::sin(deg * kDeg) + fLaunchDown * std::cos(deg * kDeg);
	if (fSequence == 0) {
		_Steer(v, dir, down, 1.0 * kDeg, dt);
		// Center (inboard) engine cutoff, to hold the acceleration near 4 g.
		double ceco = fSaturnV ? 135.2 : 140.6;
		if (!fCenterOut && t >= ceco) {
			fCenterOut = true;
			if (fSaturnV)
				FailEngine("F-1 5", false);
			else
				for (int i = 1; i <= 4; i++)
					FailEngine("H-1 " + std::to_string(i), false);
			f.Log(LogEntry::Event, fSaturnV ? "Center engine cutoff"
				: "Inboard engine cutoff");
		}
		if (_StageDepleted(v, first) || _StageThrust(v, first) <= 0) {
			_Cutoff(v, first);
			f.Log(LogEntry::Event, "Outboard engine cutoff");
			fEventGET = f.GET();
			fFrozenDir = dir;
			fSequence = 1;
		}
	} else if (fSequence == 1) {
		_Steer(v, fFrozenDir, down, 0.5 * kDeg, dt);
		if (f.GET() - fEventGET >= 0.7) {
			f.JettisonStage(first, 2.0);
			fSequence = 2;
		}
	} else if (fSequence == 2) {
		_Steer(v, fFrozenDir, down, 0.5 * kDeg, dt);
		if (f.GET() - fEventGET >= 2.1) {
			// Ullage rockets fired at separation; the J-2s start.
			std::string next = fSaturnV ? "S-II" : "S-IVB";
			_Ignite(v, next);
			f.Log(LogEntry::Event, next + " ignition");
			fEventGET = f.GET();
			fSequence = 0;
			fCenterOut = false;
			fMode = fSaturnV ? Mode::SecondStage : Mode::ThirdStage;
		}
	}
}


double
SaturnIU::_TimeForDV(Vehicle& v, double dv) const
{
	double m = v.Mass();
	double t = 0;
	const char* stages[] = {"S-II", "S-IVB"};
	for (const char* name : stages) {
		const Module* mod = v.FindModule(name);
		if (!mod)
			continue;
		double F = _StageThrust(v, name);
		bool burning = F > 0;
		if (!burning) {
			// Not yet lit: what it will deliver.
			F = 0;
			for (const Thruster& th : mod->thrusters)
				if (IsMain(th) && !th.failedOff)
					F += th.maxThrust;
		}
		if (F <= 0)
			continue;
		double ve = 421 * kG0;
		double prop = mod->tanks.empty() ? 0 : mod->tanks[0].mass;
		double dvStage = ve * std::log(m / std::max(1.0, m - prop));
		double mdot = F / ve;
		if (dv <= dvStage) {
			t += m / mdot * (1 - std::exp(-dv / ve));
			return t;
		}
		t += prop / mdot;
		dv -= dvStage;
		m -= prop + mod->dryMass;
	}
	return t;   // not enough: all of it, to burnout
}


Vec3
SaturnIU::_IGM(Vehicle& v)
{
	double mu = world::kMuEarth;
	Vec3 r = v.position, vel = v.velocity;
	double R = r.Length();
	Vec3 rh = r / R;
	Vec3 hh = fPlane.Cross(rh).Unit();
	double vr = vel.Dot(rh), vh = vel.Dot(hh);
	double y = r.Dot(fPlane), vy = vel.Dot(fPlane);
	double rp = world::kREarth + targetPerigee;
	double ra = world::kREarth + targetApogee;
	double rt = rp;
	double vt = std::sqrt(mu * (2 / rp - 2 / (rp + ra)));
	double geff = mu / (R * R) - vh * vh / R;
	// Time to go, with the radial losses folded in (iterated).
	double tgo = std::max(1.0, fTGo);
	for (int i = 0; i < 4; i++) {
		double gAvg = 0.5 * (geff + std::max(0.0, mu / (rt * rt) - vt * vt / rt));
		double dvh = vt - vh;
		double dvr = -vr + gAvg * tgo;
		double dv = std::sqrt(dvh * dvh + dvr * dvr + vy * vy);
		tgo = _TimeForDV(v, dv);
	}
	fTGo = tgo;
	double F = 0;
	for (const char* s : {"S-II", "S-IVB"})
		F += _StageThrust(v, s);
	double a = F / v.Mass();
	if (a <= 0)
		return vel.Unit();
	double T = std::max(tgo, 10.0);
	double ar = 6 * (rt - R) / (T * T) - 4 * vr / T + geff;
	double ay = -6 * y / (T * T) - 4 * vy / T;
	ar = Clamp(ar, -0.7 * a, 0.7 * a);
	ay = Clamp(ay, -0.3 * a, 0.3 * a);
	double ah = std::sqrt(std::max(0.0, a * a - ar * ar - ay * ay));
	return (rh * ar + hh * ah + fPlane * ay).Unit();
}


void
SaturnIU::_UpperStages(double dt)
{
	Vehicle& v = f.CSMVehicle();
	double sinceIgn = f.GET() - fEventGET;
	Vec3 down = -v.position.Unit();
	std::string stage = fMode == Mode::SecondStage ? "S-II" : "S-IVB";

	if (fSequence == 0) {
		// Tower jettison on cue, then iterative guidance.
		if (f.TowerAttached() && autoTowerJettison
			&& sinceIgn >= (fSaturnV ? 35.0 : 20.0))
			f.JettisonTower();
		double igmStart = fSaturnV && fMode == Mode::SecondStage ? 40.0 : 5.0;
		if (!fSaturnV && fMode == Mode::ThirdStage)
			igmStart = 25.0;
		Vec3 dir = fFrozenDir.Length() > 0 ? fFrozenDir : v.velocity.Unit();
		if (sinceIgn >= igmStart && !fFrozen) {
			dir = _IGM(v);
			if (fTGo < 12 && fMode == Mode::ThirdStage) {
				fFrozen = true;    // chi freeze for the last seconds
			}
			fFrozenDir = dir;
		}
		_Steer(v, fFrozenDir.Length() > 0 ? fFrozenDir : dir, down,
			1.0 * kDeg, dt);
		// S-II center engine cutoff (pogo suppression).
		if (fSaturnV && fMode == Mode::SecondStage && !fCenterOut
			&& f.GET() - fLiftoffGET >= 460.6) {
			fCenterOut = true;
			Thruster* c = v.GetThruster("S-II J-2 5");
			if (c && c->on) {
				c->on = false;
				f.Log(LogEntry::Event, "S-II center engine cutoff");
			}
		}
		// Cutoff: the orbit's energy reaches the target's.
		double mu = world::kMuEarth;
		double R = v.position.Length();
		double rp = world::kREarth + targetPerigee;
		double ra = world::kREarth + targetApogee;
		double vNeed = std::sqrt(mu * (2 / R - 2 / (rp + ra)));
		bool depleted = _StageDepleted(v, stage) || _StageThrust(v, stage) <= 0;
		double tail = 0;
		if (fMode == Mode::ThirdStage) {
			const Thruster* j2 = v.GetThruster("J-2");
			if (j2)
				tail = 0.5 * j2->rampTime * j2->level
					* _StageThrust(v, "S-IVB") / v.Mass();
		}
		if (v.velocity.Length() >= vNeed - tail && sinceIgn > 10) {
			_Cutoff(v, "S-II");
			_Cutoff(v, "S-IVB");
			f.Log(LogEntry::Event, "S-IVB cutoff");
			fMode = Mode::Orbit;
			fHoldDir = v.velocity.Unit();
			fSequence = 0;
			f.SetPhase(Phase::EarthParkingOrbit);
			char buf[120];
			Situation s = f.Situate(v);
			std::snprintf(buf, sizeof(buf), "Insertion: %.1f x %.1f nmi",
				s.apoapsis / 1852, s.periapsis / 1852);
			if (getenv("IUDIAG"))
				std::printf("cutoff v %.1f need %.1f tail %.1f r %.0f\n",
					v.velocity.Length(), vNeed, tail, R);
			f.Log(LogEntry::Event, buf);
			f.Log(LogEntry::Capcom, "Houston: you are GO for orbit.");
			return;
		}
		if (depleted) {
			_Cutoff(v, stage);
			if (fMode == Mode::ThirdStage) {
				// S-IVB out of propellant short of orbit.
				f.Log(LogEntry::Warning, "S-IVB propellant depletion before "
					"orbital velocity");
				fMode = Mode::Safed;
				return;
			}
			f.Log(LogEntry::Event, "S-II cutoff");
			fEventGET = f.GET();
			fSequence = 1;
		}
	} else if (fSequence == 1) {
		_Steer(v, fFrozenDir, down, 0.5 * kDeg, dt);
		if (f.GET() - fEventGET >= 0.8) {
			f.JettisonStage("S-II", 2.0);
			fSequence = 2;
		}
	} else if (fSequence == 2) {
		_Steer(v, fFrozenDir, down, 0.5 * kDeg, dt);
		if (f.GET() - fEventGET >= 3.5) {
			_Ignite(v, "S-IVB");
			f.Log(LogEntry::Event, "S-IVB ignition");
			fEventGET = f.GET();
			fSequence = 0;
			fMode = Mode::ThirdStage;
		}
	}
}


// ---------------------------------------------------------------------------
// Orbit and TLI


void
SaturnIU::_Orbit(double dt)
{
	Vehicle* v = Carrier();
	if (v == nullptr) {
		fMode = Mode::Safed;
		return;
	}
	const FlightPlan& plan = f.Mission().plan;
	if (fMode == Mode::Orbit) {
		// Local horizontal, heads down, at orbital rate.
		Vec3 dir = (v->velocity - v->position.Unit()
			* v->velocity.Dot(v->position.Unit())).Unit();
		_Steer(*v, dir, -v->position.Unit(), 0.5 * kDeg, dt);
		if (plan.tli > 0 && tliEnabled && !tliInhibit && f.BoosterAttached()
			&& f.GET() >= plan.tli - 578) {
			_PrepareTLI();
		}
		return;
	}
	if (fMode == Mode::TLIPrep) {
		if (tliInhibit) {
			f.Log(LogEntry::Caution, "TLI inhibited");
			fMode = Mode::Orbit;
			return;
		}
		// Maneuver to the burn attitude five minutes ahead.
		Vec3 dir = fHoldDir;
		if (f.GET() >= fTIG - 300) {
			State s{v->position, v->velocity, f.JulianDate()};
			Vec3 vreq;
			double tof = (fAim.jd - s.jd) * kDay;
			if (Lambert(s.r, fAim.r, tof, world::kMuEarth, fPlane, &vreq))
				dir = (vreq + fBias - s.v).Unit();
		} else {
			dir = (v->velocity - v->position.Unit()
				* v->velocity.Dot(v->position.Unit())).Unit();
		}
		_Steer(*v, dir, -v->position.Unit(), 0.3 * kDeg, dt);
		if (f.GET() >= fTIG) {
			_Ignite(*v, "S-IVB");
			f.SetPhase(Phase::TranslunarInjection);
			f.Log(LogEntry::Event, "S-IVB restart: TLI");
			fMode = Mode::TLI;
			fBestVGo = 1e9;
			fFrozen = false;
		}
		return;
	}
	// Coast after TLI: inertial hold for transposition and docking, then
	// the slingshot once the LM is out.
	if (fHoldDir.Length() == 0)
		fHoldDir = v->ToInertial(Vec3(1, 0, 0));
	_Steer(*v, fHoldDir, v->ToInertial(Vec3(0, 0, 1)), 0.3 * kDeg, dt);
	if (!f.BoosterAttached() && f.LMDocked() && !f.LMVehicle()) {
		if (fExtractGET == 0)
			fExtractGET = f.GET();
		if (!fSlingshot && f.GET() - fExtractGET > 5400) {
			// LOX dump through the J-2 and an APS burn: about 40 m/s
			// retrograde, so the stage passes behind the Moon into solar orbit.
			fSlingshot = true;
			v->velocity -= v->velocity.Unit() * 40.0;
			f.Log(LogEntry::Event, "S-IVB LOX dump: slingshot maneuver");
			fMode = Mode::Safed;
		}
	}
}


void
SaturnIU::_PrepareTLI()
{
	Vehicle* v = Carrier();
	const FlightPlan& plan = f.Mission().plan;
	double jd0 = f.JulianDate() - f.GET() / kDay;
	State now{v->position, v->velocity, f.JulianDate()};
	// The RTCC's impulsive solution near the planned mid-burn time.
	double tMid = plan.tli + 173;
	State park = Propagate(now, tMid - f.GET());
	Vec3 dv;
	double arrive;
	State ign;
	if (!TargetTLI(park, 111000, &dv, &arrive, &ign, 400,
			plan.loi > 0 ? jd0 + plan.loi / kDay : 0)) {
		f.Log(LogEntry::Warning, "TLI targeting failed: no go for TLI");
		tliEnabled = false;
		return;
	}
	State post = ign;
	post.v += dv;
	fAim = Propagate(post, 7200);
	Vec3 vl;
	if (!Lambert(ign.r, fAim.r, 7200, world::kMuEarth, fPlane, &vl)) {
		f.Log(LogEntry::Warning, "TLI targeting failed: no go for TLI");
		tliEnabled = false;
		return;
	}
	fBias = post.v - vl;
	double F = 0;
	for (const Thruster& t : v->FindModule("S-IVB")->thrusters)
		if (IsMain(t) && !t.failedOff)
			F += t.maxThrust;
	double ve = 421 * kG0;
	double m = v->Mass();
	double tb = m * ve / F * (1 - std::exp(-dv.Length() / ve));
	double midGET = (ign.jd - jd0) * kDay;
	fTIG = midGET - tb / 2;
	fMode = Mode::TLIPrep;
	char buf[160];
	int tig = int(fTIG);
	std::snprintf(buf, sizeof(buf), "TB6: TLI ignition %d:%02d:%02d, "
		"delta-V %.0f ft/s, burn %.0f s", tig / 3600, tig / 60 % 60, tig % 60,
		dv.Length() / 0.3048, tb);
	f.Log(LogEntry::Event, buf);
	f.Log(LogEntry::Capcom, f.Mission().csmCallsign + ", Houston. You are "
		"GO for TLI.");
}


void
SaturnIU::_TLI(double dt)
{
	Vehicle* v = Carrier();
	if (v == nullptr) {
		fMode = Mode::Safed;
		return;
	}
	State s{v->position, v->velocity, f.JulianDate()};
	double tof = (fAim.jd - s.jd) * kDay;
	Vec3 vreq;
	if (Lambert(s.r, fAim.r, tof, world::kMuEarth, fPlane, &vreq))
		fVGo = vreq + fBias - s.v;
	double vgo = fVGo.Length();
	double a = _StageThrust(*v, "S-IVB") / v->Mass();
	if (vgo < 60 && !fFrozen) {
		fFrozen = true;
		fFrozenDir = fVGo.Unit();
	}
	Vec3 dir = fFrozen ? fFrozenDir : fVGo.Unit();
	_Steer(*v, dir, -v->position.Unit(), 1.0 * kDeg, dt);
	fTGo = a > 0 ? vgo / a : 0;
	// Cut off early by the velocity the J-2 adds while its thrust decays.
	const Thruster* j2 = v->GetThruster("J-2");
	double tail = j2 ? 0.5 * j2->rampTime * j2->level : 0;
	bool done = fVGo.Dot(dir) <= a * tail || vgo > fBestVGo + 1.0;
	fBestVGo = std::min(fBestVGo, vgo);
	if (done || _StageDepleted(*v, "S-IVB")) {
		_Cutoff(*v, "S-IVB");
		fTLICutoffGET = f.GET();
		fMode = Mode::Coast;
		fHoldDir = v->ToInertial(Vec3(1, 0, 0));
		f.SetPhase(Phase::TranslunarCoast);
		char buf[120];
		std::snprintf(buf, sizeof(buf), "S-IVB cutoff: TLI complete, residual "
			"%.1f ft/s", vgo / 0.3048);
		f.Log(LogEntry::Event, buf);
		f.Log(LogEntry::Capcom, "Houston: TLI looks good. You're on your way.");
	}
}

}  // namespace flight
