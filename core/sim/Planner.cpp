// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Planner.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>

namespace sim {

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();

typedef std::function<double(double)> Fn;

// Illinois false position on a bracket [a, b] with f(a), f(b) of opposite
// sign. Falls back to bisection where f is undefined.
bool
Root(const Fn& f, double a, double b, double fa, double fb, double xtol,
	double* x)
{
	int side = 0;
	double c = a;
	for (int i = 0; i < 80; i++) {
		c = (a * fb - b * fa) / (fb - fa);
		if (!(c > std::min(a, b) && c < std::max(a, b)))
			c = (a + b) / 2;
		double fc = f(c);
		if (std::isnan(fc)) {
			c = (a + b) / 2;
			fc = f(c);
			if (std::isnan(fc))
				return false;
		}
		if (fc == 0 || std::fabs(b - a) < xtol) {
			*x = c;
			return true;
		}
		if ((fc > 0) == (fb > 0)) {
			b = c;
			fb = fc;
			if (side == -1)
				fa /= 2;
			side = -1;
		} else {
			a = c;
			fa = fc;
			if (side == 1)
				fb /= 2;
			side = 1;
		}
	}
	*x = c;
	return true;
}

// Scans [from, to] in `steps` intervals and solves within the first
// interval where f changes sign. Returns false if none is found.
bool
ScanAndSolve(const Fn& f, double from, double to, int steps, double xtol,
	double* x, bool preferLast = false)
{
	double prevX = from, prevF = f(from);
	bool found = false;
	double bestA = 0, bestB = 0, bestFA = 0, bestFB = 0;
	for (int i = 1; i <= steps; i++) {
		double xi = from + (to - from) * i / steps;
		double fi = f(xi);
		if (!std::isnan(prevF) && !std::isnan(fi) && (prevF > 0) != (fi > 0)) {
			bestA = prevX; bestB = xi; bestFA = prevF; bestFB = fi;
			found = true;
			if (!preferLast)
				break;
		}
		prevX = xi;
		prevF = fi;
	}
	if (!found)
		return false;
	return Root(f, bestA, bestB, bestFA, bestFB, xtol, x);
}

double
OrbitPeriod(const Model& model, const VehicleState& s)
{
	bool moon = model.NearMoon(s.r, s.t);
	Vec3 r = s.r, v = s.v;
	double mu = kMuEarth;
	if (moon) {
		r = r - model.MoonPosition(s.t);
		v = v - model.MoonVelocity(s.t);
		mu = kMuMoon;
	}
	double energy = v.Dot(v) / 2 - mu / r.Length();
	if (energy >= 0)
		return 0;
	double a = -mu / (2 * energy);
	return 2 * kPi * std::sqrt(a * a * a / mu);
}

// Burn duration for a delta-v with an engine at a given mass.
double
BurnDuration(EngineId engine, double mass, double dv)
{
	const EngineSpec& e = Engine(engine);
	double ve = e.isp * kG0;
	double mdot = e.thrust / ve;
	return mass * (1 - std::exp(-std::fabs(dv) / ve)) / mdot;
}

}  // namespace


Planner::Planner(const Model& model, const VehicleState& now,
	const FlightPlan& plan)
	:
	fModel(model),
	fNow(now),
	fPlan(plan)
{
}


FlightPlan
Planner::_Before(double t) const
{
	FlightPlan before;
	for (const Burn& b : fPlan) {
		if (b.time < t)
			before.push_back(b);
	}
	return before;
}


uint32_t
Planner::_NextId() const
{
	uint32_t id = 1;
	for (const Burn& b : fPlan)
		id = std::max(id, b.id + 1);
	for (uint32_t c : fNow.completed)
		id = std::max(id, c + 1);
	return id;
}


double
Planner::_Earliest() const
{
	double t = fNow.t + kLeadTime;
	// Not before a burn that is already in progress or scheduled.
	for (const Burn& b : fPlan) {
		if (!fNow.Completed(b.id))
			t = std::max(t, b.time + kLeadTime);
	}
	return t;
}


PlanResult
Planner::TransLunarInjection(double periAlt, double perigeeAlt)
{
	PlanResult result;
	if (!fNow.sivbAttached || fNow.sivbPropellant <= 0) {
		result.message = "The S-IVB is no longer available.";
		return result;
	}
	if (fModel.NearMoon(fNow.r, fNow.t)) {
		result.message = "TLI is only possible from Earth orbit.";
		return result;
	}
	double period = OrbitPeriod(fModel, fNow);
	if (period <= 0 || period > 6 * 3600) {
		result.message = "TLI needs a closed parking orbit.";
		return result;
	}

	Burn burn;
	burn.id = _NextId();
	burn.label = "TLI";
	burn.engine = ENGINE_SIVB;
	const double targetRadius = -(kRMoon + periAlt);

	// Delta-v that puts pericynthion at the target for ignition time t.
	auto solveDV = [&](double t, double* dvOut) {
		FlightPlan base = _Before(t);
		Fn g = [&](double dv) {
			FlightPlan p = base;
			Burn b = burn;
			b.time = t;
			b.prograde = dv;
			p.push_back(b);
			Prediction pr = Predict(fModel, fNow, p, 5.5 * 86400, false,
				false);
			if (!pr.hasPericynthion)
				return kNaN;
			return pr.signedPericynthion - targetRadius;
		};
		return ScanAndSolve(g, 3100, 3300, 20, 0.001, dvOut);
	};
	auto perigeeError = [&](double t) {
		double dv;
		if (!solveDV(t, &dv))
			return kNaN;
		FlightPlan p = _Before(t);
		Burn b = burn;
		b.time = t;
		b.prograde = dv;
		p.push_back(b);
		Prediction pr = Predict(fModel, fNow, p, 10 * 86400, false);
		if (!pr.hasReturnPerigee)
			return kNaN;
		return pr.perigeeAltitude - perigeeAlt;
	};

	double t0 = _Earliest();
	double best = 0;
	bool found = false;
	// Search the next two revolutions for an injection window. Each window
	// has a fast and a slow solution; the slow one (the last crossing) is
	// cheaper and closer to the Apollo profile.
	for (int rev = 0; rev < 2 && !found; rev++) {
		double from = t0 + rev * period;
		found = ScanAndSolve(perigeeError, from, from + period, 120, 0.05,
			&best, true);
	}
	if (!found) {
		result.message = "No free-return window in the next two orbits.";
		return result;
	}
	double dv;
	solveDV(best, &dv);
	burn.time = best;
	burn.prograde = dv;
	result.ok = true;
	result.burn = burn;
	char text[160];
	std::snprintf(text, sizeof(text),
		"Free return: pericynthion %.0f km, entry perigee %.0f km.",
		periAlt / 1000, perigeeAlt / 1000);
	result.message = text;
	return result;
}


PlanResult
Planner::LunarOrbitInsertion()
{
	PlanResult result;
	Prediction pr = Predict(fModel, fNow, fPlan, 6 * 86400, true);
	if (!pr.hasPericynthion || pr.pericynthionAltitude < 20e3) {
		result.message = pr.hasPericynthion
			? "Pericynthion is too low for orbit insertion."
			: "No lunar encounter is predicted.";
		return result;
	}
	if (pr.pericynthionTime < _Earliest()) {
		result.message = "Pericynthion is too close to plan a burn.";
		return result;
	}

	// State at pericynthion.
	VehicleState at = fNow;
	Propagator prop(fModel);
	prop.Advance(at, fPlan, pr.pericynthionTime);
	Vec3 r = at.r - fModel.MoonPosition(at.t);
	Vec3 v = at.v - fModel.MoonVelocity(at.t);
	double rPeri = r.Length();
	double dv0 = v.Length() - std::sqrt(kMuMoon / rPeri);
	if (dv0 <= 0) {
		result.message = "Already captured by the Moon.";
		return result;
	}

	Burn burn;
	burn.id = _NextId();
	burn.label = "LOI";
	burn.engine = ENGINE_SPS;
	VehicleState massState = at;
	massState.sivbAttached = false;
	double mass = massState.Mass();

	// Choose delta-v so the resulting orbit's semi-major axis equals the
	// pericynthion radius: a circular orbit at that altitude.
	Fn g = [&](double dv) {
		double dur = BurnDuration(ENGINE_SPS, mass, dv);
		FlightPlan p = _Before(pr.pericynthionTime - dur / 2);
		Burn b = burn;
		b.time = pr.pericynthionTime - dur / 2;
		b.prograde = -dv;
		p.push_back(b);
		VehicleState s = fNow;
		prop.Advance(s, p, b.time + dur + 120);
		if (s.outcome != Outcome::None)
			return kNaN;
		Vec3 rr = s.r - fModel.MoonPosition(s.t);
		Vec3 vv = s.v - fModel.MoonVelocity(s.t);
		double energy = vv.Dot(vv) / 2 - kMuMoon / rr.Length();
		if (energy >= 0)
			return 1e9;
		return -kMuMoon / (2 * energy) - rPeri;
	};
	double dv;
	double lo = dv0 * 0.8, hi = dv0 * 1.2;
	double flo = g(lo), fhi = g(hi);
	if (std::isnan(flo) || std::isnan(fhi) || (flo > 0) == (fhi > 0)
		|| !Root(g, lo, hi, flo, fhi, 0.01, &dv)) {
		dv = dv0;
	}
	double dur = BurnDuration(ENGINE_SPS, mass, dv);
	burn.time = pr.pericynthionTime - dur / 2;
	burn.prograde = -dv;
	if (dv > Engine(ENGINE_SPS).isp * kG0
			* std::log(mass / (mass - massState.spsPropellant))) {
		result.message = "Not enough SPS propellant for orbit insertion.";
		return result;
	}
	result.ok = true;
	result.burn = burn;
	char text[160];
	std::snprintf(text, sizeof(text),
		"Circular lunar orbit at %.0f km, %.0f s burn.",
		(rPeri - kRMoon) / 1000, dur);
	result.message = text;
	return result;
}


PlanResult
Planner::TransEarthInjection(double perigeeAlt, double maxCoast)
{
	PlanResult result;
	VehicleState start = fNow;
	double t0 = _Earliest();
	Propagator prop(fModel);
	prop.Advance(start, _Before(t0), t0);
	if (!fModel.NearMoon(start.r, start.t)) {
		result.message = "TEI is only possible from lunar orbit.";
		return result;
	}
	double period = OrbitPeriod(fModel, start);
	if (period <= 0) {
		result.message = "Not in a closed lunar orbit.";
		return result;
	}

	Burn burn;
	burn.id = _NextId();
	burn.label = "TEI";
	burn.engine = ENGINE_SPS;

	// Delta-v for ignition time t, and the resulting coast to entry.
	auto solveDV = [&](double t, double* dvOut, double* coastOut) {
		FlightPlan base = _Before(t);
		Fn g = [&](double dv) {
			FlightPlan p = base;
			Burn b = burn;
			b.time = t;
			b.prograde = dv;
			p.push_back(b);
			Prediction pr = Predict(fModel, fNow, p, 6 * 86400, false);
			if (pr.final.outcome == Outcome::LunarImpact)
				return kNaN;
			if (!pr.hasReturnPerigee)
				return 1e9;   // still bound to the Moon
			return pr.perigeeAltitude - perigeeAlt;
		};
		// The lowest delta-v solution is the first sign change.
		if (!ScanAndSolve(g, 400, 2000, 32, 0.001, dvOut))
			return false;
		if (coastOut != nullptr) {
			FlightPlan p = base;
			Burn b = burn;
			b.time = t;
			b.prograde = *dvOut;
			p.push_back(b);
			Prediction pr = Predict(fModel, fNow, p, 6 * 86400, false);
			*coastOut = pr.hasReturnPerigee ? pr.perigeeTime - t : 1e9;
		}
		return true;
	};
	// Cheapest burn, with a penalty for coasting home longer than
	// maxCoast (Apollo returns took about 60 hours).
	auto cost = [&](double t) {
		double dv, coast;
		if (!solveDV(t, &dv, &coast))
			return 1e12;
		return dv + 25.0 * std::max(0.0, (coast - maxCoast) / 3600.0);
	};

	double bestT = 0, bestCost = 1e12;
	const int steps = 36;
	for (int i = 0; i < steps; i++) {
		double t = t0 + period * i / steps;
		double c = cost(t);
		if (c < bestCost) {
			bestCost = c;
			bestT = t;
		}
	}
	if (bestCost >= 1e12) {
		result.message = "No return trajectory found from this orbit.";
		return result;
	}
	// Golden-section refinement of the ignition time.
	double a = std::max(t0, bestT - period / steps);
	double b = bestT + period / steps;
	const double gr = 0.6180339887;
	double c = b - gr * (b - a), d = a + gr * (b - a);
	double fc = cost(c), fd = cost(d);
	for (int i = 0; i < 18; i++) {
		if (fc < fd) {
			b = d; d = c; fd = fc;
			c = b - gr * (b - a);
			fc = cost(c);
		} else {
			a = c; c = d; fc = fd;
			d = a + gr * (b - a);
			fd = cost(d);
		}
	}
	double t = fc < fd ? c : d;
	double dv, coast;
	if (!solveDV(t, &dv, &coast)) {
		t = bestT;
		solveDV(t, &dv, &coast);
	}
	VehicleState massState = start;
	massState.sivbAttached = false;
	double mass = massState.Mass();
	if (dv > Engine(ENGINE_SPS).isp * kG0
			* std::log(mass / (mass - massState.spsPropellant))) {
		result.message = "Not enough SPS propellant for TEI.";
		return result;
	}
	burn.time = t;
	burn.prograde = dv;
	result.ok = true;
	result.burn = burn;
	char text[160];
	std::snprintf(text, sizeof(text),
		"Entry perigee %.0f km after a %.0f h coast.", perigeeAlt / 1000,
		coast / 3600);
	result.message = text;
	return result;
}


PlanResult
Planner::PericynthionCorrection(double at, double periAlt)
{
	PlanResult result;
	at = std::max(at, _Earliest());
	Burn burn;
	burn.id = _NextId();
	burn.label = "MCC";
	burn.engine = ENGINE_SPS;
	const double targetRadius = -(kRMoon + periAlt);
	FlightPlan base = _Before(at);
	Fn g = [&](double dv) {
		FlightPlan p = base;
		Burn b = burn;
		b.time = at;
		b.prograde = dv;
		p.push_back(b);
		Prediction pr = Predict(fModel, fNow, p, 5 * 86400, false, false);
		if (!pr.hasPericynthion)
			return kNaN;
		return pr.signedPericynthion - targetRadius;
	};
	double dv;
	bool ok = false;
	for (double span : {5.0, 20.0, 80.0, 250.0}) {
		if (ScanAndSolve(g, -span, span, 20, 0.0005, &dv)) {
			ok = true;
			break;
		}
	}
	if (!ok) {
		result.message = "No correction found; is the Moon reachable?";
		return result;
	}
	burn.time = at;
	burn.prograde = dv;
	result.ok = true;
	result.burn = burn;
	result.message = "Pericynthion corrected.";
	return result;
}


PlanResult
Planner::EntryCorrection(double at, double perigeeAlt)
{
	PlanResult result;
	at = std::max(at, _Earliest());
	Burn burn;
	burn.id = _NextId();
	burn.label = "MCC";
	burn.engine = ENGINE_SPS;
	FlightPlan base = _Before(at);
	Fn g = [&](double dv) {
		FlightPlan p = base;
		Burn b = burn;
		b.time = at;
		b.prograde = dv;
		p.push_back(b);
		Prediction pr = Predict(fModel, fNow, p, 8 * 86400, false);
		if (!pr.hasReturnPerigee)
			return kNaN;
		return pr.perigeeAltitude - perigeeAlt;
	};
	double dv;
	bool ok = false;
	for (double span : {5.0, 20.0, 80.0, 250.0}) {
		if (ScanAndSolve(g, -span, span, 20, 0.0005, &dv)) {
			ok = true;
			break;
		}
	}
	if (!ok) {
		result.message = "No correction found; is the flight returning?";
		return result;
	}
	burn.time = at;
	burn.prograde = dv;
	result.ok = true;
	result.burn = burn;
	result.message = "Entry perigee corrected.";
	return result;
}

}  // namespace sim
