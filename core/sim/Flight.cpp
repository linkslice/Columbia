// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Flight.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace sim {

namespace {

const EngineSpec kEngines[] = {
	{"S-IVB (J-2)", 1000000.0, 421.0},
	{"SPS", 91190.0, 314.0},
};

struct Deriv {
	Vec3 dr, dv;
};

}  // namespace


const EngineSpec&
Engine(EngineId id)
{
	return kEngines[id == ENGINE_SIVB ? 0 : 1];
}


double
Burn::DeltaV() const
{
	return std::sqrt(prograde * prograde + normal * normal
		+ radial * radial);
}


const char*
OutcomeText(Outcome outcome)
{
	switch (outcome) {
		case Outcome::None: return "In flight";
		case Outcome::Splashdown: return "Splashdown — entry within the corridor";
		case Outcome::SkipOut: return "Skip-out — entry too shallow";
		case Outcome::TooSteep: return "Burn-up — entry too steep";
		case Outcome::LunarImpact: return "Lunar impact";
		case Outcome::OutOfPropellant: return "Out of propellant";
	}
	return "";
}


double
VehicleState::Mass() const
{
	double m = kCSMDryMass + spsPropellant;
	if (sivbAttached)
		m += kSIVBDryMass + sivbPropellant;
	return m;
}


bool
VehicleState::Completed(uint32_t id) const
{
	return std::find(completed.begin(), completed.end(), id)
		!= completed.end();
}


// ---------------------------------------------------------------------------
// Model


double
Model::MoonRate() const
{
	return std::sqrt((kMuEarth + kMuMoon)
		/ (kMoonDistance * kMoonDistance * kMoonDistance));
}


Vec3
Model::MoonPosition(double t) const
{
	double a = moonPhase0 + MoonRate() * t;
	return {kMoonDistance * std::cos(a), kMoonDistance * std::sin(a), 0};
}


Vec3
Model::MoonVelocity(double t) const
{
	double n = MoonRate();
	double a = moonPhase0 + n * t;
	return {-kMoonDistance * n * std::sin(a), kMoonDistance * n * std::cos(a),
		0};
}


Vec3
Model::Gravity(const Vec3& r, double t) const
{
	Vec3 rm = MoonPosition(t);
	Vec3 d = r - rm;
	double re = r.Length(), rd = d.Length(), rmLen = rm.Length();
	Vec3 a = r * (-kMuEarth / (re * re * re));
	a -= d * (kMuMoon / (rd * rd * rd));
	a -= rm * (kMuMoon / (rmLen * rmLen * rmLen));   // indirect term
	return a;
}


bool
Model::NearMoon(const Vec3& r, double t) const
{
	return (r - MoonPosition(t)).Length() < kMoonSOI;
}


OrbitSummary
Summarize(const Model& model, const VehicleState& s)
{
	OrbitSummary o;
	o.aroundMoon = model.NearMoon(s.r, s.t);
	Vec3 r = s.r, v = s.v;
	double mu = kMuEarth, radius = kREarth;
	if (o.aroundMoon) {
		r = r - model.MoonPosition(s.t);
		v = v - model.MoonVelocity(s.t);
		mu = kMuMoon;
		radius = kRMoon;
	}
	double rl = r.Length(), vl = v.Length();
	o.altitude = rl - radius;
	o.speed = vl;
	o.flightPathAngle = std::asin(std::max(-1.0, std::min(1.0,
		r.Dot(v) / (rl * vl)))) * 180.0 / kPi;
	double energy = vl * vl / 2 - mu / rl;
	Vec3 h = r.Cross(v);
	Vec3 e = v.Cross(h) / mu - r / rl;
	double ecc = e.Length();
	double p = h.Dot(h) / mu;
	o.periapsis = p / (1 + ecc) - radius;
	if (energy < 0 && ecc < 1) {
		double a = -mu / (2 * energy);
		o.apoapsis = a * (1 + ecc) - radius;
	} else {
		o.apoapsis = -1;
	}
	return o;
}


// ---------------------------------------------------------------------------
// Propagator


void
Propagator::_StartDueBurn(VehicleState& s, const FlightPlan& plan,
	double tEnd, double* nextEvent) const
{
	*nextEvent = tEnd;
	const Burn* due = nullptr;
	for (const Burn& b : plan) {
		if (s.Completed(b.id))
			continue;
		if (b.time <= s.t + 1e-9) {
			if (due == nullptr || b.time < due->time)
				due = &b;
		} else if (b.time < *nextEvent) {
			*nextEvent = b.time;
		}
	}
	if (due == nullptr)
		return;

	s.completed.push_back(due->id);
	double dv = due->DeltaV();
	if (dv <= 0)
		return;
	if (due->engine == ENGINE_SIVB
		&& (!s.sivbAttached || s.sivbPropellant <= 0))
		return;  // no S-IVB to burn
	if (due->engine == ENGINE_SPS) {
		s.sivbAttached = false;  // separate before the first SPS burn
		if (s.spsPropellant <= 0)
			return;
	}

	s.activeBurn = due->id;
	s.burnDone = 0;
	s.burnTarget = dv;
	s.burnComponents = Vec3(due->prograde, due->normal, due->radial) / dv;
	s.burnAroundMoon = fModel.NearMoon(s.r, s.t);
	s.burnEngine = due->engine;
	*nextEvent = s.t;
}


Vec3
Propagator::_ThrustDirection(const VehicleState& s, const Vec3& rIn,
	const Vec3& vIn, double t) const
{
	// The attitude is held in the local orbital frame, as the guidance
	// did by steering at orbital rate during long burns.
	Vec3 r = rIn, v = vIn;
	if (s.burnAroundMoon) {
		r = r - fModel.MoonPosition(t);
		v = v - fModel.MoonVelocity(t);
	}
	Vec3 P = v.Unit();
	Vec3 N = r.Cross(v).Unit();
	Vec3 R = N.Cross(P);
	const Vec3& c = s.burnComponents;
	return P * c.x + N * c.y + R * c.z;
}


bool
Propagator::_BurnStep(VehicleState& s, double dt) const
{
	const EngineSpec& e = Engine(s.burnEngine);
	double* prop = s.burnEngine == ENGINE_SIVB
		? &s.sivbPropellant : &s.spsPropellant;
	double mdot = e.thrust / (e.isp * kG0);
	double m = s.Mass();

	// Shorten the step to land exactly on the commanded delta-v or on
	// propellant depletion.
	double accel = e.thrust / m;
	double remaining = s.burnTarget - s.burnDone;
	bool finish = false;
	if (accel * dt >= remaining) {
		dt = remaining / accel;
		finish = true;
	}
	if (mdot * dt >= *prop) {
		dt = *prop / mdot;
		finish = true;
	}
	if (dt <= 0) {
		s.activeBurn = 0;
		return true;
	}

	// RK4 with the thrust direction fixed and mass varying linearly.
	auto f = [&](const Vec3& r, const Vec3& v, double t, double mm) {
		Deriv d;
		d.dr = v;
		d.dv = fModel.Gravity(r, t)
			+ _ThrustDirection(s, r, v, t) * (e.thrust / mm);
		return d;
	};
	double m1 = m, m2 = m - mdot * dt / 2, m3 = m - mdot * dt;
	Deriv k1 = f(s.r, s.v, s.t, m1);
	Deriv k2 = f(s.r + k1.dr * (dt / 2), s.v + k1.dv * (dt / 2),
		s.t + dt / 2, m2);
	Deriv k3 = f(s.r + k2.dr * (dt / 2), s.v + k2.dv * (dt / 2),
		s.t + dt / 2, m2);
	Deriv k4 = f(s.r + k3.dr * dt, s.v + k3.dv * dt, s.t + dt, m3);
	s.r += (k1.dr + k2.dr * 2 + k3.dr * 2 + k4.dr) * (dt / 6);
	s.v += (k1.dv + k2.dv * 2 + k3.dv * 2 + k4.dv) * (dt / 6);
	s.t += dt;
	*prop = std::max(0.0, *prop - mdot * dt);
	s.burnDone += e.isp * kG0 * std::log(m1 / m3);

	if (finish || s.burnDone >= s.burnTarget - 1e-6) {
		if (*prop <= 1e-6 && s.burnDone < s.burnTarget - 0.01
			&& onPropellantDepleted) {
			Burn b;
			b.id = s.activeBurn;
			onPropellantDepleted(b, s.burnDone);
		}
		s.activeBurn = 0;
	}
	return true;
}


double
Propagator::_CoastStep(VehicleState& s, double h, double hMax,
	double* hNext) const
{
	// Dormand-Prince 5(4) with step-size control.
	static const double c2 = 1.0 / 5, c3 = 3.0 / 10, c4 = 4.0 / 5,
		c5 = 8.0 / 9;
	static const double a21 = 1.0 / 5;
	static const double a31 = 3.0 / 40, a32 = 9.0 / 40;
	static const double a41 = 44.0 / 45, a42 = -56.0 / 15, a43 = 32.0 / 9;
	static const double a51 = 19372.0 / 6561, a52 = -25360.0 / 2187,
		a53 = 64448.0 / 6561, a54 = -212.0 / 729;
	static const double a61 = 9017.0 / 3168, a62 = -355.0 / 33,
		a63 = 46732.0 / 5247, a64 = 49.0 / 176, a65 = -5103.0 / 18656;
	static const double b1 = 35.0 / 384, b3 = 500.0 / 1113,
		b4 = 125.0 / 192, b5 = -2187.0 / 6784, b6 = 11.0 / 84;
	static const double e1 = 71.0 / 57600, e3 = -71.0 / 16695,
		e4 = 71.0 / 1920, e5 = -17253.0 / 339200, e6 = 22.0 / 525,
		e7 = -1.0 / 40;

	auto f = [&](const Vec3& r, const Vec3& v, double t) {
		Deriv d;
		d.dr = v;
		d.dv = fModel.Gravity(r, t);
		return d;
	};

	h = std::min(h, hMax);
	for (int attempt = 0; attempt < 60; attempt++) {
		const Vec3& r = s.r;
		const Vec3& v = s.v;
		double t = s.t;
		Deriv k1 = f(r, v, t);
		Deriv k2 = f(r + k1.dr * (h * a21), v + k1.dv * (h * a21), t + c2 * h);
		Deriv k3 = f(r + (k1.dr * a31 + k2.dr * a32) * h,
			v + (k1.dv * a31 + k2.dv * a32) * h, t + c3 * h);
		Deriv k4 = f(r + (k1.dr * a41 + k2.dr * a42 + k3.dr * a43) * h,
			v + (k1.dv * a41 + k2.dv * a42 + k3.dv * a43) * h, t + c4 * h);
		Deriv k5 = f(r + (k1.dr * a51 + k2.dr * a52 + k3.dr * a53
				+ k4.dr * a54) * h,
			v + (k1.dv * a51 + k2.dv * a52 + k3.dv * a53 + k4.dv * a54) * h,
			t + c5 * h);
		Deriv k6 = f(r + (k1.dr * a61 + k2.dr * a62 + k3.dr * a63
				+ k4.dr * a64 + k5.dr * a65) * h,
			v + (k1.dv * a61 + k2.dv * a62 + k3.dv * a63 + k4.dv * a64
				+ k5.dv * a65) * h, t + h);
		Vec3 rn = r + (k1.dr * b1 + k3.dr * b3 + k4.dr * b4 + k5.dr * b5
			+ k6.dr * b6) * h;
		Vec3 vn = v + (k1.dv * b1 + k3.dv * b3 + k4.dv * b4 + k5.dv * b5
			+ k6.dv * b6) * h;
		Deriv k7 = f(rn, vn, t + h);
		Vec3 er = (k1.dr * e1 + k3.dr * e3 + k4.dr * e4 + k5.dr * e5
			+ k6.dr * e6 + k7.dr * e7) * h;
		Vec3 ev = (k1.dv * e1 + k3.dv * e3 + k4.dv * e4 + k5.dv * e5
			+ k6.dv * e6 + k7.dv * e7) * h;

		double scaleR = 0.5 + 1e-11 * rn.Length();
		double scaleV = 5e-4 + 1e-11 * vn.Length();
		double err = std::max(er.Length() / scaleR, ev.Length() / scaleV);
		double factor = err > 0 ? 0.9 * std::pow(err, -0.2) : 5.0;
		factor = std::max(0.2, std::min(5.0, factor));
		if (err <= 1.0) {
			s.r = rn;
			s.v = vn;
			s.t = t + h;
			*hNext = h * factor;
			return h;
		}
		h *= factor;
	}
	// Could not meet tolerance; take a tiny step anyway.
	*hNext = h;
	return 0;
}


bool
Propagator::_CheckEvents(VehicleState& s, const VehicleState& before) const
{
	double alt = s.r.Length() - kREarth;
	double altBefore = before.r.Length() - kREarth;
	if (alt < kEntryInterface && altBefore >= kEntryInterface) {
		// Linear interpolation to the interface is accurate to metres
		// at the step sizes used near the Earth.
		double f = (altBefore - kEntryInterface) / (altBefore - alt);
		s.t = before.t + (s.t - before.t) * f;
		s.r = before.r + (s.r - before.r) * f;
		s.v = before.v + (s.v - before.v) * f;
		double speed = s.v.Length();
		double gamma = std::asin(s.r.Unit().Dot(s.v.Unit())) * 180.0 / kPi;
		s.entrySpeed = speed;
		s.entryAngle = gamma;
		double shallow = speed > 9000 ? -5.3 : -0.3;
		double steep = speed > 9000 ? -7.4 : -9.0;
		if (gamma > shallow)
			s.outcome = Outcome::SkipOut;
		else if (gamma < steep)
			s.outcome = Outcome::TooSteep;
		else
			s.outcome = Outcome::Splashdown;
		return true;
	}
	if (alt < kEntryInterface && s.v.Dot(s.r) < 0 && altBefore < alt) {
		// Started below the interface (should not happen in flight).
		return false;
	}
	double moonDist = (s.r - fModel.MoonPosition(s.t)).Length();
	if (stopAtMoon && moonDist < kRMoon) {
		s.outcome = Outcome::LunarImpact;
		return true;
	}
	return false;
}


void
Propagator::Advance(VehicleState& s, const FlightPlan& plan, double tEnd,
	std::vector<Sample>* samples, double maxSpacing) const
{
	auto terminal = [](Outcome o) {
		return o != Outcome::None && o != Outcome::OutOfPropellant;
	};
	if (samples != nullptr && samples->empty())
		samples->push_back({s.t, s.r, s.v, s.activeBurn != 0});

	double h = std::max(1.0, std::min(600.0,
		0.01 * s.r.Length() / std::max(1.0, s.v.Length())));

	while (s.t < tEnd - 1e-9 && !terminal(s.outcome)) {
		if (s.activeBurn != 0) {
			VehicleState before = s;
			_BurnStep(s, std::min(1.0, tEnd - s.t));
			if (samples != nullptr)
				samples->push_back({s.t, s.r, s.v, true});
			if (_CheckEvents(s, before)) {
				if (samples != nullptr)
					samples->back() = {s.t, s.r, s.v, false};
				break;
			}
			continue;
		}
		double nextEvent;
		_StartDueBurn(s, plan, tEnd, &nextEvent);
		if (s.activeBurn != 0)
			continue;
		while (s.t < nextEvent - 1e-9 && !terminal(s.outcome)) {
			VehicleState before = s;
			double hNext;
			double hMax = std::min(maxSpacing, nextEvent - s.t);
			double took = _CoastStep(s, h, hMax, &hNext);
			if (took <= 0)
				break;
			// Keep h unaffected by short steps forced by events.
			if (took < hMax || h <= hMax)
				h = hNext;
			if (_CheckEvents(s, before)) {
				if (samples != nullptr)
					samples->push_back({s.t, s.r, s.v, false});
				break;
			}
			if (samples != nullptr)
				samples->push_back({s.t, s.r, s.v, false});
		}
	}
}


// ---------------------------------------------------------------------------
// Prediction


Prediction
Predict(const Model& model, const VehicleState& from, const FlightPlan& plan,
	double horizon, bool keepSamples, bool stopAtMoon)
{
	Prediction p;
	Propagator prop(model);
	prop.stopAtMoon = stopAtMoon;
	p.final = from;
	prop.Advance(p.final, plan, from.t + horizon, &p.samples, 600.0);

	// Pericynthion: closest sample to the Moon, refined with a parabola.
	const std::vector<Sample>& S = p.samples;
	size_t best = 0;
	double bestDist = 1e300;
	for (size_t i = 0; i < S.size(); i++) {
		double d = (S[i].r - model.MoonPosition(S[i].t)).Length();
		if (d < bestDist) {
			bestDist = d;
			best = i;
		}
	}
	auto refine = [&](size_t i, auto dist, double* tOut, double* dOut) {
		*tOut = S[i].t;
		*dOut = dist(i);
		if (i == 0 || i + 1 >= S.size())
			return;
		double t0 = S[i - 1].t, t1 = S[i].t, t2 = S[i + 1].t;
		double d0 = dist(i - 1), d1 = dist(i), d2 = dist(i + 1);
		// Fit d(t) = a t^2 + b t + c through three points.
		double denom = (t0 - t1) * (t0 - t2) * (t1 - t2);
		if (std::fabs(denom) < 1e-12)
			return;
		double a = (t2 * (d1 - d0) + t1 * (d0 - d2) + t0 * (d2 - d1)) / denom;
		double b = (t2 * t2 * (d0 - d1) + t1 * t1 * (d2 - d0)
			+ t0 * t0 * (d1 - d2)) / denom;
		double c = d1 - a * t1 * t1 - b * t1;
		if (a <= 0)
			return;
		double tm = -b / (2 * a);
		if (tm < t0 || tm > t2)
			return;
		*tOut = tm;
		*dOut = a * tm * tm + b * tm + c;
	};
	auto moonDist = [&](size_t i) {
		return (S[i].r - model.MoonPosition(S[i].t)).Length();
	};
	for (size_t i = 0; i < S.size(); i++) {
		if (S[i].burning)
			continue;
		Vec3 r = S[i].r - model.MoonPosition(S[i].t);
		if (r.Length() > kMoonSOI)
			continue;
		if (i > 0 && (S[i - 1].r - model.MoonPosition(S[i - 1].t)).Length()
				< kMoonSOI)
			continue;   // only the moment of entry
		Vec3 v = S[i].v - model.MoonVelocity(S[i].t);
		Vec3 h = r.Cross(v);
		Vec3 e = v.Cross(h) / kMuMoon - r / r.Length();
		double rp = h.Dot(h) / kMuMoon / (1 + e.Length());
		p.hasLunarApproach = true;
		p.lunarApproachTime = S[i].t;
		p.approachPeriRadius = h.z < 0 ? -rp : rp;
		break;
	}
	if (bestDist < kMoonSOI) {
		p.hasPericynthion = true;
		double t, d;
		refine(best, moonDist, &t, &d);
		p.pericynthionTime = t;
		p.pericynthionAltitude = d - kRMoon;
		Vec3 rr = S[best].r - model.MoonPosition(S[best].t);
		Vec3 vv = S[best].v - model.MoonVelocity(S[best].t);
		p.signedPericynthion = rr.Cross(vv).z < 0 ? -d : d;
	}

	// Earth perigee after the lunar encounter (or after the last burn if
	// the flight never reaches the Moon).
	double after = p.hasPericynthion ? p.pericynthionTime : from.t;
	if (p.final.outcome == Outcome::Splashdown
		|| p.final.outcome == Outcome::SkipOut
		|| p.final.outcome == Outcome::TooSteep) {
		// Report the vacuum perigee of the arriving conic.
		const VehicleState& at = p.final;
		Vec3 r = at.r, v = at.v;
		double rl = r.Length();
		Vec3 h = r.Cross(v);
		Vec3 e = v.Cross(h) / kMuEarth - r / rl;
		double pr = h.Dot(h) / kMuEarth / (1 + e.Length());
		p.hasReturnPerigee = true;
		p.perigeeTime = at.t;
		p.perigeeAltitude = pr - kREarth;
	} else if (p.hasPericynthion) {
		auto earthDist = [&](size_t i) { return S[i].r.Length(); };
		size_t bi = 0;
		double bd = 1e300;
		for (size_t i = 0; i < S.size(); i++) {
			if (S[i].t <= after)
				continue;
			double d = S[i].r.Length();
			if (d < bd) {
				bd = d;
				bi = i;
			}
		}
		// Only a true minimum counts (not the end of the horizon).
		if (bi > 0 && bi + 1 < S.size()) {
			double t, d;
			refine(bi, earthDist, &t, &d);
			p.hasReturnPerigee = true;
			p.perigeeTime = t;
			p.perigeeAltitude = d - kREarth;
		}
	}
	if (!keepSamples)
		p.samples.clear();
	return p;
}


// ---------------------------------------------------------------------------
// Misc


std::string
FormatGET(double seconds)
{
	bool negative = seconds < 0;
	long total = long(std::floor(std::fabs(seconds) + 0.5));
	char buffer[32];
	std::snprintf(buffer, sizeof(buffer), "%s%03ld:%02ld:%02ld",
		negative ? "-" : "", total / 3600, (total / 60) % 60, total % 60);
	return buffer;
}


bool
ParseGET(const std::string& text, double* seconds)
{
	double parts[3] = {0, 0, 0};
	int count = 0;
	std::string field;
	for (size_t i = 0; i <= text.size(); i++) {
		char c = i < text.size() ? text[i] : ':';
		if (c == ':') {
			if (count >= 3 || field.empty())
				return false;
			char* end = nullptr;
			parts[count++] = std::strtod(field.c_str(), &end);
			if (end == nullptr || *end != '\0')
				return false;
			field.clear();
		} else if (c != ' ') {
			field += c;
		}
	}
	double s = 0;
	for (int i = 0; i < count; i++)
		s = s * 60 + parts[i];
	*seconds = s;
	return true;
}


VehicleState
ParkingOrbit(const Model& model, double altitude, double longitude)
{
	(void)model;
	VehicleState s;
	double r = kREarth + altitude;
	double v = std::sqrt(kMuEarth / r);
	s.r = {r * std::cos(longitude), r * std::sin(longitude), 0};
	s.v = {-v * std::sin(longitude), v * std::cos(longitude), 0};
	return s;
}

}  // namespace sim
