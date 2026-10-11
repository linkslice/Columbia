// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Trajectory.h"

#include "../sim/World.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>

namespace flight {

using namespace sim;

namespace {

const double kDay = 86400.0;
const double kEIAltitude = 121920.0;   // 400,000 ft

}  // namespace


Vec3
Gravity(const Vec3& r, double jd)
{
	return world::Gravity::Acceleration(r, jd, true);
}


Vec3
MoonPos(double jd)
{
	return world::Ephemeris::MoonPosition(jd);
}


Vec3
MoonVel(double jd)
{
	return world::Ephemeris::MoonVelocity(jd);
}


State
Propagate(const State& s0, double seconds,
	const std::function<bool(const State&, const State&)>& stop, double tol)
{
	// Dormand-Prince 5(4).
	static const double c2 = 1.0 / 5, c3 = 3.0 / 10, c4 = 4.0 / 5, c5 = 8.0 / 9;
	static const double a21 = 1.0 / 5;
	static const double a31 = 3.0 / 40, a32 = 9.0 / 40;
	static const double a41 = 44.0 / 45, a42 = -56.0 / 15, a43 = 32.0 / 9;
	static const double a51 = 19372.0 / 6561, a52 = -25360.0 / 2187,
		a53 = 64448.0 / 6561, a54 = -212.0 / 729;
	static const double a61 = 9017.0 / 3168, a62 = -355.0 / 33,
		a63 = 46732.0 / 5247, a64 = 49.0 / 176, a65 = -5103.0 / 18656;
	static const double b1 = 35.0 / 384, b3 = 500.0 / 1113, b4 = 125.0 / 192,
		b5 = -2187.0 / 6784, b6 = 11.0 / 84;
	static const double e1 = 71.0 / 57600, e3 = -71.0 / 16695, e4 = 71.0 / 1920,
		e5 = -17253.0 / 339200, e6 = 22.0 / 525, e7 = -1.0 / 40;

	State s = s0;
	if (seconds == 0)
		return s;
	double dir = seconds > 0 ? 1 : -1;
	double remaining = std::fabs(seconds);
	double h = std::min(remaining, 60.0);
	auto f = [&](const Vec3& r, double jd) { return Gravity(r, jd); };
	while (remaining > 1e-9) {
		h = std::min(h, remaining);
		double dt = h * dir;
		double jd = s.jd;
		Vec3 r = s.r, v = s.v;
		Vec3 k1r = v, k1v = f(r, jd);
		Vec3 r2 = r + k1r * (a21 * dt), v2 = v + k1v * (a21 * dt);
		Vec3 k2r = v2, k2v = f(r2, jd + c2 * dt / kDay);
		Vec3 r3 = r + (k1r * a31 + k2r * a32) * dt;
		Vec3 v3 = v + (k1v * a31 + k2v * a32) * dt;
		Vec3 k3r = v3, k3v = f(r3, jd + c3 * dt / kDay);
		Vec3 r4 = r + (k1r * a41 + k2r * a42 + k3r * a43) * dt;
		Vec3 v4 = v + (k1v * a41 + k2v * a42 + k3v * a43) * dt;
		Vec3 k4r = v4, k4v = f(r4, jd + c4 * dt / kDay);
		Vec3 r5 = r + (k1r * a51 + k2r * a52 + k3r * a53 + k4r * a54) * dt;
		Vec3 v5 = v + (k1v * a51 + k2v * a52 + k3v * a53 + k4v * a54) * dt;
		Vec3 k5r = v5, k5v = f(r5, jd + c5 * dt / kDay);
		Vec3 r6 = r + (k1r * a61 + k2r * a62 + k3r * a63 + k4r * a64 + k5r * a65) * dt;
		Vec3 v6 = v + (k1v * a61 + k2v * a62 + k3v * a63 + k4v * a64 + k5v * a65) * dt;
		Vec3 k6r = v6, k6v = f(r6, jd + dt / kDay);
		Vec3 rn = r + (k1r * b1 + k3r * b3 + k4r * b4 + k5r * b5 + k6r * b6) * dt;
		Vec3 vn = v + (k1v * b1 + k3v * b3 + k4v * b4 + k5v * b5 + k6v * b6) * dt;
		Vec3 k7v = f(rn, jd + dt / kDay);
		Vec3 er = (k1r * e1 + k3r * e3 + k4r * e4 + k5r * e5 + k6r * e6 + vn * e7) * dt;
		Vec3 ev = (k1v * e1 + k3v * e3 + k4v * e4 + k5v * e5 + k6v * e6 + k7v * e7) * dt;
		double scaleR = tol * std::max(1.0e6, rn.Length());
		double scaleV = tol * std::max(1.0e3, vn.Length());
		double err = std::max(er.Length() / scaleR, ev.Length() / scaleV);
		if (err <= 1.0 || h < 1e-3) {
			State prev = s;
			s.r = rn;
			s.v = vn;
			s.jd = jd + dt / kDay;
			remaining -= h;
			if (stop && stop(prev, s))
				return s;
		}
		double factor = err > 0 ? 0.9 * std::pow(err, -0.2) : 5.0;
		h *= std::max(0.2, std::min(5.0, factor));
		h = std::min(h, 3600.0);
	}
	return s;
}


Conic
ConicOf(const Vec3& r, const Vec3& v, double mu)
{
	Conic c;
	c.h = r.Cross(v);
	double rl = r.Length();
	c.e = v.Cross(c.h) / mu - r / rl;
	c.ecc = c.e.Length();
	double energy = v.Dot(v) / 2 - mu / rl;
	c.sma = -mu / (2 * energy);
	double p = c.h.Dot(c.h) / mu;
	c.rp = p / (1 + c.ecc);
	c.ra = c.ecc < 1 ? p / (1 - c.ecc) : -1;
	c.inc = std::acos(Clamp(c.h.z / std::max(1e-9, c.h.Length()), -1, 1));
	return c;
}


Approach
ClosestMoonApproach(const State& s, double maxSeconds)
{
	Approach best;
	auto rangeRate = [](const State& st) {
		Vec3 rr = st.r - MoonPos(st.jd);
		Vec3 vv = st.v - MoonVel(st.jd);
		return rr.Dot(vv);
	};
	State bracketLo, bracketHi;
	bool bracketed = false;
	Propagate(s, maxSeconds, [&](const State& a, const State& b) {
		if (rangeRate(a) < 0 && rangeRate(b) >= 0) {
			bracketLo = a;
			bracketHi = b;
			bracketed = true;
			return true;
		}
		// Impact: stop.
		return (b.r - MoonPos(b.jd)).Length() < world::kRMoon;
	});
	if (!bracketed)
		return best;
	// Refine by bisection in time.
	State lo = bracketLo;
	double span = (bracketHi.jd - bracketLo.jd) * kDay;
	for (int i = 0; i < 40 && span > 0.01; i++) {
		State mid = Propagate(lo, span / 2);
		if (rangeRate(mid) < 0) {
			lo = mid;
		}
		span /= 2;
	}
	best.state = lo;
	best.jd = lo.jd;
	best.radius = (lo.r - MoonPos(lo.jd)).Length();
	best.found = true;
	return best;
}


Interface
EntryInterface(const State& s, double maxSeconds)
{
	Interface out;
	State a, b;
	bool hit = false;
	double target = world::kREarth + kEIAltitude;
	Propagate(s, maxSeconds, [&](const State& p, const State& q) {
		if (p.r.Length() > target && q.r.Length() <= target) {
			a = p;
			b = q;
			hit = true;
			return true;
		}
		return false;
	});
	if (!hit)
		return out;
	State lo = a;
	double span = (b.jd - a.jd) * kDay;
	for (int i = 0; i < 40 && span > 0.01; i++) {
		State mid = Propagate(lo, span / 2);
		if (mid.r.Length() > target)
			lo = mid;
		span /= 2;
	}
	out.state = lo;
	out.jd = lo.jd;
	out.speed = lo.v.Length();
	out.gamma = std::asin(Clamp(lo.r.Unit().Dot(lo.v.Unit()), -1, 1));
	out.found = true;
	return out;
}


Vec3
ToLVLH(const Vec3& r, const Vec3& v, const Vec3& in)
{
	Vec3 z = (-r).Unit();
	Vec3 y = v.Cross(r).Unit();        // -h
	Vec3 x = y.Cross(z);
	return Vec3(in.Dot(x), in.Dot(y), in.Dot(z));
}


Vec3
FromLVLH(const Vec3& r, const Vec3& v, const Vec3& l)
{
	Vec3 z = (-r).Unit();
	Vec3 y = v.Cross(r).Unit();
	Vec3 x = y.Cross(z);
	return x * l.x + y * l.y + z * l.z;
}


// ---------------------------------------------------------------------------
// Targeting


namespace {

// Solve F(x) = 0 for x in R^n (n <= 3) by Newton with finite differences.
template <int N>
bool
Newton(const std::function<bool(const double*, double*)>& F, double* x,
	const double* step, const double* tol, int iterations)
{
	for (int it = 0; it < iterations; it++) {
		double f0[N];
		if (!F(x, f0))
			return false;
		bool done = true;
		for (int i = 0; i < N; i++)
			if (std::fabs(f0[i]) > tol[i])
				done = false;
		if (done)
			return true;
		double J[N][N];
		for (int j = 0; j < N; j++) {
			double xp[N];
			for (int i = 0; i < N; i++)
				xp[i] = x[i];
			xp[j] += step[j];
			double f1[N];
			if (!F(xp, f1))
				return false;
			for (int i = 0; i < N; i++)
				J[i][j] = (f1[i] - f0[i]) / step[j];
		}
		// Gaussian elimination.
		double A[N][N + 1];
		for (int i = 0; i < N; i++) {
			for (int j = 0; j < N; j++)
				A[i][j] = J[i][j];
			A[i][N] = -f0[i];
		}
		for (int c = 0; c < N; c++) {
			int p = c;
			for (int r = c + 1; r < N; r++)
				if (std::fabs(A[r][c]) > std::fabs(A[p][c]))
					p = r;
			if (std::fabs(A[p][c]) < 1e-30)
				return false;
			for (int k = 0; k <= N; k++)
				std::swap(A[c][k], A[p][k]);
			for (int r = 0; r < N; r++) {
				if (r == c)
					continue;
				double m = A[r][c] / A[c][c];
				for (int k = c; k <= N; k++)
					A[r][k] -= m * A[c][k];
			}
		}
		double dx[N];
		double scale = 1;
		for (int i = 0; i < N; i++) {
			dx[i] = A[i][N] / A[i][i];
			// Damp very large steps.
			double lim = 50 * step[i];
			if (std::fabs(dx[i]) > lim)
				scale = std::min(scale, lim / std::fabs(dx[i]));
		}
		for (int i = 0; i < N; i++)
			x[i] += dx[i] * scale;
	}
	return false;
}

}  // namespace


bool
TargetTLI(const State& parking0, double periAlt, Vec3* deltaV, double* arrivalJD,
	State* ignition, double window, double wantArrivalJD)
{
	double target = world::kRMoon + periAlt;
	// Coarse search over ignition time and prograde magnitude.
	double bestR = 1e30, bestT = 0, bestMag = 3150;
	for (double dt = -window; dt <= window; dt += 120) {
		State p = Propagate(parking0, dt);
		for (double mag = 3020; mag <= 3260; mag += 15) {
			State s = p;
			s.v += s.v.Unit() * mag;
			Approach ap = ClosestMoonApproach(s, 5 * kDay);
			double score = ap.radius;
			if (wantArrivalJD > 0)
				score += std::max(0.0, std::fabs(ap.jd - wantArrivalJD) * 24 - 3)
					* 5.0e6;
			if (ap.found && score < bestR) {
				bestR = score;
				bestT = dt;
				bestMag = mag;
			}
		}
	}
	if (getenv("TLIDEBUG"))
		fprintf(stderr, "TLI coarse: best %.0f km at dt %.0f s, %.0f m/s\n",
			bestR / 1000, bestT, bestMag);
	if (bestR > 1.5e8)
		return false;
	// Finer search around the best cell.
	{
		double t0 = bestT, m0 = bestMag;
		for (double dt = t0 - 150; dt <= t0 + 150; dt += 15) {
			State p = Propagate(parking0, dt);
			for (double mag = m0 - 16; mag <= m0 + 16; mag += 2) {
				State s = p;
				s.v += s.v.Unit() * mag;
				Approach ap = ClosestMoonApproach(s, 5 * kDay);
				if (!ap.found)
					continue;
				double score = std::fabs(ap.radius - target);
				if (wantArrivalJD > 0)
					score += std::max(0.0, std::fabs(ap.jd - wantArrivalJD) * 24 - 3)
						* 5.0e6;
				if (score < bestR) {
					bestR = score;
					bestT = dt;
					bestMag = mag;
				}
			}
		}
		if (getenv("TLIDEBUG"))
			fprintf(stderr, "TLI fine: score %.0f km at dt %.0f s, %.0f m/s\n",
				bestR / 1000, bestT, bestMag);
	}
	State parking = Propagate(parking0, bestT);
	Vec3 along = parking.v.Unit();
	Vec3 normal = parking.r.Cross(parking.v).Unit();
	auto miss = [&](double mag, double yaw, Approach* ap) {
		State s = parking;
		s.v += (along * std::cos(yaw) + normal * std::sin(yaw)) * mag;
		*ap = ClosestMoonApproach(s, 5 * kDay);
		return ap->found;
	};
	// In-plane magnitude for the pericynthion radius. (Yaw at TLI barely
	// moves the arrival point, which lies near the line of nodes; the
	// plane comes from launch, and midcourse corrections trim it.)
	double x[2] = {bestMag, 0};
	auto radiusAt = [&](double mag) {
		Approach ap;
		return miss(mag, 0, &ap) ? ap.radius : 1e30;
	};
	// Polish with a dense local scan (r(mag) is not monotonic: impact on
	// one side, the far-side flyby on the other).
	for (double span = 2.0; span >= 0.01; span /= 10) {
		double best = std::fabs(radiusAt(x[0]) - target), at = x[0];
		for (double mag = x[0] - span; mag <= x[0] + span; mag += span / 20) {
			double e = std::fabs(radiusAt(mag) - target);
			if (e < best) {
				best = e;
				at = mag;
			}
		}
		x[0] = at;
	}
	if (getenv("TLIDEBUG"))
		fprintf(stderr, "TLI: %.2f m/s -> pericynthion radius %.0f km\n", x[0],
			radiusAt(x[0]) / 1000);
	if (radiusAt(x[0]) > target + 2.0e7)
		return false;
	Approach ap;
	miss(x[0], x[1], &ap);
	*deltaV = (along * std::cos(x[1]) + normal * std::sin(x[1])) * x[0];
	if (arrivalJD)
		*arrivalJD = ap.jd;
	if (ignition)
		*ignition = parking;
	return true;
}


bool
TargetLOI(const State& s, double apoAlt, double periAlt, Vec3* deltaV)
{
	Vec3 rr = s.r - MoonPos(s.jd);
	Vec3 vv = s.v - MoonVel(s.jd);
	double ra = world::kRMoon + apoAlt, rp = world::kRMoon + periAlt;
	double r = rr.Length();
	// Burn at the current point; it becomes the apsis nearest its radius.
	double a = (ra + rp) / 2;
	double vWant = std::sqrt(world::kMuMoon * (2 / r - 1 / a));
	Vec3 h = rr.Cross(vv).Unit();
	Vec3 dir = h.Cross(rr).Unit();
	*deltaV = dir * vWant - vv;
	return true;
}


Vec3
CircularizeAt(const State& s, double mu, const Vec3& bodyPos,
	const Vec3& bodyVel)
{
	Vec3 rr = s.r - bodyPos, vv = s.v - bodyVel;
	double vc = std::sqrt(mu / rr.Length());
	Vec3 h = rr.Cross(vv).Unit();
	Vec3 dir = h.Cross(rr).Unit();
	return dir * vc - vv;
}


bool
TargetTEI(const State& orbit0, double gamma, Vec3* deltaV, double* eiJD,
	State* burnState)
{
	// The burn point matters as much as the size: scan one orbit for the
	// place and magnitude whose return perigee is lowest, then solve for
	// the entry angle with magnitude (and a touch of timing).
	Vec3 rr0 = orbit0.r - MoonPos(orbit0.jd);
	Vec3 vv0 = orbit0.v - MoonVel(orbit0.jd);
	double period = 2 * M_PI * std::sqrt(std::pow(rr0.Length(), 3) / world::kMuMoon);
	auto burn = [&](const State& at, double mag) {
		State s = at;
		Vec3 vr = s.v - MoonVel(s.jd);
		s.v += vr.Unit() * mag;
		return s;
	};
	auto perigee = [&](const State& s, double* jdOut) {
		// Earth closest approach within 4.5 days.
		double best = 1e30, bestJD = 0;
		State prev = s;
		Propagate(s, 4.5 * kDay, [&](const State& a, const State& b) {
			double rb = b.r.Length();
			if (rb < best) {
				best = rb;
				bestJD = b.jd;
			}
			(void)a;
			prev = b;
			return rb < world::kREarth + 100000;
		});
		if (jdOut)
			*jdOut = bestJD;
		return best;
	};
	double bestScore = 1e30, bestDt = 0, bestMag = 1000;
	for (double dt = 0; dt < period; dt += period / 72) {
		State at = Propagate(orbit0, dt);
		for (double mag = 850; mag <= 1250; mag += 25) {
			double jd;
			double rp = perigee(burn(at, mag), &jd);
			double transit = (jd - at.jd) * 24;
			double score = std::fabs(rp - (world::kREarth + 40000))
				+ std::max(0.0, std::fabs(transit - 60) - 6) * 2.0e6;
			if (score < bestScore) {
				bestScore = score;
				bestDt = dt;
				bestMag = mag;
			}
		}
	}
	if (getenv("TLIDEBUG"))
		fprintf(stderr, "TEI coarse: perigee error %.0f km at dt %.0f s, %.0f m/s\n",
			bestScore / 1000, bestDt, bestMag);
	if (bestScore > 3.0e7)
		return false;
	// Fine: timing and magnitude for the lowest perigee near the target.
	for (int pass = 0; pass < 3; pass++) {
		double tStep = pass == 0 ? 20 : (pass == 1 ? 4 : 1);
		double mStep = pass == 0 ? 4 : (pass == 1 ? 1 : 0.2);
		double t0 = bestDt, m0 = bestMag;
		for (double dt = t0 - 6 * tStep; dt <= t0 + 6 * tStep; dt += tStep) {
			State at = Propagate(orbit0, dt);
			for (double mag = m0 - 6 * mStep; mag <= m0 + 6 * mStep; mag += mStep) {
				double jd;
				double rp = perigee(burn(at, mag), &jd);
				double transit = (jd - at.jd) * 24;
				double score = std::fabs(rp - (world::kREarth + 40000))
					+ std::max(0.0, std::fabs(transit - 60) - 6) * 2.0e6;
				if (score < bestScore) {
					bestScore = score;
					bestDt = dt;
					bestMag = mag;
				}
			}
		}
	}
	State at = Propagate(orbit0, bestDt);
	// Entry angle by magnitude.
	double x = bestMag;
	for (int it = 0; it < 30; it++) {
		Interface in = EntryInterface(burn(at, x), 4.5 * kDay);
		if (!in.found)
			break;
		double err = in.gamma - gamma;
		if (std::fabs(err) < 0.0003)
			break;
		Interface in2 = EntryInterface(burn(at, x + 0.05), 4.5 * kDay);
		if (!in2.found)
			break;
		double slope = (in2.gamma - in.gamma) / 0.05;
		if (std::fabs(slope) < 1e-9)
			break;
		x -= Clamp(err / slope, -2.0, 2.0);
	}
	Interface in = EntryInterface(burn(at, x), 4.5 * kDay);
	if (getenv("TLIDEBUG"))
		fprintf(stderr, "TEI: %.2f m/s at dt %.0f s -> %s gamma %.2f deg\n", x,
			bestDt, in.found ? "EI" : "no EI", in.gamma / kDeg);
	if (!in.found)
		return false;
	Vec3 vr = at.v - MoonVel(at.jd);
	*deltaV = vr.Unit() * x;
	if (eiJD)
		*eiJD = in.jd;
	if (burnState)
		*burnState = at;
	(void)vv0;
	return true;
}


bool
TargetMCCMoon(const State& s, double periAlt, Vec3* deltaV)
{
	double target = world::kRMoon + periAlt;
	Vec3 rr = s.r - MoonPos(s.jd);
	(void)rr;
	Vec3 along = s.v.Unit();
	Vec3 normal = s.r.Cross(s.v).Unit();
	Vec3 radial = normal.Cross(along);
	double x[2] = {0, 0};
	const double step[2] = {0.05, 0.05}, tol[2] = {200, 200};
	auto apply = [&](const double* p, Approach* ap) {
		State st = s;
		st.v += along * p[0] + radial * p[1];
		*ap = ClosestMoonApproach(st, 4 * kDay);
		return ap->found;
	};
	bool ok = Newton<2>([&](const double* p, double* out) {
		Approach ap;
		if (!apply(p, &ap))
			return false;
		Vec3 r2 = ap.state.r - MoonPos(ap.jd);
		Vec3 v2 = ap.state.v - MoonVel(ap.jd);
		Vec3 h = r2.Cross(v2).Unit();
		Vec3 moonPlane = MoonPos(ap.jd).Cross(MoonVel(ap.jd)).Unit();
		out[0] = ap.radius - target;
		out[1] = h.Cross(moonPlane).Dot(r2.Unit()) * 1e5;
		return true;
	}, x, step, tol, 12);
	if (!ok)
		return false;
	*deltaV = along * x[0] + radial * x[1];
	return true;
}


bool
TargetMCCEarth(const State& s, double gamma, Vec3* deltaV)
{
	Vec3 along = s.v.Unit();
	Vec3 normal = s.r.Cross(s.v).Unit();
	Vec3 radial = normal.Cross(along);
	double x[1] = {0};
	const double step[1] = {0.02}, tol[1] = {0.0005};
	bool ok = Newton<1>([&](const double* p, double* out) {
		State st = s;
		st.v += radial * p[0];
		Interface in = EntryInterface(st, 4 * kDay);
		if (!in.found) {
			State end = Propagate(st, 3 * kDay);
			Conic c = ConicOf(end.r, end.v, world::kMuEarth);
			out[0] = (c.rp - (world::kREarth + 40000)) / 1e7;
			return true;
		}
		out[0] = in.gamma - gamma;
		return true;
	}, x, step, tol, 20);
	if (!ok)
		return false;
	*deltaV = radial * x[0];
	return true;
}


namespace {

double
StumpffC(double z)
{
	if (z > 1e-6)
		return (1 - std::cos(std::sqrt(z))) / z;
	if (z < -1e-6)
		return (std::cosh(std::sqrt(-z)) - 1) / -z;
	return 0.5 - z / 24 + z * z / 720;
}

double
StumpffS(double z)
{
	if (z > 1e-6) {
		double q = std::sqrt(z);
		return (q - std::sin(q)) / (q * q * q);
	}
	if (z < -1e-6) {
		double q = std::sqrt(-z);
		return (std::sinh(q) - q) / (q * q * q);
	}
	return 1.0 / 6 - z / 120 + z * z / 5040;
}

}  // namespace


bool
Lambert(const Vec3& r1, const Vec3& r2, double tof, double mu,
	const Vec3& normal, Vec3* v1, Vec3* v2)
{
	double R1 = r1.Length(), R2 = r2.Length();
	if (tof <= 0 || R1 <= 0 || R2 <= 0)
		return false;
	double cosd = sim::Clamp(r1.Dot(r2) / (R1 * R2), -1, 1);
	double dtheta = std::acos(cosd);
	if (r1.Cross(r2).Dot(normal) < 0)
		dtheta = 2 * M_PI - dtheta;
	double A = std::sin(dtheta) * std::sqrt(R1 * R2 / (1 - cosd));
	if (std::fabs(A) < 1e-9)
		return false;
	auto y = [&](double z) {
		return R1 + R2 + A * (z * StumpffS(z) - 1) / std::sqrt(StumpffC(z));
	};
	auto F = [&](double z) {
		double yz = y(z);
		if (yz < 0)
			return -1e30;
		return std::pow(yz / StumpffC(z), 1.5) * StumpffS(z) + A * std::sqrt(yz)
			- std::sqrt(mu) * tof;
	};
	// Bracket the root in z (F increases with z), then bisect/secant.
	double lo = -4 * M_PI * M_PI, hi = 4 * M_PI * M_PI * 0.999;
	while (y(lo) < 0 && lo < 0)
		lo *= 0.5;
	// Make sure y is positive somewhere in the bracket.
	double zlo = lo;
	for (int i = 0; i < 200 && (y(zlo) < 0 || F(zlo) > 0); i++)
		zlo += (hi - lo) / 400;
	if (F(zlo) > 0 || F(hi) < 0)
		return false;
	double a = zlo, b = hi;
	for (int i = 0; i < 200; i++) {
		double m = 0.5 * (a + b);
		if (F(m) < 0)
			a = m;
		else
			b = m;
		if (b - a < 1e-12)
			break;
	}
	double z = 0.5 * (a + b);
	double yz = y(z);
	double f = 1 - yz / R1;
	double g = A * std::sqrt(yz / mu);
	double gdot = 1 - yz / R2;
	if (std::fabs(g) < 1e-12)
		return false;
	*v1 = (r2 - r1 * f) / g;
	if (v2)
		*v2 = (r2 * gdot - r1) / g;
	return true;
}

}  // namespace flight
