// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Coasting trajectories under Earth, Moon and Sun gravity (the same
// force model the simulation flies), conic helpers, and the targeting the
// Real-Time Computer Complex did for Mission Control: TLI, LOI, TEI and
// midcourse corrections, solved by shooting on the full dynamics.

#ifndef COLUMBIA_TRAJECTORY_H
#define COLUMBIA_TRAJECTORY_H

#include "../sim/Math.h"

#include <functional>

namespace flight {

using sim::Vec3;

struct State {
	Vec3 r, v;        // Earth-centred inertial (m, m/s)
	double jd = 0;
};

// Gravitational acceleration of the force model at r (Earth-centred).
Vec3 Gravity(const Vec3& r, double jd);

// Adaptive Dormand-Prince propagation for `seconds` (may be negative).
// `stop` is checked after each step (with the step's start and end) and
// can end the propagation early; returns the final state.
State Propagate(const State& s, double seconds,
	const std::function<bool(const State&, const State&)>& stop = nullptr,
	double tolerance = 1e-9);

// Moon-relative quantities.
Vec3 MoonPos(double jd);
Vec3 MoonVel(double jd);

struct Conic {
	double sma = 0, ecc = 0, inc = 0;    // m, -, rad
	double rp = 0, ra = 0;               // radii (ra < 0 if hyperbolic)
	Vec3 h, e;                           // angular momentum, eccentricity vec
};
Conic ConicOf(const Vec3& r, const Vec3& v, double mu);

// Moon closest approach of a coast starting at s (within `maxSeconds`).
struct Approach {
	double jd = 0;
	double radius = 0;       // from the Moon's centre (m)
	State state;
	bool found = false;
};
Approach ClosestMoonApproach(const State& s, double maxSeconds);

// Earth's atmosphere interface crossing (400,000 ft) of a return coast.
struct Interface {
	double jd = 0;
	double gamma = 0;        // flight-path angle (rad), negative descending
	double speed = 0;        // inertial (m/s)
	State state;
	bool found = false;
};
Interface EntryInterface(const State& s, double maxSeconds);

// --- RTCC targeting ---------------------------------------------------------
// Impulsive delta-V (inertial) at s.jd that puts pericynthion at
// `pericynthionAlt` above the Moon (circumlunar free return or hybrid).
// Returns false if it doesn't converge.
// The ignition may move up to `window` seconds either way along the
// parking orbit; `ignition` returns the chosen pre-burn state.
// `wantArrivalJD` (if nonzero) is the planned pericynthion time; the
// search keeps to trajectories arriving within a few hours of it.
bool TargetTLI(const State& parking, double pericynthionAlt, Vec3* deltaV,
	double* arrivalJD, State* ignition = nullptr, double window = 1800,
	double wantArrivalJD = 0);
// Retrograde LOI at pericynthion into an orbit of the given apsides.
bool TargetLOI(const State& atPericynthion, double apoAlt, double periAlt,
	Vec3* deltaV);
// Circularization (or DOI) at an apsis.
Vec3 CircularizeAt(const State& s, double mu, const Vec3& bodyPos,
	const Vec3& bodyVel);
// TEI from lunar orbit: delta-V at s.jd that brings the CSM to entry
// interface with flight-path angle `gamma`. Returns the transit time.
// The burn point is searched over one orbit; `burnState` returns it.
bool TargetTEI(const State& orbit, double gamma, Vec3* deltaV, double* eiJD,
	State* burnState = nullptr);
// Midcourse correction to bring a coast to the given pericynthion or
// entry angle.
bool TargetMCCMoon(const State& s, double pericynthionAlt, Vec3* deltaV);
bool TargetMCCEarth(const State& s, double gamma, Vec3* deltaV);

// Two-body Lambert problem (universal variables): the velocities at r1
// and r2 for a transfer of `tof` seconds, the short way round in the
// prograde sense of `normal` (any vector along the orbit's angular
// momentum). Returns false if it doesn't converge.
bool Lambert(const Vec3& r1, const Vec3& r2, double tof, double mu,
	const Vec3& normal, Vec3* v1, Vec3* v2 = nullptr);

// A constant-attitude burn: thrust along `direction` (inertial) from
// s until the accelerometers have counted `deltaV` (the way the CMC's VG
// cuts off). Returns the state at cutoff.
State FiniteBurn(const State& s, const Vec3& direction, double deltaV,
	double thrust, double mass, double isp);
// The finite burn that does what an impulsive delta-V `dv` at impulseJD
// would: ignition time and the sensed delta-V vector (constant inertial
// direction). `pre` is any state before the burn.
bool FiniteEquivalent(const State& pre, double impulseJD, const Vec3& dv,
	double thrust, double mass, double isp, double* tigJD, Vec3* dvFinite);

// LVLH components (x along-track, y -orbit normal, z toward the body) of
// an inertial vector at a body-relative state.
Vec3 ToLVLH(const Vec3& rRel, const Vec3& vRel, const Vec3& inertial);
Vec3 FromLVLH(const Vec3& rRel, const Vec3& vRel, const Vec3& lvlh);

}  // namespace flight

#endif
