// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Translunar flight dynamics. The frame is Earth-centred and non-rotating,
// with the Moon on a circular orbit in the x-y plane. Gravity is the
// restricted three-body field (Earth + Moon, plus the indirect term that
// accounts for the Earth's own acceleration toward the Moon).

#ifndef COLUMBIA_FLIGHT_H
#define COLUMBIA_FLIGHT_H

#include "Vec3.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sim {

constexpr double kMuEarth = 3.986004418e14;      // m^3/s^2
constexpr double kMuMoon = 4.9048695e12;
constexpr double kREarth = 6378137.0;            // m
constexpr double kRMoon = 1737400.0;
constexpr double kMoonDistance = 384400.0e3;
constexpr double kMoonSOI = 64373.76e3;   // the AGC's RSPHERE
constexpr double kEntryInterface = 121920.0;     // 400,000 ft
constexpr double kG0 = 9.80665;
constexpr double kPi = 3.14159265358979323846;

enum EngineId { ENGINE_SIVB = 0, ENGINE_SPS = 1 };

struct EngineSpec {
	const char* name;
	double thrust;   // N
	double isp;      // s
};

const EngineSpec& Engine(EngineId id);

// Vehicle masses (kg). Command/Service Module flying alone (no LM).
constexpr double kCSMDryMass = 11900.0;
constexpr double kSPSPropellant = 18400.0;
constexpr double kSIVBDryMass = 13300.0;
constexpr double kSIVBPropellant = 60000.0;

struct Burn {
	uint32_t id = 0;
	std::string label;
	EngineId engine = ENGINE_SPS;
	double time = 0;        // ignition, seconds GET
	double prograde = 0;    // m/s, local frame of the dominant body
	double normal = 0;
	double radial = 0;

	double DeltaV() const;
};

typedef std::vector<Burn> FlightPlan;

enum class Outcome {
	None,
	Splashdown,       // entered within the corridor
	SkipOut,          // entry too shallow
	TooSteep,         // entry too steep (g-load)
	LunarImpact,
	OutOfPropellant   // informational; flight continues
};

const char* OutcomeText(Outcome outcome);

struct VehicleState {
	double t = 0;                 // seconds GET
	Vec3 r, v;                    // Earth-centred inertial
	double sivbPropellant = kSIVBPropellant;
	double spsPropellant = kSPSPropellant;
	bool sivbAttached = true;

	// Burn in progress.
	uint32_t activeBurn = 0;      // 0 = none
	double burnDone = 0;          // m/s achieved so far
	double burnTarget = 0;
	Vec3 burnComponents;          // unit (prograde, normal, radial)
	bool burnAroundMoon = false;  // local frame of the Moon or the Earth
	EngineId burnEngine = ENGINE_SPS;
	std::vector<uint32_t> completed;

	Outcome outcome = Outcome::None;
	double entrySpeed = 0, entryAngle = 0;   // at entry interface

	double Mass() const;
	bool Completed(uint32_t id) const;
};

struct Model {
	double moonPhase0 = 0;   // Moon's angle at GET 0, radians

	double MoonRate() const;
	Vec3 MoonPosition(double t) const;
	Vec3 MoonVelocity(double t) const;
	Vec3 Gravity(const Vec3& r, double t) const;
	// The body whose sphere of influence contains r (true = Moon).
	bool NearMoon(const Vec3& r, double t) const;
};

struct Sample {
	double t;
	Vec3 r;
	Vec3 v;
	bool burning;
};

// Osculating two-body elements relative to the dominant body.
struct OrbitSummary {
	bool aroundMoon = false;
	double altitude = 0;        // m above the surface
	double speed = 0;           // m/s relative to the body
	double periapsis = 0;       // altitude, m
	double apoapsis = 0;        // altitude, m; < 0 for escape
	double flightPathAngle = 0; // degrees
};

OrbitSummary Summarize(const Model& model, const VehicleState& state);

class Propagator {
public:
	explicit Propagator(const Model& model) : fModel(model) {}

	// When false, the Moon is treated as a point mass and impacts are not
	// detected. Targeting uses this so its functions stay continuous.
	bool stopAtMoon = true;

	// Advances `state` to tEnd, executing due burns from `plan`.
	// Stops early when a terminal outcome occurs. If `samples` is not null,
	// appends trajectory points no further apart than `maxSpacing` seconds.
	void Advance(VehicleState& state, const FlightPlan& plan, double tEnd,
		std::vector<Sample>* samples = nullptr,
		double maxSpacing = 600.0) const;

	// Called when a burn ends early because the propellant ran out.
	std::function<void(const Burn&, double achieved)> onPropellantDepleted;

private:
	void _StartDueBurn(VehicleState& s, const FlightPlan& plan,
		double tEnd, double* nextEvent) const;
	bool _BurnStep(VehicleState& s, double dt) const;
	Vec3 _ThrustDirection(const VehicleState& s, const Vec3& r,
		const Vec3& v, double t) const;
	double _CoastStep(VehicleState& s, double dtTry, double dtMax,
		double* dtNext) const;
	bool _CheckEvents(VehicleState& s, const VehicleState& before) const;

	const Model& fModel;
};

// Summary of a predicted trajectory, for the planner display.
struct Prediction {
	std::vector<Sample> samples;
	VehicleState final;
	bool hasPericynthion = false;
	double pericynthionTime = 0, pericynthionAltitude = 0;
	double signedPericynthion = 0;   // radius, m; negative = clockwise
	// Lunar approach conic, taken on entering the Moon's sphere of
	// influence. Its periapsis radius is signed by the direction of travel
	// around the Moon (negative = clockwise seen from +z, the free-return
	// sense), which makes it continuous even for trajectories that would
	// hit the Moon. Used by the targeting solvers.
	bool hasLunarApproach = false;
	double lunarApproachTime = 0;
	double approachPeriRadius = 0;
	bool hasReturnPerigee = false;   // perigee after the lunar encounter
	double perigeeTime = 0, perigeeAltitude = 0;
};

Prediction Predict(const Model& model, const VehicleState& from,
	const FlightPlan& plan, double horizon, bool keepSamples = true,
	bool stopAtMoon = true);

// Formatting helpers.
std::string FormatGET(double seconds);     // "075:43:10"
bool ParseGET(const std::string& text, double* seconds);

// Initial state: circular parking orbit, 185 km (100 nmi).
VehicleState ParkingOrbit(const Model& model, double altitude = 185.2e3,
	double longitude = 0);

}  // namespace sim

#endif
