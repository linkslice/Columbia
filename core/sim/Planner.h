// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Maneuver targeting, in the spirit of the ground's RTCC: each helper
// proposes one burn by shooting with the same propagator the flight uses.

#ifndef COLUMBIA_PLANNER_H
#define COLUMBIA_PLANNER_H

#include "Flight.h"

#include <string>

namespace sim {

struct PlanResult {
	bool ok = false;
	Burn burn;
	std::string message;
};

class Planner {
public:
	// `plan` holds the burns already scheduled; helpers keep those that
	// happen before the burn they compute and ignore later ones.
	Planner(const Model& model, const VehicleState& now,
		const FlightPlan& plan);

	// Free-return translunar injection with the S-IVB.
	PlanResult TransLunarInjection(double pericynthionAltitude = 110e3,
		double perigeeAltitude = 40e3);
	// Retrograde SPS burn at the next pericynthion into a circular orbit.
	PlanResult LunarOrbitInsertion();
	// Cheapest prograde SPS burn from lunar orbit back to the corridor.
	PlanResult TransEarthInjection(double perigeeAltitude = 40e3,
		double maxCoast = 66 * 3600.0);
	// Mid-course corrections at time `at`.
	PlanResult PericynthionCorrection(double at,
		double pericynthionAltitude = 110e3);
	PlanResult EntryCorrection(double at, double perigeeAltitude = 40e3);

	// Smallest lead time before a planned ignition (s).
	static constexpr double kLeadTime = 600.0;

private:
	FlightPlan _Before(double t) const;
	uint32_t _NextId() const;
	double _Earliest() const;

	const Model& fModel;
	VehicleState fNow;
	FlightPlan fPlan;
};

}  // namespace sim

#endif
