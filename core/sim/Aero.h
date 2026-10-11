// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Aerodynamics: plain drag for the launch vehicle and jettisoned stages,
// and the Command Module's lifting entry. The CM's offset centre of
// gravity trims it, heat shield first, at about -20 degrees angle of
// attack, where it develops a lift-to-drag ratio of about 0.3 in the plane
// normal to its Y axis; rolling about the velocity vector points that lift.
// The conventions match Colossus/Comanche's CM/POSE, so the angles the CMC
// computes are the ones the model flies.

#ifndef COLUMBIA_AERO_H
#define COLUMBIA_AERO_H

#include "Vehicle.h"

namespace sim {

struct EntryAeroState {
	double mach = 0;
	double dynamicPressure = 0;   // Pa
	double alpha = 0;             // rad (CMC convention)
	double roll = 0;              // rad: lift-vector bank, 0 = lift up
	double gLoad = 0;             // non-gravitational acceleration, g
	double heatRate = 0;          // W/m^2 at the stagnation point
};

// Applies drag (and, for the CM, lift and trim moments) for this step.
// `airVelocity` is the vehicle's velocity relative to the atmosphere,
// `up` the local vertical. Returns what a display or the heat-shield
// model wants to know.
EntryAeroState ApplyAerodynamics(Vehicle& vehicle, const Vec3& airVelocity,
	const Vec3& up, double altitude, bool commandModule);

}  // namespace sim

#endif
