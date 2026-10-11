// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The Apollo inertial measurement unit: a three-gimbal stable platform
// (gimbal order inner Y, middle Z, outer X, as the flight software's
// *SMNB* routine assumes), its CDUs, PIPA accelerometers, gyro torquing for
// fine alignment, coarse alignment through the CDU error counters, and
// gimbal lock.

#ifndef COLUMBIA_IMU_H
#define COLUMBIA_IMU_H

#include "../sim/Math.h"

#include <cstdint>

namespace gnc {

class IMU {
public:
	IMU();

	// Platform orientation: columns are the stable-member axes expressed
	// in the inertial frame. REFSMMAT is the transpose of this.
	sim::Mat3 platform;

	bool powered = true;
	bool caged = false;
	bool failed = false;          // stabilization loops lost
	bool cduFailed = false;
	bool tumbled = false;         // lost alignment through gimbal lock

	// Gimbal angles (rad): x = outer, y = inner, z = middle.
	sim::Vec3 Gimbals(const sim::Quat& bodyToInertial) const;
	// Re-orients the platform so the gimbals read `gimbals` at this
	// vehicle attitude (coarse alignment, caging).
	void SetGimbals(const sim::Quat& bodyToInertial, const sim::Vec3& gimbals);
	// Rotates the platform about one of its own axes (gyro torquing).
	void Torque(int axis, double angle);
	// Aligns the platform to a REFSMMAT (rows: SM axes in inertial).
	void Align(const sim::Mat3& refsmmat) { platform = refsmmat.Transposed(); }
	sim::Mat3 REFSMMAT() const { return platform.Transposed(); }

	// Vector conversions.
	sim::Vec3 InertialToPlatform(const sim::Vec3& v) const
		{ return platform.Transposed() * v; }

	// Called every step: tracks gimbal lock. Returns true if the middle
	// gimbal is in the warning region (> 70 degrees).
	bool Update(const sim::Quat& bodyToInertial, double dt);

	// Accelerometers: accumulate a sensed velocity change (inertial) and
	// return whole pulses per platform axis.
	void Sense(const sim::Vec3& deltaVInertial, double pulseSize,
		int counts[3]);

	double MiddleGimbal() const { return fMiddle; }

private:
	sim::Vec3 fResidual;
	double fMiddle = 0;
};

// CDU counts: 2^15 per revolution, two's complement.
uint16_t AngleToCDU(double radians);
double CDUToAngle(uint16_t counts);

}  // namespace gnc

#endif
