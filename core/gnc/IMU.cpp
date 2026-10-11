// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "IMU.h"

#include <cmath>

namespace gnc {

using sim::Mat3;
using sim::Quat;
using sim::Vec3;

namespace {

const double kTwoPi = 6.28318530717958647692;

}  // namespace


uint16_t
AngleToCDU(double radians)
{
	double revs = radians / kTwoPi;
	revs -= std::floor(revs);
	long counts = std::lround(revs * 32768.0) & 077777;
	return uint16_t(counts);
}


double
CDUToAngle(uint16_t counts)
{
	int c = counts & 077777;
	if (c >= 040000)
		c -= 0100000;
	return c * kTwoPi / 32768.0;
}


IMU::IMU()
{
	platform = Mat3::Identity();
}


Vec3
IMU::Gimbals(const Quat& bodyToInertial) const
{
	// M = (body->inertial)^T * (SM->inertial): SM coordinates -> NB.
	// M = Rx(outer) * Rz(middle) * Ry(inner) as frame rotations.
	Mat3 B = bodyToInertial.ToMatrix();
	Mat3 M = B.Transposed() * platform;
	double middle = std::asin(sim::Clamp(M.m[0][1], -1, 1));
	double inner = std::atan2(-M.m[0][2], M.m[0][0]);
	double outer = std::atan2(-M.m[2][1], M.m[1][1]);
	return Vec3(outer, inner, middle);
}


void
IMU::SetGimbals(const Quat& bodyToInertial, const Vec3& g)
{
	Mat3 M = Mat3::RotX(g.x) * Mat3::RotZ(g.z) * Mat3::RotY(g.y);
	// SM->inertial = B * M.
	platform = (bodyToInertial.ToMatrix() * M).Orthonormalized().Transposed()
		.Orthonormalized().Transposed();
}


void
IMU::Torque(int axis, double angle)
{
	// Rotate the platform about its own axis by `angle` (active).
	Vec3 a = platform.Column(axis);
	Quat q = Quat::AxisAngle(a, angle);
	Mat3 R = q.ToMatrix();
	platform = R * platform;
}


bool
IMU::Update(const Quat& bodyToInertial, double dt)
{
	(void)dt;
	if (!powered || failed)
		return false;
	Vec3 g = Gimbals(bodyToInertial);
	fMiddle = g.z;
	// Hardware gimbal lock: the middle gimbal reaching 90 degrees aligns
	// the outer and inner axes and the platform can no longer hold its
	// inertial orientation; it is dragged along and alignment is lost.
	if (std::fabs(g.z) > 89.0 * sim::kDeg && !caged) {
		tumbled = true;
		Vec3 limited = g;
		limited.z = (g.z > 0 ? 1 : -1) * 85.0 * sim::kDeg;
		SetGimbals(bodyToInertial, limited);
	}
	if (caged)
		SetGimbals(bodyToInertial, Vec3());
	return std::fabs(g.z) > 70.0 * sim::kDeg;
}


void
IMU::Sense(const Vec3& deltaV, double pulseSize, int counts[3])
{
	Vec3 sm = InertialToPlatform(deltaV);
	if (!powered || failed || caged)
		sm = Vec3();
	fResidual += sm;
	double* r[3] = {&fResidual.x, &fResidual.y, &fResidual.z};
	for (int i = 0; i < 3; i++) {
		double n = std::trunc(*r[i] / pulseSize);
		counts[i] = int(n);
		*r[i] -= n * pulseSize;
	}
}

}  // namespace gnc
