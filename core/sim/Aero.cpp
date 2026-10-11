// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Aero.h"

#include "World.h"

#include <cmath>

namespace sim {

namespace {

const double kTrimAlpha = -20.0 * kDeg;
const double kNominalLD = 0.30;
const double kCMNoseRadius = 4.69;    // heat-shield radius of curvature, m

double
AreaAndDrag(const Vehicle& v, double* cd)
{
	double area = 0, cdA = 0;
	for (const Module& m : v.Modules()) {
		if (m.referenceArea > area)
			area = m.referenceArea;
		if (m.referenceArea * m.dragCoefficient > cdA)
			cdA = m.referenceArea * m.dragCoefficient;
	}
	*cd = area > 0 ? cdA / area : 0;
	return area;
}

}  // namespace


EntryAeroState
ApplyAerodynamics(Vehicle& vehicle, const Vec3& airVelocity, const Vec3& up,
	double altitude, bool commandModule)
{
	EntryAeroState s;
	world::Atmosphere air = world::Atmosphere::At(altitude);
	double speed = airVelocity.Length();
	if (air.density <= 0 || speed < 1e-3)
		return s;
	double cd;
	double area = AreaAndDrag(vehicle, &cd);
	if (area <= 0)
		return s;
	double q = 0.5 * air.density * speed * speed;
	s.dynamicPressure = q;
	s.mach = speed / std::max(1.0, air.soundSpeed);
	Vec3 vhat = airVelocity / speed;
	Vec3 drag = vhat * (-q * area * cd);
	Vec3 force = drag;
	Vec3 torque;

	if (commandModule) {
		Vec3 xb = vehicle.ToInertial(Vec3(1, 0, 0));
		Vec3 yb = vehicle.ToInertial(Vec3(0, 1, 0));
		Vec3 zb = vehicle.ToInertial(Vec3(0, 0, 1));
		Vec3 ua = -vhat;                       // CMC's UXA
		Vec3 ul = ua.Cross(yb);                // lift direction (CM/POSE)
		double ulLen = ul.Length();
		if (ulLen > 1e-6) {
			ul = ul / ulLen;
			double alpha = std::atan2(-ul.Dot(xb), ul.Dot(zb));
			s.alpha = alpha;
			// Lift scales with how close the CM sits to its trim.
			double ld = kNominalLD * Clamp(std::sin(alpha) / std::sin(kTrimAlpha),
				-1.2, 1.2);
			force += ul * (ld * q * area * cd);
			// Roll of the lift vector from "up" about the velocity (CM/POSE's
			// trajectory triad: UYA = UR x UXA, UZA = UXA x UYA).
			Vec3 uya = up.Cross(ua);
			double ny = uya.Length();
			if (ny > 1e-6) {
				uya = uya / ny;
				Vec3 una = ua.Cross(uya);
				s.roll = std::atan2(-ul.Dot(uya), ul.Dot(una));
			}
			// Static stability about the trim attitude in pitch and yaw,
			// with aerodynamic damping: the desired X axis lies in the
			// lift plane at the trim angle.
			Vec3 xTrim = ua * std::cos(kTrimAlpha) - ul * std::sin(kTrimAlpha);
			Vec3 err = xb.Cross(xTrim);                // inertial
			Vec3 errBody = vehicle.ToBody(err);
			const double kStiff = 0.06, kDamp = 0.9;   // per unit q*S*d
			double scale = q * area * 3.9;             // 3.9 m = CM diameter
			Vec3 rates = vehicle.rates;
			// Roll about the X axis is essentially undamped aerodynamically;
			// the RCS does all the work there.
			torque = Vec3(0,
				(kStiff * errBody.y - kDamp * rates.y * 0.05) * scale,
				(kStiff * errBody.z - kDamp * rates.z * 0.05) * scale);
		}
		// Stagnation-point convective heating (Sutton-Graves).
		s.heatRate = 1.7415e-4 * std::sqrt(air.density / kCMNoseRadius)
			* speed * speed * speed;
	}
	vehicle.AddExternalForce(force, torque);
	s.gLoad = force.Length() / std::max(1.0, vehicle.Mass()) / 9.80665;
	return s;
}

}  // namespace sim
