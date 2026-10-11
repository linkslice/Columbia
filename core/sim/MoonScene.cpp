// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "MoonScene.h"

#include <cmath>

namespace sim {

void
MoonScene::SetTime(double jd)
{
	if (jd == fJD)
		return;
	fJD = jd;
	fMoonPos = world::Ephemeris::MoonPosition(jd);
	fMoonVel = world::Ephemeris::MoonVelocity(jd);
	fFixedToInertial = world::Ephemeris::MoonFixedToInertial(jd);
	// The Moon's spin axis is the fixed frame's z axis; one sidereal
	// rotation per 27.321661 days.
	double rate = 2 * 3.14159265358979323846 / (27.321661 * 86400.0);
	fOmega = fFixedToInertial.Column(2) * rate;
}


Vec3
MoonScene::ToFixed(const Vec3& inertial) const
{
	return fFixedToInertial.Transposed() * (inertial - fMoonPos);
}


Vec3
MoonScene::FromFixed(const Vec3& fixed) const
{
	return fMoonPos + fFixedToInertial * fixed;
}


void
MoonScene::LatLon(const Vec3& inertial, double* lat, double* lon,
	double* radius) const
{
	world::VectorToLatLon(ToFixed(inertial), lat, lon, radius);
}


double
MoonScene::SurfaceRadiusBelow(const Vec3& inertial) const
{
	double lat, lon, r;
	LatLon(inertial, &lat, &lon, &r);
	return fTerrain.SurfaceRadius(lat, lon);
}


double
MoonScene::Altitude(const Vec3& inertial) const
{
	double lat, lon, r;
	LatLon(inertial, &lat, &lon, &r);
	return r - fTerrain.SurfaceRadius(lat, lon);
}


Vec3
MoonScene::LocalVertical(const Vec3& inertial) const
{
	return (inertial - fMoonPos).Unit();
}


bool
MoonScene::SlantRange(const Vec3& from, const Vec3& direction,
	double maxRange, double* range) const
{
	Vec3 d = direction.Unit();
	// The beam must point at least somewhat downward.
	if (d.Dot(LocalVertical(from)) > -0.05)
		return false;
	double step = std::max(5.0, Altitude(from) * 0.02);
	double prev = 0;
	double prevAlt = Altitude(from);
	if (prevAlt <= 0) {
		*range = 0;
		return true;
	}
	for (double s = step; s <= maxRange; s += step) {
		double alt = Altitude(from + d * s);
		if (alt <= 0) {
			// Bisect between prev and s.
			double lo = prev, hi = s;
			for (int i = 0; i < 30; i++) {
				double mid = 0.5 * (lo + hi);
				if (Altitude(from + d * mid) > 0)
					lo = mid;
				else
					hi = mid;
			}
			*range = 0.5 * (lo + hi);
			return true;
		}
		prev = s;
		prevAlt = alt;
		step = std::max(2.0, alt * 0.25);
	}
	return false;
}


Vec3
MoonScene::SurfaceVelocity(const Vec3& point) const
{
	return fMoonVel + fOmega.Cross(point - fMoonPos);
}

}  // namespace sim
