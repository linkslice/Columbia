// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The lunar surface as seen by radars, landing gear and cameras: converts
// between the Earth-centred inertial frame and Moon-fixed coordinates, and
// queries the terrain.

#ifndef COLUMBIA_MOONSCENE_H
#define COLUMBIA_MOONSCENE_H

#include "World.h"
#include "../gnc/LMPGNCS.h"

namespace sim {

class MoonScene : public gnc::RadarScene {
public:
	explicit MoonScene(const world::Terrain& terrain) : fTerrain(terrain) {}

	void SetTime(double jd);
	double Time() const { return fJD; }

	// Frames.
	const Vec3& MoonPosition() const { return fMoonPos; }
	const Vec3& MoonVelocity() const { return fMoonVel; }
	const Mat3& FixedToInertial() const { return fFixedToInertial; }
	Vec3 ToFixed(const Vec3& inertial) const;        // position
	Vec3 FromFixed(const Vec3& fixed) const;         // position
	void LatLon(const Vec3& inertial, double* lat, double* lon,
		double* radius) const;

	// Height above the terrain directly below an inertial point (m).
	double Altitude(const Vec3& inertial) const;
	double SurfaceRadiusBelow(const Vec3& inertial) const;
	Vec3 LocalVertical(const Vec3& inertial) const;   // unit, outward

	// gnc::RadarScene
	bool SlantRange(const Vec3& from, const Vec3& direction, double maxRange,
		double* range) const override;
	Vec3 SurfaceVelocity(const Vec3& point) const override;

	const world::Terrain& Terrain() const { return fTerrain; }

private:
	const world::Terrain& fTerrain;
	double fJD = 0;
	Vec3 fMoonPos, fMoonVel;
	Mat3 fFixedToInertial;
	Vec3 fOmega;   // Moon's spin, inertial (rad/s)
};

}  // namespace sim

#endif
