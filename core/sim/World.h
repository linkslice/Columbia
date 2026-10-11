// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The real-date world: Moon and Sun ephemerides (Meeus' truncated ELP-2000
// and solar theory), Earth rotation, the Moon's body-fixed frame (IAU),
// gravity with Earth J2 and third bodies, the atmosphere, and lunar terrain.
//
// Frame: Earth-centred, mean equator and equinox of date (the Apollo
// computers' "basic reference coordinate system" was the mean equator and
// equinox of the mission year, which this matches to well under an arc
// minute). Time: Julian date (UTC is used as TT; under a minute apart).

#ifndef COLUMBIA_WORLD_H
#define COLUMBIA_WORLD_H

#include "Math.h"

#include <cstdint>
#include <string>
#include <vector>

namespace sim {

namespace world {

constexpr double kMuEarth = 3.986004418e14;
constexpr double kMuMoon = 4.9048695e12;
constexpr double kMuSun = 1.32712440018e20;
constexpr double kREarth = 6378137.0;
constexpr double kRMoon = 1737400.0;
constexpr double kJ2Earth = 1.08262668e-3;
constexpr double kEarthRate = 7.2921158553e-5;   // rad/s
constexpr double kMoonSOI = 66100.0e3;
constexpr double kAU = 1.495978707e11;
constexpr double kG0 = 9.80665;
constexpr double kEntryInterface = 121920.0;      // 400,000 ft

// Julian date from a UTC calendar date and time.
double JulianDate(int year, int month, int day, int hour = 0, int minute = 0,
	double second = 0);
std::string FormatDate(double jd);   // "1969-07-20 20:17:40 UTC"

struct Ephemeris {
	// Positions (m) and velocities (m/s) relative to the Earth.
	static Vec3 MoonPosition(double jd);
	static Vec3 MoonVelocity(double jd);
	static Vec3 SunPosition(double jd);

	// Greenwich mean sidereal angle (rad).
	static double GMST(double jd);
	// Earth-fixed -> inertial (mean of date; nutation ignored).
	static Mat3 EarthFixedToInertial(double jd);
	// Moon body-fixed (mean Earth / polar axis) -> inertial (IAU model).
	static Mat3 MoonFixedToInertial(double jd);
};

// Geodetic-ish helpers (spherical Earth and Moon for positions).
Vec3 LatLonToVector(double latDeg, double lonDeg, double radius);
void VectorToLatLon(const Vec3& v, double* latDeg, double* lonDeg,
	double* radius);

struct Gravity {
	// Acceleration in the Earth-centred inertial frame, including the
	// Earth's J2, the Moon and the Sun (with indirect terms).
	static Vec3 Acceleration(const Vec3& r, double jd, bool includeSun = true);
};

struct Atmosphere {
	double density;      // kg/m^3
	double pressure;     // Pa
	double temperature;  // K
	double soundSpeed;   // m/s

	static Atmosphere At(double altitude);
};

// Lunar terrain around the Apollo sites: a smooth reference sphere with
// procedural craters, ridges and boulder fields that are deterministic
// for a given location, plus hand-placed features at the landing sites.
class Terrain {
public:
	struct Feature {
		double lat, lon;     // degrees
		double diameter;     // m
		double depthRatio;   // depth / diameter (craters) or <0 for hills
		bool boulders;       // strewn with large boulders
		std::string name;
	};

	// Local datum: radius of the mean surface near the site (m).
	void SetDatum(double latDeg, double lonDeg, double radius);
	void AddFeature(const Feature& feature);
	void ClearFeatures();

	// Height above kRMoon at a selenographic location (m).
	double Height(double latDeg, double lonDeg) const;
	// Height of the largest boulder near the location (m), 0 if none.
	double BoulderHeight(double latDeg, double lonDeg) const;
	// Surface slope in degrees.
	double Slope(double latDeg, double lonDeg) const;
	// Radius of the surface (m from the Moon's centre).
	double SurfaceRadius(double latDeg, double lonDeg) const
		{ return kRMoon + Height(latDeg, lonDeg); }

	const std::vector<Feature>& Features() const { return fFeatures; }

private:
	double _Datum(double latDeg, double lonDeg) const;
	double _Craters(double x, double y, double cell, double minD,
		double maxD, uint32_t seed) const;

	double fDatumLat = 0, fDatumLon = 0, fDatumRadius = kRMoon;
	std::vector<Feature> fFeatures;
};

}  // namespace world
}  // namespace sim

#endif
