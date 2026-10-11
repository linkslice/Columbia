// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "World.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sim {
namespace world {

namespace {

const double kPi = 3.14159265358979323846;

double
Rad(double deg)
{
	return deg * kPi / 180.0;
}

double
Norm360(double deg)
{
	deg = std::fmod(deg, 360.0);
	return deg < 0 ? deg + 360.0 : deg;
}

// Meeus, Astronomical Algorithms, tables 47.A and 47.B (largest terms).
struct LunarTerm {
	signed char d, m, mp, f;
	double l, r;
};

const LunarTerm kLongitudeDistance[] = {
	{0, 0, 1, 0, 6288774, -20905355}, {2, 0, -1, 0, 1274027, -3699111},
	{2, 0, 0, 0, 658314, -2955968}, {0, 0, 2, 0, 213618, -569925},
	{0, 1, 0, 0, -185116, 48888}, {0, 0, 0, 2, -114332, -3149},
	{2, 0, -2, 0, 58793, 246158}, {2, -1, -1, 0, 57066, -152138},
	{2, 0, 1, 0, 53322, -170733}, {2, -1, 0, 0, 45758, -204586},
	{0, 1, -1, 0, -40923, -129620}, {1, 0, 0, 0, -34720, 108743},
	{0, 1, 1, 0, -30383, 104755}, {2, 0, 0, -2, 15327, 10321},
	{0, 0, 1, 2, -12528, 0}, {0, 0, 1, -2, 10980, 79661},
	{4, 0, -1, 0, 10675, -34782}, {0, 0, 3, 0, 10034, -23210},
	{4, 0, -2, 0, 8548, -21636}, {2, 1, -1, 0, -7888, 24208},
	{2, 1, 0, 0, -6766, 30824}, {1, 0, -1, 0, -5163, -8379},
	{1, 1, 0, 0, 4987, -16675}, {2, -1, 1, 0, 4036, -12831},
	{2, 0, 2, 0, 3994, -10445}, {4, 0, 0, 0, 3861, -11650},
	{2, 0, -3, 0, 3665, 14403}, {0, 1, -2, 0, -2689, -7003},
	{2, 0, -1, 2, -2602, 0}, {2, -1, -2, 0, 2390, 10056},
	{1, 0, 1, 0, -2348, 6322}, {2, -2, 0, 0, 2236, -9884},
	{0, 1, 2, 0, -2120, 5751}, {0, 2, 0, 0, -2069, 0},
	{2, -2, -1, 0, 2048, -4950}, {2, 0, 1, -2, -1773, 4130},
	{2, 0, 0, 2, -1595, 0}, {4, -1, -1, 0, 1215, -3958},
	{0, 0, 2, 2, -1110, 0}, {3, 0, -1, 0, -892, 3258},
};

const LunarTerm kLatitude[] = {
	{0, 0, 0, 1, 5128122, 0}, {0, 0, 1, 1, 280602, 0},
	{0, 0, 1, -1, 277693, 0}, {2, 0, 0, -1, 173237, 0},
	{2, 0, -1, 1, 55413, 0}, {2, 0, -1, -1, 46271, 0},
	{2, 0, 0, 1, 32573, 0}, {0, 0, 2, 1, 17198, 0},
	{2, 0, 1, -1, 9266, 0}, {0, 0, 2, -1, 8822, 0},
	{2, -1, 0, -1, 8216, 0}, {2, 0, -2, -1, 4324, 0},
	{2, 0, 1, 1, 4200, 0}, {2, 1, 0, -1, -3359, 0},
	{2, -1, -1, 1, 2463, 0}, {2, -1, 0, 1, 2211, 0},
	{2, -1, -1, -1, 2065, 0}, {0, 1, -1, -1, -1870, 0},
	{4, 0, -1, -1, 1828, 0}, {0, 1, 0, 1, -1794, 0},
};

// Mean obliquity of the ecliptic of date (degrees).
double
Obliquity(double T)
{
	return 23.4392911 - (46.8150 * T + 0.00059 * T * T
		- 0.001813 * T * T * T) / 3600.0;
}

// Ecliptic of date -> mean equator and equinox of date.
Vec3
EclipticToEquatorial(double lonDeg, double latDeg, double distance, double T)
{
	double lon = Rad(lonDeg);
	double lat = Rad(latDeg);
	Vec3 ecl(distance * std::cos(lat) * std::cos(lon),
		distance * std::cos(lat) * std::sin(lon), distance * std::sin(lat));
	double e = Rad(Obliquity(T));
	return Vec3(ecl.x, ecl.y * std::cos(e) - ecl.z * std::sin(e),
		ecl.y * std::sin(e) + ecl.z * std::cos(e));
}

// IAU 1976 precession: J2000 -> mean equator and equinox of date.
Mat3
Precession(double T)
{
	double as = 3.14159265358979323846 / (180.0 * 3600.0);
	double zeta = (2306.2181 * T + 0.30188 * T * T + 0.017998 * T * T * T) * as;
	double z = (2306.2181 * T + 1.09468 * T * T + 0.018203 * T * T * T) * as;
	double theta = (2004.3109 * T - 0.42665 * T * T - 0.041833 * T * T * T)
		* as;
	auto rz = [](double a) {
		double c = std::cos(a), s = std::sin(a);
		return Mat3::Rows({c, -s, 0}, {s, c, 0}, {0, 0, 1});
	};
	auto ry = [](double a) {
		double c = std::cos(a), s = std::sin(a);
		return Mat3::Rows({c, 0, s}, {0, 1, 0}, {-s, 0, c});
	};
	return rz(z) * ry(-theta) * rz(zeta);
}

uint32_t
Hash(int32_t a, int32_t b, uint32_t seed)
{
	uint32_t h = seed * 0x9E3779B1u;
	h ^= uint32_t(a) * 0x85EBCA77u;
	h = (h << 13) | (h >> 19);
	h ^= uint32_t(b) * 0xC2B2AE3Du;
	h ^= h >> 16;
	h *= 0x7FEB352Du;
	h ^= h >> 15;
	h *= 0x846CA68Bu;
	h ^= h >> 16;
	return h;
}

double
Unit(uint32_t& state)
{
	state = state * 1664525u + 1013904223u;
	return (state >> 8) / double(1 << 24);
}

double
CraterProfile(double r, double diameter, double depthRatio)
{
	// r: distance from centre / radius.
	double depth = depthRatio * diameter;
	double rim = 0.04 * diameter;
	if (r < 1.0)
		return -depth + (depth + rim) * r * r;
	if (r < 1.6) {
		double t = (1.6 - r) / 0.6;
		return rim * t * t;
	}
	return 0;
}

}  // namespace


double
JulianDate(int year, int month, int day, int hour, int minute, double second)
{
	if (month <= 2) {
		year -= 1;
		month += 12;
	}
	int a = year / 100;
	int b = 2 - a + a / 4;
	double jd = std::floor(365.25 * (year + 4716))
		+ std::floor(30.6001 * (month + 1)) + day + b - 1524.5;
	return jd + (hour + minute / 60.0 + second / 3600.0) / 24.0;
}


std::string
FormatDate(double jd)
{
	double z = std::floor(jd + 0.5);
	double f = jd + 0.5 - z;
	double alpha = std::floor((z - 1867216.25) / 36524.25);
	double a = z + 1 + alpha - std::floor(alpha / 4);
	double b = a + 1524;
	double c = std::floor((b - 122.1) / 365.25);
	double d = std::floor(365.25 * c);
	double e = std::floor((b - d) / 30.6001);
	int day = int(b - d - std::floor(30.6001 * e));
	int month = int(e < 14 ? e - 1 : e - 13);
	int year = int(month > 2 ? c - 4716 : c - 4715);
	int seconds = int(std::floor(f * 86400 + 0.5));
	char text[64];
	std::snprintf(text, sizeof(text), "%04d-%02d-%02d %02d:%02d:%02d UTC", year,
		month, day, seconds / 3600, (seconds / 60) % 60, seconds % 60);
	return text;
}


Vec3
Ephemeris::MoonPosition(double jd)
{
	double T = (jd - 2451545.0) / 36525.0;
	double Lp = Norm360(218.3164477 + 481267.88123421 * T);
	double D = Norm360(297.8501921 + 445267.1114034 * T);
	double M = Norm360(357.5291092 + 35999.0502909 * T);
	double Mp = Norm360(134.9633964 + 477198.8675055 * T);
	double F = Norm360(93.2720950 + 483202.0175233 * T);
	double E = 1 - 0.002516 * T - 0.0000074 * T * T;

	double sl = 0, sr = 0, sb = 0;
	for (const LunarTerm& t : kLongitudeDistance) {
		double arg = Rad(t.d * D + t.m * M + t.mp * Mp + t.f * F);
		double ecc = t.m == 0 ? 1 : (std::abs(t.m) == 1 ? E : E * E);
		sl += t.l * ecc * std::sin(arg);
		sr += t.r * ecc * std::cos(arg);
	}
	for (const LunarTerm& t : kLatitude) {
		double arg = Rad(t.d * D + t.m * M + t.mp * Mp + t.f * F);
		double ecc = t.m == 0 ? 1 : (std::abs(t.m) == 1 ? E : E * E);
		sb += t.l * ecc * std::sin(arg);
	}
	// Additive terms (Venus, Jupiter, flattening).
	double A1 = Norm360(119.75 + 131.849 * T);
	double A2 = Norm360(53.09 + 479264.290 * T);
	double A3 = Norm360(313.45 + 481266.484 * T);
	sl += 3958 * std::sin(Rad(A1)) + 1962 * std::sin(Rad(Lp - F))
		+ 318 * std::sin(Rad(A2));
	sb += -2235 * std::sin(Rad(Lp)) + 382 * std::sin(Rad(A3))
		+ 175 * std::sin(Rad(A1 - F)) + 175 * std::sin(Rad(A1 + F))
		+ 127 * std::sin(Rad(Lp - Mp)) - 115 * std::sin(Rad(Lp + Mp));

	double lon = Lp + sl / 1e6;
	double lat = sb / 1e6;
	double distance = (385000.56 + sr / 1000.0) * 1000.0;
	return EclipticToEquatorial(lon, lat, distance, T);
}


Vec3
Ephemeris::MoonVelocity(double jd)
{
	const double h = 60.0 / 86400.0;
	return (MoonPosition(jd + h) - MoonPosition(jd - h)) / 120.0;
}


Vec3
Ephemeris::SunPosition(double jd)
{
	double T = (jd - 2451545.0) / 36525.0;
	double L0 = Norm360(280.46646 + 36000.76983 * T);
	double M = Norm360(357.52911 + 35999.05029 * T);
	double C = (1.914602 - 0.004817 * T) * std::sin(Rad(M))
		+ 0.019993 * std::sin(Rad(2 * M)) + 0.000289 * std::sin(Rad(3 * M));
	double lon = L0 + C;
	double v = M + C;
	double e = 0.016708634 - 0.000042037 * T;
	double R = 1.000001018 * (1 - e * e) / (1 + e * std::cos(Rad(v)));
	return EclipticToEquatorial(lon, 0, R * kAU, T);
}


double
Ephemeris::GMST(double jd)
{
	double d = jd - 2451545.0;
	return Rad(Norm360(280.46061837 + 360.98564736629 * d));
}


Mat3
Ephemeris::EarthFixedToInertial(double jd)
{
	// Columns: Earth-fixed axes expressed in inertial coordinates.
	double g = GMST(jd);
	double c = std::cos(g), s = std::sin(g);
	return Mat3::Rows({c, -s, 0}, {s, c, 0}, {0, 0, 1});
}


Mat3
Ephemeris::MoonFixedToInertial(double jd)
{
	// IAU 2009 lunar rotation, main terms.
	double d = jd - 2451545.0;
	double T = d / 36525.0;
	double E1 = Rad(125.045 - 0.0529921 * d);
	double E2 = Rad(250.089 - 0.1059842 * d);
	double E3 = Rad(260.008 + 13.0120009 * d);
	double alpha = 269.9949 + 0.0031 * T - 3.8787 * std::sin(E1)
		- 0.1204 * std::sin(E2) + 0.0700 * std::sin(E3);
	double delta = 66.5392 + 0.0130 * T + 1.5419 * std::cos(E1)
		+ 0.0239 * std::cos(E2) - 0.0278 * std::cos(E3);
	double W = 38.3213 + 13.17635815 * d - 1.4e-12 * d * d
		+ 3.5610 * std::sin(E1) + 0.1208 * std::sin(E2)
		- 0.0642 * std::sin(E3);
	// Body-fixed -> inertial = Rz(alpha+90) * Rx(90-delta) * Rz(W) (active).
	auto rz = [](double a) {
		double c = std::cos(a), s = std::sin(a);
		return Mat3::Rows({c, -s, 0}, {s, c, 0}, {0, 0, 1});
	};
	auto rx = [](double a) {
		double c = std::cos(a), s = std::sin(a);
		return Mat3::Rows({1, 0, 0}, {0, c, -s}, {0, s, c});
	};
	// IAU angles are referred to J2000; carry them to the frame of date.
	return Precession(T) * rz(Rad(alpha + 90)) * rx(Rad(90 - delta))
		* rz(Rad(W));
}


Vec3
LatLonToVector(double latDeg, double lonDeg, double radius)
{
	double lat = Rad(latDeg), lon = Rad(lonDeg);
	return Vec3(radius * std::cos(lat) * std::cos(lon),
		radius * std::cos(lat) * std::sin(lon), radius * std::sin(lat));
}


void
VectorToLatLon(const Vec3& v, double* latDeg, double* lonDeg, double* radius)
{
	double r = v.Length();
	*radius = r;
	*latDeg = std::asin(Clamp(v.z / r, -1, 1)) * 180 / kPi;
	*lonDeg = std::atan2(v.y, v.x) * 180 / kPi;
}


// ---------------------------------------------------------------------------
// Gravity


Vec3
Gravity::Acceleration(const Vec3& r, double jd, bool includeSun)
{
	double rl = r.Length();
	double r2 = rl * rl;
	// Earth with J2 (z is the pole in J2000 to within a fraction of a
	// degree, close enough for zonal gravity).
	double zr = r.z / rl;
	double factor = 1.5 * kJ2Earth * (kREarth * kREarth) / r2;
	double common = -kMuEarth / (r2 * rl);
	Vec3 a(r.x * common * (1 + factor * (1 - 5 * zr * zr)),
		r.y * common * (1 + factor * (1 - 5 * zr * zr)),
		r.z * common * (1 + factor * (3 - 5 * zr * zr)));

	Vec3 rm = Ephemeris::MoonPosition(jd);
	Vec3 d = r - rm;
	double dl = d.Length(), rml = rm.Length();
	a -= d * (kMuMoon / (dl * dl * dl));
	a -= rm * (kMuMoon / (rml * rml * rml));

	if (includeSun) {
		Vec3 rs = Ephemeris::SunPosition(jd);
		Vec3 ds = r - rs;
		double dsl = ds.Length(), rsl = rs.Length();
		a -= ds * (kMuSun / (dsl * dsl * dsl));
		a -= rs * (kMuSun / (rsl * rsl * rsl));
	}
	return a;
}


// ---------------------------------------------------------------------------
// Atmosphere (US Standard 1976 layers, extended exponentially above 86 km)


Atmosphere
Atmosphere::At(double h)
{
	struct Layer { double base, temperature, lapse, pressure; };
	static const Layer layers[] = {
		{0, 288.15, -0.0065, 101325.0},
		{11000, 216.65, 0.0, 22632.06},
		{20000, 216.65, 0.001, 5474.889},
		{32000, 228.65, 0.0028, 868.0187},
		{47000, 270.65, 0.0, 110.9063},
		{51000, 270.65, -0.0028, 66.93887},
		{71000, 214.65, -0.002, 3.956420},
		{86000, 186.87, 0.0, 0.3734},
	};
	const double R = 287.053, g = 9.80665;
	Atmosphere a;
	if (h < 0)
		h = 0;
	if (h > 86000) {
		a.temperature = 186.87;
		a.pressure = 0.3734 * std::exp(-(h - 86000) / 6000.0);
		if (h > 400000)
			a.pressure = 0;
	} else {
		int i = 7;
		while (i > 0 && h < layers[i].base)
			i--;
		const Layer& L = layers[i];
		double dh = h - L.base;
		if (L.lapse == 0) {
			a.temperature = L.temperature;
			a.pressure = L.pressure * std::exp(-g * dh / (R * L.temperature));
		} else {
			a.temperature = L.temperature + L.lapse * dh;
			a.pressure = L.pressure * std::pow(a.temperature / L.temperature,
				-g / (L.lapse * R));
		}
	}
	a.density = a.pressure / (R * a.temperature);
	a.soundSpeed = std::sqrt(1.4 * R * a.temperature);
	return a;
}


// ---------------------------------------------------------------------------
// Terrain


void
Terrain::SetDatum(double latDeg, double lonDeg, double radius)
{
	fDatumLat = latDeg;
	fDatumLon = lonDeg;
	fDatumRadius = radius;
}


void
Terrain::AddFeature(const Feature& feature)
{
	fFeatures.push_back(feature);
}


void
Terrain::ClearFeatures()
{
	fFeatures.clear();
}


double
Terrain::_Datum(double latDeg, double lonDeg) const
{
	// The site datum covers a mare-sized plain (several hundred km, so the
	// whole powered descent flies over it) and blends into the mean sphere
	// beyond, with a gentle regional undulation.
	double dlat = (latDeg - fDatumLat) * kRMoon * kPi / 180;
	double dlon = (lonDeg - fDatumLon) * kRMoon * kPi / 180
		* std::cos(Rad(fDatumLat));
	double dist = std::sqrt(dlat * dlat + dlon * dlon);
	double q = dist / 650e3;
	double blend = std::exp(-q * q * q * q);
	double base = (fDatumRadius - kRMoon) * blend;
	double x = lonDeg * kRMoon * kPi / 180 * std::cos(Rad(latDeg));
	double y = latDeg * kRMoon * kPi / 180;
	base += 35 * std::sin(x / 9100.0) * std::cos(y / 7300.0)
		+ 18 * std::sin(x / 3100.0 + 1.3) * std::sin(y / 4300.0 + 0.4);
	return base;
}


double
Terrain::_Craters(double x, double y, double cell, double minD, double maxD,
	uint32_t seed) const
{
	double z = 0;
	int32_t ci = int32_t(std::floor(x / cell));
	int32_t cj = int32_t(std::floor(y / cell));
	for (int32_t i = ci - 1; i <= ci + 1; i++) {
		for (int32_t j = cj - 1; j <= cj + 1; j++) {
			uint32_t state = Hash(i, j, seed);
			int count = int(Unit(state) * 2.2);
			for (int k = 0; k < count; k++) {
				double cx = (i + Unit(state)) * cell;
				double cy = (j + Unit(state)) * cell;
				double u = Unit(state);
				double d = minD * std::pow(maxD / minD, u * u);
				double depth = 0.12 + 0.08 * Unit(state);
				double r = std::hypot(x - cx, y - cy) / (d / 2);
				if (r < 1.6)
					z += CraterProfile(r, d, depth);
			}
		}
	}
	return z;
}


double
Terrain::Height(double latDeg, double lonDeg) const
{
	double x = lonDeg * kRMoon * kPi / 180 * std::cos(Rad(latDeg));
	double y = latDeg * kRMoon * kPi / 180;
	double h = _Datum(latDeg, lonDeg);
	h += _Craters(x, y, 1800, 180, 1100, 11);
	h += _Craters(x, y, 360, 25, 220, 23);
	h += _Craters(x, y, 70, 4, 40, 37);
	for (const Feature& f : fFeatures) {
		double fx = (lonDeg - f.lon) * kRMoon * kPi / 180
			* std::cos(Rad(latDeg));
		double fy = (latDeg - f.lat) * kRMoon * kPi / 180;
		double r = std::hypot(fx, fy) / (f.diameter / 2);
		if (f.depthRatio > 0) {
			if (r < 1.6)
				h += CraterProfile(r, f.diameter, f.depthRatio);
		} else if (r < 1.0) {
			// Hill or mountain: smooth cosine dome.
			h += -f.depthRatio * f.diameter
				* 0.5 * (1 + std::cos(r * 3.14159265358979323846));
		}
	}
	return h;
}


double
Terrain::BoulderHeight(double latDeg, double lonDeg) const
{
	double x = lonDeg * kRMoon * kPi / 180 * std::cos(Rad(latDeg));
	double y = latDeg * kRMoon * kPi / 180;
	// Base boulder density, raised near the rims of boulder-strewn
	// features (like West crater at Tranquility Base).
	double density = 0.015;
	for (const Feature& f : fFeatures) {
		if (!f.boulders)
			continue;
		double fx = (lonDeg - f.lon) * kRMoon * kPi / 180
			* std::cos(Rad(latDeg));
		double fy = (latDeg - f.lat) * kRMoon * kPi / 180;
		double r = std::hypot(fx, fy) / (f.diameter / 2);
		if (r < 3.0)
			density = std::max(density, 0.35 * std::exp(-std::fabs(r - 1.1)));
	}
	const double cell = 6.0;
	double tallest = 0;
	int32_t ci = int32_t(std::floor(x / cell));
	int32_t cj = int32_t(std::floor(y / cell));
	for (int32_t i = ci - 1; i <= ci + 1; i++) {
		for (int32_t j = cj - 1; j <= cj + 1; j++) {
			uint32_t state = Hash(i, j, 97);
			if (Unit(state) > density)
				continue;
			double bx = (i + Unit(state)) * cell;
			double by = (j + Unit(state)) * cell;
			double size = 0.3 + 2.2 * std::pow(Unit(state), 3);
			if (std::hypot(x - bx, y - by) < 4.5 + size)
				tallest = std::max(tallest, size);
		}
	}
	return tallest;
}


double
Terrain::Slope(double latDeg, double lonDeg) const
{
	const double step = 4.0;   // m
	double dlat = step / (kRMoon * kPi / 180);
	double dlon = dlat / std::max(0.05, std::cos(Rad(latDeg)));
	double hx = Height(latDeg, lonDeg + dlon) - Height(latDeg, lonDeg - dlon);
	double hy = Height(latDeg + dlat, lonDeg) - Height(latDeg - dlat, lonDeg);
	double grad = std::hypot(hx, hy) / (2 * step);
	return std::atan(grad) * 180 / kPi;
}

}  // namespace world
}  // namespace sim
