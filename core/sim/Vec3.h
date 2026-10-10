// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#ifndef COLUMBIA_VEC3_H
#define COLUMBIA_VEC3_H

#include <cmath>

namespace sim {

struct Vec3 {
	double x = 0, y = 0, z = 0;

	Vec3() = default;
	Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

	Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
	Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
	Vec3 operator-() const { return {-x, -y, -z}; }
	Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
	Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }

	double Dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
	Vec3 Cross(const Vec3& o) const
	{
		return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
	}
	double Length() const { return std::sqrt(Dot(*this)); }
	Vec3 Unit() const
	{
		double l = Length();
		return l > 0 ? *this / l : Vec3();
	}
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

}  // namespace sim

#endif
