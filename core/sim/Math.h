// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// 3x3 matrices and quaternions for attitude work.

#ifndef COLUMBIA_MATH_H
#define COLUMBIA_MATH_H

#include "Vec3.h"

#include <cmath>

namespace sim {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

struct Mat3 {
	// Row-major: m[row][column].
	double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	static Mat3 Identity() { return Mat3(); }
	static Mat3 Rows(const Vec3& a, const Vec3& b, const Vec3& c)
	{
		Mat3 r;
		r.m[0][0] = a.x; r.m[0][1] = a.y; r.m[0][2] = a.z;
		r.m[1][0] = b.x; r.m[1][1] = b.y; r.m[1][2] = b.z;
		r.m[2][0] = c.x; r.m[2][1] = c.y; r.m[2][2] = c.z;
		return r;
	}
	static Mat3 Columns(const Vec3& a, const Vec3& b, const Vec3& c)
	{
		return Rows(a, b, c).Transposed();
	}
	// Frame (passive) rotations: coordinates of a fixed vector in a frame
	// rotated by `a` radians about the axis.
	static Mat3 RotX(double a)
	{
		double c = std::cos(a), s = std::sin(a);
		return Rows({1, 0, 0}, {0, c, s}, {0, -s, c});
	}
	static Mat3 RotY(double a)
	{
		double c = std::cos(a), s = std::sin(a);
		return Rows({c, 0, -s}, {0, 1, 0}, {s, 0, c});
	}
	static Mat3 RotZ(double a)
	{
		double c = std::cos(a), s = std::sin(a);
		return Rows({c, s, 0}, {-s, c, 0}, {0, 0, 1});
	}

	Vec3 Row(int i) const { return {m[i][0], m[i][1], m[i][2]}; }
	Vec3 Column(int i) const { return {m[0][i], m[1][i], m[2][i]}; }

	Mat3 Transposed() const
	{
		Mat3 t;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				t.m[i][j] = m[j][i];
		return t;
	}
	Mat3 operator*(const Mat3& o) const
	{
		Mat3 r;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j]
					+ m[i][2] * o.m[2][j];
		return r;
	}
	Vec3 operator*(const Vec3& v) const
	{
		return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
			m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
			m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
	}
	// Re-orthonormalizes (Gram-Schmidt on rows).
	Mat3 Orthonormalized() const
	{
		Vec3 a = Row(0).Unit();
		Vec3 b = (Row(1) - a * a.Dot(Row(1))).Unit();
		Vec3 c = a.Cross(b);
		return Rows(a, b, c);
	}
};

// Unit quaternion. Represents the rotation taking body coordinates to the
// inertial frame: v_inertial = q.Rotate(v_body).
struct Quat {
	double w = 1, x = 0, y = 0, z = 0;

	Quat() = default;
	Quat(double w_, double x_, double y_, double z_)
		: w(w_), x(x_), y(y_), z(z_) {}

	static Quat AxisAngle(const Vec3& axis, double angle)
	{
		Vec3 u = axis.Unit();
		double s = std::sin(angle / 2);
		return {std::cos(angle / 2), u.x * s, u.y * s, u.z * s};
	}

	Quat operator*(const Quat& o) const
	{
		return {w * o.w - x * o.x - y * o.y - z * o.z,
			w * o.x + x * o.w + y * o.z - z * o.y,
			w * o.y - x * o.z + y * o.w + z * o.x,
			w * o.z + x * o.y - y * o.x + z * o.w};
	}
	Quat Conjugate() const { return {w, -x, -y, -z}; }
	Quat Normalized() const
	{
		double n = std::sqrt(w * w + x * x + y * y + z * z);
		return n > 0 ? Quat(w / n, x / n, y / n, z / n) : Quat();
	}
	Vec3 Rotate(const Vec3& v) const
	{
		Quat p(0, v.x, v.y, v.z);
		Quat r = (*this) * p * Conjugate();
		return {r.x, r.y, r.z};
	}
	// Matrix whose columns are the body axes in inertial coordinates
	// (body -> inertial).
	Mat3 ToMatrix() const
	{
		Mat3 r;
		r.m[0][0] = 1 - 2 * (y * y + z * z);
		r.m[0][1] = 2 * (x * y - w * z);
		r.m[0][2] = 2 * (x * z + w * y);
		r.m[1][0] = 2 * (x * y + w * z);
		r.m[1][1] = 1 - 2 * (x * x + z * z);
		r.m[1][2] = 2 * (y * z - w * x);
		r.m[2][0] = 2 * (x * z - w * y);
		r.m[2][1] = 2 * (y * z + w * x);
		r.m[2][2] = 1 - 2 * (x * x + y * y);
		return r;
	}
	static Quat FromMatrix(const Mat3& a)
	{
		double tr = a.m[0][0] + a.m[1][1] + a.m[2][2];
		Quat q;
		if (tr > 0) {
			double s = std::sqrt(tr + 1.0) * 2;
			q = {0.25 * s, (a.m[2][1] - a.m[1][2]) / s,
				(a.m[0][2] - a.m[2][0]) / s, (a.m[1][0] - a.m[0][1]) / s};
		} else if (a.m[0][0] > a.m[1][1] && a.m[0][0] > a.m[2][2]) {
			double s = std::sqrt(1.0 + a.m[0][0] - a.m[1][1] - a.m[2][2]) * 2;
			q = {(a.m[2][1] - a.m[1][2]) / s, 0.25 * s,
				(a.m[0][1] + a.m[1][0]) / s, (a.m[0][2] + a.m[2][0]) / s};
		} else if (a.m[1][1] > a.m[2][2]) {
			double s = std::sqrt(1.0 + a.m[1][1] - a.m[0][0] - a.m[2][2]) * 2;
			q = {(a.m[0][2] - a.m[2][0]) / s, (a.m[0][1] + a.m[1][0]) / s,
				0.25 * s, (a.m[1][2] + a.m[2][1]) / s};
		} else {
			double s = std::sqrt(1.0 + a.m[2][2] - a.m[0][0] - a.m[1][1]) * 2;
			q = {(a.m[1][0] - a.m[0][1]) / s, (a.m[0][2] + a.m[2][0]) / s,
				(a.m[1][2] + a.m[2][1]) / s, 0.25 * s};
		}
		return q.Normalized();
	}
	// Integrates a body-frame angular velocity over dt.
	Quat Integrated(const Vec3& omegaBody, double dt) const
	{
		double rate = omegaBody.Length();
		if (rate * dt < 1e-12)
			return *this;
		return ((*this) * AxisAngle(omegaBody, rate * dt)).Normalized();
	}
};

inline double Clamp(double v, double lo, double hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

inline double WrapPi(double a)
{
	a = std::fmod(a + 3.14159265358979323846, 2 * 3.14159265358979323846);
	if (a < 0)
		a += 2 * 3.14159265358979323846;
	return a - 3.14159265358979323846;
}

}  // namespace sim

#endif
