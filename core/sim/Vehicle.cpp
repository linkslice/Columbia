// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Vehicle.h"

#include <algorithm>
#include <cmath>

namespace sim {

namespace {

const double kG0 = 9.80665;
const double kSeaLevelPressure = 101325.0;

Mat3
Inverse(const Mat3& a)
{
	const double (*m)[3] = a.m;
	double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
		- m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
		+ m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	Mat3 r;
	if (std::fabs(det) < 1e-12)
		return r;
	double inv = 1.0 / det;
	r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv;
	r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
	r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
	r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv;
	r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
	r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
	r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv;
	r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
	r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
	return r;
}

}  // namespace


double
Module::Mass() const
{
	double m = dryMass;
	for (const Tank& t : tanks)
		m += t.mass;
	return m;
}


int
Module::FindTank(const std::string& n) const
{
	for (size_t i = 0; i < tanks.size(); i++)
		if (tanks[i].name == n)
			return int(i);
	return -1;
}


int
Module::FindThruster(const std::string& n) const
{
	for (size_t i = 0; i < thrusters.size(); i++)
		if (thrusters[i].name == n)
			return int(i);
	return -1;
}


Vec3
GimballedDirection(const Thruster& t)
{
	if (t.gimbalPitch == 0 && t.gimbalYaw == 0)
		return t.direction;
	Vec3 d = t.direction;
	// Small-angle rotations about the two gimbal axes.
	auto rotate = [](const Vec3& v, const Vec3& axis, double angle) {
		Vec3 k = axis.Unit();
		double c = std::cos(angle), s = std::sin(angle);
		return v * c + k.Cross(v) * s + k * (k.Dot(v) * (1 - c));
	};
	d = rotate(d, t.gimbalPitchAxis, t.gimbalPitch);
	d = rotate(d, t.gimbalYawAxis, t.gimbalYaw);
	return d.Unit();
}


Module&
Vehicle::AddModule(const Module& module)
{
	fModules.push_back(module);
	UpdateMassProperties();
	return fModules.back();
}


Module*
Vehicle::FindModule(const std::string& n)
{
	for (Module& m : fModules)
		if (m.name == n)
			return &m;
	return nullptr;
}


const Module*
Vehicle::FindModule(const std::string& n) const
{
	for (const Module& m : fModules)
		if (m.name == n)
			return &m;
	return nullptr;
}


Module
Vehicle::RemoveModule(const std::string& n)
{
	Module removed;
	for (size_t i = 0; i < fModules.size(); i++) {
		if (fModules[i].name == n) {
			removed = fModules[i];
			fModules.erase(fModules.begin() + i);
			break;
		}
	}
	Vec3 oldCG = fCG;
	UpdateMassProperties();
	// Keep the inertial position of the vehicle frame continuous.
	position += ToInertial(fCG - oldCG);
	return removed;
}


void
Vehicle::UpdateMassProperties()
{
	double total = 0;
	Vec3 moment;
	for (const Module& m : fModules) {
		double mm = m.dryMass;
		Vec3 c = m.cg * m.dryMass;
		for (const Tank& t : m.tanks) {
			mm += t.mass;
			c += t.position * t.mass;
		}
		if (mm <= 0)
			continue;
		Vec3 cgModule = c / mm;
		Vec3 cgVehicle = m.offset + m.rotation * cgModule;
		total += mm;
		moment += cgVehicle * mm;
	}
	fMass = std::max(total, 1.0);
	fCG = total > 0 ? moment / total : Vec3();

	Mat3 I;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			I.m[i][j] = 0;
	for (const Module& m : fModules) {
		double mm = m.Mass();
		if (mm <= 0)
			continue;
		double scale = m.inertiaMass > 0 ? mm / m.inertiaMass : 1.0;
		Mat3 local;
		local.m[0][0] = m.inertia.x * scale;
		local.m[1][1] = m.inertia.y * scale;
		local.m[2][2] = m.inertia.z * scale;
		local.m[0][1] = local.m[0][2] = local.m[1][0] = 0;
		local.m[1][2] = local.m[2][0] = local.m[2][1] = 0;
		Mat3 rotated = m.rotation * local * m.rotation.Transposed();
		Vec3 cgVehicle = m.offset + m.rotation * m.cg;
		Vec3 d = cgVehicle - fCG;
		double d2 = d.Dot(d);
		double dv[3] = {d.x, d.y, d.z};
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				I.m[i][j] += rotated.m[i][j]
					+ mm * ((i == j ? d2 : 0) - dv[i] * dv[j]);
	}
	for (int i = 0; i < 3; i++)
		if (I.m[i][i] < 1)
			I.m[i][i] = 1;
	fInertia = I;
	fInertiaInv = Inverse(I);
}


Vec3
Vehicle::PointPosition(const Vec3& vehiclePoint) const
{
	return position + ToInertial(vehiclePoint - fCG);
}


bool
Vehicle::FindThruster(const std::string& n, int* module, int* thruster) const
{
	for (size_t i = 0; i < fModules.size(); i++) {
		int t = fModules[i].FindThruster(n);
		if (t >= 0) {
			*module = int(i);
			*thruster = t;
			return true;
		}
	}
	return false;
}


Thruster*
Vehicle::GetThruster(const std::string& n)
{
	int m, t;
	if (!FindThruster(n, &m, &t))
		return nullptr;
	return &fModules[m].thrusters[t];
}


void
Vehicle::_Thrust(Vec3* force, Vec3* torque, double* massFlow, double dt,
	bool consume)
{
	*force = Vec3();
	*torque = Vec3();
	*massFlow = 0;
	for (Module& m : fModules) {
		for (Thruster& t : m.thrusters) {
			bool commanded = (t.on || t.failedOn) && !t.failedOff;
			Tank* tank = t.tank >= 0 && t.tank < int(m.tanks.size())
				? &m.tanks[t.tank] : nullptr;
			if (tank != nullptr && tank->mass <= 0)
				commanded = false;
			double target = commanded ? Clamp(t.throttle, 0, 1) : 0;
			if (t.rampTime > 0) {
				double step = dt / t.rampTime;
				if (t.level < target)
					t.level = std::min(target, t.level + step);
				else
					t.level = std::max(target, t.level - step);
			} else {
				t.level = target;
			}
			if (commanded && consume)
				t.burnTime += dt;
			if (t.level <= 0)
				continue;

			double vac = t.maxThrust;
			double thrust = vac;
			if (t.seaLevelThrust > 0 && ambientPressure > 0) {
				thrust = vac - (vac - t.seaLevelThrust)
					* (ambientPressure / kSeaLevelPressure);
			}
			thrust *= t.level * t.health;
			double flow = t.maxThrust * t.level / (t.isp * kG0);
			if (tank != nullptr && consume) {
				double used = flow * dt;
				if (used > tank->mass) {
					thrust *= tank->mass / used;
					used = tank->mass;
				}
				tank->mass -= used;
			}
			*massFlow += flow;

			Vec3 dir = m.rotation * GimballedDirection(t);
			Vec3 pos = m.offset + m.rotation * t.position;
			Vec3 f = dir * thrust;
			*force += f;
			*torque += (pos - fCG).Cross(f);
		}
	}
}

}  // namespace sim
