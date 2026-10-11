// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Six-degree-of-freedom spacecraft built from modules (CM, SM, LM ascent
// stage, LM descent stage, S-IVB...). Modules carry their own tanks and
// thrusters, so staging, docking and undocking just move modules between
// vehicles.

#ifndef COLUMBIA_VEHICLE_H
#define COLUMBIA_VEHICLE_H

#include "Math.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sim {

struct Tank {
	std::string name;
	double mass = 0;          // kg remaining
	double capacity = 0;      // kg
	Vec3 position;            // module frame (for CG)
};

enum ThrusterKind { THRUSTER_MAIN, THRUSTER_RCS };

struct Thruster {
	std::string name;
	ThrusterKind kind = THRUSTER_RCS;
	Vec3 position;            // module frame, m
	Vec3 direction;           // force direction on the vehicle (unit)
	double maxThrust = 0;     // N (vacuum)
	double seaLevelThrust = 0;// N (0 = vacuum engine)
	double isp = 300;         // s (vacuum)
	int tank = -1;            // index in the module's tanks
	// Command and state.
	bool on = false;
	double throttle = 1.0;    // 0..1 of maxThrust
	double level = 0;         // actual thrust fraction (start/stop ramp)
	double rampTime = 0.0;    // s to full thrust (0 = instant)
	// Gimbal (main engines): rotation of `direction` about two axes.
	double gimbalPitch = 0, gimbalYaw = 0;   // rad
	Vec3 gimbalPitchAxis, gimbalYawAxis;      // module frame
	double gimbalLimit = 0;   // rad
	// Failures.
	bool failedOff = false;   // will not fire
	bool failedOn = false;    // stuck firing
	double health = 1.0;      // thrust multiplier (leaks, damage)
	// Statistics.
	double burnTime = 0;
	uint32_t firings = 0;
};

struct Module {
	std::string name;
	double dryMass = 0;
	Vec3 cg;                  // dry CG, module frame
	Vec3 inertia;             // principal moments about the dry CG (kg m^2)
	double inertiaMass = 0;   // mass at which `inertia` applies
	std::vector<Tank> tanks;
	std::vector<Thruster> thrusters;

	// Placement in the vehicle frame.
	Vec3 offset;
	Mat3 rotation;            // module -> vehicle

	// Aerodynamics (zero = none).
	double referenceArea = 0; // m^2
	double dragCoefficient = 0;

	double Mass() const;
	int FindTank(const std::string& name) const;
	int FindThruster(const std::string& name) const;
};

struct ContactPoint {
	Vec3 position;            // vehicle frame
	std::string name;
};

class Vehicle {
public:
	explicit Vehicle(const std::string& name = "") : fName(name) {}

	const std::string& Name() const { return fName; }
	void SetName(const std::string& name) { fName = name; }

	// --- Modules ------------------------------------------------------------
	Module& AddModule(const Module& module);
	std::vector<Module>& Modules() { return fModules; }
	const std::vector<Module>& Modules() const { return fModules; }
	Module* FindModule(const std::string& name);
	const Module* FindModule(const std::string& name) const;
	// Removes a module, returning it (used for staging and undocking).
	Module RemoveModule(const std::string& name);
	// Docking: takes over all of `other`'s modules, placed so that other's
	// frame sits at `offset` with `rotation` (other frame -> this frame).
	// Inertial positions stay put; momentum is conserved.
	void Merge(Vehicle& other, const Vec3& offset, const Mat3& rotation);
	// Undocking/staging: splits off the named modules as a new vehicle
	// whose frame is at `offset`, `rotation` (new frame -> this frame) in
	// this vehicle's frame. `separation` is the relative velocity given
	// to the new vehicle along its own +X (springs, pyros).
	Vehicle Detach(const std::vector<std::string>& modules, const Vec3& offset,
		const Mat3& rotation, const std::string& name, double separation = 0);
	void UpdateMassProperties();

	double Mass() const { return fMass; }
	const Vec3& CenterOfMass() const { return fCG; }   // vehicle frame
	const Mat3& Inertia() const { return fInertia; }

	// --- State (CG in the Earth-centred inertial frame) -----------------------
	Vec3 position, velocity;
	Quat attitude;            // vehicle -> inertial
	Vec3 rates;               // body rad/s
	double jd = 0;            // time of this state

	Vec3 ToInertial(const Vec3& vehicleVector) const
		{ return attitude.Rotate(vehicleVector); }
	Vec3 ToBody(const Vec3& inertialVector) const
		{ return attitude.Conjugate().Rotate(inertialVector); }
	// Inertial position of a point given in the vehicle frame.
	Vec3 PointPosition(const Vec3& vehiclePoint) const;

	// --- Forces ------------------------------------------------------------------
	// Thruster lookup across modules: returns (module, thruster) indices.
	bool FindThruster(const std::string& name, int* module,
		int* thruster) const;
	Thruster* GetThruster(const std::string& name);

	// External force and torque for this step (aerodynamics, contact).
	void AddExternalForce(const Vec3& inertialForce, const Vec3& bodyTorque)
	{
		fExternalForce += inertialForce;
		fExternalTorque += bodyTorque;
	}

	// Advances the rigid-body state by dt seconds with fixed-step RK4.
	// `gravity` gives the gravitational acceleration at a position/time.
	// Thrust from all "on" thrusters is applied and propellant consumed.
	template <class GravityFn>
	void Step(double dt, GravityFn gravity);

	// Sensed (non-gravitational) acceleration over the last step, in the
	// inertial frame. This is what accelerometers measure.
	const Vec3& SensedAcceleration() const { return fSensed; }
	// Same, as accumulated velocity change during the last step.
	const Vec3& SensedDeltaV() const { return fSensedDV; }

	// --- Contact -------------------------------------------------------------------
	std::vector<ContactPoint> contacts;

	// Free-form flags for systems.
	bool landed = false;      // fixed to a surface

private:
	void _Thrust(Vec3* force, Vec3* torque, double* massFlow,
		double dt, bool consume);
	double _AtmosphericPressure() const;

	std::string fName;
	std::vector<Module> fModules;
	double fMass = 0;
	Vec3 fCG;
	Mat3 fInertia;
	Mat3 fInertiaInv;
	Vec3 fExternalForce, fExternalTorque;
	Vec3 fSensed, fSensedDV;

public:
	// Ambient pressure provider for engine back-pressure (Pa); set by the
	// simulation for vehicles in the atmosphere.
	double ambientPressure = 0;
};

// Rotates a vector by the thruster's gimbal angles (module frame).
Vec3 GimballedDirection(const Thruster& t);

// --- Template implementation ---------------------------------------------------

template <class GravityFn>
void
Vehicle::Step(double dt, GravityFn gravity)
{
	if (dt <= 0)
		return;
	if (landed) {
		// Engines still start, run up and burn propellant on the pad or
		// the surface (hold-down); the vehicle doesn't move.
		Vec3 f, t;
		double flow;
		_Thrust(&f, &t, &flow, dt, true);
		fExternalForce = fExternalTorque = Vec3();
		// The ground holds it up: accelerometers read the support force.
		fSensed = -gravity(position, jd);
		fSensedDV = fSensed * dt;
		jd += dt / 86400.0;
		return;
	}

	// Thrust is evaluated once per step (engine commands change at the
	// computer's rate, much slower than the step).
	Vec3 thrustBody, torqueBody;
	double massFlow;
	_Thrust(&thrustBody, &torqueBody, &massFlow, dt, true);
	double mass0 = fMass;
	UpdateMassProperties();
	double massAvg = 0.5 * (mass0 + fMass);

	Vec3 extForce = fExternalForce;
	Vec3 extTorque = fExternalTorque;
	fExternalForce = fExternalTorque = Vec3();

	// Translational RK4 (attitude varies little within a step; use the
	// mid-step attitude for the thrust direction).
	Quat qMid = attitude.Integrated(rates, dt / 2);
	Vec3 accelNonGrav = qMid.Rotate(thrustBody) / massAvg + extForce / massAvg;
	double t0 = jd;
	auto acc = [&](const Vec3& r, double tOffset) {
		return gravity(r, t0 + tOffset / 86400.0) + accelNonGrav;
	};
	Vec3 r = position, v = velocity;
	Vec3 k1v = acc(r, 0), k1r = v;
	Vec3 k2v = acc(r + k1r * (dt / 2), dt / 2), k2r = v + k1v * (dt / 2);
	Vec3 k3v = acc(r + k2r * (dt / 2), dt / 2), k3r = v + k2v * (dt / 2);
	Vec3 k4v = acc(r + k3r * dt, dt), k4r = v + k3v * dt;
	position = r + (k1r + k2r * 2 + k3r * 2 + k4r) * (dt / 6);
	velocity = v + (k1v + k2v * 2 + k3v * 2 + k4v) * (dt / 6);
	fSensed = accelNonGrav;
	fSensedDV = accelNonGrav * dt;

	// Rotational dynamics: Euler's equations, RK4 on the rates.
	Vec3 torque = torqueBody + extTorque;
	auto wdot = [&](const Vec3& w) {
		Vec3 Iw = fInertia * w;
		return fInertiaInv * (torque - w.Cross(Iw));
	};
	Vec3 w = rates;
	Vec3 a1 = wdot(w);
	Vec3 a2 = wdot(w + a1 * (dt / 2));
	Vec3 a3 = wdot(w + a2 * (dt / 2));
	Vec3 a4 = wdot(w + a3 * dt);
	Vec3 wNew = w + (a1 + a2 * 2 + a3 * 2 + a4) * (dt / 6);
	attitude = attitude.Integrated((w + wNew) * 0.5, dt);
	rates = wNew;
	jd += dt / 86400.0;
}

}  // namespace sim

#endif
