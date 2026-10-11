// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Geometry notes. RCS jet layouts were derived from the flight software's
// own jet-selection tables (Luminary's TYPEPOLY/JETSALL and Comanche's
// PYTABLE/RTABLE): each channel bit's force direction and torque sign fix
// which quad it is on. Masses and engine figures are Apollo 11 values from
// the press kit and mission reports, adjusted per mission elsewhere.

#include "Spacecraft.h"

#include <cmath>
#include <cstdio>

namespace sim {
namespace craft {

namespace {

Thruster
Jet(const char* name, const Vec3& pos, const Vec3& dir, double thrust,
	double isp, int tank)
{
	Thruster t;
	t.name = name;
	t.kind = THRUSTER_RCS;
	t.position = pos;
	t.direction = dir.Unit();
	t.maxThrust = thrust;
	t.isp = isp;
	t.tank = tank;
	return t;
}

// LM quads, at the corners between the Y and Z axes.
struct LMQuad { const char* name; double y, z; };
const LMQuad kLMQuads[4] = {
	{"A", +1.68, +1.68}, {"B", +1.68, -1.68},
	{"C", -1.68, -1.68}, {"D", -1.68, +1.68}
};

// Channel 5 (X jets): bit -> (quad, force sign). From Luminary TYPEPOLY:
// -X jets 1 5 9 13 on bits 1 3 5 7, +X jets 2 6 10 14 on bits 2 4 6 8.
struct LMJetSpec { int channel, bit; int quad; Vec3 dir; const char* name; };
const LMJetSpec kLMJets[16] = {
	{5, 1, 0, {-1, 0, 0}, "LM1"},  {5, 2, 0, {+1, 0, 0}, "LM2"},
	{5, 3, 1, {-1, 0, 0}, "LM5"},  {5, 4, 1, {+1, 0, 0}, "LM6"},
	{5, 5, 2, {-1, 0, 0}, "LM9"},  {5, 6, 2, {+1, 0, 0}, "LM10"},
	{5, 7, 3, {-1, 0, 0}, "LM13"}, {5, 8, 3, {+1, 0, 0}, "LM14"},
	// Channel 6 (horizontal jets), from JETSALL: +Z bits 1,4; -Z 2,3;
	// +Y 5,8; -Y 6,7, placed by their roll (P-axis) torque signs.
	{6, 1, 1, {0, 0, +1}, "LM-B-Z+"}, {6, 2, 0, {0, 0, -1}, "LM-A-Z-"},
	{6, 3, 3, {0, 0, -1}, "LM-D-Z-"}, {6, 4, 2, {0, 0, +1}, "LM-C-Z+"},
	{6, 5, 2, {0, +1, 0}, "LM-C-Y+"}, {6, 6, 1, {0, -1, 0}, "LM-B-Y-"},
	{6, 7, 0, {0, -1, 0}, "LM-A-Y-"}, {6, 8, 3, {0, +1, 0}, "LM-D-Y+"},
};

// CSM SM RCS quads: A +Z, B +Y, C -Z, D -Y (the 7.25 degree clocking of
// the real quads is ignored).
struct CSMJetSpec { int channel, bit; Vec3 pos; Vec3 dir; const char* name; };
const double kSMQuadX = -1.0, kSMQuadR = 2.0;
const CSMJetSpec kCSMJets[16] = {
	// Channel 5: pitch jets (A/C) bits 1-4, yaw jets (B/D) bits 5-8.
	{5, 1, {kSMQuadX, 0, +kSMQuadR}, {+1, 0, 0}, "SM-A-X+"},
	{5, 2, {kSMQuadX, 0, +kSMQuadR}, {-1, 0, 0}, "SM-A-X-"},
	{5, 3, {kSMQuadX, 0, -kSMQuadR}, {-1, 0, 0}, "SM-C-X-"},
	{5, 4, {kSMQuadX, 0, -kSMQuadR}, {+1, 0, 0}, "SM-C-X+"},
	{5, 5, {kSMQuadX, -kSMQuadR, 0}, {+1, 0, 0}, "SM-D-X+"},
	{5, 6, {kSMQuadX, -kSMQuadR, 0}, {-1, 0, 0}, "SM-D-X-"},
	{5, 7, {kSMQuadX, +kSMQuadR, 0}, {-1, 0, 0}, "SM-B-X-"},
	{5, 8, {kSMQuadX, +kSMQuadR, 0}, {+1, 0, 0}, "SM-B-X+"},
	// Channel 6: BD roll jets bits 1-4 (Z forces), AC roll jets 5-8
	// (Y forces).
	{6, 1, {kSMQuadX, +kSMQuadR, 0}, {0, 0, +1}, "SM-B-Z+"},
	{6, 2, {kSMQuadX, +kSMQuadR, 0}, {0, 0, -1}, "SM-B-Z-"},
	{6, 3, {kSMQuadX, -kSMQuadR, 0}, {0, 0, -1}, "SM-D-Z-"},
	{6, 4, {kSMQuadX, -kSMQuadR, 0}, {0, 0, +1}, "SM-D-Z+"},
	{6, 5, {kSMQuadX, 0, -kSMQuadR}, {0, +1, 0}, "SM-C-Y+"},
	{6, 6, {kSMQuadX, 0, -kSMQuadR}, {0, -1, 0}, "SM-C-Y-"},
	{6, 7, {kSMQuadX, 0, +kSMQuadR}, {0, -1, 0}, "SM-A-Y-"},
	{6, 8, {kSMQuadX, 0, +kSMQuadR}, {0, +1, 0}, "SM-A-Y+"},
};

// CM RCS (used after CM/SM separation): couples about each axis. The CMC
// drives them through the same channel bits; pitch/yaw jets pair across
// the CM, roll jets fire tangentially.
const CSMJetSpec kCMJets[16] = {
	{5, 1, {0.4, 0, +1.6}, {+1, 0, 0}, "CM-P+1"},
	{5, 2, {0.4, 0, +1.6}, {-1, 0, 0}, "CM-P-1"},
	{5, 3, {0.4, 0, -1.6}, {-1, 0, 0}, "CM-P+2"},
	{5, 4, {0.4, 0, -1.6}, {+1, 0, 0}, "CM-P-2"},
	{5, 5, {0.4, -1.6, 0}, {+1, 0, 0}, "CM-Y+1"},
	{5, 6, {0.4, -1.6, 0}, {-1, 0, 0}, "CM-Y-1"},
	{5, 7, {0.4, +1.6, 0}, {-1, 0, 0}, "CM-Y+2"},
	{5, 8, {0.4, +1.6, 0}, {+1, 0, 0}, "CM-Y-2"},
	{6, 1, {0.4, +1.4, 0}, {0, 0, +1}, "CM-R+1"},
	{6, 2, {0.4, +1.4, 0}, {0, 0, -1}, "CM-R-1"},
	{6, 3, {0.4, -1.4, 0}, {0, 0, -1}, "CM-R+2"},
	{6, 4, {0.4, -1.4, 0}, {0, 0, +1}, "CM-R-2"},
	{6, 5, {0.4, 0, -1.4}, {0, +1, 0}, "CM-R+3"},
	{6, 6, {0.4, 0, -1.4}, {0, -1, 0}, "CM-R-3"},
	{6, 7, {0.4, 0, +1.4}, {0, -1, 0}, "CM-R+4"},
	{6, 8, {0.4, 0, +1.4}, {0, +1, 0}, "CM-R-4"},
};

}  // namespace


// ---------------------------------------------------------------------------
// LM


Module
MakeLMDescentStage(const LMConfig& c)
{
	Module m;
	m.name = "LM descent stage";
	m.dryMass = c.descentDry;
	m.cg = Vec3(-1.3, 0, 0);
	m.inertiaMass = c.descentDry + c.descentPropellant;
	m.inertia = Vec3(19500, 15500, 15500);

	Tank fuel;
	fuel.name = "DPS";
	fuel.capacity = c.descentPropellant;
	fuel.mass = c.descentPropellant;
	fuel.position = Vec3(-1.2, 0, 0);
	m.tanks.push_back(fuel);

	Thruster dps;
	dps.name = "DPS";
	dps.kind = THRUSTER_MAIN;
	dps.position = Vec3(-2.4, 0, 0);
	dps.direction = Vec3(1, 0, 0);
	dps.maxThrust = kDPSFullThrust;
	dps.isp = 311;
	dps.tank = 0;
	dps.throttle = kDPSMinThrust / kDPSFullThrust;
	dps.rampTime = 0.6;
	dps.gimbalPitchAxis = Vec3(0, 1, 0);   // pitch trim tilts toward Z
	dps.gimbalYawAxis = Vec3(0, 0, 1);     // roll trim tilts toward Y
	dps.gimbalLimit = kDPSGimbalLimit;
	m.thrusters.push_back(dps);
	return m;
}


Module
MakeLMAscentStage(const LMConfig& c)
{
	Module m;
	m.name = "LM ascent stage";
	m.dryMass = c.ascentDry;
	m.cg = Vec3(1.15, 0, 0.1);
	m.inertiaMass = c.ascentDry + c.ascentPropellant + c.rcsPropellant;
	m.inertia = Vec3(6300, 5400, 4300);

	Tank aps;
	aps.name = "APS";
	aps.capacity = aps.mass = c.ascentPropellant;
	aps.position = Vec3(0.9, 0, 0);
	m.tanks.push_back(aps);
	Tank rcsA, rcsB;
	rcsA.name = "RCS A";
	rcsB.name = "RCS B";
	rcsA.capacity = rcsA.mass = c.rcsPropellant / 2;
	rcsB.capacity = rcsB.mass = c.rcsPropellant / 2;
	rcsA.position = rcsB.position = Vec3(1.3, 0, 0);
	m.tanks.push_back(rcsA);
	m.tanks.push_back(rcsB);

	Thruster apsEngine;
	apsEngine.name = "APS";
	apsEngine.kind = THRUSTER_MAIN;
	apsEngine.position = Vec3(0.1, 0, 0);
	apsEngine.direction = Vec3(1, 0, 0);
	apsEngine.maxThrust = 15568;
	apsEngine.isp = 311;
	apsEngine.tank = 0;
	apsEngine.rampTime = 0.3;
	m.thrusters.push_back(apsEngine);

	for (const LMJetSpec& j : kLMJets) {
		const LMQuad& q = kLMQuads[j.quad];
		// Systems A and B alternate between jets in each quad.
		int tank = (j.bit % 2) ? 1 : 2;
		m.thrusters.push_back(Jet(j.name, Vec3(1.45, q.y, q.z), j.dir,
			kLMPropellantRCSJet, 290, tank));
	}
	return m;
}


void
AddLMContacts(Vehicle& lm)
{
	lm.contacts.clear();
	const double padX = -3.35, reach = 4.65;
	lm.contacts.push_back({Vec3(padX, 0, +reach), "+Z footpad"});
	lm.contacts.push_back({Vec3(padX, 0, -reach), "-Z footpad"});
	lm.contacts.push_back({Vec3(padX, +reach, 0), "+Y footpad"});
	lm.contacts.push_back({Vec3(padX, -reach, 0), "-Y footpad"});
}


const char*
LMJetName(int channel, int bit)
{
	for (const LMJetSpec& j : kLMJets)
		if (j.channel == channel && j.bit == bit)
			return j.name;
	return nullptr;
}


// ---------------------------------------------------------------------------
// CSM


Module
MakeCommandModule(const CSMConfig& c)
{
	Module m;
	m.name = "Command Module";
	m.dryMass = c.cmMass;
	// The CM's CG sits slightly toward +Z, which trims it at an angle in
	// entry and gives it lift.
	m.cg = Vec3(1.05, 0, 0.13);
	m.inertiaMass = c.cmMass;
	m.inertia = Vec3(6100, 5600, 5100);
	m.referenceArea = 12.02;
	m.dragCoefficient = 1.29;

	Tank rcs;
	rcs.name = "CM RCS";
	rcs.capacity = rcs.mass = c.cmRcsPropellant;
	rcs.position = Vec3(0.8, 0, 0);
	m.tanks.push_back(rcs);
	for (const CSMJetSpec& j : kCMJets) {
		Thruster t = Jet(j.name, j.pos, j.dir, 412, 290, 0);
		t.failedOff = true;   // pressurized only at CM/SM separation
		m.thrusters.push_back(t);
	}
	return m;
}


Module
MakeServiceModule(const CSMConfig& c)
{
	Module m;
	m.name = "Service Module";
	m.dryMass = c.smDry;
	m.cg = Vec3(-2.4, 0, 0);
	m.inertiaMass = c.smDry + c.spsPropellant + c.smRcsPropellant;
	m.inertia = Vec3(17500, 60000, 60000);
	m.referenceArea = 12.02;
	m.dragCoefficient = 0.9;

	Tank sps;
	sps.name = "SPS";
	sps.capacity = sps.mass = c.spsPropellant;
	sps.position = Vec3(-2.3, 0, 0);
	m.tanks.push_back(sps);
	const char* quads[4] = {"RCS A", "RCS B", "RCS C", "RCS D"};
	for (const char* q : quads) {
		Tank t;
		t.name = q;
		t.capacity = t.mass = c.smRcsPropellant / 4;
		t.position = Vec3(-1.0, 0, 0);
		m.tanks.push_back(t);
	}

	Thruster spsEngine;
	spsEngine.name = "SPS";
	spsEngine.kind = THRUSTER_MAIN;
	spsEngine.position = Vec3(-6.4, 0, 0);
	spsEngine.direction = Vec3(1, 0, 0);
	spsEngine.maxThrust = kSPSThrust;
	spsEngine.isp = kSPSIsp;
	spsEngine.tank = 0;
	spsEngine.rampTime = 0.5;
	spsEngine.gimbalPitchAxis = Vec3(0, 1, 0);
	spsEngine.gimbalYawAxis = Vec3(0, 0, 1);
	spsEngine.gimbalLimit = 6.0 * kDeg;
	m.thrusters.push_back(spsEngine);

	for (const CSMJetSpec& j : kCSMJets) {
		char quad = j.name[3];
		int tank = 1 + (quad - 'A');
		m.thrusters.push_back(Jet(j.name, j.pos, j.dir, 445, 290, tank));
	}
	return m;
}


const char*
CSMJetName(int channel, int bit)
{
	for (const CSMJetSpec& j : kCSMJets)
		if (j.channel == channel && j.bit == bit)
			return j.name;
	return nullptr;
}


const char*
CMJetName(int channel, int bit)
{
	for (const CSMJetSpec& j : kCMJets)
		if (j.channel == channel && j.bit == bit)
			return j.name;
	return nullptr;
}


// ---------------------------------------------------------------------------
// Saturn V


Module
MakeSIVB(double propellant)
{
	Module m;
	m.name = "S-IVB";
	m.dryMass = 13500;   // including the Instrument Unit
	m.cg = Vec3(-17.0, 0, 0);
	m.inertiaMass = 13500 + 109000;
	m.inertia = Vec3(60000, 1.6e6, 1.6e6);
	m.referenceArea = 33.0;
	m.dragCoefficient = 0.5;
	Tank t;
	t.name = "S-IVB";
	t.capacity = 109000;
	t.mass = propellant;
	t.position = Vec3(-19.0, 0, 0);
	m.tanks.push_back(t);
	Thruster j2;
	j2.name = "J-2";
	j2.kind = THRUSTER_MAIN;
	j2.position = Vec3(-27.0, 0, 0);
	j2.direction = Vec3(1, 0, 0);
	j2.maxThrust = 1023000;
	j2.isp = 421;
	j2.tank = 0;
	j2.rampTime = 2.5;
	j2.gimbalPitchAxis = Vec3(0, 1, 0);
	j2.gimbalYawAxis = Vec3(0, 0, 1);
	j2.gimbalLimit = 7 * kDeg;
	m.thrusters.push_back(j2);
	// APS ullage/attitude modules (lumped).
	m.thrusters.push_back(Jet("SIVB-APS-P+", Vec3(-25, 0, 3.3),
		Vec3(1, 0, 0), 670, 290, 0));
	return m;
}


Module
MakeSII()
{
	Module m;
	m.name = "S-II";
	m.dryMass = 36000;
	m.cg = Vec3(-42.0, 0, 0);
	m.inertiaMass = 36000 + 444000;
	m.inertia = Vec3(5.0e5, 1.2e7, 1.2e7);
	m.referenceArea = 78.5;
	m.dragCoefficient = 0.5;
	Tank t;
	t.name = "S-II";
	t.capacity = t.mass = 444000;
	t.position = Vec3(-42.0, 0, 0);
	m.tanks.push_back(t);
	for (int i = 0; i < 5; i++) {
		Thruster e;
		char name[16];
		std::snprintf(name, sizeof(name), "S-II J-2 %d", i + 1);
		e.name = name;
		e.kind = THRUSTER_MAIN;
		double angle = i * 3.14159265358979 / 2;
		e.position = i == 4 ? Vec3(-55, 0, 0)
			: Vec3(-55, 2.3 * std::cos(angle), 2.3 * std::sin(angle));
		e.direction = Vec3(1, 0, 0);
		e.maxThrust = 1023000;
		e.isp = 421;
		e.tank = 0;
		e.rampTime = 2.5;
		e.gimbalPitchAxis = Vec3(0, 1, 0);
		e.gimbalYawAxis = Vec3(0, 0, 1);
		e.gimbalLimit = 7 * kDeg;
		m.thrusters.push_back(e);
	}
	return m;
}


Module
MakeSIC()
{
	Module m;
	m.name = "S-IC";
	m.dryMass = 131000;
	m.cg = Vec3(-82.0, 0, 0);
	m.inertiaMass = 131000 + 2077000;
	m.inertia = Vec3(5.0e6, 2.0e8, 2.0e8);
	m.referenceArea = 78.5;
	m.dragCoefficient = 0.5;
	Tank t;
	t.name = "S-IC";
	t.capacity = t.mass = 2077000;
	t.position = Vec3(-80.0, 0, 0);
	m.tanks.push_back(t);
	for (int i = 0; i < 5; i++) {
		Thruster e;
		char name[16];
		std::snprintf(name, sizeof(name), "F-1 %d", i + 1);
		e.name = name;
		e.kind = THRUSTER_MAIN;
		double angle = i * 3.14159265358979 / 2 + 3.14159265358979 / 4;
		e.position = i == 4 ? Vec3(-110, 0, 0)
			: Vec3(-110, 4.0 * std::cos(angle), 4.0 * std::sin(angle));
		e.direction = Vec3(1, 0, 0);
		e.maxThrust = 7770000;
		e.seaLevelThrust = 6770000;
		e.isp = 304;
		e.tank = 0;
		e.rampTime = 1.0;
		e.gimbalPitchAxis = Vec3(0, 1, 0);
		e.gimbalYawAxis = Vec3(0, 0, 1);
		e.gimbalLimit = 5.15 * kDeg;
		m.thrusters.push_back(e);
	}
	return m;
}


Module
MakeLaunchEscapeTower()
{
	Module m;
	m.name = "Launch escape tower";
	m.dryMass = 2100;
	m.cg = Vec3(6.0, 0, 0);
	m.inertiaMass = 4200;
	m.inertia = Vec3(400, 18000, 18000);
	Tank t;
	t.name = "LES";
	t.capacity = t.mass = 2100;
	t.position = Vec3(5.5, 0, 0);
	m.tanks.push_back(t);
	Thruster les;
	les.name = "LES";
	les.kind = THRUSTER_MAIN;
	les.position = Vec3(4.5, 0, 0);
	les.direction = Vec3(1, 0, 0.02);
	les.maxThrust = 653000;
	les.isp = 250;
	les.tank = 0;
	m.thrusters.push_back(les);
	Thruster jett;
	jett.name = "Tower jettison";
	jett.kind = THRUSTER_MAIN;
	jett.position = Vec3(8.0, 0, 0);
	jett.direction = Vec3(1, 0, 0);
	jett.maxThrust = 140000;
	jett.isp = 220;
	jett.tank = 0;
	m.thrusters.push_back(jett);
	return m;
}

}  // namespace craft
}  // namespace sim
