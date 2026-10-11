// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "FlightSim.h"

#include "Saturn.h"
#include "Trajectory.h"
#include "../gnc/MissionPads.h"
#include "../gnc/PadLoads.h"
#include "../sim/Spacecraft.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace flight {

using namespace sim;

namespace {

const double kDay = 86400.0;
const double kStep = 0.005;            // lockstep physics/AGC step (s)
const double kCoastStep = 20.0;        // longest coast step (s)
// Docking hardware: CM probe tip and LM drogue in their own frames.
const Vec3 kCMProbe(3.55, 0, 0);
const Vec3 kLMDrogue(3.05, 0, 0);
// LM stowed in the SLA, in the stack (CSM) frame.
const Vec3 kLMInSLA(-11.0, 0, 0);

struct Pad { const char* name; double lat, lon; };
const Pad kPads[] = {
	{"LC-39A", 28.60822, -80.60411},
	{"LC-39B", 28.62719, -80.62082},
	{"LC-34", 28.52182, -80.56126},
};

const Pad&
FindPad(const std::string& name)
{
	for (const Pad& p : kPads)
		if (name == p.name)
			return p;
	return kPads[0];
}


// Engines that make coasting impossible (attitude-hold RCS pulses don't).
bool
MainThrust(const Vehicle& v)
{
	for (const Module& m : v.Modules())
		for (const Thruster& t : m.thrusters)
			if (t.failedOn || ((t.on || t.level > 0) && t.kind == THRUSTER_MAIN))
				return true;
	return false;
}

}  // namespace


const char*
PhaseName(Phase p)
{
	switch (p) {
		case Phase::Prelaunch: return "Prelaunch";
		case Phase::Boost: return "Boost";
		case Phase::EarthParkingOrbit: return "Earth parking orbit";
		case Phase::TranslunarInjection: return "Translunar injection";
		case Phase::TranslunarCoast: return "Translunar coast";
		case Phase::LunarOrbit: return "Lunar orbit";
		case Phase::PoweredDescent: return "Powered descent";
		case Phase::LunarSurface: return "Lunar surface";
		case Phase::LunarAscent: return "Lunar ascent";
		case Phase::Rendezvous: return "Rendezvous";
		case Phase::TransearthCoast: return "Transearth coast";
		case Phase::Entry: return "Entry";
		case Phase::Recovery: return "Recovery";
		case Phase::Lost: return "Mission lost";
	}
	return "";
}


const char*
StartPointName(StartPoint s)
{
	switch (s) {
		case StartPoint::Launch: return "Launch";
		case StartPoint::EarthOrbit: return "Earth parking orbit";
		case StartPoint::TranslunarCoast: return "Translunar coast";
		case StartPoint::LunarOrbit: return "Lunar orbit";
		case StartPoint::PoweredDescent: return "Powered descent";
		case StartPoint::LunarSurface: return "On the surface";
		case StartPoint::LunarAscent: return "Lunar ascent";
		case StartPoint::TransearthCoast: return "Transearth coast";
		case StartPoint::Entry: return "Entry";
	}
	return "";
}


FlightSim::FlightSim(const MissionSpec& mission, const std::string& ropeDir)
	:
	fMission(mission),
	fRopeDir(ropeDir)
{
	fGETZeroJD = world::JulianDate(mission.year, mission.month, mission.day,
		mission.hour, mission.minute, mission.second);
	fHasLM = !mission.lgcProgram.empty();
	_BuildWorld();
	crew.members[0].name = mission.commander;
	crew.members[0].role = "CDR";
	crew.members[1].name = mission.cmPilot;
	crew.members[1].role = "CMP";
	crew.members[2].name = mission.lmPilot;
	crew.members[2].role = "LMP";
	lioh.canistersCM = 20;
	lioh.canistersLM = !fHasLM ? 0 : mission.number >= 15 ? 4 : 2;
	lioh.activeLife = 36;
	crew.foodDays = mission.plan.splashdown > 0
		? mission.plan.splashdown / kDay + 2 : 14;
}


FlightSim::~FlightSim()
{
}


double
FlightSim::JulianDate() const
{
	return fGETZeroJD + fGET / kDay;
}


void
FlightSim::_BuildWorld()
{
	if (fHasLM && fMission.site.latitude != 0)
		fTerrain.SetDatum(fMission.site.latitude, fMission.site.longitude,
			fMission.site.radius);
	fMoonScene.reset(new MoonScene(fTerrain));
}


Vec3
FlightSim::_Gravity(const Vec3& r, double jd) const
{
	return world::Gravity::Acceleration(r, jd, true);
}


void
FlightSim::Log(LogEntry::Kind kind, const std::string& text)
{
	LogEntry e{fGET, kind, text};
	fLog.push_back(e);
	while (fLog.size() > 500)
		fLog.pop_front();
	if (onLog)
		onLog(e);
}


void
FlightSim::Lose(const std::string& reason)
{
	if (fPhase == Phase::Lost)
		return;
	fLossReason = reason;
	fPhase = Phase::Lost;
	Log(LogEntry::Warning, reason);
}


void
FlightSim::SetPhase(Phase phase)
{
	if (phase == fPhase || fPhase == Phase::Lost)
		return;
	fPhase = phase;
	Log(LogEntry::Event, std::string("Phase: ") + PhaseName(phase));
}


// ---------------------------------------------------------------------------
// Vehicles


Vehicle
FlightSim::_MakeCSM(bool withSM)
{
	craft::CSMConfig cfg;
	cfg.spsPropellant = fMission.csmSpsPropellant;
	Vehicle v(fMission.csmCallsign);
	v.AddModule(craft::MakeCommandModule(cfg));
	if (withSM)
		v.AddModule(craft::MakeServiceModule(cfg));
	return v;
}


Vehicle
FlightSim::_MakeLM(bool descent)
{
	craft::LMConfig cfg;
	cfg.descentDry = fMission.lmDescentDry;
	cfg.descentPropellant = fMission.lmDescentPropellant;
	cfg.ascentDry = fMission.lmAscentDry;
	cfg.ascentPropellant = fMission.lmAscentPropellant;
	Vehicle v(fMission.lmCallsign);
	if (descent)
		v.AddModule(craft::MakeLMDescentStage(cfg));
	v.AddModule(craft::MakeLMAscentStage(cfg));
	craft::AddLMContacts(v);
	return v;
}


void
FlightSim::_DockLMGeometry(Vec3* offset, Mat3* rotation) const
{
	// Nose to nose: LM +X along CSM -X, rolled so the LM's +Z sits along
	// the CSM's -Z... (180 degrees about Z).
	*rotation = Mat3::RotZ(3.14159265358979323846).Transposed();
	*offset = kCMProbe - *rotation * kLMDrogue;
}


Mat3
FlightSim::LVLHReference(const Vec3& r, const Vec3& v)
{
	// Stable member: X along track, Y opposite the orbit normal, Z down.
	Vec3 z = (-r).Unit();
	Vec3 y = v.Cross(r).Unit();
	Vec3 x = y.Cross(z);
	return Mat3::Rows(x, y, z);
}


void
FlightSim::_UpdatePGNCSAttachments()
{
	if (fCMPGNCS)
		fCMPGNCS->Attach(*fCSM);
	if (fLMPGNCS) {
		if (fLMDocked || fLMInSLA || !fLM) {
			Vec3 off;
			Mat3 rot;
			if (fLMInSLA)
				rot = Mat3();
			else
				_DockLMGeometry(&off, &rot);
			fLMPGNCS->Attach(*fCSM, Quat::FromMatrix(rot));
		} else {
			fLMPGNCS->Attach(*fLM);
		}
	}
}


// ---------------------------------------------------------------------------
// Computers


bool
FlightSim::_LoadComputers(std::string* error)
{
	if (!fCMCRope.Load(fRopeDir, fMission.cmcProgram, error))
		return false;
	fCMC.reset(new agc::Computer);
	fCMC->LoadRope(fCMCRope.Image().data(), fCMCRope.Image().size());
	fCMC->onChannelWrite = [this](int ch, uint16_t v) {
		fCMCDSKY.ChannelWritten(ch, v);
	};
	fCMPGNCS.reset(new gnc::CMPGNCS(*fCMC, *fCSM));
	fCMCPad.reset(new gnc::PadLoader(*fCMC, fCMCRope));
	if (fHasLM) {
		if (!fLGCRope.Load(fRopeDir, fMission.lgcProgram, error))
			return false;
		fLGC.reset(new agc::Computer);
		fLGC->LoadRope(fLGCRope.Image().data(), fLGCRope.Image().size());
		fLGC->onChannelWrite = [this](int ch, uint16_t v) {
			fLGCDSKY.ChannelWritten(ch, v);
		};
		fLMPGNCS.reset(new gnc::LMPGNCS(*fLGC, fLM ? *fLM : *fCSM));
		fLGCPad.reset(new gnc::PadLoader(*fLGC, fLGCRope));
	}
	return true;
}


namespace {

// Key a string into a computer that runs on its own (setup only).
void
KeySequence(agc::Computer& c, const std::string& keys)
{
	for (char k : keys) {
		int code;
		switch (k) {
			case 'V': code = agc::KEY_VERB; break;
			case 'N': code = agc::KEY_NOUN; break;
			case 'E': code = agc::KEY_ENTR; break;
			case 'R': code = agc::KEY_RSET; break;
			case '+': code = agc::KEY_PLUS; break;
			case '-': code = agc::KEY_MINUS; break;
			default: code = k == '0' ? agc::KEY_0 : k - '0';
		}
		c.PressKey(code);
		c.Run(0.25);
	}
}

double
TephemFor(double getZeroJD)
{
	// Seconds from July 1.0 UT of the launch year (the AGC's epoch).
	int year = 1969;
	double jd = getZeroJD;
	for (int y = 1967; y <= 1973; y++)
		if (jd >= world::JulianDate(y, 7, 1))
			year = y;
	if (jd < world::JulianDate(year, 7, 1))
		year--;
	return (jd - world::JulianDate(year, 7, 1)) * kDay;
}

double
AzoFor(double getZeroJD)
{
	int year = 1969;
	for (int y = 1967; y <= 1973; y++)
		if (getZeroJD >= world::JulianDate(y, 7, 1))
			year = y;
	return world::Ephemeris::GMST(world::JulianDate(year, 7, 1)) / (2 * M_PI);
}

}  // namespace


void
FlightSim::_PadLoadCMC(const Mat3& refsmmat)
{
	agc::Computer& c = *fCMC;
	c.PowerOn();
	c.Run(1.0);
	KeySequence(c, "V36E");
	c.Run(3.0);
	gnc::PadLoader& p = *fCMCPad;
	p.Apply(gnc::MissionPads(fMission.padLoads, false));
	gnc::CMCPadLoad load;
	load.agcTime = fGET;
	load.tephem = TephemFor(fGETZeroJD);
	load.azo = AzoFor(fGETZeroJD);
	load.refsmmat = refsmmat;
	double jd = JulianDate();
	fMoonScene->SetTime(jd);
	Vec3 csmPos = fCSM->position, csmVel = fCSM->velocity;
	bool moon = (csmPos - fMoonScene->MoonPosition()).Length() < world::kMoonSOI;
	Vec3 origin = moon ? fMoonScene->MoonPosition() : Vec3();
	Vec3 originV = moon ? fMoonScene->MoonVelocity() : Vec3();
	load.csm.position = csmPos - origin;
	load.csm.velocity = csmVel - originV;
	load.csm.agcTime = fGET;
	load.csm.moon = moon;
	if (fLM) {
		load.lmState = true;
		load.lm.position = fLM->position - origin;
		load.lm.velocity = fLM->velocity - originV;
		load.lm.agcTime = fGET;
		load.lm.moon = moon;
		load.lmMass = fLM->Mass();
	}
	load.csmMass = fCSM->Mass();
	if (fLMDocked) {
		const Module* d = fCSM->FindModule("LM descent stage");
		const Module* a = fCSM->FindModule("LM ascent stage");
		load.lmMass = (d ? d->Mass() : 0) + (a ? a->Mass() : 0);
		load.csmMass -= load.lmMass;
		load.dapData1 = 021102;   // CSM + LM
	}
	gnc::LoadCMCMission(p, load);
	gnc::SetComputerClock(c, fGET);
	fCMPGNCS->imu.Align(refsmmat);
	// On the pad the CMC waits in P02 (gyrocompassing) for the liftoff
	// discrete, which starts P11.
	KeySequence(c, "V37E00E");
	c.Run(1.0);
	if (fOnPad) {
		// Prelaunch: with the IMU's discretes live, P01 levels the
		// platform and hands over to P02, gyrocompassing on the pad until
		// the liftoff discrete starts P11.
		auto run = [&](double s) {
			for (double t = 0; t < s; t += 0.01)
				fCMPGNCS->Step(0.01);
		};
		// The IMU turn-on sequence (cage, warm-up) blocks V37 for ~90 s.
		run(100.0);
		// Gyrocompass erasable load: X of the stable member downrange,
		// the pad's latitude, the azimuth of the vehicle's +Z (all in
		// revolutions).
		const Pad& pad = FindPad(fMission.pad);
		double az = FlightAzimuth() / 360.0;
		bool ok = p.Double("LAUNCHAZ", az, 0);
		ok = p.Double("AZIMUTH", az, 0) && ok;
		ok = p.Double("LATITUDE", pad.lat / 360.0, 0) && ok;
		if (!ok)
			Log(LogEntry::Info, "CMC: gyrocompass pad load incomplete");
		for (char k : std::string("V37E01E")) {
			int code = k == 'V' ? agc::KEY_VERB : k == 'E' ? agc::KEY_ENTR
				: k == '0' ? agc::KEY_0 : k - '0';
			c.PressKey(code);
			run(0.3);
		}
		run(1.0);
	}
	// The keying above ran the computer without the simulation's clock.
	gnc::SetComputerClock(c, fGET);
	fCMCDSKY.Sync(c);
}


void
FlightSim::_PadLoadLGC(const Mat3& refsmmat)
{
	if (!fLGC)
		return;
	agc::Computer& c = *fLGC;
	c.PowerOn();
	c.Run(1.0);
	KeySequence(c, "V36E");
	c.Run(3.0);
	gnc::PadLoader& p = *fLGCPad;
	gnc::LoadLuminaryInvariants(p);
	p.Apply(gnc::MissionPads(fMission.padLoads, true));
	gnc::LMPadLoad load;
	load.agcTime = fGET;
	load.tland = fMission.plan.landing;
	load.tephem = TephemFor(fGETZeroJD);
	load.azo = AzoFor(fGETZeroJD);
	load.rls = world::LatLonToVector(fMission.site.latitude,
		fMission.site.longitude, fMission.site.radius);
	load.refsmmat = refsmmat;
	double jd = JulianDate();
	fMoonScene->SetTime(jd);
	Vec3 origin = fMoonScene->MoonPosition(), originV = fMoonScene->MoonVelocity();
	Vehicle& lm = fLM ? *fLM : *fCSM;
	load.lm.position = lm.position - origin;
	load.lm.velocity = lm.velocity - originV;
	load.lm.agcTime = fGET;
	load.csm.position = fCSM->position - origin;
	load.csm.velocity = fCSM->velocity - originV;
	load.csm.agcTime = fGET;
	load.lmMass = fLM ? fLM->Mass() : 15000;
	load.csmMass = fLM ? fCSM->Mass() : fCSM->Mass() - 15000;
	load.onSurface = fLMLanded;
	// On the surface the LGC treats the LM as the ascent stage (the
	// landing program sets APSFLAG); the crew's V48 masses are ascent ones.
	load.descentStage = !fLMAscentOnly && !fLMLanded;
	if (fLMLanded && fLM) {
		const Module* a = fLM->FindModule("LM ascent stage");
		if (a)
			load.lmMass = a->Mass();
	}
	gnc::LoadLuminaryMission(p, load);
	gnc::SetComputerClock(c, fGET);
	p.Octal("DAPBOOLS", fLMDocked ? 002102 : 002002);
	fLMPGNCS->imu.Align(refsmmat);
	KeySequence(c, "V37E00E");
	c.Run(1.0);
	gnc::SetComputerClock(c, fGET);
	fLGCDSKY.Sync(c);
	fLGCPowered = true;
	lmSystems.powered = true;
}


void
FlightSim::PressCMC(int key)
{
	fCMC->PressKey(key);
}


void
FlightSim::PressLGC(int key)
{
	if (fLGC && fLGCPowered)
		fLGC->PressKey(key);
}


void
FlightSim::ProceedCMC(bool down)
{
	fCMC->SetProceed(down);
}


void
FlightSim::ProceedLGC(bool down)
{
	if (fLGC)
		fLGC->SetProceed(down);
}


// ---------------------------------------------------------------------------
// Start points


void
FlightSim::_PlaceOnPad()
{
	const Pad& pad = FindPad(fMission.pad);
	double jd = JulianDate();
	Mat3 ef = world::Ephemeris::EarthFixedToInertial(jd);
	Vec3 up = world::LatLonToVector(pad.lat, pad.lon, 1.0);
	fPadFixed = up * (world::kREarth + 30.0);
	Vec3 east = Vec3(0, 0, 1).Cross(up).Unit();
	Vec3 north = up.Cross(east);
	double az = FlightAzimuth() * kDeg;
	Vec3 downrange = north * std::cos(az) + east * std::sin(az);
	// +X up the stack, +Z toward the downrange side (the pitch program
	// rotates about +Y toward +Z... heads down).
	Vec3 x = up, z = downrange, y = z.Cross(x);
	fPadAttitude = Quat::FromMatrix(Mat3::Columns(x, y, z));
	fCSM->position = ef * fPadFixed;
	fCSM->velocity = Vec3(0, 0, world::kEarthRate).Cross(fCSM->position);
	fCSM->attitude = Quat::FromMatrix(ef) * fPadAttitude;
	fCSM->rates = Vec3();
	fCSM->landed = true;
	fOnPad = true;
}


bool
FlightSim::Start(StartPoint start, std::string* error)
{
	const FlightPlan& plan = fMission.plan;
	bool lunar = plan.tli > 0;
	double get = 0;
	switch (start) {
		case StartPoint::Launch: get = -600.0; break;
		case StartPoint::EarthOrbit: get = lunar ? plan.tli - 3600 : 3 * 3600; break;
		case StartPoint::TranslunarCoast: get = plan.tli + 4 * 3600; break;
		case StartPoint::LunarOrbit: get = plan.undock > 0 ? plan.undock - 3600
			: plan.loi + 6 * 3600; break;
		case StartPoint::PoweredDescent: get = plan.pdi - 600; break;
		case StartPoint::LunarSurface: get = plan.landing + 120; break;
		case StartPoint::LunarAscent: get = plan.liftoff - 600; break;
		case StartPoint::TransearthCoast: get = plan.tei + 2 * 3600; break;
		case StartPoint::Entry: get = plan.entry - 1800; break;
	}
	if ((start == StartPoint::PoweredDescent || start == StartPoint::LunarSurface
			|| start == StartPoint::LunarAscent) && plan.landing == 0) {
		if (error)
			*error = fMission.name + " did not land";
		return false;
	}
	if ((start != StartPoint::Launch && start != StartPoint::EarthOrbit
			&& start != StartPoint::Entry) && !lunar) {
		if (error)
			*error = fMission.name + " stayed in Earth orbit";
		return false;
	}
	fGET = get;
	_StateForStart(start);
	if (!_LoadComputers(error))
		return false;
	_UpdatePGNCSAttachments();

	// Platform alignment and the computers' pad loads.
	double jd = JulianDate();
	fMoonScene->SetTime(jd);
	Mat3 refsmmat;
	if (start == StartPoint::Launch) {
		// Launch REFSMMAT: X down the local vertical... the CMC's P11
		// expects the platform aligned to the launch azimuth at liftoff.
		Vec3 up = fCSM->ToInertial(Vec3(1, 0, 0));
		Vec3 dr = fCSM->ToInertial(Vec3(0, 0, 1));
		Vec3 x = -up, z = dr, y = z.Cross(x);
		refsmmat = Mat3::Rows(x, y, z);
	} else {
		bool moon = (fCSM->position - fMoonScene->MoonPosition()).Length()
			< world::kMoonSOI;
		Vec3 r = fCSM->position - (moon ? fMoonScene->MoonPosition() : Vec3());
		Vec3 v = fCSM->velocity - (moon ? fMoonScene->MoonVelocity() : Vec3());
		refsmmat = LVLHReference(r, v);
	}
	_PadLoadCMC(refsmmat);
	bool lgcOn = start == StartPoint::PoweredDescent
		|| start == StartPoint::LunarSurface || start == StartPoint::LunarAscent;
	if (lgcOn) {
		// Landing-site REFSMMAT: X up through the site at landing, Z
		// downrange (the orientation P63 expects).
		Vec3 site = fMoonScene->FixedToInertial() * world::LatLonToVector(
			fMission.site.latitude, fMission.site.longitude, 1.0);
		Vec3 vrel = fCSM->velocity - fMoonScene->MoonVelocity();
		Vec3 rrel = fCSM->position - fMoonScene->MoonPosition();
		Vec3 h = rrel.Cross(vrel).Unit();
		Vec3 downrange = h.Cross(site).Unit();
		Mat3 lmRef = Mat3::Rows(site, downrange.Cross(site), downrange);
		_PadLoadLGC(lmRef);
	}
	fIU.reset();
	boosterGuidance = nullptr;
	if (fBoosterAttached) {
		fIU.reset(new SaturnIU(*this));
		boosterGuidance = [this](FlightSim&, double dt) { fIU->Step(dt); };
	}
	Log(LogEntry::Event, fMission.name + ": " + StartPointName(start));
	return true;
}


void
FlightSim::_StateForStart(StartPoint start)
{
	const FlightPlan& plan = fMission.plan;
	double jd = JulianDate();
	fMoonScene->SetTime(jd);
	auto moonPos = [](double j) { return world::Ephemeris::MoonPosition(j); };
	auto moonVel = [](double j) { return world::Ephemeris::MoonVelocity(j); };

	// Lunar orbit geometry: retrograde, over the site at landing time.
	auto siteOrbit = [&](double atJD, double alt, double leadDeg, Vec3* r,
		Vec3* v) {
		world::Terrain t;
		MoonScene ms(t);
		double landJD = fGETZeroJD + plan.landing / kDay;
		ms.SetTime(landJD);
		Vec3 site = ms.FixedToInertial() * world::LatLonToVector(
			fMission.site.latitude, fMission.site.longitude, 1.0);
		Vec3 north = ms.FixedToInertial().Column(2);
		Vec3 h = (-north - site * (-north).Dot(site)).Unit();
		Vec3 downrange = h.Cross(site).Unit();
		double rc = world::kRMoon + alt;
		double vc = std::sqrt(world::kMuMoon / rc);
		double n = vc / rc;
		double angle = leadDeg * kDeg + n * (atJD - landJD) * kDay;
		Vec3 p = site * std::cos(angle) + downrange * std::sin(angle);
		Vec3 q = h.Cross(p);
		*r = moonPos(atJD) + p * rc;
		*v = moonVel(atJD) + q * vc;
	};
	auto attitudeAlong = [](const Vec3& rRel, const Vec3& vRel) {
		Vec3 x = vRel.Unit();
		Vec3 z = (-rRel).Unit();
		z = (z - x * z.Dot(x)).Unit();
		Vec3 y = z.Cross(x);
		return Quat::FromMatrix(Mat3::Columns(x, y, z));
	};

	fCSM.reset(new Vehicle(_MakeCSM(true)));
	fLM.reset();
	fBooster.reset();
	fDebris.clear();
	fLMDocked = fLMInSLA = fLMLanded = fLMAscentOnly = false;
	fBoosterAttached = false;
	fSMAttached = true;
	fCrewInCM = 3;
	fCrewInLM = 0;
	fCSM->jd = jd;

	switch (start) {
		case StartPoint::Launch: {
			fTowerAttached = true;
			fCSM->AddModule(craft::MakeLaunchEscapeTower());
			if (fHasLM) {
				Vehicle lm = _MakeLM(true);
				lm.velocity = fCSM->velocity;   // stowed: moving with the stack
				fCSM->Merge(lm, kLMInSLA, Mat3());
				fLMInSLA = true;
			}
			if (fMission.saturnV) {
				fCSM->AddModule(craft::MakeSIVB());
				fCSM->AddModule(craft::MakeSII());
				fCSM->AddModule(craft::MakeSIC());
			} else {
				Module sivb = craft::MakeSIVB(105000);
				sivb.dryMass = 12400;     // S-IVB-200 with its IU
				fCSM->AddModule(sivb);
				fCSM->AddModule(craft::MakeSIB());
			}
			holdDown = true;
			fBoosterAttached = true;
			_PlaceOnPad();
			fPhase = Phase::Prelaunch;
			break;
		}
		case StartPoint::EarthOrbit:
		case StartPoint::TranslunarCoast: {
			// Parking orbit in the plane that holds the Moon at arrival.
			const Pad& pad = FindPad(fMission.pad);
			Vec3 padR = world::Ephemeris::EarthFixedToInertial(fGETZeroJD)
				* world::LatLonToVector(pad.lat, pad.lon, 1.0);
			Vec3 n = ParkingOrbitNormal();
			double r = world::kREarth + 185200;
			double vc = std::sqrt(world::kMuEarth / r);
			double period = 2 * M_PI * std::sqrt(r * r * r / world::kMuEarth);
			double tIns = 709;
			Vec3 b = n.Cross(padR);
			auto orbitAt = [&](double get, Vec3* pos, Vec3* vel) {
				double th = 2100e3 / world::kREarth + 2 * M_PI * (get - tIns) / period;
				*pos = (padR * std::cos(th) + b * std::sin(th)) * r;
				*vel = (padR * -std::sin(th) + b * std::cos(th)) * vc;
			};
			if (start == StartPoint::EarthOrbit) {
				orbitAt(fGET, &fCSM->position, &fCSM->velocity);
				if (fHasLM) {
					Vehicle lm = _MakeLM(true);
					lm.velocity = fCSM->velocity;   // stowed: moving with the stack
					fCSM->Merge(lm, kLMInSLA, Mat3());
					fLMInSLA = true;
				}
				fCSM->AddModule(craft::MakeSIVB(73000));
				fBoosterAttached = true;
				fCSM->attitude = attitudeAlong(fCSM->position, fCSM->velocity);
				fPhase = Phase::EarthParkingOrbit;
			} else {
				Vec3 pos, vel;
				double tTLI = plan.tli - 160;
				orbitAt(tTLI, &pos, &vel);
				State s{pos, vel, fGETZeroJD + tTLI / kDay};
				Vec3 dv;
				double arrive;
				State ign;
				if (!TargetTLI(s, 111000, &dv, &arrive, &ign, 2700,
						fGETZeroJD + plan.loi / kDay)) {
					ign = s;
					dv = vel.Unit() * 3170;
				}
				ign.v += dv;
				State now = Propagate(ign, (jd - ign.jd) * kDay);
				fCSM->position = now.r;
				fCSM->velocity = now.v;
				if (fHasLM) {
					Vehicle lm = _MakeLM(true);
					Vec3 off;
					Mat3 rot;
					_DockLMGeometry(&off, &rot);
					lm.velocity = fCSM->velocity;   // stowed: moving with the stack
					fCSM->Merge(lm, off, rot);
					fLMDocked = true;
				}
				fCSM->attitude = attitudeAlong(now.r, now.v);
				// The spent S-IVB, slingshotted away.
				fBooster.reset(new Vehicle("S-IVB"));
				fBooster->AddModule(craft::MakeSIVB(2000));
				fBooster->position = now.r - now.v.Unit() * 3.0e5;
				fBooster->velocity = now.v * 0.995;
				fBooster->jd = jd;
				fPhase = Phase::TranslunarCoast;
			}
			break;
		}
		case StartPoint::LunarOrbit: {
			siteOrbit(jd, 111000, 0, &fCSM->position, &fCSM->velocity);
			if (fHasLM) {
				Vehicle lm = _MakeLM(true);
				Vec3 off;
				Mat3 rot;
				_DockLMGeometry(&off, &rot);
				lm.velocity = fCSM->velocity;   // stowed: moving with the stack
				fCSM->Merge(lm, off, rot);
				fLMDocked = true;
			}
			fCSM->attitude = attitudeAlong(fCSM->position - moonPos(jd),
				fCSM->velocity - moonVel(jd));
			fPhase = Phase::LunarOrbit;
			break;
		}
		case StartPoint::PoweredDescent: {
			// LM in the 60 x 8 nmi descent orbit, perilune ~16 deg uprange
			// of the site at PDI; the CSM in its 60 nmi orbit above.
			world::Terrain t;
			MoonScene ms(t);
			double landJD = fGETZeroJD + plan.landing / kDay;
			ms.SetTime(landJD);
			Vec3 site = ms.FixedToInertial() * world::LatLonToVector(
				fMission.site.latitude, fMission.site.longitude,
				fMission.site.radius);
			Vec3 north = ms.FixedToInertial().Column(2);
			Vec3 S = site.Unit();
			Vec3 h = (-north - S * (-north).Dot(S)).Unit();
			Vec3 downrange = h.Cross(S).Unit();
			double rp = fMission.site.radius + 15240, ra = world::kRMoon + 111120;
			double a = 0.5 * (rp + ra), e = (ra - rp) / (ra + rp);
			double theta = -16.2 * kDeg;
			Vec3 P = S * std::cos(theta) + downrange * std::sin(theta);
			Vec3 Q = h.Cross(P);
			double n = std::sqrt(world::kMuMoon / (a * a * a));
			double M0 = -n * (plan.pdi - fGET);
			double E = M0;
			for (int i = 0; i < 30; i++)
				E -= (E - e * std::sin(E) - M0) / (1 - e * std::cos(E));
			double nu = 2 * std::atan2(std::sqrt(1 + e) * std::sin(E / 2),
				std::sqrt(1 - e) * std::cos(E / 2));
			double rr = a * (1 - e * std::cos(E));
			double p = a * (1 - e * e);
			Vec3 relPos = (P * std::cos(nu) + Q * std::sin(nu)) * rr;
			Vec3 relVel = (P * -std::sin(nu) + Q * (e + std::cos(nu)))
				* std::sqrt(world::kMuMoon / p);
			fLM.reset(new Vehicle(_MakeLM(true)));
			fLM->position = moonPos(jd) + relPos;
			fLM->velocity = moonVel(jd) + relVel;
			fLM->jd = jd;
			Vec3 xb = (-relVel).Unit(), zb = (-relPos).Unit();
			zb = (zb - xb * zb.Dot(xb)).Unit();
			fLM->attitude = Quat::FromMatrix(Mat3::Columns(xb, zb.Cross(xb), zb));
			double rc = world::kRMoon + 111120;
			double vc = std::sqrt(world::kMuMoon / rc);
			Vec3 cp = (P * std::cos(nu + 0.05) + Q * std::sin(nu + 0.05)) * rc;
			Vec3 cv = (P * -std::sin(nu + 0.05) + Q * std::cos(nu + 0.05)) * vc;
			fCSM->position = moonPos(jd) + cp;
			fCSM->velocity = moonVel(jd) + cv;
			fCSM->attitude = attitudeAlong(cp, cv);
			fCrewInCM = 1;
			fCrewInLM = 2;
			fPhase = Phase::PoweredDescent;
			break;
		}
		case StartPoint::LunarSurface:
		case StartPoint::LunarAscent: {
			fLM.reset(new Vehicle(_MakeLM(true)));
			fLM->FindModule("LM descent stage")->tanks[0].mass = 800;
			double h = fTerrain.SurfaceRadius(fMission.site.latitude,
				fMission.site.longitude);
			fLMFixedPosition = world::LatLonToVector(fMission.site.latitude,
				fMission.site.longitude, h + 3.35 + 0.3);
			Vec3 up = fLMFixedPosition.Unit();
			Vec3 east = Vec3(0, 0, 1).Cross(up).Unit();
			// LM +X up, +Z facing west (down the approach path).
			Vec3 x = up, z = -east, y = z.Cross(x);
			fLMFixedAttitude = Quat::FromMatrix(Mat3::Columns(x, y, z));
			fLMLanded = true;
			fLM->landed = true;
			fLM->jd = jd;
			fMoonScene->SetTime(jd);
			fLM->position = fMoonScene->FromFixed(fLMFixedPosition);
			fLM->velocity = fMoonScene->SurfaceVelocity(fLM->position);
			fLM->attitude = Quat::FromMatrix(fMoonScene->FixedToInertial())
				* fLMFixedAttitude;
			// CSM overhead at landing; for the ascent, leading by ~20 deg.
			double lead = start == StartPoint::LunarAscent ? 20.0 : 0.0;
			double ref = start == StartPoint::LunarAscent
				? fGETZeroJD + plan.liftoff / kDay : jd;
			Vec3 r, v;
			siteOrbit(jd, 111000, lead, &r, &v);
			(void)ref;
			fCSM->position = r;
			fCSM->velocity = v;
			fCSM->attitude = attitudeAlong(r - moonPos(jd), v - moonVel(jd));
			fCrewInCM = 1;
			fCrewInLM = 2;
			fPhase = start == StartPoint::LunarSurface ? Phase::LunarSurface
				: Phase::LunarSurface;
			break;
		}
		case StartPoint::TransearthCoast: {
			Vec3 r, v;
			double teiJD = fGETZeroJD + plan.tei / kDay;
			siteOrbit(teiJD - 2 * 3600 / kDay, 111000, 0, &r, &v);
			State s{r, v, teiJD - 2 * 3600 / kDay};
			Vec3 dv;
			double ei;
			State burnAt;
			if (TargetTEI(s, -6.5 * kDeg, &dv, &ei, &burnAt)) {
				burnAt.v += dv;
				State now = Propagate(burnAt, (jd - burnAt.jd) * kDay);
				fCSM->position = now.r;
				fCSM->velocity = now.v;
			} else {
				fCSM->position = r;
				fCSM->velocity = v + (v - moonVel(s.jd)).Unit() * 1000;
			}
			fCSM->FindModule("Service Module")->tanks[0].mass =
				std::min(fMission.csmSpsPropellant, 2500.0);
			fCSM->attitude = attitudeAlong(fCSM->position, fCSM->velocity);
			fPhase = Phase::TransearthCoast;
			break;
		}
		case StartPoint::Entry: {
			double eiJD = fGETZeroJD + plan.entry / kDay;
			double splashJD = eiJD + 840 / kDay;
			Vec3 target = (world::Ephemeris::EarthFixedToInertial(splashJD)
				* world::LatLonToVector(fMission.splashLatitude,
					fMission.splashLongitude, 1.0)).Unit();
			Vec3 eastT = Vec3(0, 0, 1).Cross(target).Unit();
			Vec3 northT = target.Cross(eastT);
			double heading = 60 * kDeg;
			Vec3 dirT = northT * std::cos(heading) + eastT * std::sin(heading);
			Vec3 axis = target.Cross(dirT).Unit();
			double arc = 2250e3 / world::kREarth;
			Vec3 eiHat = target * std::cos(arc) - dirT * std::sin(arc);
			Vec3 along = axis.Cross(eiHat).Unit();
			double rEI = world::kREarth + 121920;
			double gamma = -6.48 * kDeg;
			State s{eiHat * rEI,
				(along * std::cos(gamma) + eiHat * std::sin(gamma)) * 11032, eiJD};
			State now = Propagate(s, (jd - eiJD) * kDay);
			fCSM->position = now.r;
			fCSM->velocity = now.v;
			fCSM->FindModule("Service Module")->tanks[0].mass = 300;
			Vec3 xb = (-now.v).Unit(), zb = (-now.r).Unit();
			zb = (zb - xb * zb.Dot(xb)).Unit();
			fCSM->attitude = Quat::FromMatrix(Mat3::Columns(xb, zb.Cross(xb), zb));
			fHasLM = false;
			fPhase = Phase::TransearthCoast;
			break;
		}
	}
	if (fLM)
		fLM->jd = jd;
	if (fBooster)
		fBooster->jd = jd;
	// Consumables already used, roughly in proportion to elapsed time.
	if (fGET > 3600) {
		double hours = fGET / 3600;
		for (CryoTank* t : {&csmSystems.o2[0], &csmSystems.o2[1]})
			t->quantity = std::max(20.0, t->quantity - hours * 0.4);
		for (CryoTank* t : {&csmSystems.h2[0], &csmSystems.h2[1]})
			t->quantity = std::max(2.0, t->quantity - hours * 0.04);
		crew.foodDays = std::max(1.0, crew.foodDays - hours / 24);
	}
}


// ---------------------------------------------------------------------------
// Hardware actions


bool
FlightSim::JettisonTower()
{
	if (!fTowerAttached)
		return false;
	Vehicle tower = fCSM->Detach({"Launch escape tower"}, Vec3(), Mat3(),
		"LES", 30);
	fDebris.emplace_back(new Vehicle(tower));
	fTowerAttached = false;
	Log(LogEntry::Event, "Tower jettison");
	return true;
}


bool
FlightSim::SeparateCSMFromBooster()
{
	if (!fBoosterAttached)
		return false;
	std::vector<std::string> stack;
	for (const Module& m : fCSM->Modules())
		if (m.name != "Command Module" && m.name != "Service Module"
			&& m.name != "Launch escape tower")
			stack.push_back(m.name);
	Vehicle booster = fCSM->Detach(stack, Vec3(), Mat3(), "S-IVB", -0.6);
	fBooster.reset(new Vehicle(booster));
	fBoosterAttached = false;
	_UpdatePGNCSAttachments();
	if (fLMInSLA && fLMPGNCS) {
		// The LM rides with the S-IVB until extraction.
		fLMPGNCS->Attach(*fBooster);
	}
	Log(LogEntry::Event, "CSM/LV separation");
	return true;
}


void
FlightSim::SCEToAux()
{
	csmSystems.sceAux = true;
	if (faults.sceFailed) {
		faults.sceFailed = false;
		Log(LogEntry::Crew, "SCE to AUX: the instrumentation is back");
	}
}


void
FlightSim::ResetFuelCells()
{
	int n = 0;
	for (FuelCell& fc : csmSystems.fuelCell)
		if (!fc.failed && !fc.online) {
			fc.online = true;
			n++;
		}
	if (n > 0)
		Log(LogEntry::Crew, "Fuel cells reconnected to the main buses");
}


void
FlightSim::PenInBreaker()
{
	if (fLMPGNCS && fLMPGNCS->ascentArmBreakerOpen) {
		fLMPGNCS->ascentArmBreakerOpen = false;
		Log(LogEntry::Crew, "Engine arm breaker pushed in with a felt-tip pen");
	}
}


void
FlightSim::RRModeToLGC()
{
	if (fLMPGNCS && fLMPGNCS->rrCDUHunting) {
		fLMPGNCS->rrCDUHunting = false;
		Log(LogEntry::Crew, "Rendezvous radar mode switch to LGC");
	}
}


Vec3
FlightSim::ParkingOrbitNormal() const
{
	const Pad& pad = FindPad(fMission.pad);
	Vec3 padR = world::Ephemeris::EarthFixedToInertial(fGETZeroJD)
		* world::LatLonToVector(pad.lat, pad.lon, 1.0);
	Vec3 east = Vec3(0, 0, 1).Cross(padR).Unit();
	Vec3 north = padR.Cross(east);
	double az = fMission.launchAzimuth * kDeg;
	Vec3 n = padR.Cross(north * std::cos(az) + east * std::sin(az)).Unit();
	if (fMission.plan.loi > 0) {
		Vec3 m = world::Ephemeris::MoonPosition(fGETZeroJD
			+ fMission.plan.loi / kDay).Unit();
		Vec3 n2 = padR.Cross(m).Unit();
		if (n2.Dot(n) < 0)
			n2 = -n2;
		n = n2;
	}
	return n;
}


double
FlightSim::FlightAzimuth() const
{
	const Pad& pad = FindPad(fMission.pad);
	Vec3 padR = world::Ephemeris::EarthFixedToInertial(fGETZeroJD)
		* world::LatLonToVector(pad.lat, pad.lon, 1.0);
	Vec3 east = Vec3(0, 0, 1).Cross(padR).Unit();
	Vec3 north = padR.Cross(east);
	Vec3 dir = ParkingOrbitNormal().Cross(padR).Unit();
	return std::atan2(dir.Dot(east), dir.Dot(north)) / kDeg;
}


bool
FlightSim::JettisonStage(const std::string& module, double separation)
{
	if (fCSM->FindModule(module) == nullptr)
		return false;
	// The stage's own frame is the stack frame; it leaves along -X.
	Vehicle stage = fCSM->Detach({module}, Vec3(), Mat3::RotZ(M_PI), module,
		separation);
	fDebris.emplace_back(new Vehicle(stage));
	Log(LogEntry::Event, module + " separation");
	return true;
}


bool
FlightSim::Docking(DockingTarget* out) const
{
	const Vehicle* target = nullptr;
	Vec3 local;
	if (fLMInSLA && fBooster) {
		target = fBooster.get();
		local = kLMInSLA + kLMDrogue;
	} else if (fLM && !fLMDocked) {
		target = fLM.get();
		local = kLMDrogue;
	}
	if (target == nullptr)
		return false;
	out->drogue = target->PointPosition(local);
	out->drogueVelocity = target->velocity
		+ target->ToInertial(target->rates.Cross(local - target->CenterOfMass()));
	out->axis = target->ToInertial(Vec3(1, 0, 0));
	out->probe = fCSM->PointPosition(kCMProbe);
	return true;
}


bool
FlightSim::DockingGeometry(double* range, double* rate, double* angle) const
{
	const Vehicle* target = nullptr;
	Vec3 drogue;
	Vec3 targetAxis;
	if (fLMInSLA && fBooster) {
		target = fBooster.get();
		drogue = target->PointPosition(kLMInSLA + kLMDrogue);
		targetAxis = target->ToInertial(Vec3(1, 0, 0));
	} else if (fLM && !fLMDocked) {
		target = fLM.get();
		drogue = target->PointPosition(kLMDrogue);
		targetAxis = target->ToInertial(Vec3(1, 0, 0));
	}
	if (target == nullptr)
		return false;
	Vec3 probe = fCSM->PointPosition(kCMProbe);
	Vec3 rel = drogue - probe;
	Vec3 relV = target->velocity - fCSM->velocity;
	*range = rel.Length();
	*rate = rel.Length() > 1e-6 ? relV.Dot(rel.Unit()) : 0;
	Vec3 csmAxis = fCSM->ToInertial(Vec3(1, 0, 0));
	*angle = std::acos(Clamp(-csmAxis.Dot(targetAxis), -1, 1)) / kDeg;
	return true;
}


bool
FlightSim::DockToLM()
{
	double range, rate, angle;
	if (!DockingGeometry(&range, &rate, &angle))
		return false;
	if (range > 0.35 || std::fabs(rate) > 0.5 || angle > 10)
		return false;
	if (faults.probeMisses > 0) {
		faults.probeMisses--;
		Log(LogEntry::Caution, "Contact, but no capture: the probe latches "
			"did not engage");
		// The probe pushes the two apart again.
		Vehicle* target = fLMInSLA ? fBooster.get() : fLM.get();
		if (target) {
			Vec3 axis = fCSM->ToInertial(Vec3(1, 0, 0));
			fCSM->velocity -= axis * 0.1;
		}
		return false;
	}
	Vec3 off;
	Mat3 rot;
	_DockLMGeometry(&off, &rot);
	if (fLMInSLA && fBooster) {
		// Hard dock to the LM still in the SLA: the S-IVB comes along
		// until extraction.
		Vehicle lmPart = fBooster->Detach({"LM descent stage", "LM ascent stage"},
			kLMInSLA, Mat3(), fMission.lmCallsign);
		fCSM->Merge(lmPart, off, rot);
		fLMInSLA = false;
		fLMDocked = true;
		// Leave the S-IVB just behind the LM, matched in velocity.
		fBooster->velocity = fCSM->velocity;
		Log(LogEntry::Event, "Docked with " + fMission.lmCallsign);
		Log(LogEntry::Event, "LM extraction");
	} else if (fLM) {
		// The capture latches pull the two into the nominal nose-to-nose
		// alignment (within the 10 degree capture cone).
		fLM->contacts.clear();
		fCSM->Merge(*fLM, off, rot);
		fLM.reset();
		fLMDocked = true;
		Log(LogEntry::Event, "Docked with " + fMission.lmCallsign);
	}
	_UpdatePGNCSAttachments();
	return true;
}


bool
FlightSim::ExtractLM()
{
	// Extraction happens as part of the hard dock to the SLA-stowed LM.
	return fLMDocked;
}


bool
FlightSim::UndockLM()
{
	if (!fLMDocked)
		return false;
	Vec3 off;
	Mat3 rot;
	_DockLMGeometry(&off, &rot);
	std::vector<std::string> parts;
	for (const Module& m : fCSM->Modules())
		if (m.name == "LM descent stage" || m.name == "LM ascent stage")
			parts.push_back(m.name);
	Vehicle lm = fCSM->Detach(parts, off, rot, fMission.lmCallsign, -0.15);
	craft::AddLMContacts(lm);
	fLM.reset(new Vehicle(lm));
	fLMDocked = false;
	_UpdatePGNCSAttachments();
	Log(LogEntry::Event, "Undocked: " + fMission.lmCallsign);
	return true;
}


bool
FlightSim::JettisonLM()
{
	if (fLMDocked) {
		UndockLM();
	}
	if (!fLM)
		return false;
	fDebris.emplace_back(fLM.release());
	fLGCPowered = false;
	Log(LogEntry::Event, fMission.lmCallsign + " jettisoned");
	return true;
}


bool
FlightSim::StageLM()
{
	if (!fLM || fLMAscentOnly)
		return false;
	Vehicle descent = fLM->Detach({"LM descent stage"}, Vec3(), Mat3(),
		fMission.lmCallsign + " descent stage");
	if (fLMLanded) {
		// The ascent stage sits on the descent stage (its launch pad)
		// until the ascent engine lifts it off.
		descent.landed = true;
	}
	fDebris.emplace_back(new Vehicle(descent));
	fLM->contacts.clear();
	if (faults.stagingGyration) {
		// The AGS, left in AUTO, swings the ascent stage to point at the
		// CSM the instant it's free.
		fLM->rates += Vec3(0.35, -0.25, 0.5);
		Log(LogEntry::Warning, "Ascent stage gyrating after staging");
		faults.stagingGyration = false;
	}
	fLMAscentOnly = true;
	lmSystems.descentStage = false;
	_UpdatePGNCSAttachments();
	Log(LogEntry::Event, "Staging: ascent stage free");
	return true;
}


bool
FlightSim::SeparateCMFromSM()
{
	if (!fSMAttached)
		return false;
	Vehicle sm = fCSM->Detach({"Service Module"}, Vec3(), Mat3(),
		"Service Module", -1.5);
	fDebris.emplace_back(new Vehicle(sm));
	fSMAttached = false;
	csmSystems.attached = false;
	for (Module& m : fCSM->Modules())
		for (Thruster& t : m.thrusters)
			t.failedOff = false;
	fCMPGNCS->panel.cmSmSeparated = true;
	csmSystems.batteryA.online = csmSystems.batteryB.online = true;
	csmSystems.batteryC.online = true;
	_UpdatePGNCSAttachments();
	Log(LogEntry::Event, "CM/SM separation");
	return true;
}


void
FlightSim::DeployDrogues()
{
	if (!fDrogues) {
		fDrogues = true;
		Log(LogEntry::Event, "Drogue parachutes");
	}
}


void
FlightSim::DeployMains()
{
	if (!fMains) {
		fMains = true;
		Log(LogEntry::Event, "Main parachutes");
	}
}


bool
FlightSim::TransferCrew(int toLM)
{
	if (!fLMDocked && !fLMInSLA)
		return false;
	int total = fCrewInCM + fCrewInLM;
	toLM = std::max(0, std::min(total, toLM));
	fCrewInLM = toLM;
	fCrewInCM = total - toLM;
	return true;
}


// ---------------------------------------------------------------------------
// Situation


Situation
FlightSim::Situate(const Vehicle& v) const
{
	Situation s;
	s.get = fGET;
	double jd = v.jd;
	s.moon = world::Ephemeris::MoonPosition(jd);
	s.sun = world::Ephemeris::SunPosition(jd);
	Vec3 mv = world::Ephemeris::MoonVelocity(jd);
	Vec3 rm = v.position - s.moon;
	s.nearMoon = rm.Length() < world::kMoonSOI;
	Vec3 r = s.nearMoon ? rm : v.position;
	Vec3 vel = s.nearMoon ? v.velocity - mv : v.velocity;
	double mu = s.nearMoon ? world::kMuMoon : world::kMuEarth;
	double R = s.nearMoon ? world::kRMoon : world::kREarth;
	s.altitude = r.Length() - R;
	s.speed = vel.Length();
	Conic c = ConicOf(r, vel, mu);
	s.periapsis = c.rp - R;
	s.apoapsis = c.ra > 0 ? c.ra - R : -1;
	s.inclination = c.inc / kDeg;
	if (s.nearMoon) {
		Vec3 fixed = world::Ephemeris::MoonFixedToInertial(jd).Transposed() * rm;
		double rr;
		world::VectorToLatLon(fixed, &s.latitude, &s.longitude, &rr);
	} else {
		Vec3 fixed = world::Ephemeris::EarthFixedToInertial(jd).Transposed()
			* v.position;
		double rr;
		world::VectorToLatLon(fixed, &s.latitude, &s.longitude, &rr);
	}
	return s;
}


// ---------------------------------------------------------------------------
// Stepping


bool
FlightSim::_CanCoast() const
{
	if (fPhase == Phase::Boost || fPhase == Phase::Prelaunch
		|| fPhase == Phase::PoweredDescent || fPhase == Phase::LunarAscent
		|| fPhase == Phase::Entry || fPhase == Phase::Lost)
		return false;
	if (MainThrust(*fCSM) || (fLM && MainThrust(*fLM)))
		return false;
	if (fIU && (fIU->CurrentMode() == SaturnIU::Mode::TLI
			|| (fIU->CurrentMode() == SaturnIU::Mode::TLIPrep
				&& fGET > fIU->TLIIgnitionGET() - 400)))
		return false;
	if (fCSM->position.Length() < world::kREarth + 150000)
		return false;
	if (fGET >= lockstepFrom - 0.5)
		return false;
	if (fLM && !fLMDocked && !fLMLanded) {
		if ((fLM->position - fCSM->position).Length() < 2000)
			return false;
	}
	if (fLMInSLA && fBooster
		&& (fBooster->position - fCSM->position).Length() < 2000)
		return false;
	return true;
}


double
FlightSim::MaxWarp() const
{
	if (fPhase == Phase::Lost || fPhase == Phase::Recovery)
		return 1;
	return _CanCoast() ? 100000 : fLockstepLimit;
}


void
FlightSim::SetWarp(double w)
{
	fWarp = std::max(0.1, std::min(w, 100000.0));
}


void
FlightSim::Advance(double realSeconds)
{
	if (fPaused || fPhase == Phase::Lost)
		return;
	double warp = std::min(fWarp, MaxWarp());
	if (warp != fWarp && fWarp > MaxWarp())
		fWarp = warp;
	Run(realSeconds * warp);
}


void
FlightSim::Run(double simSeconds)
{
	double remaining = simSeconds + fCarry;
	while (remaining >= kStep) {
		if (fPhase == Phase::Lost)
			break;
		if (fWarp > fLockstepLimit && _CanCoast() && remaining > 1.0) {
			double dt = std::min(remaining, kCoastStep);
			if (lockstepFrom < 1e29)
				dt = std::min(dt, std::max(kStep, lockstepFrom - 0.5 - fGET));
			_Coast(dt);
			remaining -= dt;
		} else {
			_Step(kStep);
			remaining -= kStep;
		}
	}
	fCarry = remaining;
}


void
FlightSim::_Step(double dt)
{
	fGET += dt;
	double jd = JulianDate();
	fMoonScene->SetTime(jd);
	Vec3 earthRate(0, 0, world::kEarthRate);

	auto stepVehicle = [&](Vehicle& v, bool isCM) {
		if (v.landed) {
			if (&v == fLM.get() && fLMLanded) {
				v.position = fMoonScene->FromFixed(fLMFixedPosition);
				v.velocity = fMoonScene->SurfaceVelocity(v.position);
				v.attitude = Quat::FromMatrix(fMoonScene->FixedToInertial())
					* fLMFixedAttitude;
				v.rates = Vec3();
			} else if (&v == fCSM.get() && (fOnPad || fSplashed)) {
				Mat3 ef = world::Ephemeris::EarthFixedToInertial(jd);
				v.position = ef * fPadFixed;
				v.velocity = earthRate.Cross(v.position);
				v.attitude = Quat::FromMatrix(ef) * fPadAttitude;
			}
			// Engines can still start (liftoff from either body).
			bool thrust = MainThrust(v);   // RCS pulses don't lift it off
			if (!thrust || (&v == fCSM.get() && fOnPad && holdDown)) {
				// Engines running up against the hold-down arms, or
				// just the accelerometers feeling the ground.
				v.Step(dt, [this](const Vec3& r, double j) {
					return _Gravity(r, j); });
				v.jd = jd;
				return;
			}
			v.jd = jd;
			v.landed = false;
			if (&v == fLM.get())
				fLMLanded = false;
			if (&v == fCSM.get())
				fOnPad = false;
		}
		double alt = v.position.Length() - world::kREarth;
		if (alt < 150000) {
			Vec3 air = v.velocity - earthRate.Cross(v.position);
			EntryAeroState st = ApplyAerodynamics(v, air, v.position.Unit(), alt,
				isCM);
			if (isCM)
				fEntry = st;
			world::Atmosphere a = world::Atmosphere::At(alt);
			v.ambientPressure = a.pressure;
			if (isCM && (fDrogues || fMains)) {
				double cdA = fMains ? (faults.mainChuteFouled ? 2 : 3)
					* 0.8 * 0.25 * M_PI * 25.5 * 25.5
					: 2 * 0.6 * 0.25 * M_PI * 5 * 5;
				v.AddExternalForce(air * (-0.5 * a.density * air.Length() * cdA),
					v.rates * -2000.0);
			}
		} else {
			v.ambientPressure = 0;
		}
		v.Step(dt, [this](const Vec3& r, double j) { return _Gravity(r, j); });
		// Julian dates don't survive millions of tiny increments: the
		// simulation's own clock (seconds) is the reference.
		v.jd = jd;
	};

	bool cmAlone = !fSMAttached;
	stepVehicle(*fCSM, cmAlone);
	if (fLM)
		stepVehicle(*fLM, false);
	if (fBooster)
		stepVehicle(*fBooster, false);
	for (auto& d : fDebris)
		if (!d->landed)
			stepVehicle(*d, false);

	if (boosterGuidance && (fBoosterAttached || fBooster))
		boosterGuidance(*this, dt);

	fCMPGNCS->Step(dt);
	if (fLMPGNCS && fLGCPowered)
		fLMPGNCS->Step(dt, fMoonScene.get());

	for (auto& hook : stepHooks)
		hook(*this, dt);

	_Contacts();
	fSecondAccum += dt;
	if (fSecondAccum >= 1.0) {
		_SecondTick();
		fSecondAccum -= 1.0;
	}
}


void
FlightSim::_Coast(double dt)
{
	fGET += dt;
	double jd = JulianDate();
	auto coast = [&](Vehicle& v) {
		if (v.landed) {
			if (&v == fLM.get() && fLMLanded) {
				fMoonScene->SetTime(jd);
				v.position = fMoonScene->FromFixed(fLMFixedPosition);
				v.velocity = fMoonScene->SurfaceVelocity(v.position);
				v.attitude = Quat::FromMatrix(fMoonScene->FixedToInertial())
					* fLMFixedAttitude;
			}
			v.jd = jd;
			return;
		}
		State s{v.position, v.velocity, v.jd};
		State e = Propagate(s, (jd - v.jd) * kDay);
		v.position = e.r;
		v.velocity = e.v;
		v.attitude = v.attitude.Integrated(v.rates, dt).Normalized();
		v.jd = jd;
	};
	coast(*fCSM);
	if (fLM)
		coast(*fLM);
	if (fBooster)
		coast(*fBooster);
	for (auto& d : fDebris)
		coast(*d);
	fMoonScene->SetTime(jd);
	// The computers' clocks run on; their programs are idle.
	if (boosterGuidance && (fBoosterAttached || fBooster))
		boosterGuidance(*this, dt);
	fCMC->AdvanceClock(dt);
	if (fLGC && fLGCPowered)
		fLGC->AdvanceClock(dt);
	fSecondAccum += dt;
	while (fSecondAccum >= 1.0) {
		double chunk = std::floor(fSecondAccum);
		// Systems in one larger step.
		double save = fSecondAccum;
		fSecondAccum = 1.0;
		_SecondTick();
		fSecondAccum = save - chunk;
		if (chunk > 1) {
			// Account the rest of the interval in one go.
			int crewCM = fCrewInCM, crewLM = fCrewInLM;
			csmSystems.Update(chunk - 1, crewCM, cmCabin, lioh);
			if (fHasLM && (fLM || fLMDocked))
				lmSystems.Update(chunk - 1, crewLM, lmCabin, lioh);
			crew.Update(chunk - 1, crewLM > 0 ? lmCabin : cmCabin,
				csmSystems.potableWater, sleepPeriod);
		}
	}
	_RemoveDistantDebris();
}


void
FlightSim::_Contacts()
{
	// LM touchdown.
	if (fLM && !fLMLanded && !fLMAscentOnly) {
		Vec3 up = fMoonScene->LocalVertical(fLM->position);
		for (const ContactPoint& c : fLM->contacts) {
			Vec3 pt = fLM->PointPosition(c.position);
			if (fMoonScene->Altitude(pt) > 0)
				continue;
			Vec3 vrel = fLM->velocity - fMoonScene->SurfaceVelocity(fLM->position);
			double vz = vrel.Dot(up);
			double vh = (vrel - up * vz).Length();
			double tilt = std::acos(Clamp(fLM->ToInertial(Vec3(1, 0, 0)).Dot(up),
				-1, 1)) / kDeg;
			double lat, lon, rr;
			fMoonScene->LatLon(fLM->position, &lat, &lon, &rr);
			double slope = fTerrain.Slope(lat, lon);   // degrees
			char buf[200];
			std::snprintf(buf, sizeof(buf), "Contact: %.1f ft/s down, %.1f ft/s"
				" across, %.1f deg tilt, %.1f deg slope", -vz / 0.3048,
				vh / 0.3048, tilt, slope);
			Log(LogEntry::Event, buf);
			if (vz < -5.0 || vh > 4.0 || tilt + slope > 30) {
				Lose(fMission.lmCallsign + " crashed on the lunar surface");
				return;
			}
			if (vz < -3.0 || vh > 1.5 || tilt + slope > 15) {
				Log(LogEntry::Warning, "Hard landing: gear stroked, the LM is "
					"damaged and leaning");
				crew.members[0].health -= 10;
				crew.members[2].health -= 10;
			}
			// Settle on the surface.
			fLMLanded = true;
			fLM->landed = true;
			for (Module& m : fLM->Modules())
				for (Thruster& t : m.thrusters) {
					t.on = false;
					t.level = 0;
				}
			double h = fTerrain.SurfaceRadius(lat, lon);
			Vec3 fixedPos = fMoonScene->ToFixed(fLM->position);
			fLMFixedPosition = fixedPos.Unit() * (h + 3.35 + 0.3);
			Vec3 x = fixedPos.Unit();
			Mat3 toFixed = fMoonScene->FixedToInertial().Transposed();
			Vec3 zf = toFixed * fLM->ToInertial(Vec3(0, 0, 1));
			zf = (zf - x * zf.Dot(x)).Unit();
			fLMFixedAttitude = Quat::FromMatrix(Mat3::Columns(x, zf.Cross(x), zf));
			SetPhase(Phase::LunarSurface);
			Log(LogEntry::Capcom, "We copy you down, " + fMission.lmCallsign + ".");
			break;
		}
	}
	// Lunar impact of anything else.
	if (fLM && !fLMLanded && fLMAscentOnly
		&& fMoonScene->Altitude(fLM->position) < 0) {
		Lose(fMission.lmCallsign + " ascent stage struck the Moon");
	}
	if (fMoonScene->Altitude(fCSM->position) < 0)
		Lose(fMission.csmCallsign + " struck the Moon");
	// Earth: splashdown or impact.
	double alt = fCSM->position.Length() - world::kREarth;
	if (!fOnPad && alt < 0 && !fSplashed) {
		Vec3 air = fCSM->velocity
			- Vec3(0, 0, world::kEarthRate).Cross(fCSM->position);
		fSplashed = true;
		char buf[120];
		std::snprintf(buf, sizeof(buf), "Splashdown at %.1f m/s", air.Length());
		Log(LogEntry::Event, buf);
		if (fSMAttached || air.Length() > 15) {
			Lose(fMission.csmCallsign + " hit the ocean too fast");
		} else {
			SetPhase(Phase::Recovery);
			// Afloat: ride the Earth's rotation from here.
			Mat3 ef = world::Ephemeris::EarthFixedToInertial(JulianDate());
			fPadFixed = ef.Transposed() * (fCSM->position.Unit() * world::kREarth);
			fPadAttitude = Quat::FromMatrix(ef.Transposed()) * fCSM->attitude;
			fCSM->landed = true;
			fCSM->rates = Vec3();
			Log(LogEntry::Capcom, fMission.csmCallsign
				+ ", Houston. Welcome home.");
		}
	}
	if (fSMAttached || fPhase == Phase::Boost)
		return;
	if (!fDrogues && alt < 7315 && fEntry.mach < 1.0 && fPhase == Phase::Entry)
		DeployDrogues();
	if (fDrogues && !fMains && alt < 3048)
		DeployMains();
	if (fEntry.gLoad > fMaxG)
		fMaxG = fEntry.gLoad;
}


void
FlightSim::_SecondTick()
{
	csmSystems.Update(1.0, fCrewInCM, cmCabin, lioh);
	if (fHasLM && (fLM || fLMDocked || fLMInSLA))
		lmSystems.Update(1.0, fCrewInLM, lmCabin, lioh);
	const Cabin& where = fCrewInLM > fCrewInCM ? lmCabin : cmCabin;
	double water = fCrewInLM > fCrewInCM ? lmSystems.WaterKg()
		: csmSystems.potableWater;
	crew.Update(1.0, where, water, sleepPeriod);
	for (auto& hook : secondHooks)
		hook(*this, 1.0);
	// Phase bookkeeping from the geometry.
	if (fPhase == Phase::TranslunarCoast) {
		Situation s = Situate(*fCSM);
		if (s.nearMoon && s.apoapsis > 0 && s.apoapsis < 2.0e7)
			SetPhase(Phase::LunarOrbit);
	} else if (fPhase == Phase::LunarOrbit || fPhase == Phase::Rendezvous) {
		Situation s = Situate(*fCSM);
		if (!s.nearMoon || s.apoapsis < 0)
			SetPhase(Phase::TransearthCoast);
	} else if (fPhase == Phase::TransearthCoast) {
		if (fCSM->position.Length() < world::kREarth + 121920)
			SetPhase(Phase::Entry);
	}
	if (crew.AverageHealth() < 5)
		Lose("The crew did not survive");
	if (cmCabin.pressure < 1.0 && fCrewInCM > 0)
		Lose("Cabin pressure lost");
	if (lmCabin.pressure < 1.0 && fCrewInLM > 0)
		Lose("LM cabin pressure lost");
}


void
FlightSim::_RemoveDistantDebris()
{
	fDebris.erase(std::remove_if(fDebris.begin(), fDebris.end(),
		[&](const std::unique_ptr<Vehicle>& d) {
			if (d->landed)
				return false;
			double r = d->position.Length();
			return r < world::kREarth || (d->position - fCSM->position).Length() > 5.0e8;
		}), fDebris.end());
}

}  // namespace flight
