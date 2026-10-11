// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Recorder.h"

#include "FlightSim.h"

#include <cmath>
#include <cstring>

namespace flight {

using namespace sim;

namespace {

uint16_t
ModulesOf(const Vehicle& v)
{
	uint16_t m = 0;
	for (const Module& mod : v.Modules()) {
		const std::string& n = mod.name;
		if (n == "S-IC")
			m |= REC_SIC;
		else if (n == "S-II")
			m |= REC_SII;
		else if (n == "S-IVB")
			m |= REC_SIVB;
		else if (n == "S-IB")
			m |= REC_SIB;
		else if (n == "Service Module")
			m |= REC_SM;
		else if (n == "Command Module")
			m |= REC_CM;
		else if (n == "LM descent stage")
			m |= REC_LM_DESCENT;
		else if (n == "LM ascent stage")
			m |= REC_LM_ASCENT;
		else if (n == "Launch escape tower")
			m |= REC_TOWER;
	}
	return m;
}

void
Fill(RecVehicle* out, const Vehicle* v)
{
	*out = RecVehicle();
	if (v == nullptr)
		return;
	out->flags = REC_PRESENT;
	out->modules = ModulesOf(*v);
	out->r = v->position;
	out->v = v->velocity;
	out->q = v->attitude;
	for (const Module& m : v->Modules())
		for (const Thruster& t : m.thrusters) {
			bool on = t.on || t.level > 0.05 || t.failedOn;
			if (!on)
				continue;
			out->flags |= t.kind == THRUSTER_MAIN ? REC_ENGINE : REC_RCS;
		}
	if (v->landed)
		out->flags |= REC_LANDED;
}

RecDisplay
FromDSKY(const agc::DSKYState& s, double get)
{
	RecDisplay d;
	d.get = get;
	std::memcpy(d.prog, s.prog, 2);
	std::memcpy(d.verb, s.verb, 2);
	std::memcpy(d.noun, s.noun, 2);
	std::memcpy(d.reg, s.reg, sizeof(d.reg));
	std::memcpy(d.sign, s.sign, 3);
	uint32_t l = 0;
	if (s.compActy) l |= LAMP_COMP_ACTY;
	if (s.uplinkActy) l |= LAMP_UPLINK;
	if (s.temp) l |= LAMP_TEMP;
	if (s.keyRel) l |= LAMP_KEY_REL;
	if (s.verbNounFlash) l |= LAMP_FLASH;
	if (s.oprErr) l |= LAMP_OPR_ERR;
	if (s.noAtt) l |= LAMP_NO_ATT;
	if (s.gimbalLock) l |= LAMP_GIMBAL_LOCK;
	if (s.progAlarm) l |= LAMP_PROG;
	if (s.tracker) l |= LAMP_TRACKER;
	if (s.restart) l |= LAMP_RESTART;
	if (s.standby) l |= LAMP_STBY;
	if (s.alt) l |= LAMP_ALT;
	if (s.vel) l |= LAMP_VEL;
	if (s.prioDisp) l |= LAMP_PRIO_DISP;
	if (s.noDap) l |= LAMP_NO_DAP;
	d.lamps = l;
	return d;
}

}  // namespace


FlightRecorder::FlightRecorder(FlightSim& sim)
	:
	f(sim)
{
	const MissionSpec& m = sim.Mission();
	RecHeader& h = fRec.header;
	h.missionNumber = m.number;
	h.missionName = m.name;
	h.csmCallsign = m.csmCallsign;
	h.lmCallsign = m.lmCallsign;
	h.siteName = m.site.name;
	h.siteLatitude = m.site.latitude;
	h.siteLongitude = m.site.longitude;
	h.siteRadius = m.site.radius;
	h.splashLatitude = m.splashLatitude;
	h.splashLongitude = m.splashLongitude;
	h.getZeroJD = sim.JulianDate() - sim.GET() / 86400.0;
}


void
FlightRecorder::Attach()
{
	if (fAttached)
		return;
	fAttached = true;
	f.logHooks.push_back([this](const LogEntry& e) {
		RecEvent ev;
		ev.get = e.get;
		ev.kind = uint8_t(e.kind);
		ev.text = e.text;
		fRec.events.push_back(ev);
		// Every event gets a sample, so the replay shows it where it was.
		_Sample();
	});
	f.stepHooks.push_back([this](FlightSim& sim, double) {
		double get = sim.GET();
		double every = _Busy() ? busyInterval : quietInterval;
		if (get - fLastSample >= every)
			_Sample();
		if (get - fLastDisplay >= 0.25)
			_Displays();
	});
	f.secondHooks.push_back([this](FlightSim& sim, double) {
		// Coasts step in large chunks: the per-second hook still runs.
		double get = sim.GET();
		double every = _Busy() ? busyInterval : quietInterval;
		if (get - fLastSample >= every)
			_Sample();
		if (get - fLastDisplay >= 0.25)
			_Displays();
	});
	_Sample();
	_Displays();
}


bool
FlightRecorder::_Busy() const
{
	switch (f.CurrentPhase()) {
		case Phase::Prelaunch:
		case Phase::Boost:
		case Phase::TranslunarInjection:
		case Phase::PoweredDescent:
		case Phase::LunarAscent:
		case Phase::Rendezvous:
		case Phase::Entry:
			return true;
		default:
			break;
	}
	if (f.CSMVehicle().Modules().empty())
		return false;
	const Vehicle& csm = f.CSMVehicle();
	for (const Module& m : csm.Modules())
		for (const Thruster& t : m.thrusters)
			if (t.kind == THRUSTER_MAIN && (t.on || t.level > 0))
				return true;
	if (const Vehicle* lm = const_cast<FlightSim&>(f).LMVehicle()) {
		if ((lm->position - csm.position).Length() < 5000)
			return true;
		for (const Module& m : lm->Modules())
			for (const Thruster& t : m.thrusters)
				if (t.kind == THRUSTER_MAIN && (t.on || t.level > 0))
					return true;
	}
	if (const Vehicle* b = const_cast<FlightSim&>(f).BoosterVehicle())
		if ((b->position - csm.position).Length() < 2000)
			return true;
	return false;
}


void
FlightRecorder::_Sample()
{
	double get = f.GET();
	RecSample s;
	s.get = get;
	s.phase = uint8_t(f.CurrentPhase());
	s.moon = f.Moon().MoonPosition();
	s.moonV = f.Moon().MoonVelocity();
	Fill(&s.veh[REC_CSM], &f.CSMVehicle());
	Fill(&s.veh[REC_LM], f.LMVehicle());
	Fill(&s.veh[REC_BOOSTER], f.BoosterVehicle());
	if (f.DroguesOut())
		s.veh[REC_CSM].flags |= REC_DROGUES;
	if (f.MainsOut())
		s.veh[REC_CSM].flags |= REC_MAINS;
	const Vehicle& crewed = (f.LMVehicle() && f.CrewInLM() > 0)
		? *f.LMVehicle() : f.CSMVehicle();
	Situation sit = f.Situate(crewed);
	if (sit.nearMoon) {
		s.altitude = f.Moon().Altitude(crewed.position);
		s.speed = (crewed.velocity - s.moonV).Length();
	} else {
		s.altitude = crewed.position.Length() - world::kREarth;
		// Relative to the turning Earth near it, inertial far out.
		Vec3 air = crewed.velocity - Vec3(-world::kEarthRate * crewed.position.y,
			world::kEarthRate * crewed.position.x, 0);
		s.speed = s.altitude < 200e3 ? air.Length() : crewed.velocity.Length();
	}
	// Two samples at the same moment: keep the later.
	if (!fRec.samples.empty() && fRec.samples.back().get >= get)
		fRec.samples.back() = s;
	else
		fRec.samples.push_back(s);
	fLastSample = get;
}


void
FlightRecorder::_Displays()
{
	double get = f.GET();
	fLastDisplay = get;
	RecDisplay c = FromDSKY(f.CMCDSKY().State(), get);
	if (fRec.cmc.empty() || !fRec.cmc.back().SameAs(c))
		fRec.cmc.push_back(c);
	if (f.LGC() && f.LGCPowered()) {
		RecDisplay l = FromDSKY(f.LGCDSKY().State(), get);
		if (fRec.lgc.empty() || !fRec.lgc.back().SameAs(l))
			fRec.lgc.push_back(l);
	}
}


void
FlightRecorder::Finish(const std::string& outcome, bool success)
{
	_Sample();
	_Displays();
	fRec.header.outcome = outcome;
	fRec.header.success = success;
}

}  // namespace flight
