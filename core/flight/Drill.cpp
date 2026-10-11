// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Drill.h"

#include "AutoCrew.h"
#include "MissionDirector.h"
#include "Saturn.h"

namespace flight {

using sim::Vec3;

const std::vector<DrillSpec>&
Drills()
{
	static const std::vector<DrillSpec> drills = {
		{DrillId::TranspositionDocking, "td", "Transposition and docking",
			"The S-IVB has finished TLI and is holding attitude. Separate "
			"(CSM/LV SEP), turn the CSM around, line the COAS up on the LM's "
			"docking target, close at a tenth of a metre a second and dock. "
			"Then LM EXTRACT.",
			StartPoint::EarthOrbit, 6},
		{DrillId::LOI, "loi", "Lunar orbit insertion",
			"Behind the Moon in twenty minutes. Houston's LOI-1 PAD is up: "
			"P30 with its TIG and delta-V (V37E30E), then P40, the attitude, "
			"and the SPS burns you into orbit. Too short and you're on a "
			"flyby; too long and the orbit dips into the hills.",
			StartPoint::TranslunarCoast, 120},
		{DrillId::LastFiveHundred, "p66", "The last 500 feet",
			"Five hundred feet over the site, P64 flying and the LPD scale on "
			"the window. Redesignate with the controller if you don't like "
			"where it's going, or go to P66 (ROD switch) and fly the rate of "
			"descent yourself. Fuel is short. Contact light, then ENG STOP.",
			StartPoint::PoweredDescent, 2},
		{DrillId::RendezvousInsertion, "rndz", "Rendezvous from insertion",
			"The ascent stage has just reached a 9 by 45 nautical mile orbit "
			"with the CSM ahead and above. Plan TPI (the CSM 26.6 degrees "
			"above the horizon), burn it with the RCS, two midcourses, brake "
			"along the line of sight and station-keep.",
			StartPoint::LunarAscent, 4},
		{DrillId::TPI, "tpi", "Terminal phase initiation",
			"Ten minutes to TPI on the direct (Apollo 14 style) rendezvous: "
			"the CSM about a hundred nautical miles ahead and above. Point at it, burn TPI along the line of sight, null "
			"the line-of-sight rates in the midcourses and brake in stages.",
			StartPoint::LunarAscent, 5},
		{DrillId::Docking, "dock", "Docking",
			"Sixty metres apart in lunar orbit, the LM station-keeping. "
			"You're the CMP: bring the probe into the LM's drogue at a tenth "
			"of a metre a second, centred and aligned.",
			StartPoint::LunarAscent, 6},
	};
	return drills;
}


const DrillSpec*
FindDrill(DrillId id)
{
	for (const DrillSpec& d : Drills())
		if (d.id == id)
			return &d;
	return nullptr;
}


const DrillSpec*
FindDrill(const std::string& key)
{
	for (const DrillSpec& d : Drills())
		if (key == d.key)
			return &d;
	return nullptr;
}


bool
DrillAvailable(const DrillSpec& drill, const MissionSpec& m)
{
	bool lunar = m.plan.tli > 0;
	bool lm = !m.lgcProgram.empty();
	bool landing = lm && m.plan.landing > 0;
	switch (drill.id) {
		case DrillId::None: return true;
		case DrillId::TranspositionDocking: return lunar && lm;
		case DrillId::LOI: return lunar;
		case DrillId::LastFiveHundred:
		case DrillId::RendezvousInsertion:
		case DrillId::TPI:
		case DrillId::Docking:
			return landing;
	}
	return false;
}


bool
DrillReady(const DrillSpec& drill, FlightSim& f, AutoCrew& crew,
	MissionDirector& director)
{
	Procedure* now = crew.Current();
	bool rendezvous = now && now->Name() == "Rendezvous";
	switch (drill.id) {
		case DrillId::None:
			return true;
		case DrillId::TranspositionDocking: {
			SaturnIU* iu = f.IU();
			return iu && iu->CurrentMode() == SaturnIU::Mode::Coast
				&& f.LMInSLA();
		}
		case DrillId::LOI:
			for (const ManeuverPAD& pad : director.PADs())
				if (pad.purpose == "LOI-1" && f.GET() >= pad.tig - 1200)
					return f.CurrentPhase() == Phase::TranslunarCoast;
			return false;
		case DrillId::LastFiveHundred: {
			sim::Vehicle* lm = f.LMVehicle();
			if (!lm || f.LMLanded() || f.CurrentPhase() != Phase::PoweredDescent)
				return false;
			Vec3 fixed = f.Moon().ToFixed(lm->position);
			double lat, lon, r;
			sim::world::VectorToLatLon(fixed, &lat, &lon, &r);
			double h = r - f.Terrain().SurfaceRadius(lat, lon);
			return h < 152.4;
		}
		case DrillId::RendezvousInsertion:
			return rendezvous && f.LMVehicle() && !f.LMLanded();
		case DrillId::TPI: {
			if (!rendezvous)
				return false;
			double wake = now->WakeGET(f);
			// Coasting to TPI (wakes two minutes ahead of it): ten minutes
			// out.
			return wake > f.GET() + 60 && f.GET() >= wake + 120 - 600;
		}
		case DrillId::Docking: {
			if (rendezvous || !f.LMVehicle() || f.LMDocked() || f.LMLanded())
				return false;
			double range, rate, angle;
			return f.DockingGeometry(&range, &rate, &angle) && range < 60;
		}
	}
	return false;
}

}  // namespace flight
