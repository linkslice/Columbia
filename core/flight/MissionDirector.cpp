// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "MissionDirector.h"

#include "../sim/Spacecraft.h"

#include "FlightSim.h"
#include "Saturn.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace flight {

using namespace sim;

namespace {

const double kH = 3600.0;
const double kSPSThrust = 91188, kSPSIsp = 314;

std::string
FormatGET(double get)
{
	char b[32];
	long t = long(get);
	std::snprintf(b, sizeof(b), "%03ld:%02ld:%02ld", t / 3600, t / 60 % 60, t % 60);
	return b;
}

// Time (GET) of the next apsis after `from`: periapsis if `low`.
double
NextApsis(const State& s, double nowGET, double from, bool low, double* radius)
{
	State a = Propagate(s, from - nowGET);
	double prev = (a.r - MoonPos(a.jd)).Length(), prevPrev = prev;
	double t = from;
	for (int i = 0; i < 2000; i++) {
		State b = Propagate(a, 10);
		double r = (b.r - MoonPos(b.jd)).Length();
		bool turned = low ? (prev < prevPrev && prev < r) : (prev > prevPrev && prev > r);
		if (i > 1 && turned) {
			*radius = prev;
			return t;
		}
		prevPrev = prev;
		prev = r;
		a = b;
		t += 10;
	}
	*radius = prev;
	return t;
}

}  // namespace


std::string
ManeuverPAD::Text() const
{
	char b[256];
	std::snprintf(b, sizeof(b), "%s PAD: %s, TIG %s, dVX %+.1f dVY %+.1f dVZ %+.1f "
		"ft/s, dVT %.1f ft/s, burn %d:%02d", purpose.c_str(), engine.c_str(),
		FormatGET(tig).c_str(), dvLVLH.x / 0.3048, dvLVLH.y / 0.3048,
		dvLVLH.z / 0.3048, dvMagnitude / 0.3048, int(burnTime) / 60,
		int(burnTime) % 60);
	return b;
}


MissionDirector::MissionDirector(FlightSim& sim, AutoCrew& crew)
	:
	f(sim),
	fCrew(crew)
{
	fSplashLat = sim.Mission().splashLatitude;
	fSplashLon = sim.Mission().splashLongitude;
	_Build();
	if (const char* env = std::getenv("COLUMBIA_CHOICES")) {
		std::string all = env;
		size_t at = 0;
		while (at < all.size()) {
			size_t comma = all.find(',', at);
			std::string item = all.substr(at, comma == std::string::npos
				? std::string::npos : comma - at);
			size_t eq = item.find('=');
			if (eq != std::string::npos)
				presetChoices[item.substr(0, eq)] = item.substr(eq + 1);
			if (comma == std::string::npos)
				break;
			at = comma + 1;
		}
	}
	// Starting partway through the flight: what's long past is done.
	for (Step& s : fSteps)
		if (s.get > 0 && s.get < f.GET() - 2 * kH)
			s.done = true;
	if (f.LMDocked() || !f.HasLM())
		for (Step& s : fSteps)
			if (s.name == "Transposition and docking")
				s.done = true;
	Phase ph = f.CurrentPhase();
	// Starting in the LM's part of the flight: the descent (or the stay)
	// and everything after it.
	if (ph == Phase::PoweredDescent || ph == Phase::LunarSurface
			|| ph == Phase::LunarAscent) {
		for (Step& s : fSteps)
			s.done = true;
		const FlightPlan& p = f.Mission().plan;
		if (ph == Phase::PoweredDescent) {
			fPDI = p.pdi;
			_Add("Powered descent", 0, [this]() {
				if (autopilot)
					fCrew.Add(AutoCrew::Landing(fPDI));
			});
		} else
			fLanded = p.landing;
		_AddSurface();
	}
	// Already on the way home: everything before the midcourses is done,
	// and the entry interface comes from the tracking.
	if (ph == Phase::TransearthCoast || ph == Phase::Entry) {
		bool haveReturn = false;
		for (const Step& s : fSteps)
			haveReturn |= s.name == "MCC-5";
		if (!haveReturn) {
			for (Step& s : fSteps)
				s.done = true;
			_AddReturn();
		}
		for (Step& s : fSteps) {
			if (s.name == "MCC-5")
				break;
			s.done = true;
		}
		Interface in = EntryInterface(_Now(), 4 * 86400);
		if (in.found)
			fEI = (in.jd - (f.JulianDate() - f.GET() / 86400)) * 86400;
	}
	fBuilt = true;
}


void
MissionDirector::Install()
{
	if (fInstalled)
		return;
	fInstalled = true;
	f.secondHooks.push_back([this](FlightSim&, double) { Tick(); });
}


void
MissionDirector::_Add(const std::string& name, double get,
	std::function<void()> run, std::function<bool()> when)
{
	Step s;
	s.name = name;
	s.get = get;
	s.run = run;
	s.when = when;
	// Once the flight is under way, steps may be added while a step runs
	// (the step list can't grow under the loop): they join next tick.
	if (fBuilt)
		fLater.push_back(s);
	else
		fSteps.push_back(s);
}


void
MissionDirector::_AddLater(const std::string& name, double get,
	std::function<void()> run, std::function<bool()> when)
{
	Step s;
	s.name = name;
	s.get = get;
	s.run = run;
	s.when = when;
	fLater.push_back(s);
}


// ---------------------------------------------------------------------------
// Flight's calls


const Decision*
MissionDirector::PendingDecision() const
{
	for (const Decision& d : fDecisions)
		if (d.chosen < 0)
			return &d;
	return nullptr;
}


bool
MissionDirector::Decide(const std::string& id, const std::string& key,
	const std::string& by)
{
	for (size_t i = 0; i < fDecisions.size(); i++) {
		Decision& d = fDecisions[i];
		if (d.id != id || d.chosen >= 0)
			continue;
		for (size_t k = 0; k < d.options.size(); k++) {
			if (d.options[k].key != key)
				continue;
			d.chosen = int(k);
			d.chosenBy = by;
			const DecisionOption& o = d.options[k];
			f.Log(LogEntry::Event, "FLIGHT: " + d.title + " -- " + o.label
				+ (o.historical ? " (as flown)" : " (not as flown)"));
			auto then = fDecisionThen[i];
			then(key);
			return true;
		}
	}
	return false;
}


void
MissionDirector::_Ask(Decision d, std::function<void(const std::string&)> then)
{
	d.postedGET = f.GET();
	fDecisions.push_back(d);
	fDecisionThen.push_back(then);
	f.Log(LogEntry::Capcom, "Flight: " + d.title + ". " + d.situation);
	for (const DecisionOption& o : d.options)
		f.Log(LogEntry::Info, "  " + o.label + ": " + o.detail);
	std::string key, by;
	auto preset = presetChoices.find(d.id);
	if (preset != presetChoices.end()) {
		key = preset->second;
		by = "preset";
	} else if (!waitForDecisions) {
		for (const DecisionOption& o : d.options)
			if (o.historical)
				key = o.key;
		if (key.empty())
			key = d.options.front().key;
		by = "Flight (as flown)";
	}
	if (onDecision)
		onDecision(fDecisions.back());
	if (!key.empty())
		Decide(d.id, key, by);
}


void
MissionDirector::_CheckEmergencies()
{
	// The service module damaged on the way out (Apollo 13): abort.
	if (!fEmergencyAsked && f.faults.smDamaged && f.HasLM() && f.LMDocked()
			&& f.CurrentPhase() == Phase::TranslunarCoast) {
		fEmergencyAsked = true;
		const LMSystems& lm = f.lmSystems;
		char lmLine[200];
		std::snprintf(lmLine, sizeof(lmLine), "%s carries %.0f kg of oxygen, "
			"%.0f kg of water and %.0f amp-hours: built for two men for two days.",
			f.Mission().lmCallsign.c_str(), lm.OxygenKg(), lm.WaterKg(),
			lm.AmpHours());
		Decision d;
		d.id = "sm-abort";
		d.title = "Abort the mission";
		d.situation = std::string("The service module has lost its oxygen and two "
			"fuel cells; the CM has its entry batteries and a little surge-tank "
			"oxygen. There is no landing. ") + lmLine;
		d.options.push_back({"lifeboat", "Around the Moon in the LM lifeboat",
			"Power up the LM, move all three crew in, and burn its descent engine "
			"to get back on a free return around the Moon. Home in about four "
			"days -- if the LM's water, power and CO2 scrubbers hold out.", true});
		d.options.push_back({"direct", "Direct abort with the SPS",
			"Turn around now: jettison the LM (the stack is too heavy for the SPS "
			"to reverse), a burn of about 5,000 ft/s, home in about two days. "
			"The SPS sits beside the blown-out panel, and the crew rides home in "
			"a dead CM with no lifeboat.", false});
		_Ask(d, [this](const std::string& key) { _SMDamageAbort(key); });
	}
	// In the lifeboat: the square-in-round adapter when the LM's CO2 climbs.
	if (fLifeboat && !f.lioh.squareInRound && f.lmCabin.co2 > 7.0) {
		f.lioh.squareInRound = true;
		f.Log(LogEntry::Capcom, f.Mission().lmCallsign + ", Houston. We'd like you"
			" to build a little adapter for the CM's lithium hydroxide canisters: "
			"a flight plan cover, a sock, two bags and the gray tape.");
		f.Log(LogEntry::Crew, "LMP: Mailbox is up and running. CO2's coming down.");
	}
}


void
MissionDirector::_EnterLifeboat()
{
	fLifeboat = true;
	if (!f.LGCPowered())
		f.PowerUpLGC();
	f.TransferCrew(f.CrewInCM() + f.CrewInLM());
	// The CM shut down to save its batteries for entry.
	CSMSystems& cs = f.csmSystems;
	cs.poweredDown = true;
	cs.powerTransfer = true;
	cs.loads.computer = cs.loads.imu = cs.loads.scs = cs.loads.comms = false;
	cs.loads.heaters = cs.loads.cabinFans = cs.loads.lights = false;
	cs.loads.glycolPumps = cs.loads.optics = cs.loads.radar = cs.loads.tv = false;
	f.CMPGNCS().panel.scContCMC = false;
	f.CMPGNCS().panel.mode = gnc::CMPanel::MODE_FREE;
	f.CMPGNCS().imu.powered = false;     // the CM platform off for the trip
	f.Log(LogEntry::Capcom, f.Mission().lmCallsign + ", Houston. Get the LM powered"
		" up; we want you to power down the CM and move in.");
	f.Log(LogEntry::Crew, "CDR: Okay, we're in the LM. The CM is powered down.");
}


void
MissionDirector::_SMDamageAbort(const std::string& choice)
{
	fAborted = true;
	for (Step& st : fSteps)
		st.done = true;
	double jd0 = f.JulianDate() - f.GET() / 86400;
	if (choice == "direct") {
		// The LM goes; the SPS turns them around.
		_AddLater("LM jettison for the abort", 0, [this]() {
			f.TransferCrew(0);
			f.JettisonLM();
			f.Log(LogEntry::Crew, "CDR: LM jettison. We're on our own.");
		});
		_AddLater("Direct abort", 0, [this, jd0]() {
			double tig = f.GET() + 1.5 * kH;
			State at = Propagate(_Now(), tig - f.GET());
			Vec3 dv;
			Interface entry;
			if (!TargetReturn(at, -at.v, -6.5 * kDeg, 0, 2400, &dv, &entry)) {
				f.Log(LogEntry::Capcom, "Houston: no direct abort solution. "
					"We'll go around the Moon.");
				_AddLater("Free return", 0, [this]() {});
				_AddEarthReturn();
				return;
			}
			fEI = (entry.jd - jd0) * 86400;
			char buf[160];
			std::snprintf(buf, sizeof(buf), "RTCC: direct abort %.0f ft/s, entry "
				"interface at %s", dv.Length() / 0.3048, FormatGET(fEI).c_str());
			f.Log(LogEntry::Info, buf);
			_Burn("Direct abort", tig, dv, false);
			_AddEarthReturn();
		});
		return;
	}
	_EnterLifeboat();
	_AddLifeboatHome(true);
}


void
MissionDirector::_AddLifeboatHome(bool pc2Decision)
{
	double jd0 = f.JulianDate() - f.GET() / 86400;
	// DPS-1: back onto a free return (Apollo 13's at 61:30, 38 ft/s).
	_AddLater("DPS-1: free return", 0, [this, jd0]() {
		double tig = f.GET() + 2 * kH;
		State at = Propagate(_Now(), tig - f.GET());
		Vec3 dv;
		Interface entry;
		if (!TargetReturn(at, at.v, -6.5 * kDeg, 0, 300, &dv, &entry)) {
			f.Log(LogEntry::Capcom, "Houston: still working the free-return burn.");
			for (Step& st : fSteps)
				if (st.name == "DPS-1: free return") {
					st.done = false;
					st.get = f.GET() + 1800;
				}
			return;
		}
		fEI = (entry.jd - jd0) * 86400;
		State post = at;
		post.v += dv;
		Approach ap = ClosestMoonApproach(post, 3 * 86400);
		fPericynthion = ap.found ? (ap.jd - jd0) * 86400 : tig + 15 * kH;
		char buf[200];
		std::snprintf(buf, sizeof(buf), "RTCC: free return %.1f ft/s; pericynthion "
			"%s at %.0f nmi, entry interface %s", dv.Length() / 0.3048,
			FormatGET(fPericynthion).c_str(), (ap.radius - world::kRMoon) / 1852,
			FormatGET(fEI).c_str());
		f.Log(LogEntry::Info, buf);
		_Burn("DPS-1", tig, dv, true);
	}, [this]() { return !fCrew.Busy(); });
	if (!pc2Decision) {
		_AddEarthReturn();
		return;
	}
	// Before pericynthion: speed the return up, and where to come down.
	_AddLater("PC+2 decision", 0, [this, jd0]() {
		const LMSystems& lm = f.lmSystems;
		double hoursHome = (fEI - f.GET()) / 3600;
		char state[240];
		std::snprintf(state, sizeof(state), "On the free return, entry is in %.0f "
			"hours, in the Indian Ocean. %s has %.0f hours of water and %.0f of "
			"power at the present rates.", hoursHome,
			f.Mission().lmCallsign.c_str(), lm.WaterHours(), lm.PowerHours());
		Decision d;
		d.id = "pc2";
		d.title = "The PC+2 burn";
		d.situation = state;
		d.options.push_back({"pacific", "PC+2 to the Pacific",
			"A DPS burn two hours after pericynthion of about 860 ft/s: about ten "
			"hours sooner, and the recovery fleet in the mid-Pacific.", true});
		d.options.push_back({"fast", "Fast return, SM jettisoned",
			"Jettison the service module first, then a much bigger DPS burn: home "
			"about a day sooner. The CM's heat shield, sheltered by the SM until "
			"now, spends days in deep cold.", false});
		d.options.push_back({"none", "No burn: ride the free return",
			"Save the DPS for corrections. The longest way home, with the LM's "
			"consumables right at their limits, and a splash where the fleet "
			"isn't.", false});
		_Ask(d, [this, jd0](const std::string& key) {
			if (key == "none") {
				_AddEarthReturn();
				return;
			}
			bool fast = key == "fast";
			const FlightPlan& plan = f.Mission().plan;
			// Apollo 13's took about ten hours off the free return.
			double wantEI = fast ? fEI - 28 * kH : fEI - 10 * kH;
			(void)plan;
			if (fast)
				_AddLater("SM jettison for the fast return", 0, [this]() {
					f.SeparateCMFromSM();
				}, [this]() { return !fCrew.Busy() && f.GET() > fPericynthion; });
			_AddLater("PC+2", 0, [this, jd0, wantEI, fast]() {
				double tig = fPericynthion + 2 * kH;
				State at = Propagate(_Now(), tig - f.GET());
				Vec3 vm = at.v - MoonVel(at.jd);
				Vec3 dv;
				Interface entry;
				// The fast return: as early as the DPS can make it.
				bool ok = false;
				for (double back = 0; back <= (fast ? 18 : 0) * kH && !ok; back += 3 * kH)
					ok = TargetReturn(at, vm, -6.5 * kDeg, jd0 + (wantEI + back) / 86400,
						fast ? 1500 : 500, &dv, &entry);
				if (!ok) {
					f.Log(LogEntry::Capcom, "Houston: the RTCC can't make that return; "
						"you'll stay on the free return.");
					_AddEarthReturn();
					return;
				}
				fEI = (entry.jd - jd0) * 86400;
				char buf[160];
				std::snprintf(buf, sizeof(buf), "RTCC: PC+2 %.0f ft/s, entry "
					"interface %s", dv.Length() / 0.3048, FormatGET(fEI).c_str());
				f.Log(LogEntry::Info, buf);
				_Burn("PC+2", tig, dv, true);
				_AddEarthReturn();
			}, [this]() { return !fCrew.Busy() && f.GET() > fPericynthion - 30 * 60; });
		});
	}, [this]() { return fPericynthion > 0 && !fCrew.Busy()
		&& f.GET() > fPericynthion - 4 * kH; });
}


State
MissionDirector::_Now() const
{
	return State{f.CSMVehicle().position, f.CSMVehicle().velocity, f.JulianDate()};
}


Vec3
MissionDirector::_SiteAtLanding() const
{
	const MissionSpec& m = f.Mission();
	double jd0 = f.JulianDate() - f.GET() / 86400;
	double jd = jd0 + (m.plan.landing > 0 ? m.plan.landing : m.plan.loi + 6 * kH) / 86400;
	world::Terrain t;
	MoonScene ms(t);
	ms.SetTime(jd);
	return ms.FixedToInertial() * world::LatLonToVector(m.site.latitude,
		m.site.longitude, 1.0);
}


Vec3
MissionDirector::_LunarPlane() const
{
	// Retrograde, through the landing site at the landing time (or the
	// equator for a flight that doesn't land).
	const MissionSpec& m = f.Mission();
	double jd0 = f.JulianDate() - f.GET() / 86400;
	double jd = jd0 + (m.plan.landing > 0 ? m.plan.landing : m.plan.loi + 6 * kH) / 86400;
	world::Terrain t;
	MoonScene ms(t);
	ms.SetTime(jd);
	Vec3 site = ms.FixedToInertial() * world::LatLonToVector(m.site.latitude,
		m.site.longitude, 1.0);
	Vec3 north = ms.FixedToInertial().Column(2);
	Vec3 h = (-north - site * (-north).Dot(site)).Unit();
	return h;
}


void
MissionDirector::_Burn(const std::string& purpose, double impulseGET,
	const Vec3& dv, bool docked)
{
	if (!std::isfinite(dv.Length()) || !std::isfinite(impulseGET)) {
		f.Log(LogEntry::Capcom, purpose + ": the RTCC has no solution; stand by.");
		return;
	}
	ManeuverPAD pad;
	pad.purpose = purpose;
	// In the lifeboat the LM's descent engine pushes the docked stack.
	bool dps = fLifeboat && f.LMDocked();
	pad.engine = dps ? "DPS" : "SPS";
	pad.dvMagnitude = dv.Length();
	double thrust = dps ? 0.4 * sim::craft::kDPSFullThrust : kSPSThrust;
	double isp = dps ? 305 : kSPSIsp;
	AutoCrew::ExternalDeltaV(f, impulseGET, dv, &pad.tig, &pad.dvLVLH, thrust, isp);
	double m = f.CSMVehicle().Mass(), ve = isp * 9.80665;
	pad.burnTime = m * ve / thrust * (1 - std::exp(-dv.Length() / ve));
	(void)docked;
	fPADs.push_back(pad);
	f.Log(LogEntry::Capcom, (dps ? f.Mission().lmCallsign : f.Mission().csmCallsign)
		+ ", Houston. " + pad.Text());
	if (onPAD)
		onPAD(pad);
	if (autopilot)
		fCrew.Add(dps ? AutoCrew::DockedDPSBurn(pad.tig, pad.dvLVLH, purpose)
			: AutoCrew::CSMBurn(pad.tig, pad.dvLVLH, purpose));
}


void
MissionDirector::_Build()
{
	const MissionSpec& m = f.Mission();
	const FlightPlan& p = m.plan;
	bool lunar = p.tli > 0;
	bool landing = p.pdi > 0 && !m.lgcProgram.empty();
	if (!lunar)
		return;     // Earth-orbit flights are flown by hand (for now)

	// --- Transposition and docking -------------------------------------------
	_Add("Transposition and docking", p.tli, [this]() {
		if (autopilot)
			fCrew.Add(AutoCrew::TransposeAndDock(f.GET() + 600));
	}, [this]() {
		SaturnIU* iu = f.IU();
		return f.HasLM() && !f.LMDocked() && iu
			&& iu->CurrentMode() == SaturnIU::Mode::Coast;
	});

	// --- Translunar midcourses -----------------------------------------------
	auto mcc = [this](const std::string& name) {
		return [this, name]() {
			double tig = f.GET() + kH;
			State at = Propagate(_Now(), kH);
			Vec3 dv;
			Approach ap;
			Vec3 site = _SiteAtLanding();
			bool lands = f.Mission().plan.landing > 0;
			if (!TargetLunarApproach(at, 111000, _LunarPlane(), &dv, &ap,
					lands ? &site : nullptr)) {
				f.Log(LogEntry::Capcom, name + ": no solution; we'll look at it "
					"again later.");
				return;
			}
			Vec3 h = (ap.state.r - MoonPos(ap.jd)).Cross(ap.state.v - MoonVel(ap.jd));
			{
				char buf[160];
				std::snprintf(buf, sizeof(buf), "RTCC: %s %.1f m/s gives pericynthion"
					" %.0f km at GET %.1f h, the site %.2f deg off the plane", name.c_str(),
					dv.Length(), (ap.radius - world::kRMoon) / 1000,
					(ap.jd - (f.JulianDate() - f.GET() / 86400)) * 24,
					std::asin(Clamp(h.Unit().Dot(_SiteAtLanding()), -1.0, 1.0)) / kDeg);
				f.Log(LogEntry::Info, buf);
				if (std::getenv("DOIDIAG"))
					std::printf("%s\n", buf);
			}
			// The arrival moves with the targeting: MCC-4 and LOI-1 keep
			// their places before it.
			{
				double arriveGET = (ap.jd - (f.JulianDate() - f.GET() / 86400)) * 86400;
				for (Step& st : fSteps) {
					if (st.done)
						continue;
					if (st.name == "MCC-4" && name == "MCC-2")
						st.get = std::max(f.GET() + kH, arriveGET - 6 * kH);
					if (st.name == "LOI-1")
						st.get = std::max(f.GET() + kH, arriveGET - 3 * kH);
				}
			}
			if (h.Dot(_LunarPlane()) < 0)
				f.Log(LogEntry::Capcom, name + ": the approach is in the wrong "
					"sense for the landing site plane.");
			if (dv.Length() < 0.6) {
				f.Log(LogEntry::Capcom, name + " is not required.");
				return;
			}
			_Burn(name, tig, dv, f.LMDocked());
		};
	};
	_Add("MCC-2", p.tli + 21 * kH, mcc("MCC-2"), [this]() { return !fCrew.Busy(); });
	_Add("MCC-4", p.loi - 6 * kH, mcc("MCC-4"), [this]() { return !fCrew.Busy(); });

	// --- Lunar orbit insertion ------------------------------------------------
	_Add("LOI-1", p.loi - 3 * kH, [this]() {
		State s = _Now();
		Approach ap = ClosestMoonApproach(s, 2 * 86400);
		if (!ap.found) {
			f.Log(LogEntry::Capcom, "LOI-1: no lunar approach in the tracking; "
				"we'll look again.");
			return;
		}
		Vec3 dv;
		if (!TargetLOI(ap.state, 313000, 111000, &dv)) {
			char buf[120];
			std::snprintf(buf, sizeof(buf), "LOI-1: no solution (pericynthion "
				"%.0f km); we'll look again.", (ap.radius - world::kRMoon) / 1000);
			f.Log(LogEntry::Capcom, buf);
			return;
		}
		double jd0 = f.JulianDate() - f.GET() / 86400;
		fLOI = (ap.jd - jd0) * 86400;
		_Burn("LOI-1", fLOI, dv, f.LMDocked());
	}, [this]() { return !fCrew.Busy() && f.CurrentPhase() == Phase::TranslunarCoast; });

	_Add("LOI-2", p.loi + 3.6 * kH, [this]() {
		// Circularize at 60 nmi: at the next pericynthion, lower the
		// apocynthion.
		double rp;
		double t = NextApsis(_Now(), f.GET(), f.GET() + 1200, true, &rp);
		State at = Propagate(_Now(), t - f.GET());
		Vec3 dv = CircularizeAt(at, world::kMuMoon, MoonPos(at.jd), MoonVel(at.jd));
		_Burn("LOI-2", t, dv, f.LMDocked());
	}, [this]() { return !fCrew.Busy() && f.CurrentPhase() == Phase::LunarOrbit; });

	if (!landing) {
		// Lunar orbit only (Apollo 8, 10): TEI as planned.
		_AddReturn();
	} else {
		// --- Landing -----------------------------------------------------------
		_Add("LM activation", p.undock - 2.5 * kH, [this]() {
			f.TransferCrew(2);
			f.PowerUpLGC();
		}, [this]() { return !fCrew.Busy() && f.LMDocked(); });
		// (Undocking, DOI and the descent: see Steps added on the fly.)
		_Add("DOI", p.undock - 2 * kH, [this]() { _PlanDescent(); },
			[this]() { return !fCrew.Busy() && f.LGCPowered(); });
	}
}


void
MissionDirector::_PlanDescent()
{
	// DOI with the docked stack (as Apollo 14-17 flew it): retrograde at
	// the point opposite the PDI point, a whole transfer orbit before the
	// pass that puts perilune (50,000 ft) over the PDI point at about the
	// planned time. Then undock, circularize the CSM, and descend.
	const MissionSpec& m = f.Mission();
	const FlightPlan& p = m.plan;
	State now = _Now();
	double siteR = m.site.radius;
	Vec3 rr = now.r - MoonPos(now.jd);
	double rc = rr.Length();
	double a = 0.5 * (rc + siteR + 15240);
	double halfT = M_PI * std::sqrt(a * a * a / world::kMuMoon);
	double fullT = 2 * halfT;
	double jd0 = f.JulianDate() - f.GET() / 86400;
	world::Terrain terrain;
	MoonScene ms(terrain);
	auto pdiDir = [&](double get, const Vec3& h) {
		ms.SetTime(jd0 + (get + 720) / 86400);
		Vec3 site = ms.FixedToInertial() * world::LatLonToVector(m.site.latitude,
			m.site.longitude, 1.0);
		Vec3 down = h.Cross(site).Unit();
		double th = -16.2 * kDeg;
		return site * std::cos(th) + down * std::sin(th);
	};
	// Scan for the CSM passing opposite the PDI point half a transfer later.
	double best = 0, bestErr = 1e30;
	double prevAng = 0;
	bool havePrev = false;
	for (double t = f.GET() + 1800 + fullT; t < f.GET() + 10 * kH; t += 15) {
		State st = Propagate(now, t - f.GET());
		Vec3 r = st.r - MoonPos(st.jd), v = st.v - MoonVel(st.jd);
		Vec3 h = r.Cross(v).Unit();
		Vec3 anti = -pdiDir(t + halfT, h);
		anti = (anti - h * anti.Dot(h)).Unit();
		double ang = std::atan2(h.Dot(r.Unit().Cross(anti)), r.Unit().Dot(anti));
		if (havePrev && prevAng > 0 && ang <= 0 && std::fabs(ang) < 0.5) {
			double pdi = t + halfT;
			double err = std::fabs(pdi - p.pdi);
			if (err < bestErr) {
				bestErr = err;
				best = t;
			}
		}
		prevAng = ang;
		havePrev = true;
	}
	if (best == 0) {
		f.Log(LogEntry::Capcom, "Houston: no DOI solution. Holding in orbit.");
		return;
	}
	fCirc = best;
	fDOI = best - fullT;
	fPDI = best + halfT;
	// Refine: DOI time and size so the perilune (50,000 ft) sits exactly
	// over the PDI point.
	Vec3 target = pdiDir(fPDI, rr.Cross(now.v - MoonVel(now.jd)).Unit());
	double targetR = siteR + 15240;
	double x[2] = {0, 22};
	auto solve = [&](const double* q, double* out) {
		State at = Propagate(now, fDOI + q[0] - f.GET());
		Vec3 v0 = at.v - MoonVel(at.jd);
		at.v -= v0.Unit() * q[1];
		Approach ap = ClosestMoonApproach(at, 2 * kH, true);
		if (!ap.found)
			return false;
		Vec3 r = ap.state.r - MoonPos(ap.jd);
		Vec3 h = r.Cross(ap.state.v - MoonVel(ap.jd)).Unit();
		Vec3 tp = (target - h * target.Dot(h)).Unit();
		out[0] = ap.radius - targetR;
		out[1] = std::atan2(h.Dot(r.Unit().Cross(tp)), r.Unit().Dot(tp)) * 1e5;
		return true;
	};
	{
		// Newton with finite differences (2 x 2).
		for (int it = 0; it < 12; it++) {
			double f0[2], f1[2], f2[2];
			if (!solve(x, f0))
				break;
			if (std::fabs(f0[0]) < 30 && std::fabs(f0[1]) < 20)
				break;
			double xa[2] = {x[0] + 5, x[1]}, xb[2] = {x[0], x[1] + 0.1};
			if (!solve(xa, f1) || !solve(xb, f2))
				break;
			double J[2][2] = {{(f1[0] - f0[0]) / 5, (f2[0] - f0[0]) / 0.1},
				{(f1[1] - f0[1]) / 5, (f2[1] - f0[1]) / 0.1}};
			double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
			if (std::fabs(det) < 1e-12)
				break;
			x[0] -= (f0[0] * J[1][1] - f0[1] * J[0][1]) / det;
			x[1] -= (J[0][0] * f0[1] - J[1][0] * f0[0]) / det;
		}
	}
	{
		double res[2] = {1e9, 1e9};
		bool ok = solve(x, res);
		char buf[160];
		std::snprintf(buf, sizeof(buf), "RTCC: DOI %+.0f s, %.1f m/s; perilune %.0f ft"
			" from the target height, %.1f deg from the PDI point%s", x[0], x[1],
			res[0] / 0.3048, res[1] / 1e5 / kDeg, ok ? "" : " (no solution)");
		f.Log(LogEntry::Info, buf);
		if (std::getenv("DOIDIAG"))
			std::printf("%s\n", buf);
	}
	fDOI += x[0];
	double dvm = x[1];
	State atDOI = Propagate(now, fDOI - f.GET());
	Vec3 v0 = atDOI.v - MoonVel(atDOI.jd);
	{
		// PDI: the second perilune after DOI.
		State post = atDOI;
		post.v -= v0.Unit() * dvm;
		Approach first = ClosestMoonApproach(post, 2 * kH);
		if (first.found) {
			Approach second = ClosestMoonApproach(Propagate(first.state, 1800),
				3 * kH);
			if (second.found)
				fPDI = (second.jd - jd0) * 86400;
			fCirc = (first.jd - jd0) * 86400 + halfT;
		}
	}
	_Burn("DOI", fDOI, -v0.Unit() * dvm, true);
	// The LGC's landing time follows the descent actually flown.
	if (gnc::PadLoader* pad = f.LGCPad())
		pad->Double("TLAND", (fPDI + p.landing - p.pdi) * 100.0, -28);

	_Add("Undocking", fDOI + 1200, [this]() {
		f.UndockLM();
		if (autopilot)
			fCrew.Add(AutoCrew::ConfigureCMCDAP());
	}, [this]() { return !fCrew.Busy(); });
	_Add("CSM circularization", fCirc - 1800, [this]() {
		// At the apolune where DOI was done, a transfer orbit later: back up
		// to 60 nmi circular.
		State at = Propagate(_Now(), fCirc - f.GET());
		Vec3 dv = CircularizeAt(at, world::kMuMoon, MoonPos(at.jd), MoonVel(at.jd));
		_Burn("CSM circularization", fCirc, dv, false);
		if (autopilot)
			fCrew.Add(AutoCrew::Landing(fPDI));
	});
	_AddSurface();
}


void
MissionDirector::_AddSurface()
{
	const FlightPlan& p = f.Mission().plan;
	_Add("Touchdown", 0, [this]() {
		if (fLanded <= 0)
			fLanded = f.GET();
		f.Log(LogEntry::Capcom, "Roger, " + f.Mission().lmCallsign
			+ ". We copy you on the ground.");
	}, [this]() { return f.LMLanded(); });
	double stay = p.liftoff - p.landing;
	// Lunar orbit plane change (Apollo 15-17's LOPC): the Moon turns under
	// the CSM's orbit during the stay, carrying the site out of its plane;
	// a few hours before liftoff the CSM changes plane to pass over the
	// site again, so the ascent needn't steer sideways.
	_Add("LOPC", 0, [this, stay]() {
		double jd0 = f.JulianDate() - f.GET() / 86400;
		double liftJD = jd0 + (fLanded + stay) / 86400;
		world::Terrain t;
		MoonScene ms(t);
		ms.SetTime(liftJD);
		Vec3 site = ms.FixedToInertial() * world::LatLonToVector(
			f.Mission().site.latitude, f.Mission().site.longitude, 1.0);
		State now = _Now();
		Vec3 r = now.r - MoonPos(now.jd), v = now.v - MoonVel(now.jd);
		Vec3 n = r.Cross(v).Unit();
		double off = std::asin(Clamp(n.Dot(site), -1.0, 1.0)) / kDeg;
		char buf[160];
		// The J missions trimmed even small offsets; the shorter stays left
		// up to a few degrees for the ascent to steer out.
		bool longStay = f.Mission().type == MissionType::LandingExtended;
		if (std::fabs(off) < (longStay ? 0.15 : 2.5)) {
			std::snprintf(buf, sizeof(buf), "RTCC: the site will be %.2f deg out of "
				"the CSM's plane at liftoff; no plane change.", off);
			f.Log(LogEntry::Info, buf);
			return;
		}
		Vec3 n2 = (n - site * n.Dot(site)).Unit();
		Vec3 node = n.Cross(n2).Unit();
		// The next node at least 20 minutes out: step around the orbit.
		double period = 2 * M_PI * std::sqrt(std::pow(r.Length(), 3) / world::kMuMoon);
		double best = -1, bestDot = 0;
		for (double dt = 1200; dt < 1200 + period; dt += 15) {
			State at = Propagate(now, dt);
			Vec3 ru = (at.r - MoonPos(at.jd)).Unit();
			double d = std::fabs(ru.Dot(node));
			if (d > bestDot) {
				bestDot = d;
				best = dt;
			}
		}
		State at = Propagate(now, best);
		Vec3 rb = at.r - MoonPos(at.jd), vb = at.v - MoonVel(at.jd);
		Vec3 ru = rb.Unit();
		double radial = vb.Dot(ru);
		double horiz = (vb - ru * radial).Length();
		Vec3 vNew = n2.Cross(ru) * horiz + ru * radial;
		Vec3 dv = vNew - vb;
		std::snprintf(buf, sizeof(buf), "RTCC: LOPC %.1f m/s puts the site in the "
			"CSM's plane at liftoff (now %.2f deg out)", dv.Length(), off);
		f.Log(LogEntry::Info, buf);
		_Burn("LOPC", f.GET() + best, dv, false);
	}, [this, stay]() {
		// The J missions' long stays only (Apollo 11-14's ascents steered
		// out the small offsets of a day or so on the surface).
		return fLanded > 0 && f.GET() > fLanded + stay - 7 * kH && !fCrew.Busy()
			&& !f.LMDocked();
	});
	_Add("Lunar liftoff", 0, [this]() {
		// The RTCC times the liftoff for the rendezvous: the CSM just
		// about overhead at ascent ignition.
		double lift = AutoCrew::LiftoffTime(f, f.GET() + 900, -0.6);
		char buf[80];
		std::snprintf(buf, sizeof(buf), "Liftoff time %s", FormatGET(lift).c_str());
		f.Log(LogEntry::Capcom, buf);
		if (autopilot) {
			fCrew.Add(AutoCrew::Ascent(lift));
			fCrew.Add(AutoCrew::Rendezvous());
			fCrew.Add(AutoCrew::TransposeAndDock(0, false));
		}
	}, [this, stay]() { return fLanded > 0 && f.GET() > fLanded + stay - 1800; });
	_Add("LM jettison", 0, [this]() {
		fDocked = f.GET();
		f.TransferCrew(0);
		f.JettisonLM();
		if (autopilot)
			fCrew.Add(AutoCrew::ConfigureCMCDAP());
	}, [this]() { return f.LMDocked() && fLanded > 0 && !fCrew.Busy(); });
	_AddReturn();
}


void
MissionDirector::_AddReturn()
{
	const MissionSpec& m = f.Mission();
	const FlightPlan& p = m.plan;
	fTEIGiveUp = p.tei + 24 * kH;
	_Add("TEI", p.tei - 3 * kH, [this]() {
		State at = Propagate(_Now(), 1800);
		Vec3 dv;
		double ei;
		State burn;
		// Entry interface at the planned time of day (the recovery line
		// turns under the returning spacecraft): the planned EI moved by
		// whole days to the one nearest the planned transit (Apollo 11's
		// 60 hours, Apollo 16's 90: a slower return costs less delta-V).
		const FlightPlan& plan = f.Mission().plan;
		double jd0 = f.JulianDate() - f.GET() / 86400;
		double wantEI = 0;
		if (plan.splashdown > 0) {
			double base = plan.splashdown - 900;
			double transit = plan.entry > plan.tei && plan.tei > 0
				? Clamp(plan.entry - plan.tei, 55 * kH, 110 * kH) : 60 * kH;
			double k = std::round((f.GET() + 1800 + transit - base) / (24 * kH));
			wantEI = jd0 + (base + k * 24 * kH) / 86400;
		}
		if (!TargetTEI(at, -6.5 * kDeg, &dv, &ei, &burn, wantEI)
				&& !TargetTEI(at, -6.5 * kDeg, &dv, &ei, &burn)) {
			f.Log(LogEntry::Capcom, "Houston: TEI targeting failed; next rev.");
			// Try again on the next revolution, for a day.
			for (Step& st : fSteps)
				if (st.name == "TEI" && f.GET() < fTEIGiveUp) {
					st.done = false;
					st.get = f.GET() + 2 * kH;
				}
			return;
		}
		fEI = (ei - jd0) * 86400;
		_Burn("TEI", (burn.jd - jd0) * 86400, dv, false);
	}, [this]() {
		return !fCrew.Busy() && !f.LMDocked() && f.CurrentPhase() == Phase::LunarOrbit;
	});
	_AddEarthReturn();
	(void)p;
}


void
MissionDirector::_AddEarthReturn()
{
	auto mccEarth = [this](const std::string& name, double minDV) {
		return [this, name, minDV]() {
			State at = Propagate(_Now(), kH);
			Vec3 dv;
			if (!TargetMCCEarth(at, -6.5 * kDeg, &dv)) {
				f.Log(LogEntry::Capcom, name + ": no solution.");
				return;
			}
			if (dv.Length() < minDV) {
				f.Log(LogEntry::Capcom, name + " is not required.");
				return;
			}
			_Burn(name, f.GET() + kH, dv, false);
		};
	};
	_Add("MCC-5", 0, mccEarth("MCC-5", 0.6), [this]() {
		return fEI > 0 && !fCrew.Busy() && f.GET() > fEI - 45 * kH;
	});
	_Add("MCC-7", 0, mccEarth("MCC-7", 0.3), [this]() {
		return fEI > 0 && !fCrew.Busy() && f.GET() > fEI - 5 * kH;
	});
	if (fLifeboat) {
		// Apollo 13's last day: the SM goes (the crew photographed the
		// damage), the crew moves back to the powered-up CM, Aquarius goes.
		_Add("SM jettison", 0, [this]() {
			if (f.SMAttached()) {
				f.SeparateCMFromSM();
				f.Log(LogEntry::Crew, "CDR: And there's one whole side of that"
					" spacecraft missing.");
			}
		}, [this]() { return fEI > 0 && !fCrew.Busy() && f.GET() > fEI - 4.6 * kH; });
		_Add("Crew to the CM", 0, [this]() {
			f.TransferCrew(0);
			CSMSystems& cs = f.csmSystems;
			cs.poweredDown = false;
			cs.powerTransfer = false;
			cs.loads = PowerLoads();
			f.CMPGNCS().imu.powered = true;   // aligned in the entry procedure
			cs.batteryA.online = cs.batteryB.online = cs.batteryC.online = true;
			f.CMPGNCS().panel.scContCMC = true;
			f.CMPGNCS().panel.mode = gnc::CMPanel::MODE_AUTO;
			char bat[160];
			std::snprintf(bat, sizeof(bat), "%s, Houston. Power up the CM; the entry "
				"checklist is on the way. Batteries A, B, C: %.0f, %.0f, %.0f amp-hours.",
				f.Mission().csmCallsign.c_str(), cs.batteryA.charge, cs.batteryB.charge,
				cs.batteryC.charge);
			f.Log(LogEntry::Capcom, bat);
		}, [this]() { return fEI > 0 && f.GET() > fEI - 3 * kH; });
		_Add("LM jettison", 0, [this]() {
			f.JettisonLM();
			f.Log(LogEntry::Capcom, "Farewell, " + f.Mission().lmCallsign
				+ ", and we thank you.");
		}, [this]() { return fEI > 0 && f.GET() > fEI - 2.5 * kH; });
	}
	const MissionSpec& m = f.Mission();
	_Add("Entry", 0, [this, &m]() {
		// The recovery forces sit on the predicted ground track, the
		// entry range downrange of the entry interface (they moved for
		// weather on Apollo 11). The planned point when it's reachable.
		Interface in = EntryInterface(_Now(), 4 * kH);
		fSplashLat = m.splashLatitude;
		fSplashLon = m.splashLongitude;
		if (in.found) {
			// Work in inertial space along the entry plane; the splash
			// comes about 14 minutes after entry interface.
			Vec3 up = in.state.r.Unit();
			Vec3 along = (in.state.v - up * in.state.v.Dot(up)).Unit();
			double splashJD = in.jd + 840 / 86400.0;
			Vec3 plan = world::Ephemeris::EarthFixedToInertial(splashJD)
				* world::LatLonToVector(m.splashLatitude, m.splashLongitude, 1.0);
			double down = std::atan2(plan.Dot(along), plan.Dot(up)) * world::kREarth;
			double cross = std::fabs(plan.Dot(up.Cross(along))) * world::kREarth;
			if (std::getenv("ENTRYDIAG"))
				std::printf("ENTRYDIAG plan downrange %.0f nmi crossrange %.0f nmi\n",
					down / 1852, cross / 1852);
			// Comanche's entry reaches 1,250 to 2,000 nmi or so without a
			// skip; aim mid-range when the planned point is out of reach.
			if (down < 1250 * 1852 || down > 2000 * 1852 || cross > 60 * 1852) {
				const double range = 1400 * 1852.0;
				double ang = range / world::kREarth;
				Vec3 aimI = up * std::cos(ang) + along * std::sin(ang);
				Vec3 aim = world::Ephemeris::EarthFixedToInertial(splashJD).Transposed()
					* aimI;
				double lat, lon, rr;
				world::VectorToLatLon(aim, &lat, &lon, &rr);
				fSplashLat = lat;
				fSplashLon = lon;
				char buf[120];
				std::snprintf(buf, sizeof(buf), "Houston: the recovery forces are moving"
					" to %.1f %s %.1f %s.", std::fabs(lat), lat >= 0 ? "N" : "S",
					std::fabs(lon), lon >= 0 ? "E" : "W");
				f.Log(LogEntry::Capcom, buf);
			}

		}
		if (autopilot)
			fCrew.Add(AutoCrew::Entry(fSplashLat, fSplashLon));
	}, [this]() { return fEI > 0 && !fCrew.Busy() && f.GET() > fEI - 2 * kH; });
}

std::string
MissionDirector::NextEvent() const
{
	for (const Step& s : fSteps)
		if (!s.done)
			return s.name;
	return "";
}


double
MissionDirector::NextEventGET() const
{
	for (const Step& s : fSteps)
		if (!s.done)
			return s.get;
	return 0;
}


void
MissionDirector::Tick()
{
	for (Step& st : fLater)
		fSteps.push_back(st);
	fLater.clear();
	_CheckEmergencies();
	for (Step& st : fLater)
		fSteps.push_back(st);
	fLater.clear();
	if (PendingDecision())
		return;             // Flight hasn't called it yet
	for (Step& s : fSteps) {
		if (s.done)
			continue;
		if (f.GET() < s.get)
			return;            // steps run in order
		if (s.when && !s.when())
			return;
		s.done = true;
		f.Log(LogEntry::Info, "Flight plan: " + s.name);
		s.run();
		return;
	}
}

}  // namespace flight
