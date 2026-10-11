// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "MissionDirector.h"

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
	fSteps.push_back(s);
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
	ManeuverPAD pad;
	pad.purpose = purpose;
	pad.engine = "SPS";
	pad.dvMagnitude = dv.Length();
	AutoCrew::ExternalDeltaV(f, impulseGET, dv, &pad.tig, &pad.dvLVLH, kSPSThrust,
		kSPSIsp);
	double m = f.CSMVehicle().Mass(), ve = kSPSIsp * 9.80665;
	pad.burnTime = m * ve / kSPSThrust * (1 - std::exp(-dv.Length() / ve));
	(void)docked;
	fPADs.push_back(pad);
	f.Log(LogEntry::Capcom, f.Mission().csmCallsign + ", Houston. " + pad.Text());
	if (onPAD)
		onPAD(pad);
	if (autopilot)
		fCrew.Add(AutoCrew::CSMBurn(pad.tig, pad.dvLVLH, purpose));
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
		if (!ap.found)
			return;
		Vec3 dv;
		if (!TargetLOI(ap.state, 313000, 111000, &dv))
			return;
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
	_Add("TEI", p.tei - 3 * kH, [this]() {
		State at = Propagate(_Now(), 1800);
		Vec3 dv;
		double ei;
		State burn;
		// Entry interface at the planned time of day (the recovery line
		// turns under the returning spacecraft): the planned EI moved by
		// whole days to the one nearest a 60-hour transit.
		const FlightPlan& plan = f.Mission().plan;
		double jd0 = f.JulianDate() - f.GET() / 86400;
		double wantEI = 0;
		if (plan.splashdown > 0) {
			double base = plan.splashdown - 900;
			double k = std::round((f.GET() + 1800 + 60 * kH - base) / (24 * kH));
			wantEI = jd0 + (base + k * 24 * kH) / 86400;
		}
		if (!TargetTEI(at, -6.5 * kDeg, &dv, &ei, &burn, wantEI)
				&& !TargetTEI(at, -6.5 * kDeg, &dv, &ei, &burn)) {
			f.Log(LogEntry::Capcom, "Houston: TEI targeting failed; next rev.");
			return;
		}
		fEI = (ei - jd0) * 86400;
		_Burn("TEI", (burn.jd - jd0) * 86400, dv, false);
	}, [this]() {
		return !fCrew.Busy() && !f.LMDocked() && f.CurrentPhase() == Phase::LunarOrbit;
	});
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
	(void)p;
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
