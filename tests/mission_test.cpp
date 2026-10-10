// Columbia — end-to-end lunar mission using only the planner helpers.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "../core/sim/Flight.h"
#include "../core/sim/Planner.h"

#include <chrono>
#include <cmath>
#include <cstdio>

using namespace sim;

static int sFailures = 0;

static void
Check(bool ok, const char* what)
{
	std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
	if (!ok)
		sFailures++;
}

static void
Report(const Model& m, const VehicleState& s, const char* when)
{
	OrbitSummary o = Summarize(m, s);
	std::printf("%s %s: %s alt %.1f km, peri %.1f km, apo %.1f km, "
		"SPS %.0f kg\n", FormatGET(s.t).c_str(), when,
		o.aroundMoon ? "Moon" : "Earth", o.altitude / 1e3, o.periapsis / 1e3,
		o.apoapsis / 1e3, s.spsPropellant);
}

static bool
Plan(const char* name, const PlanResult& r, FlightPlan& plan)
{
	std::printf("plan %-4s %s  ign %s  dv %.2f m/s — %s\n", name,
		r.ok ? "ok  " : "FAIL", FormatGET(r.burn.time).c_str(),
		r.burn.DeltaV(), r.message.c_str());
	if (r.ok)
		plan.push_back(r.burn);
	return r.ok;
}

int
main()
{
	auto wall = std::chrono::steady_clock::now();
	Model model;
	VehicleState s = ParkingOrbit(model);
	FlightPlan plan;
	Propagator truth(model);

	// --- TLI ---------------------------------------------------------------
	if (!Plan("TLI", Planner(model, s, plan).TransLunarInjection(), plan))
		return 1;
	Prediction p = Predict(model, s, plan, 10 * 86400, false);
	std::printf("  predicted pericynthion %.1f km at %s, return perigee "
		"%.1f km, %s\n", p.pericynthionAltitude / 1e3,
		FormatGET(p.pericynthionTime).c_str(), p.perigeeAltitude / 1e3,
		OutcomeText(p.final.outcome));
	Check(std::fabs(p.pericynthionAltitude - 110e3) < 5e3,
		"TLI pericynthion within 5 km of 110 km");
	Check(p.final.outcome == Outcome::Splashdown,
		"TLI alone is a free return to a safe entry");

	// Fly to 8 h after TLI; the S-IVB is still attached.
	truth.Advance(s, plan, plan[0].time + 8 * 3600);
	Report(model, s, "after TLI coast");
	Check(!s.sivbAttached || s.sivbPropellant < kSIVBPropellant,
		"S-IVB propellant consumed");

	// --- Outbound MCC (keeps the pericynthion honest after the finite burn)
	Plan("MCC1", Planner(model, s, plan).PericynthionCorrection(s.t + 3600),
		plan);

	// --- LOI ---------------------------------------------------------------
	truth.Advance(s, plan, s.t + 30 * 3600);
	if (!Plan("LOI", Planner(model, s, plan).LunarOrbitInsertion(), plan))
		return 1;
	truth.Advance(s, plan, plan.back().time + 3 * 3600);
	Report(model, s, "after LOI");
	OrbitSummary o = Summarize(model, s);
	Check(o.aroundMoon && o.apoapsis > 0 && o.apoapsis < 400e3
		&& o.periapsis > 30e3, "in a closed low lunar orbit");

	// Orbit the Moon for a day.
	truth.Advance(s, plan, s.t + 24 * 3600);
	Report(model, s, "after a day in orbit");

	// --- TEI ---------------------------------------------------------------
	if (!Plan("TEI", Planner(model, s, plan).TransEarthInjection(), plan))
		return 1;
	truth.Advance(s, plan, plan.back().time + 12 * 3600);
	Report(model, s, "12 h after TEI");

	// --- Return MCC ----------------------------------------------------------
	Plan("MCC2", Planner(model, s, plan).EntryCorrection(s.t + 3600), plan);
	truth.Advance(s, plan, s.t + 6 * 86400);
	std::printf("%s %s, entry %.0f m/s at %.2f deg\n",
		FormatGET(s.t).c_str(), OutcomeText(s.outcome), s.entrySpeed,
		s.entryAngle);
	Check(s.outcome == Outcome::Splashdown, "splashdown");
	Check(s.t < 9 * 86400, "mission shorter than 9 days");

	double dvTotal = 0;
	for (const Burn& b : plan)
		if (b.engine == ENGINE_SPS)
			dvTotal += b.DeltaV();
	std::printf("SPS delta-v used %.0f m/s, propellant left %.0f kg\n",
		dvTotal, s.spsPropellant);
	double secs = std::chrono::duration<double>(
		std::chrono::steady_clock::now() - wall).count();
	std::printf("wall time %.2f s, %d failure(s)\n", secs, sFailures);
	return sFailures == 0 ? 0 : 1;
}
