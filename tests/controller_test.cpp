// Columbia — drives the Mission controller like the UI does.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "../core/Mission.h"

#include <cmath>
#include <cstdio>
#include <string>

using namespace columbia;

static int sFailures = 0;

static void
Check(bool ok, const std::string& what)
{
	std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what.c_str());
	if (!ok)
		sFailures++;
}

static void
Drain(Mission& m)
{
	for (const std::string& e : m.TakeEvents())
		std::printf("    | %s\n", e.c_str());
}

// Runs UI ticks (30 Hz) until GET reaches `until` or the flight ends.
static void
Fly(Mission& m, double until, double warp)
{
	m.SetWarp(warp);
	int guard = 0;
	while (m.Time() < until && !m.Finished() && guard++ < 2000000) {
		m.Update(1.0 / 30.0);
		if (m.Warp() < warp && m.State().activeBurn == 0)
			m.SetWarp(warp);   // a user would turn the warp back up
		Drain(m);
	}
}

static std::string
Prog(Mission& m)
{
	const agc::DSKYState& d = m.Display();
	return std::string(d.prog, 2);
}

int
main()
{
	Mission m;
	Drain(m);
	for (int i = 0; i < 30 * 8; i++)
		m.Update(1.0 / 30.0);
	Drain(m);
	Check(Prog(m) == "00", "computer fresh-started into P00 (PROG "
		+ Prog(m) + ")");
	Check(!m.Display().progAlarm, "no program alarm after fresh start");

	// Display GET on the DSKY and compare with the flight clock.
	m.QueueKeys("V16N65E");
	for (int i = 0; i < 30 * 3; i++)
		m.Update(1.0 / 30.0);
	const agc::DSKYState& d = m.Display();
	std::printf("  DSKY V%c%c N%c%c R3 %s at GET %s\n", d.verb[0], d.verb[1],
		d.noun[0], d.noun[1], d.RegisterText(2).c_str(),
		sim::FormatGET(m.Time()).c_str());
	Check(std::string(d.verb, 2) == "16" && std::string(d.noun, 2) == "65",
		"V16N65 monitor up");

	sim::PlanResult tli = m.PlanNow(Mission::PLAN_TLI);
	std::printf("  TLI: %s %s\n", tli.ok ? "ok" : "failed", tli.message.c_str());
	Check(tli.ok, "TLI planned with the Mission's Moon phase");
	if (!tli.ok)
		return 1;

	std::string error;
	Check(m.SavePlan("/tmp/claude-0/plan.txt", &error), "plan saved");

	Fly(m, tli.burn.time + 3600, 500);
	Check(!m.State().sivbAttached || m.State().sivbPropellant < 60000,
		"TLI executed");

	// The clock on the DSKY should keep up with GET even at high warp.
	Fly(m, m.Time() + 2 * 3600, 5000);
	m.SetWarp(1);
	for (int i = 0; i < 30 * 3; i++)
		m.Update(1.0 / 30.0);
	const agc::DSKYState& d2 = m.Display();
	int hours = std::atoi(std::string(d2.reg[0], 5).c_str());
	int minutes = std::atoi(std::string(d2.reg[1], 5).c_str());
	double agcClock = hours * 3600.0 + minutes * 60.0;
	std::printf("  DSKY clock %dh %dm vs GET %s\n", hours, minutes,
		sim::FormatGET(m.Time()).c_str());
	Check(std::fabs(agcClock - m.Time()) < 120,
		"AGC mission clock tracks GET within two minutes");

	Fly(m, m.Time() + 20 * 3600, 10000);
	sim::PlanResult loi = m.PlanNow(Mission::PLAN_LOI);
	std::printf("  LOI: %s %s\n", loi.ok ? "ok" : "failed",
		loi.message.c_str());
	Check(loi.ok, "LOI planned");
	Fly(m, loi.burn.time + 4 * 3600, 10000);
	sim::OrbitSummary o = sim::Summarize(m.Model(), m.State());
	std::printf("  lunar orbit %.0f x %.0f km\n", o.periapsis / 1e3,
		o.apoapsis / 1e3);
	Check(o.aroundMoon && o.apoapsis > 0, "captured into lunar orbit");

	sim::PlanResult tei = m.PlanNow(Mission::PLAN_TEI);
	std::printf("  TEI: %s %s\n", tei.ok ? "ok" : "failed",
		tei.message.c_str());
	Check(tei.ok, "TEI planned");
	Fly(m, tei.burn.time + 20 * 3600, 10000);
	sim::PlanResult mcc = m.PlanNow(Mission::PLAN_MCC);
	std::printf("  MCC: %s %s %.2f m/s\n", mcc.ok ? "ok" : "failed",
		mcc.message.c_str(), mcc.burn.DeltaV());
	Fly(m, m.Time() + 8 * 86400, 10000);
	std::printf("  outcome: %s\n", sim::OutcomeText(m.State().outcome));
	Check(m.State().outcome == sim::Outcome::Splashdown, "splashdown");
	Check(m.Display().progAlarm == false, "computer quiet at the end");
	std::printf("  computer restarts during flight: %u\n",
		m.Computer().RestartCount() - 1);
	Check(m.Computer().RestartCount() == 1, "no AGC hardware restarts");

	// Reload the saved plan into a fresh mission.
	Mission fresh;
	Check(fresh.LoadPlan("/tmp/claude-0/plan.txt", &error),
		"plan reloaded: " + error);
	Check(fresh.Plan().size() == 1 && fresh.Plan()[0].label == "TLI"
		&& std::fabs(fresh.Plan()[0].prograde - tli.burn.prograde) < 0.01,
		"reloaded plan matches");

	std::printf("%d failure(s)\n", sFailures);
	return sFailures == 0 ? 0 : 1;
}
