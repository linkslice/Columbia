// Columbia — contingencies fire into the real hardware model at their
// historical times and the flight responds: Apollo 12's lightning and
// Apollo 13's pogo during the boost, Apollo 13's oxygen tank on the way
// to the Moon (with the LM lifeboat), the schedules the difficulty
// levels draw.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: contingency_test <rope-dir>

#include "../core/flight/Contingency.h"
#include "../core/flight/FlightSim.h"
#include "../core/flight/Saturn.h"

#include <cmath>
#include <cstdio>

using namespace flight;
using namespace sim;

namespace {

int gFailures = 0;

void
Check(bool ok, const char* what)
{
	std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
	if (!ok)
		gFailures++;
}

void
Echo(FlightSim& f)
{
	f.onLog = [](const LogEntry& e) {
		if (e.kind != LogEntry::Info)
			std::printf("    [%9.1f] %s\n", e.get, e.text.c_str());
	};
}

}  // namespace


int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";

	// --- Schedules -----------------------------------------------------------
	std::printf("Schedules for Apollo 11:\n");
	for (Difficulty d : {Difficulty::Nominal, Difficulty::AsFlown,
			Difficulty::Challenging, Difficulty::Hard, Difficulty::Nightmare}) {
		FlightSim f(*FindMission(11), ropes);
		std::string err;
		f.Start(StartPoint::Launch, &err);
		ContingencyManager m(f, d, 42);
		m.Plan();
		std::printf("  %-11s:", DifficultyName(d));
		for (const auto& s : m.Schedule())
			std::printf(" %s@%.0fh", s.spec->id.c_str(), s.get / 3600);
		std::printf("\n");
		if (d == Difficulty::AsFlown)
			Check(m.Schedule().size() == 3, "as flown: three Apollo 11 events");
		if (d == Difficulty::Nominal)
			Check(m.Schedule().empty(), "nominal: none");
	}

	// --- Apollo 12: lightning ------------------------------------------------
	{
		std::printf("Apollo 12 launch, as flown:\n");
		FlightSim f(*FindMission(12), ropes);
		std::string err;
		if (!f.Start(StartPoint::Launch, &err)) {
			std::printf("  start: %s\n", err.c_str());
			return 1;
		}
		ContingencyManager m(f, Difficulty::AsFlown, 1);
		m.Plan();
		f.secondHooks.push_back([&](FlightSim&, double) { m.Tick(); });
		Echo(f);
		f.Run(600 + 40);
		Check(m.Fired() == 1, "lightning struck");
		Check(f.csmSystems.FuelCellsOnline() == 0, "fuel cells off line");
		Check(f.faults.sceFailed, "SCE failed");
		f.SCEToAux();
		f.ResetFuelCells();
		f.Run(2);
		Check(m.Resolved() == 1, "SCE to AUX and fuel cells reset");
		while (f.IU()->CurrentMode() != SaturnIU::Mode::Orbit && f.GET() < 800
				&& !f.Lost())
			f.Run(5);
		f.Run(5);
		Situation s = f.Situate(f.CSMVehicle());
		std::printf("  orbit %.1f x %.1f km\n", s.apoapsis / 1000, s.periapsis / 1000);
		Check(s.periapsis > 150000, "the Saturn flew on to orbit");
	}

	// --- Apollo 13: pogo -----------------------------------------------------
	{
		std::printf("Apollo 13 launch, as flown:\n");
		FlightSim f(*FindMission(13), ropes);
		std::string err;
		f.Start(StartPoint::Launch, &err);
		ContingencyManager m(f, Difficulty::AsFlown, 1);
		m.Plan();
		f.secondHooks.push_back([&](FlightSim&, double) { m.Tick(); });
		Echo(f);
		while (f.IU()->CurrentMode() != SaturnIU::Mode::Orbit && f.GET() < 900
				&& !f.Lost())
			f.Run(5);
		f.Run(5);
		Situation s = f.Situate(f.CSMVehicle());
		std::printf("  orbit %.1f x %.1f km at GET %.0f\n", s.apoapsis / 1000,
			s.periapsis / 1000, f.GET());
		Check(m.Fired() >= 1, "S-II center engine out");
		Check(s.periapsis > 150000, "still made orbit");
	}

	// --- Apollo 13: the oxygen tank ----------------------------------------
	{
		std::printf("Apollo 13 translunar coast, as flown:\n");
		FlightSim f(*FindMission(13), ropes);
		std::string err;
		if (!f.Start(StartPoint::TranslunarCoast, &err)) {
			std::printf("  start: %s\n", err.c_str());
			return 1;
		}
		ContingencyManager m(f, Difficulty::AsFlown, 1);
		m.Plan();
		f.secondHooks.push_back([&](FlightSim&, double) { m.Tick(); });
		Echo(f);
		f.SetWarp(10000);
		double bang = 55 * 3600 + 54 * 60 + 53;
		while (f.GET() < bang + 60 && !f.Lost())
			f.Advance(0.1);
		Check(m.Fired() >= 1, "tank 2 failed");
		std::printf("  O2 %.1f + %.1f kg, fuel cells %d, bus %.1f V\n",
			f.csmSystems.o2[0].quantity, f.csmSystems.o2[1].quantity,
			f.csmSystems.FuelCellsOnline(), f.csmSystems.mainBusA);
		// Two hours later.
		while (f.GET() < bang + 2.5 * 3600 && !f.Lost())
			f.Advance(0.1);
		std::printf("  +2.5 h: O2 %.1f kg, bus %.1f V, batteries %.0f/%.0f/%.0f A h\n",
			f.csmSystems.TotalO2(), f.csmSystems.mainBusA,
			f.csmSystems.batteryA.charge, f.csmSystems.batteryB.charge,
			f.csmSystems.batteryC.charge);
		Check(f.csmSystems.TotalO2() < 1, "tank 1 bled out too");
		// The lifeboat: everyone into Aquarius, the CM powered down.
		f.TransferCrew(3);
		f.lmSystems.powered = true;
		f.csmSystems.loads = PowerLoads();
		f.csmSystems.loads.computer = f.csmSystems.loads.imu = false;
		f.csmSystems.loads.scs = f.csmSystems.loads.comms = false;
		f.csmSystems.loads.heaters = f.csmSystems.loads.cabinFans = false;
		f.csmSystems.loads.lights = f.csmSystems.loads.glycolPumps = false;
		double t0 = f.GET();
		double co2Peak = 0;
		bool adapter = false;
		while (f.GET() < t0 + 40 * 3600 && !f.Lost()) {
			f.Advance(0.1);
			co2Peak = std::max(co2Peak, f.lmCabin.co2);
			if (!adapter && f.lmCabin.co2 > 7.6) {
				adapter = true;
				f.lioh.squareInRound = true;
				std::printf("  CO2 %.1f mm at +%.1f h: building the adapter\n",
					f.lmCabin.co2, (f.GET() - t0) / 3600);
			}
		}
		std::printf("  +40 h in the LM: O2 %.1f kg, water %.1f kg, power %.0f A h, "
			"CO2 %.1f mm (peak %.1f), cabin %.0f F, crew health %.0f\n",
			f.lmSystems.OxygenKg(), f.lmSystems.WaterKg(), f.lmSystems.AmpHours(),
			f.lmCabin.co2, co2Peak, f.lmCabin.temperature, f.crew.AverageHealth());
		Check(!f.Lost(), "crew alive in the lifeboat");
		Check(adapter, "CO2 forced the square-in-round adapter");
	}

	std::printf(gFailures ? "CONTINGENCIES FAILED (%d)\n" : "CONTINGENCIES OK\n",
		gFailures);
	return gFailures ? 1 : 0;
}
