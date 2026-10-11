// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Spacecraft systems and consumables — the "Oregon Trail" side of the
// game. Cryogenic oxygen and hydrogen feed three fuel cells that make the
// CSM's power and water; batteries carry the LM and the CM after
// separation; lithium hydroxide scrubs the crew's CO2. Every powered
// box draws current, every hour the crew eats, drinks and tires, and
// failures turn these budgets into the mission's real problem.
//
// Rates come from the Apollo Operations Handbooks and mission reports
// (nominal CSM load ~2 kW, LM descent stage battery capacity, LiOH
// canister life, crew metabolic rates).

#ifndef COLUMBIA_SYSTEMS_H
#define COLUMBIA_SYSTEMS_H

#include <string>
#include <vector>

namespace flight {

struct CryoTank {
	std::string name;
	double quantity = 0;      // kg
	double capacity = 0;
	double pressure = 0;      // psia
	double leakRate = 0;      // kg/h (failure)
	bool heaters = true, fans = true;
	bool failed = false;
};

struct FuelCell {
	bool online = true;
	bool failed = false;
	double temperature = 420;  // F, skin
	double load = 0;           // A
};

struct Battery {
	std::string name;
	double charge = 0;         // A h
	double capacity = 0;
	bool online = false;
	bool failed = false;
};

// Electrical loads that can be switched (and are by procedures).
struct PowerLoads {
	bool computer = true;      // AGC + DSKY
	bool imu = true;           // IMU operate + heaters
	bool optics = false;
	bool radar = false;        // RR / LR / VHF ranging
	bool scs = true;           // CSM stabilization and control
	bool comms = true;         // S-band, HGA
	bool heaters = true;       // RCS/SPS/cryo heaters
	bool cabinFans = true;
	bool lights = true;
	bool glycolPumps = true;   // ECS cooling
	bool tv = false;
};

struct CrewMember {
	std::string name;
	std::string role;          // CDR, CMP, LMP
	double health = 100;       // 0..100
	double fatigue = 0;        // 0 rested .. 100 exhausted
	double morale = 80;        // 0..100
	double hydration = 100;
	double nourishment = 100;
	double radiation = 0;      // rem
	bool sick = false;
	std::string condition;     // "", "head cold", "nausea", "infection"...
	bool asleep = false;
	bool onEVA = false;
};

struct Cabin {
	double pressure = 5.0;     // psia
	double co2 = 0.5;          // mm Hg partial pressure
	double temperature = 72;   // F
	double humidity = 50;      // %
	double leakRate = 0;       // psi/h (failure)
};

struct LiOH {
	int canistersCM = 0;       // spares stowed
	int canistersLM = 0;
	double activeLife = 0;     // crew-hours left in the installed canister
	bool squareInRound = false;  // the Apollo 13 adapter in use
};

class CSMSystems {
public:
	CSMSystems();
	CryoTank o2[2], h2[2];
	FuelCell fuelCell[3];
	Battery batteryA, batteryB, batteryC;   // entry/post-landing
	Battery pyroA, pyroB;
	PowerLoads loads;
	double potableWater = 16;  // kg
	double wasteWater = 10;
	double surgeO2 = 1.7;      // kg, surge tank (for entry)
	double mainBusA = 28.5, mainBusB = 28.5;  // volts
	bool sceAux = false;       // Signal Conditioning Equipment on AUX
	bool attached = true;      // service module still on
	bool powerTransfer = false; // drawing on the LM (lifeboat)

	// Current demand in amperes at 28 V for the present loads.
	double Demand() const;
	// Advance by dt seconds; `crewAboard` breathe the cabin.
	void Update(double dt, int crewAboard, Cabin& cabin, LiOH& lioh);
	double TotalO2() const { return o2[0].quantity + o2[1].quantity; }
	double TotalH2() const { return h2[0].quantity + h2[1].quantity; }
	int FuelCellsOnline() const;
	// Hours of oxygen and power left at the present rates.
	double OxygenHours(int crew) const;
	double PowerHours() const;

	// Projected for warnings.
	bool MainBusUndervolt() const { return mainBusA < 26.25 || mainBusB < 26.25; }

private:
	double fLastDemand = 0;
};

class LMSystems {
public:
	LMSystems();
	Battery descent[4];        // 415 A h each
	Battery ascent[2];         // 296 A h each
	PowerLoads loads;
	double descentO2 = 22.0;   // kg
	double ascentO2[2] = {1.1, 1.1};
	double descentWater = 151; // kg (cooling and drinking)
	double ascentWater[2] = {19.1, 19.1};
	bool descentStage = true;
	bool powered = false;
	bool supplyingCSM = false; // LM -> CSM umbilical power (Apollo 13)
	double busVoltage = 0;

	double Demand() const;
	void Update(double dt, int crewAboard, Cabin& cabin, LiOH& lioh);
	double OxygenKg() const;
	double WaterKg() const;
	double AmpHours() const;
	double PowerHours() const;
	double WaterHours() const;

private:
	double fLastDemand = 0;
};

// The crew and their supplies.
class Crew {
public:
	CrewMember members[3];
	double foodDays = 14;      // ration-days for three
	enum Rations { Full, Reduced, Minimal } rations = Full;
	enum Water { WaterFull, WaterRationed } water = WaterFull;

	// Advance by dt with the environment the crew is in.
	void Update(double dt, const Cabin& cabin, double waterAvailable,
		bool sleepPeriod);
	double AverageHealth() const;
	double AverageMorale() const;
	bool Incapacitated(int index) const { return members[index].health < 15; }
};

}  // namespace flight

#endif
