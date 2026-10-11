// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Systems.h"

#include <algorithm>
#include <cmath>

namespace flight {

namespace {

const double kHour = 3600.0;

// Per-person metabolic rates.
const double kO2PerPersonHour = 0.035;      // kg (0.84 kg/day)
const double kWaterPerPersonHour = 0.105;   // kg (2.5 kg/day)
// Fuel-cell reactants and product water per kWh.
const double kFCO2PerKWh = 0.32;
const double kFCH2PerKWh = 0.040;
const double kFCWaterPerKWh = 0.36;
// LiOH canisters, crew-hours of scrubbing.
const double kCMCanister = 36.0;
const double kLMCanister = 40.0;   // primary cartridge, crew-hours
// Sublimator water for the LM's cooling, kg per kWh dissipated.
const double kSublimatorPerKWh = 1.0;

double
Clamp(double v, double lo, double hi)
{
	return std::max(lo, std::min(hi, v));
}


// CO2 and pressure for a cabin with `crew` aboard and the scrubber state.
void
Breathe(double hours, int crew, Cabin& cabin, LiOH& lioh, bool lm)
{
	double crewHours = crew * hours;
	if (crew > 0) {
		double& life = lm ? lioh.activeLifeLM : lioh.activeLife;
		if (life <= 0) {
			// The LM's cartridges are round, the CM's square: the CM's only
			// fit the LM's system through the improvised adapter.
			if (lm && lioh.canistersLM > 0) {
				lioh.canistersLM--;
				life = kLMCanister;
			} else if ((!lm || lioh.squareInRound) && lioh.canistersCM > 0) {
				lioh.canistersCM--;
				life = kCMCanister;
			}
		}
		if (life > 0) {
			life -= crewHours;
			// Scrubbing pulls CO2 back down toward ~1 mm Hg.
			cabin.co2 += (1.0 - cabin.co2) * std::min(1.0, hours * 0.5);
		} else {
			// About 1.4 mm Hg per crew-hour in the small cabins.
			cabin.co2 += 1.4 * crewHours / std::max(1, crew) * crew / 3.0
				* (lm ? 1.6 : 1.0);
		}
	}
	cabin.co2 = Clamp(cabin.co2, 0, 60);
	cabin.pressure = std::max(0.0, cabin.pressure - cabin.leakRate * hours);
}

}  // namespace


// ---------------------------------------------------------------------------
// CSM


CSMSystems::CSMSystems()
{
	o2[0] = {"O2 TANK 1", 145, 145, 900, 0, true, true, false};
	o2[1] = {"O2 TANK 2", 145, 145, 900, 0, true, true, false};
	h2[0] = {"H2 TANK 1", 12.7, 12.7, 245, 0, true, true, false};
	h2[1] = {"H2 TANK 2", 12.7, 12.7, 245, 0, true, true, false};
	batteryA = {"BAT A", 40, 40, false, false};
	batteryB = {"BAT B", 40, 40, false, false};
	batteryC = {"BAT C", 40, 40, false, false};
	pyroA = {"PYRO A", 0.75, 0.75, true, false};
	pyroB = {"PYRO B", 0.75, 0.75, true, false};
}


double
CSMSystems::Demand() const
{
	double a = 18;   // instrumentation, displays, C&W, essentials
	if (loads.computer) a += 3.5;
	if (loads.imu) a += 9;
	if (loads.optics) a += 1.5;
	if (loads.radar) a += 2;
	if (loads.scs) a += 6;
	if (loads.comms) a += 8;
	if (loads.heaters && attached) a += 10;
	if (loads.cabinFans) a += 3;
	if (loads.lights) a += 2;
	if (loads.glycolPumps) a += 3;
	if (loads.tv) a += 2;
	return a;
}


int
CSMSystems::FuelCellsOnline() const
{
	int n = 0;
	for (const FuelCell& f : fuelCell)
		if (f.online && !f.failed)
			n++;
	return n;
}


void
CSMSystems::Update(double dt, int crewAboard, Cabin& cabin, LiOH& lioh)
{
	double hours = dt / kHour;
	double amps = Demand();
	fLastDemand = amps;
	double kw = amps * 28.0 / 1000.0;

	// Cryogenic tanks: leaks, pressure tracks quantity.
	for (CryoTank* t : {&o2[0], &o2[1], &h2[0], &h2[1]}) {
		if (t->leakRate > 0)
			t->quantity = std::max(0.0, t->quantity - t->leakRate * hours);
		double frac = t->capacity > 0 ? t->quantity / t->capacity : 0;
		double nominal = (t == &h2[0] || t == &h2[1]) ? 245 : 900;
		t->pressure = t->quantity > 0.2 ? nominal * (0.25 + 0.75 * std::min(1.0, frac * 1.6))
			: t->quantity * 30;
	}

	// Fuel cells share the load if they have reactants; batteries pick up
	// what they can't carry.
	bool reactants = attached && TotalO2() > 0.5 && TotalH2() > 0.05;
	int cells = reactants ? FuelCellsOnline() : 0;
	double fcCapacityKW = cells * 1.42;
	double fromCells = std::min(kw, fcCapacityKW);
	double fromBatteries = kw - fromCells;
	if (powerTransfer) {
		// LM power through the umbilical carries some of it.
		fromBatteries = std::max(0.0, fromBatteries - 0.6);
	}
	for (FuelCell& f : fuelCell)
		f.load = (f.online && !f.failed && cells > 0) ? fromCells * 1000 / 28.0 / cells : 0;
	if (cells > 0) {
		double o2Use = fromCells * kFCO2PerKWh * hours;
		double h2Use = fromCells * kFCH2PerKWh * hours;
		double o2Both = o2[0].quantity + o2[1].quantity;
		double h2Both = h2[0].quantity + h2[1].quantity;
		for (CryoTank* t : {&o2[0], &o2[1]})
			if (o2Both > 0)
				t->quantity = std::max(0.0, t->quantity - o2Use * t->quantity / o2Both);
		for (CryoTank* t : {&h2[0], &h2[1]})
			if (h2Both > 0)
				t->quantity = std::max(0.0, t->quantity - h2Use * t->quantity / h2Both);
		double water = fromCells * kFCWaterPerKWh * hours;
		potableWater += water;
		if (potableWater > 16) {
			wasteWater += potableWater - 16;
			potableWater = 16;
		}
	}
	if (fromBatteries > 0) {
		double ah = fromBatteries * 1000 / 28.0 * hours;
		Battery* bats[3] = {&batteryA, &batteryB, &batteryC};
		int live = 0;
		for (Battery* b : bats)
			if (!b->failed && b->charge > 0)
				live++;
		for (Battery* b : bats)
			if (!b->failed && b->charge > 0 && live > 0)
				b->charge = std::max(0.0, b->charge - ah / live);
	}
	double available = (cells > 0 ? fcCapacityKW : 0)
		+ ((batteryA.charge + batteryB.charge + batteryC.charge) > 0.5 ? 2.0 : 0)
		+ (powerTransfer ? 0.6 : 0);
	double sag = kw > available ? (kw - available) / std::max(0.1, kw) : 0;
	mainBusA = mainBusB = available > 0 ? 28.8 - 1.2 * kw / std::max(0.5, available)
		- 10 * sag : 0;

	// Breathing oxygen comes from the cryo tanks (or the surge tank once
	// the SM is gone).
	double o2Need = crewAboard * kO2PerPersonHour * hours
		+ (cabin.leakRate > 0 ? cabin.leakRate * 0.2 * hours : 0);
	if (attached && TotalO2() > 0) {
		double both = o2[0].quantity + o2[1].quantity;
		for (CryoTank* t : {&o2[0], &o2[1]})
			t->quantity = std::max(0.0, t->quantity - o2Need * t->quantity / both);
	} else if (surgeO2 > 0) {
		surgeO2 = std::max(0.0, surgeO2 - o2Need);
	} else if (crewAboard > 0) {
		cabin.pressure = std::max(0.0, cabin.pressure - 0.4 * hours);
	}
	Breathe(hours, crewAboard, cabin, lioh, false);

	// Cabin temperature follows the heat the equipment makes.
	double target = 38 + 34 * std::min(1.0, kw / 1.8);
	cabin.temperature += (target - cabin.temperature) * std::min(1.0, hours / 6);
	// Water: the crew drinks from the potable tank.
	potableWater = std::max(0.0, potableWater
		- crewAboard * kWaterPerPersonHour * hours);
}


double
CSMSystems::OxygenHours(int crew) const
{
	double perHour = crew * kO2PerPersonHour + fLastDemand * 28 / 1000 * kFCO2PerKWh
		* (FuelCellsOnline() > 0 ? 1 : 0);
	return perHour > 0 ? TotalO2() / perHour : 1e9;
}


double
CSMSystems::PowerHours() const
{
	double kw = fLastDemand * 28 / 1000;
	if (FuelCellsOnline() > 0 && attached) {
		double perHour = kw * kFCH2PerKWh;
		return perHour > 0 ? TotalH2() / perHour : 1e9;
	}
	double ah = batteryA.charge + batteryB.charge + batteryC.charge;
	return fLastDemand > 0 ? ah / fLastDemand : 1e9;
}


// ---------------------------------------------------------------------------
// LM


LMSystems::LMSystems()
{
	for (int i = 0; i < 4; i++)
		descent[i] = {"DES " + std::to_string(i + 1), 415, 415, false, false};
	ascent[0] = {"ASC 5", 296, 296, false, false};
	ascent[1] = {"ASC 6", 296, 296, false, false};
	loads.scs = false;
	loads.heaters = true;
}


double
LMSystems::Demand() const
{
	if (!powered)
		return 1.5;   // heaters on the CSM's feed
	double a = 12;
	if (loads.computer) a += 3;
	if (loads.imu) a += 8;
	if (loads.radar) a += 5;
	if (loads.comms) a += 5;
	if (loads.heaters) a += 5;
	if (loads.cabinFans) a += 2;
	if (loads.glycolPumps) a += 2;
	if (loads.lights) a += 2;
	if (loads.tv) a += 2;
	if (supplyingCSM) a += 20;
	return a;
}


double
LMSystems::AmpHours() const
{
	double ah = ascent[0].charge + ascent[1].charge;
	if (descentStage)
		for (const Battery& b : descent)
			ah += b.charge;
	return ah;
}


double
LMSystems::OxygenKg() const
{
	return (descentStage ? descentO2 : 0) + ascentO2[0] + ascentO2[1];
}


double
LMSystems::WaterKg() const
{
	return (descentStage ? descentWater : 0) + ascentWater[0] + ascentWater[1];
}


void
LMSystems::Update(double dt, int crewAboard, Cabin& cabin, LiOH& lioh)
{
	double hours = dt / kHour;
	double amps = Demand();
	fLastDemand = amps;
	if (powered) {
		double ah = amps * hours;
		std::vector<Battery*> live;
		if (descentStage)
			for (Battery& b : descent)
				if (!b.failed && b.charge > 0)
					live.push_back(&b);
		for (Battery& b : ascent)
			if (!b.failed && b.charge > 0 && (live.empty() || !descentStage))
				live.push_back(&b);
		for (Battery* b : live)
			b->charge = std::max(0.0, b->charge - ah / live.size());
		busVoltage = live.empty() ? 0 : 28.0 + 3.0 * (1 - amps / 80.0);
		if (live.empty())
			powered = false;
		// Sublimator cooling boils water away.
		double water = amps * 28 / 1000 * kSublimatorPerKWh * hours;
		if (descentStage && descentWater > 0)
			descentWater = std::max(0.0, descentWater - water);
		else
			for (double& w : ascentWater)
				w = std::max(0.0, w - water / 2);
	} else {
		busVoltage = 0;
	}
	if (crewAboard > 0) {
		double o2 = crewAboard * kO2PerPersonHour * hours;
		if (descentStage && descentO2 > 0)
			descentO2 = std::max(0.0, descentO2 - o2);
		else
			for (double& t : ascentO2)
				t = std::max(0.0, t - o2 / 2);
		double drink = crewAboard * kWaterPerPersonHour * hours;
		if (descentStage && descentWater > 0)
			descentWater = std::max(0.0, descentWater - drink);
		else
			for (double& w : ascentWater)
				w = std::max(0.0, w - drink / 2);
		Breathe(hours, crewAboard, cabin, lioh, true);
		double target = powered ? 38 + 30 * std::min(1.0, amps / 50.0) : 38;
		cabin.temperature += (target - cabin.temperature) * std::min(1.0, hours / 6);
	}
}


double
LMSystems::PowerHours() const
{
	return fLastDemand > 0 ? AmpHours() / fLastDemand : 1e9;
}


double
LMSystems::WaterHours() const
{
	double perHour = fLastDemand * 28 / 1000 * kSublimatorPerKWh + 0.3;
	return WaterKg() / perHour;
}


// ---------------------------------------------------------------------------
// Crew


void
Crew::Update(double dt, const Cabin& cabin, double waterAvailable,
	bool sleepPeriod)
{
	double hours = dt / kHour;
	double food = rations == Full ? 1.0 : (rations == Reduced ? 0.6 : 0.3);
	foodDays = std::max(0.0, foodDays - hours / 24 * food);
	if (foodDays <= 0)
		food = 0;
	double drink = (water == WaterFull && waterAvailable > 1) ? 1.0
		: (waterAvailable > 0.2 ? 0.45 : 0.0);
	for (CrewMember& m : members) {
		m.asleep = sleepPeriod && !m.onEVA;
		m.fatigue += (m.asleep ? -12.0 : (m.onEVA ? 7.0 : 3.5)) * hours;
		m.fatigue = Clamp(m.fatigue, 0, 100);
		m.nourishment = Clamp(m.nourishment + (food - 1.0) * 4 * hours
			+ (food > 0.9 ? 2 * hours : 0), 0, 100);
		m.hydration = Clamp(m.hydration + (drink - 1.0) * 5 * hours
			+ (drink > 0.9 ? 3 * hours : 0), 0, 100);
		double harm = 0;
		if (cabin.co2 > 7.6) harm += (cabin.co2 - 7.6) * 0.15;
		if (cabin.co2 > 15) harm += (cabin.co2 - 15) * 0.6;
		if (cabin.temperature < 50) harm += (50 - cabin.temperature) * 0.02;
		if (cabin.pressure < 3.5) harm += (3.5 - cabin.pressure) * 8;
		if (m.hydration < 40) harm += (40 - m.hydration) * 0.05;
		if (m.nourishment < 30) harm += (30 - m.nourishment) * 0.02;
		if (m.fatigue > 85) harm += 0.2;
		if (m.sick) harm += 0.15;
		double heal = (harm == 0 && !m.sick) ? 0.4 : 0;
		m.health = Clamp(m.health + (heal - harm) * hours, 0, 100);
		// Morale: rest, health and comfort.
		double drift = (70 - m.morale) * 0.01;
		drift -= (m.fatigue > 70 ? 0.6 : 0) + (cabin.temperature < 55 ? 0.4 : 0)
			+ (m.sick ? 0.3 : 0) + (cabin.co2 > 7.6 ? 0.5 : 0);
		m.morale = Clamp(m.morale + drift * hours, 0, 100);
	}
}


double
Crew::AverageHealth() const
{
	return (members[0].health + members[1].health + members[2].health) / 3;
}


double
Crew::AverageMorale() const
{
	return (members[0].morale + members[1].morale + members[2].morale) / 3;
}

}  // namespace flight
