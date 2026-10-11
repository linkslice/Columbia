// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The whole flight: Earth, Moon and Sun; the Saturn, CSM and LM as rigid
// bodies; the CMC and LGC running their real programs in lockstep with
// the physics; docking, undocking and staging; landing and splashdown;
// the spacecraft systems and the crew; time acceleration.
//
// Time runs in lockstep with the computers (5 ms steps) up to the warp
// the host can sustain. Beyond that, during unpowered coasts, the
// vehicles are integrated with long steps and the computers' clocks are
// carried forward without executing (they are idle in coast; the state
// vectors they keep are time-tagged and propagate when next needed).

#ifndef COLUMBIA_FLIGHTSIM_H
#define COLUMBIA_FLIGHTSIM_H

#include "MissionSpec.h"
#include "Systems.h"
#include "../agc/AGC.h"
#include "../agc/DSKY.h"
#include "../gnc/CMPGNCS.h"
#include "../gnc/LMPGNCS.h"
#include "../gnc/Rope.h"
#include "../sim/Aero.h"
#include "../sim/MoonScene.h"
#include "../sim/Vehicle.h"
#include "../sim/World.h"

#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace flight {

class SaturnIU;

enum class Phase {
	Prelaunch, Boost, EarthParkingOrbit, TranslunarInjection, TranslunarCoast,
	LunarOrbit, PoweredDescent, LunarSurface, LunarAscent, Rendezvous,
	TransearthCoast, Entry, Recovery, Lost
};
const char* PhaseName(Phase phase);

enum class StartPoint {
	Launch,              // T-0 on the pad
	EarthOrbit,          // parking orbit, an hour before TLI
	TranslunarCoast,     // after transposition, docking and extraction
	LunarOrbit,          // CSM and LM docked in lunar orbit
	PoweredDescent,      // LM undocked, ten minutes before PDI
	LunarSurface,        // just landed
	LunarAscent,         // ten minutes before liftoff
	TransearthCoast,     // after TEI
	Entry                // half an hour before entry interface
};
const char* StartPointName(StartPoint start);

struct LogEntry {
	double get;
	enum Kind { Info, Capcom, Crew, Caution, Warning, Event } kind;
	std::string text;
};

// Where things are, for displays and the window view.
struct Situation {
	double get = 0;
	sim::Vec3 earth, moon, sun;   // inertial, Earth at the origin
	bool nearMoon = false;        // in the Moon's sphere of influence
	double altitude = 0;          // above the nearer body (m)
	double speed = 0;             // relative to that body (m/s)
	double apoapsis = 0, periapsis = 0;   // altitudes (m), < 0 = impact
	double inclination = 0;       // degrees
	double latitude = 0, longitude = 0;   // under the CSM
};

class FlightSim {
public:
	FlightSim(const MissionSpec& mission, const std::string& ropeDirectory);
	~FlightSim();

	bool Start(StartPoint start, std::string* error);
	const MissionSpec& Mission() const { return fMission; }

	// --- Time -------------------------------------------------------------
	double GET() const { return fGET; }
	double JulianDate() const;
	// Simulated seconds per real second.
	void SetWarp(double warp);
	double Warp() const { return fWarp; }
	// What the present situation allows (burns and atmosphere hold the
	// warp to what the computers can run in lockstep).
	double MaxWarp() const;
	void SetLockstepLimit(double limit) { fLockstepLimit = limit; }
	// Advance by `realSeconds` of wall time at the present warp.
	void Advance(double realSeconds);
	// Advance by exactly `simSeconds` of simulated time (tests, jumps).
	void Run(double simSeconds);
	bool Paused() const { return fPaused; }
	void SetPaused(bool paused) { fPaused = paused; }

	// --- Spacecraft -----------------------------------------------------------
	sim::Vehicle& CSMVehicle() { return *fCSM; }
	sim::Vehicle* LMVehicle() { return fLM.get(); }        // null while docked
	sim::Vehicle* BoosterVehicle() { return fBooster.get(); }
	const std::vector<std::unique_ptr<sim::Vehicle>>& Debris() const
		{ return fDebris; }
	bool HasLM() const { return fHasLM; }
	bool LMDocked() const { return fLMDocked; }
	bool LMLanded() const { return fLMLanded; }
	bool BoosterAttached() const { return fBoosterAttached; }
	bool SMAttached() const { return fSMAttached; }
	bool LMAscentOnly() const { return fLMAscentOnly; }

	agc::Computer& CMC() { return *fCMC; }
	agc::Computer* LGC() { return fLGC.get(); }
	agc::DSKY& CMCDSKY() { return fCMCDSKY; }
	agc::DSKY& LGCDSKY() { return fLGCDSKY; }
	gnc::CMPGNCS& CMPGNCS() { return *fCMPGNCS; }
	gnc::LMPGNCS* LMPGNCS() { return fLMPGNCS.get(); }
	gnc::PadLoader& CMCPad() { return *fCMCPad; }
	gnc::PadLoader* LGCPad() { return fLGCPad.get(); }
	const gnc::Rope& CMCRope() const { return fCMCRope; }

	// DSKY keys (key codes from agc::Key*; 'P' = PRO).
	void PressCMC(int key);
	void PressLGC(int key);
	void ProceedCMC(bool down);
	void ProceedLGC(bool down);

	// --- Crew actions on the hardware ----------------------------------------
	bool SeparateCSMFromBooster();       // CSM/S-IVB separation
	bool DockToLM();                     // capture, if close and slow
	bool ExtractLM();                    // pull the LM from the S-IVB
	bool UndockLM();
	bool JettisonLM();                   // after the crew's back aboard
	bool StageLM();                      // ascent stage fires the pyros
	bool SeparateCMFromSM();
	bool JettisonTower();
	void DeployDrogues();
	void DeployMains();
	// Docking geometry for the approach display: probe-to-drogue distance,
	// closing speed, and misalignment.
	bool DockingGeometry(double* range, double* rate, double* angle) const;
	// The same in vectors (inertial): the target drogue's position and
	// velocity, the target's docking axis (pointing out of the drogue), and
	// the CSM probe tip.
	struct DockingTarget {
		sim::Vec3 drogue, drogueVelocity, axis, probe;
	};
	bool Docking(DockingTarget* target) const;

	// Crew transfer between spacecraft (2 to the LM for the landing).
	int CrewInCM() const { return fCrewInCM; }
	int CrewInLM() const { return fCrewInLM; }
	bool TransferCrew(int toLM);

	// --- Systems and crew -----------------------------------------------------
	CSMSystems csmSystems;
	LMSystems lmSystems;
	Crew crew;
	Cabin cmCabin, lmCabin;
	LiOH lioh;
	bool sleepPeriod = false;

	// --- World ---------------------------------------------------------------
	const sim::MoonScene& Moon() const { return *fMoonScene; }
	const sim::world::Terrain& Terrain() const { return fTerrain; }
	sim::world::Terrain& EditTerrain() { return fTerrain; }
	Situation Situate(const sim::Vehicle& vehicle) const;
	Phase CurrentPhase() const { return fPhase; }
	void SetPhase(Phase phase);
	sim::EntryAeroState EntryState() const { return fEntry; }
	double MaxGLoad() const { return fMaxG; }

	// --- Log, CAPCOM ------------------------------------------------------------
	void Log(LogEntry::Kind kind, const std::string& text);
	const std::deque<LogEntry>& Entries() const { return fLog; }
	std::function<void(const LogEntry&)> onLog;

	// Outcome.
	bool Lost() const { return fPhase == Phase::Lost; }
	std::string LossReason() const { return fLossReason; }
	void Lose(const std::string& reason);

	// Hooks run every physics step (contingencies, crew automation).
	std::vector<std::function<void(FlightSim&, double)>> stepHooks;
	// Hooks run once per simulated second (systems, checks).
	std::vector<std::function<void(FlightSim&, double)>> secondHooks;

	// Saturn launch vehicle guidance (set by the boost module).
	std::function<void(FlightSim&, double)> boosterGuidance;
	// The Saturn's Instrument Unit (null once there's no S-IVB to fly).
	SaturnIU* IU() { return fIU.get(); }
	// Lockstep from this GET on (the crew has something to do then): long
	// coasting steps stop short of it.
	double lockstepFrom = 1e30;
	// Hold-down arms keep the Saturn on the pad until released.
	bool holdDown = true;
	// Drops a spent booster stage (S-IC, S-II, S-IB) or the S-IVB part of
	// the stack as debris; `separation` m/s along the stack's -X.
	bool JettisonStage(const std::string& module, double separation);
	bool TowerAttached() const { return fTowerAttached; }
	bool OnPad() const { return fOnPad; }

	// Hardware faults set by contingencies and read by the hardware model.
	struct Faults {
		int probeMisses = 0;          // docking: capture latches don't engage
		bool mainChuteFouled = false; // one of three mains collapsed
		bool stagingGyration = false; // LM: AGS mode switch left in AUTO
		bool sceFailed = false;       // CSM: signal conditioning lost
		bool smDamaged = false;       // CSM: service module structural damage
	} faults;
	// Crew actions for particular failures.
	void SCEToAux();                  // signal conditioning to AUX power
	void ResetFuelCells();            // reconnect the fuel cells to the buses
	void PenInBreaker();              // push the broken ascent-arm breaker
	void RRModeToLGC();               // rendezvous radar switch to LGC

	// The parking orbit's plane: through the pad at liftoff and, for a
	// lunar mission, the Moon at LOI (what the IU's launch azimuth
	// achieves). Unit normal along the orbit's angular momentum.
	sim::Vec3 ParkingOrbitNormal() const;
	// Launch azimuth that flies into that plane (degrees from north).
	double FlightAzimuth() const;

	// Platform alignment for a new start point.
	static sim::Mat3 LVLHReference(const sim::Vec3& r, const sim::Vec3& v);

private:
	void _BuildWorld();
	bool _LoadComputers(std::string* error);
	void _PadLoadCMC(const sim::Mat3& refsmmat);
	void _PadLoadLGC(const sim::Mat3& refsmmat);
	void _Step(double dt);           // lockstep
	void _Coast(double dt);          // long steps, computers idle
	void _Contacts();
	void _SecondTick();
	bool _CanCoast() const;
	sim::Vec3 _Gravity(const sim::Vec3& r, double jd) const;
	void _PlaceOnPad();
	void _StateForStart(StartPoint start);
	sim::Vehicle _MakeCSM(bool withSM);
	sim::Vehicle _MakeLM(bool descentStage);
	void _DockLMGeometry(sim::Vec3* offset, sim::Mat3* rotation) const;
	void _UpdatePGNCSAttachments();
	void _RemoveDistantDebris();

	MissionSpec fMission;
	std::string fRopeDir;
	double fGETZeroJD = 0;
	double fGET = 0;
	double fWarp = 1;
	double fLockstepLimit = 100;
	double fCarry = 0;
	double fSecondAccum = 0;
	bool fPaused = false;
	Phase fPhase = Phase::Prelaunch;
	std::string fLossReason;

	sim::world::Terrain fTerrain;
	std::unique_ptr<sim::MoonScene> fMoonScene;

	std::unique_ptr<sim::Vehicle> fCSM;      // CSM (+ docked LM, + booster)
	std::unique_ptr<sim::Vehicle> fLM;       // LM when flying separately
	std::unique_ptr<sim::Vehicle> fBooster;  // S-IVB after separation
	std::vector<std::unique_ptr<sim::Vehicle>> fDebris;
	bool fHasLM = false;
	bool fLMDocked = false;
	bool fLMInSLA = false;
	bool fLMLanded = false;
	bool fLMAscentOnly = false;
	bool fBoosterAttached = false;
	bool fSMAttached = true;
	bool fTowerAttached = false;
	bool fOnPad = false;
	bool fDrogues = false, fMains = false;
	bool fSplashed = false;
	sim::Vec3 fLMFixedPosition;              // Moon-fixed, while landed
	sim::Quat fLMFixedAttitude;
	sim::Vec3 fPadFixed;                     // Earth-fixed, on the pad
	sim::Quat fPadAttitude;
	int fCrewInCM = 3, fCrewInLM = 0;

	gnc::Rope fCMCRope, fLGCRope;
	std::unique_ptr<agc::Computer> fCMC, fLGC;
	agc::DSKY fCMCDSKY, fLGCDSKY;
	std::unique_ptr<gnc::CMPGNCS> fCMPGNCS;
	std::unique_ptr<gnc::LMPGNCS> fLMPGNCS;
	std::unique_ptr<gnc::PadLoader> fCMCPad, fLGCPad;
	bool fLGCPowered = false;

	std::unique_ptr<SaturnIU> fIU;
	sim::EntryAeroState fEntry;
	double fMaxG = 0;
	std::deque<LogEntry> fLog;
};

}  // namespace flight

#endif
