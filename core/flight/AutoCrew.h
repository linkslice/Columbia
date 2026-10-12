// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The automatic crew: checklists flown at the controls. Each procedure
// works the spacecraft the way the crew did -- keying the DSKY, answering
// its flashing requests, arming the engines, flying the hand controllers
// -- so the real flight software does the rest. It lets a player hand off
// any part of the flight ("autopilot to the next event") and it is how
// the simulator tests a whole mission end to end.

#ifndef COLUMBIA_AUTOCREW_H
#define COLUMBIA_AUTOCREW_H

#include "../agc/DSKY.h"
#include "../sim/Math.h"

#include <deque>
#include <map>
#include <functional>
#include <memory>
#include <string>

namespace flight {

class FlightSim;

// Keystrokes into one computer at a human pace: 'V' 'N' 'E' 'R' '+' '-'
// digits, and 'P' for PRO (held 0.3 s).
class DSKYDriver {
public:
	DSKYDriver(std::function<void(int)> press, std::function<void(bool)> proceed,
		std::function<const agc::DSKYState&()> state);
	void Type(const std::string& keys);
	// V37: select a major mode, retrying (after RSET) if the computer
	// refuses it for the moment (1520: busy integrating the state vector).
	void SelectProgram(int program);
	bool Idle() const { return fQueue.empty() && fWait <= 0; }
	// Idle, any program change finished, and the display settled.
	bool Ready() const
		{ return Idle() && fWantProgram < 0 && fStable > 1.0 && !State().oprErr; }
	void Step(double dt);
	void Clear() { fQueue.clear(); }

	const agc::DSKYState& State() const { return fState(); }
	std::string Program() const;
	std::string VerbNoun() const;          // e.g. "5018"
	std::string Register(int r) const;     // five digits, blanks as ' '
	bool Flashing() const { return State().verbNounFlash; }
	// Seconds the display has looked the same (a request is only answered
	// once the computer has finished writing it).
	double StableFor() const { return fStable; }
	std::string Display() const;           // one-line summary

	static std::string Signed(long value); // "+00135"
	// "V25E" + the hours / minutes / centiseconds of a GET.
	static std::string TimeEntry(double get);

private:
	std::function<void(int)> fPress;
	std::function<void(bool)> fProceed;
	std::function<const agc::DSKYState&()> fState;
	std::deque<char> fQueue;
	double fWait = 0;
	bool fProceedDown = false;
	int fWantProgram = -1;
	std::string fLastDisplay;
	double fStable = 0;
	double fSinceSelect = 0;
	int fTries = 0;
};

class AutoCrew;

class Procedure {
public:
	virtual ~Procedure() {}
	virtual std::string Name() const = 0;
	// Returns true when finished (successfully or not: see failed).
	virtual bool Step(AutoCrew& crew, FlightSim& f, double dt) = 0;
	// GET at which this procedure next needs lockstep (it may coast until
	// then); default: now.
	virtual double WakeGET(FlightSim& f) const;
	std::string status;
	bool failed = false;
	bool started = false;   // past its wake time, stepping
};

class AutoCrew {
public:
	explicit AutoCrew(FlightSim& sim);
	~AutoCrew();

	// Hooks into the simulation's step callbacks.
	void Install();

	void Add(std::unique_ptr<Procedure> procedure);
	bool Busy() const { return !fQueue.empty(); }
	Procedure* Current() { return fQueue.empty() ? nullptr : fQueue.front().get(); }
	void Abandon();               // the player takes over
	std::function<void(const std::string&)> onReport;   // crew chatter
	void Report(const std::string& text);

	DSKYDriver cmc, lgc;
	bool verbose = false;
	// Work the failures the crew notices (Apollo 12's SCE to AUX, the pen
	// in Apollo 11's breaker, ...). Off: the player's crew does nothing.
	bool troubleshoot = true;
	// Crew calls also go in the flight's log (as LogEntry::Crew).
	bool logToFlight = true;

	// --- Procedures --------------------------------------------------------
	// SPS (or docked-stack) burn: V48/V46, P30 with the PAD's TIG and LVLH
	// delta-V (m/s), P40 through ignition and cutoff.
	// The RTCC's external delta-V PAD for a burn whose (constant-attitude)
	// thrust should deliver `dvInertial` centered on `impulseGET`: returns
	// the TIG and the N81 LVLH components. P40 rotates an external delta-V
	// in the orbit plane before steering (S40.1's THETAT, for the arc flown
	// during the burn); the PAD is pre-rotated so the CMC ends up pointing
	// where Mission Control wants.
	static void ExternalDeltaV(FlightSim& f, double impulseGET,
		const sim::Vec3& dvInertial, double* tig, sim::Vec3* lvlh,
		double thrust = 91188, double isp = 314);
	// How far the CSM leads the LM around the Moon at `get` (degrees, in
	// the CSM's orbit plane; negative when it trails).
	static double PhaseAngle(FlightSim& f, double get);
	// The RTCC's liftoff time: the first ascent ignition after `earliest`
	// with the CSM leading the landed LM by `phase` degrees.
	static double LiftoffTime(FlightSim& f, double earliest, double phase);
	// A DPS burn with the CSM docked, flown by the LGC (Apollo 9's docked
	// burns, Apollo 13's lifeboat): uplinked state vector and REFSMMAT,
	// V48 for the docked stack, P30 with the PAD, P40 through cutoff.
	static std::unique_ptr<Procedure> DockedDPSBurn(double tig,
		const sim::Vec3& dvLVLH, const std::string& label);
	static std::unique_ptr<Procedure> CSMBurn(double tig, const sim::Vec3& dvLVLH,
		const std::string& label);
	// Entry: CM/SM separation, P61 to P67 to the splash point.
	static std::unique_ptr<Procedure> Entry(double splashLat, double splashLon);
	// The LM's powered descent: P63, P64, P65 automatic to touchdown.
	static std::unique_ptr<Procedure> Landing(double pdiGET);
	// Lunar liftoff: P12 with the PAD's TIG, abort stage, engine arm ascent,
	// PRO at V99; the LGC flies the ascent to orbit insertion.
	static std::unique_ptr<Procedure> Ascent(double tig);
	// LM-active rendezvous after insertion: TPI at the 26.6 degree elevation
	// angle, two midcourses, braking along the line of sight, station
	// keeping at the CSM -- then the CMP docks (CSM active).
	static std::unique_ptr<Procedure> Rendezvous();
	// Transposition, docking and extraction after TLI (or any CSM-active
	// docking with a free LM): separate, turn around, close on the drogue
	// with the hand controllers, dock, back the LM out.
	static std::unique_ptr<Procedure> TransposeAndDock(double startGET,
		bool transposition = true);
	// V48/V46 with the masses and configuration of the moment (after
	// undocking, docking or staging).
	static std::unique_ptr<Procedure> ConfigureCMCDAP();
	// Just keys (e.g. "V37E00E") into the CMC or LGC.
	static std::unique_ptr<Procedure> Keys(bool lgc, const std::string& keys,
		double atGET = 0);
	// Anything else: a function stepped until it returns true.
	static std::unique_ptr<Procedure> Do(const std::string& name,
		std::function<bool(AutoCrew&, FlightSim&, double)> step,
		double wakeGET = 0);

private:
	void _Step(double dt);
	void _Troubleshoot();

	FlightSim& f;
	std::map<std::string, double> fNoticed;   // failure -> GET first seen
	std::deque<std::unique_ptr<Procedure>> fQueue;
	bool fInstalled = false;
};

}  // namespace flight

#endif
