// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Erasable-memory pad loads. The invariant LGC values are NASA's own, from
// the pad-load section printed in the Luminary 069 listing (MIT/IL; public
// domain). Mission values (state vectors, REFSMMAT, landing site and time,
// masses, ephemeris epoch) are computed from the simulation's state, the
// way the RTCC computed and uplinked them.

#ifndef COLUMBIA_PADLOADS_H
#define COLUMBIA_PADLOADS_H

#include "Rope.h"
#include "../sim/Math.h"

namespace gnc {

// Flag helpers: flag N lives in STATE + N/15 at bit 15 - N%15.
bool SetFlag(PadLoader& pad, int flag, bool value);
bool GetFlag(const PadLoader& pad, int flag);

// Luminary flag numbers (identical across Luminary 069-210).
enum LuminaryFlag {
	FLAG_MOONFLAG = 3, FLAG_REFSMFLG = 47, FLAG_CMOONFLG = 123,
	FLAG_LMOONFLG = 124, FLAG_SURFFLAG = 127, FLAG_APSFLAG = 152
};

struct StateVector {
	sim::Vec3 position;   // m, relative to the Moon (or Earth)
	sim::Vec3 velocity;   // m/s
	double agcTime = 0;   // s on the AGC clock (GET)
	bool moon = true;
};

struct LMPadLoad {
	double agcTime = 0;          // GET now (s)
	double tland = 0;            // GET of the planned landing (s)
	double tephem = 0;           // s from July 1.0 UT of the year to GET 0
	double azo = 0;              // Earth rotation angle at July 1.0 (rev)
	sim::Vec3 rls;               // landing site, Moon-fixed (m)
	sim::Mat3 refsmmat;          // rows: stable-member axes
	StateVector lm, csm;
	double lmMass = 15100;       // kg
	double csmMass = 16500;      // kg
	bool descentStage = true;
	bool onSurface = false;
};

struct CMCPadLoad {
	double agcTime = 0;          // GET now (s)
	double tephem = 0;           // s from July 1.0 UT of the year to GET 0
	double azo = 0;              // Earth rotation angle at July 1.0 (rev)
	sim::Mat3 refsmmat;          // rows: stable-member axes
	bool refsmmatGood = true;
	StateVector csm, lm;
	bool lmState = false;        // load the LM state too
	double csmMass = 28800;      // kg
	double lmMass = 0;           // kg (0 when undocked)
	// V48 N46 digits as octal words (R1 ABCDE, R2 quads/channels).
	uint16_t dapData1 = 011102;  // CSM alone, 4-jet X, AC roll, 0.5 deg, 0.2 deg/s
	uint16_t dapData2 = 011111;
};

// Mission-dependent CMC values (Colossus, Comanche, Artemis).
void LoadCMCMission(PadLoader& pad, const CMCPadLoad& load);

// Values from the Luminary pad-load deck that do not change by mission.
void LoadLuminaryInvariants(PadLoader& pad);
// Mission-dependent values; also sets the relevant flags.
void LoadLuminaryMission(PadLoader& pad, const LMPadLoad& load);
// Replaces the LM (or CSM) state vector, as a MSFN uplink would.
void LoadStateVector(PadLoader& pad, const StateVector& sv, bool lm);

// Sets the AGC clock (TIME2/TIME1) to a GET in seconds.
void SetComputerClock(agc::Computer& computer, double seconds);

}  // namespace gnc

#endif
