// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Builders for the Apollo modules: LM descent and ascent stages, the
// Command Module, the Service Module, the S-IVB and the Saturn V's lower
// stages. Masses default to Apollo 11 and are adjusted per mission.

#ifndef COLUMBIA_SPACECRAFT_H
#define COLUMBIA_SPACECRAFT_H

#include "Vehicle.h"

namespace sim {
namespace craft {

// --- LM -------------------------------------------------------------------
// LM frame: +X up through the overhead hatch (thrust axis), +Z forward out
// of the front hatch, +Y to the right. Origin at the stage interface.

struct LMConfig {
	double descentDry = 2034;       // kg
	double descentPropellant = 8248;
	double ascentDry = 2265;        // with crew (Apollo 11: 4,888 kg at liftoff)
	double ascentPropellant = 2353;
	double rcsPropellant = 287;     // both systems
};

Module MakeLMDescentStage(const LMConfig& config);
Module MakeLMAscentStage(const LMConfig& config);
void AddLMContacts(Vehicle& lm);    // footpads and probes

// DPS characteristics (Luminary's own numbers: FMAX, FSAT, BITPERF).
constexpr double kDPSFullThrust = 43454.7;    // N, throttle stop (FTP)
constexpr double kDPSMinThrust = 4670.0;      // N, ~10%
constexpr double kDPSPulsesPerNewton = 0.0797959872;
constexpr double kDPSGimbalRate = 0.2 * kDeg; // rad/s trim
constexpr double kDPSGimbalLimit = 6.0 * kDeg;
constexpr double kLMPropellantRCSJet = 445.0; // N

// LM RCS jet for each LGC channel bit: channel 5 bits 1-8 drive the
// up/down (X) jets, channel 6 bits 1-8 the horizontal (Y/Z) jets.
const char* LMJetName(int channel, int bit);

// --- CSM -------------------------------------------------------------------
// CSM frame: +X along the long axis out of the docking probe (SPS thrust
// is +X), +Z toward the crew's heads... (Apollo body axes), origin at the
// CM/SM interface.

struct CSMConfig {
	double cmMass = 5560;           // kg
	double smDry = 6110;
	double spsPropellant = 18410;
	double smRcsPropellant = 610;   // four quads (about 1,340 lb)
	double cmRcsPropellant = 123;   // two systems
};

Module MakeCommandModule(const CSMConfig& config);
Module MakeServiceModule(const CSMConfig& config);

constexpr double kSPSThrust = 91190.0;
constexpr double kSPSIsp = 314.0;

// CSM RCS jet for each CMC channel bit (channel 5: pitch/yaw, 6: roll).
const char* CSMJetName(int channel, int bit);
// The CM's own jets on the same bits, live after CM/SM separation.
const char* CMJetName(int channel, int bit);

// --- Launch vehicle ----------------------------------------------------------
// Saturn frame shares the CSM's +X (up the stack). Origin at the CSM/SLA
// interface; stages extend toward -X.

Module MakeSIVB(double propellant = 109000);
Module MakeSII();
Module MakeSIC();
Module MakeSIB();          // Saturn IB first stage
Module MakeLaunchEscapeTower();

}  // namespace craft
}  // namespace sim

#endif
