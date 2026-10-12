// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "PadLoads.h"

#include "MissionPads.h"

#include <cstring>
#include <string>

#include <cmath>
#include <cstdint>

namespace gnc {

using sim::Vec3;

bool
SetFlag(PadLoader& pad, int flag, bool value)
{
	int word = flag / 15;
	int bit = 15 - flag % 15;
	uint16_t mask = uint16_t(1u << (bit - 1));
	uint16_t w = pad.Word("STATE", word);
	w = value ? uint16_t(w | mask) : uint16_t(w & ~mask);
	return pad.SetWord("STATE", w, word);
}


bool
GetFlag(const PadLoader& pad, int flag)
{
	int word = flag / 15;
	int bit = 15 - flag % 15;
	return (pad.Word("STATE", word) >> (bit - 1)) & 1;
}


void
SetComputerClock(agc::Computer& computer, double seconds)
{
	uint64_t cs = uint64_t(std::llround(seconds * 100.0));
	computer.SetCell(agc::REG_TIME1, uint16_t(cs & 037777));
	computer.SetCell(agc::REG_TIME2, uint16_t((cs >> 14) & 037777));
}


void
LoadLuminaryInvariants(PadLoader& p)
{
	// DAP state estimator and deadbands (R557 GSOP sec 3.3.2.3).
	p.Single("HIASCENT", 5050, -16);
	p.Octal("DKTRAP", 077001);
	p.Octal("DKOMEGAN", 000012);
	p.Octal("DKKAOSN", 000074);
	p.Octal("LMTRAP", 077001);
	p.Octal("LMOMEGAN", 000000);
	p.Octal("LMKAOSN", 000074);
	p.Octal("DKDB", 000200);
	p.Integer("ROLLTIME", 3000);
	p.Integer("PITTIME", 3000);
	p.Integer("ZOOMTIME", 2600);

	// IMU compensation: the simulated platform has no drift or bias, so
	// these are zero rather than the flight hardware's calibrations.
	for (const char* n : {"PIPASCFX", "PIPASCFY", "PIPASCFZ", "PBIASX",
			"PBIASY", "PBIASZ", "ADIAX", "ADIAY", "ADIAZ", "ADSRAX",
			"ADSRAY", "ADSRAZ", "NBDX", "NBDY", "NBDZ"})
		p.Octal(n, 0);

	// Rendezvous navigation.
	p.Double("RANGEVAR", .111111111e-4, 12);
	p.Double("RATEVAR", 1.87777e-5, 12);
	p.Single("RVARMIN", 66.0, -12);
	p.Single("VVARMIN", .17445e-5, 12);
	p.Double("ATIGINC", 42000, -28);
	p.Double("PTIGINC", 72000, -28);
	p.Octal("AOTAZ", 065252, 0);
	p.Octal("AOTAZ", 0, 1);
	p.Octal("AOTAZ", 012525, 2);
	p.Octal("AOTEL", 010000, 0);
	p.Octal("AOTEL", 010000, 1);
	p.Octal("AOTEL", 010000, 2);
	p.Octal("DUMPCNT", 010000);

	// Lunar landing targets in guidance coordinates (x up through the site,
	// z downrange), m, m/cs, m/cs^2, m/cs^3. These are the Apollo 11
	// braking- and approach-phase targets (ft, ft/s: R (171.835, -10678.596),
	// V (-105.876, -1.04), A (0.6241, -9.1044), J -0.01882677; approach R
	// (111.085, -26.794), V (-4.993, 0.248), A (-0.2624, -0.512),
	// J 0.00180772). The starred words are the TTF-polynomial forms:
	// V* = 18V/8, A* = 6A, J* = 8J.
	p.Vector("RBRFG", Vec3(52.375308, 0, -3254.83606), -24);
	p.Vector("VBRFG", Vec3(-0.322710048, 0, -0.00316992), -10);
	p.Vector("ABRFG", Vec3(1.9022568e-05, 0, -0.000277502112), 4);
	p.Double("VBRFG*", -0.00713232, -10);
	p.Double("ABRFG*", -0.00166501267, 4);
	p.Double("JBRFG*", -4.5907196e-08, 18);
	p.Vector("RAPFG", Vec3(33.858708, 0, -8.1668112), -24);
	p.Vector("VAPFG", Vec3(-0.015218664, 0, 0.000755904), -10);
	p.Vector("AAPFG", Vec3(-7.997952e-06, 0, -1.560576e-05), 4);
	p.Double("VAPFG*", 0.001700784, -10);
	p.Double("AAPFG*", -9.363456e-05, 4);
	p.Double("JAPFG*", 4.40794445e-09, 18);
	// Guidance-frame re-erection windows left closed (frame fixed at
	// ignition); Luminary 1A pads these, older ropes don't have them.
	p.Double("GAINBRAK", 0, 0);
	p.Single("TCGFBRAK", 0, -17);
	p.Single("TCGIBRAK", 0, -17);
	p.Double("GAINAPPR", 0, 0);
	p.Single("TCGFAPPR", 0, -17);
	p.Single("TCGIAPPR", 0, -17);

	// Ignition point: 15.24 km perilune, ~492 km uprange (16.25 deg of arc)
	// at 1699.5 m/s, which with these targets reproduces the Apollo 11
	// timeline (throttle-down ~6:25 after ignition, high gate ~7,500 ft).
	p.Double("VIGN", 1.69952182e1, -10);
	p.Double("RIGNX", -5.4701e4, -24);
	p.Double("RIGNZ", -4.899e5, -24);
	p.Double("KIGNX/B4", -.022499999, 0);
	p.Double("KIGNY/B8", -.174716605, 0);
	p.Double("KIGNV/B4", -.165939331, 0);
	p.Octal("LOWCRIT", 004251);
	p.Octal("HIGHCRIT", 004622);
	// Phase switching (TTF/8 units, i.e. centiseconds B-17): braking ends
	// 60 s before its target, approach 10 s before; TTF is pushed out
	// 60 s at the switch and guidance projects 2.2 s ahead.
	p.Single("TENDBRAK", 60e2, -17);
	p.Single("TENDAPPR", 10e2, -17);
	p.Single("DELTTFAP", -60e2, -17);
	p.Single("LEADTIME", -220, -17);

	// P65 velocity nulling to -3 ft/s with a 6 s time constant; P66 rate
	// of descent: 1 ft/s per click, 2 s time constant, throttle lag 0.2 s,
	// force limits 4359 N and 28024 N (kg m/cs^2 at B-12).
	p.Vector("V2FG", Vec3(-0.009144, 0, 0), -10);
	p.Double("TAUVERT", 600, -14);
	p.Octal("RODSCALE", 014370);
	p.Double("TAUROD", 200, -9);
	p.Double("LAG/TAU", 0.1, 0);
	p.Double("MINFORCE", 0.435926, -12);
	p.Double("MAXFORCE", 2.80238, -12);
	// Luminary 1D/1E/1F (Apollo 13-17): the automatic P66 after P64 nulls
	// horizontal velocity with pad-loaded gains, and skips any pass that
	// starts later than 2LATE466 after PIPTIME. Absent from the Apollo 12
	// loads, these would be zero: P66 would never throttle. Values: a 5 s
	// nulling time constant (TAUHZ, cs at 2^11), no command feedback
	// (QHZ), horizontal acceleration limited to about a 20-degree tilt
	// (AHZLIM, m/cs/cs at 2^-4), and 1.5 s of grace (cs at 2^28).
	p.Single("TAUHZ", 500, -11);
	p.Single("QHZ", 0, 0);
	p.Single("AHZLIM", 1.62 * 0.364 / 1e4, 4);
	p.Double("2LATE466", 150, -28);
	p.Single("RPCRTIME", 300e2, -17);
	p.Single("RPCRTQSW", .35356, 0);

	// Landing radar.
	p.Octal("LRALPHA", 001042);
	p.Octal("LRBETA1", 004210);
	p.Octal("LRALPHA2", 001042);
	p.Octal("LRBETA2", 000000);
	p.Integer("LRHMAX", 15240);
	p.Single("LRVMAX", .047625, 0);
	p.Single("LRVF", .0119, 0);
	p.Single("LRWH", .35, 0);
	p.Single("LRWVZ", .7, 0);
	p.Single("LRWVY", .7, 0);
	p.Single("LRWVX", .4, 0);
	p.Single("LRWVFZ", .5, 0);
	p.Single("LRWVFY", .5, 0);
	p.Single("LRWVFX", .3, 0);
	p.Single("LRWVFF", .1, 0);
	p.Double("DELQFIX", 15.24, -24);

	// Ascent and aborts.
	p.Single("TBRKPNT", 54000, -17);
	p.Double("ABTVINJ1", 16.91945, -7);
	p.Double("ABTVINJ2", 16.79448, -7);

	// Planetary orientation residuals.
	p.Vector("504LM", Vec3(), 0);
	p.Double("AXO", 0, 0);
	p.Double("-AYO", 0, 0);
}


void
LoadStateVector(PadLoader& p, const StateVector& sv, bool lm)
{
	const char* suffix = lm ? "LEM" : "CSM";
	int rs = sv.moon ? -27 : -29;    // metres, scaled B-27 (Moon) / B-29
	int vs = sv.moon ? -5 : -7;      // m/cs, scaled B-5 / B-7
	Vec3 vcs = sv.velocity / 100.0;
	std::string s(suffix);
	p.Vector("RRECT" + s, sv.position, rs);
	p.Vector("VRECT" + s, vcs, vs);
	p.Vector("RCV" + s, sv.position, rs);
	p.Vector("VCV" + s, vcs, vs);
	p.Vector("DELTA" + s, Vec3(), 0);
	p.Vector("NUV" + s, Vec3(), 0);
	p.Double("TC" + s, 0, 0);
	p.Double("XKEP" + s, 0, 0);
	p.Double("TET" + s, sv.agcTime * 100.0, -28);
	SetFlag(p, lm ? FLAG_LMOONFLG : FLAG_CMOONFLG, sv.moon);
}


void
LoadLuminaryMission(PadLoader& p, const LMPadLoad& m)
{
	// Masses (kg, B-16).
	p.Double("MASS", m.lmMass, -16);
	p.Single("LEMMASS", m.lmMass, -16);
	p.Single("CSMMASS", m.csmMass, -16);

	// Landing site and time.
	p.Vector("RLS", m.rls, -27);
	p.Double("TLAND", m.tland * 100.0, -28);

	// Ephemeris epoch: triple precision centiseconds.
	uint64_t cs = uint64_t(std::llround(m.tephem * 100.0));
	p.Octal("TEPHEM", uint16_t((cs >> 28) & 037777), 0);
	p.Octal("TEPHEM", uint16_t((cs >> 14) & 037777), 1);
	p.Octal("TEPHEM", uint16_t(cs & 037777), 2);
	p.Double("AZO", m.azo, 0);

	// REFSMMAT: unit vectors scaled B-1.
	p.Vector("REFSMMAT", m.refsmmat.Row(0), -1);
	Vec3 r1 = m.refsmmat.Row(1), r2 = m.refsmmat.Row(2);
	uint16_t hi, lo;
	double comps[6] = {r1.x, r1.y, r1.z, r2.x, r2.y, r2.z};
	for (int i = 0; i < 6; i++) {
		EncodeDP(std::ldexp(comps[i], -1), &hi, &lo);
		p.Octal("REFSMMAT", hi, 6 + 2 * i);
		p.Octal("REFSMMAT", lo, 7 + 2 * i);
	}
	SetFlag(p, FLAG_REFSMFLG, true);

	LoadStateVector(p, m.lm, true);
	LoadStateVector(p, m.csm, false);
	SetFlag(p, FLAG_MOONFLAG, m.lm.moon);
	SetFlag(p, FLAG_SURFFLAG, m.onSurface);
	SetFlag(p, FLAG_APSFLAG, !m.descentStage);
}

void
LoadCMCMission(PadLoader& p, const CMCPadLoad& m)
{
	uint64_t cs = uint64_t(std::llround(m.tephem * 100.0));
	p.Octal("TEPHEM", uint16_t((cs >> 28) & 037777), 0);
	p.Octal("TEPHEM", uint16_t((cs >> 14) & 037777), 1);
	p.Octal("TEPHEM", uint16_t(cs & 037777), 2);
	p.Double("AZO", m.azo, 0);
	// Earth's spin axis in the reference frame (UNITW, whose X and Y words
	// are -AYO and AXO): the frame's own pole, since the simulation's
	// reference frame is of date.
	p.Octal("UNITW", 0, 0);
	p.Octal("UNITW", 0, 1);
	p.Octal("UNITW", 0, 2);
	p.Octal("UNITW", 0, 3);
	p.Octal("UNITW", 037777, 4);
	p.Octal("UNITW", 037777, 5);

	p.Vector("REFSMMAT", m.refsmmat.Row(0), -1);
	Vec3 r1 = m.refsmmat.Row(1), r2 = m.refsmmat.Row(2);
	uint16_t hi, lo;
	double comps[6] = {r1.x, r1.y, r1.z, r2.x, r2.y, r2.z};
	for (int i = 0; i < 6; i++) {
		EncodeDP(std::ldexp(comps[i], -1), &hi, &lo);
		p.Octal("REFSMMAT", hi, 6 + 2 * i);
		p.Octal("REFSMMAT", lo, 7 + 2 * i);
	}
	SetFlag(p, FLAG_REFSMFLG, m.refsmmatGood);

	LoadStateVector(p, m.csm, false);
	if (m.lmState)
		LoadStateVector(p, m.lm, true);
	SetFlag(p, FLAG_MOONFLAG, m.csm.moon);

	// Masses in kg, scaled B-16 (V48 N47).
	p.Single("CSMMASS", m.csmMass, -16);
	p.Single("LEMMASS", m.lmMass, -16);
	p.Octal("DAPDATR1", m.dapData1);
	p.Octal("DAPDATR2", m.dapData2);
}


int
ApplyDescentTargetsXZ(PadLoader& pad, const PadWord* table)
{
	if (!table || !pad.Has("RBRFGX"))
		return 0;
	static const char* const kVectors[] = {"RBRFG", "RAPFG", "VBRFG", "VAPFG",
		"ABRFG", "AAPFG"};
	int loaded = 0;
	for (const char* v : kVectors) {
		for (const PadWord* w = table; w->name; w++) {
			if (std::strcmp(w->name, v) != 0 || (w->index >= 2 && w->index <= 3))
				continue;
			// Words 0-1: X, 2-3: Y (not kept), 4-5: Z.
			std::string name = std::string(v) + (w->index < 2 ? "X" : "Z");
			if (pad.Octal(name, w->value, w->index & 1))
				loaded++;
		}
	}
	return loaded;
}

}  // namespace gnc
