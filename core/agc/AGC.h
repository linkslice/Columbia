// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// A Block II Apollo Guidance Computer emulator, written from the public
// instruction-set documentation. It runs the original (public domain)
// Comanche/Colossus and Luminary rope images unmodified.
//
// Word conventions used throughout:
//   * 15-bit ones-complement words live in the low 15 bits of a uint16_t.
//   * The 16-bit registers (A, Q) carry an extra sign bit (bit 16, 0x8000)
//     so that overflow can be represented: bit16 != bit15 means overflow.

#ifndef COLUMBIA_AGC_H
#define COLUMBIA_AGC_H

#include <cstdint>
#include <functional>
#include <string>

namespace agc {

enum Register : uint16_t {
	REG_A = 0, REG_L = 1, REG_Q = 2, REG_EB = 3, REG_FB = 4, REG_Z = 5,
	REG_BB = 6, REG_ZERO = 7,
	REG_ARUPT = 010, REG_LRUPT = 011, REG_QRUPT = 012,
	REG_ZRUPT = 015, REG_BBRUPT = 016, REG_BRUPT = 017,
	REG_CYR = 020, REG_SR = 021, REG_CYL = 022, REG_EDOP = 023,
	REG_TIME2 = 024, REG_TIME1 = 025, REG_TIME3 = 026, REG_TIME4 = 027,
	REG_TIME5 = 030, REG_TIME6 = 031,
	REG_CDUX = 032, REG_CDUY = 033, REG_CDUZ = 034,
	REG_OPTY = 035, REG_OPTX = 036,
	REG_PIPAX = 037, REG_PIPAY = 040, REG_PIPAZ = 041,
	REG_RHCP = 042, REG_RHCY = 043, REG_RHCR = 044,
	REG_INLINK = 045, REG_RNRAD = 046, REG_GYROCTR = 047,
	REG_CDUXCMD = 050, REG_CDUYCMD = 051, REG_CDUZCMD = 052,
	REG_CDUTCMD = 053, REG_CDUSCMD = 054,   // optics drive; SPS TVC yaw, pitch
	REG_THRUST = 055, REG_LEMONM = 056, REG_OUTLINK = 057, REG_ALTM = 060
};

enum Interrupt {
	RUPT_T6 = 0, RUPT_T5, RUPT_T3, RUPT_T4, RUPT_KEY1, RUPT_KEY2,
	RUPT_UP, RUPT_DOWN, RUPT_RADAR, RUPT_HAND, RUPT_COUNT
};

// DSKY keycodes as delivered on input channel 015.
enum Key {
	KEY_0 = 020, KEY_1 = 1, KEY_2 = 2, KEY_3 = 3, KEY_4 = 4, KEY_5 = 5,
	KEY_6 = 6, KEY_7 = 7, KEY_8 = 010, KEY_9 = 011,
	KEY_VERB = 021, KEY_RSET = 022, KEY_KEYREL = 031, KEY_PLUS = 032,
	KEY_MINUS = 033, KEY_ENTR = 034, KEY_CLR = 036, KEY_NOUN = 037
};

// Duration of one memory cycle (MCT): 12 pulses of the 1.024 MHz clock.
constexpr double kMCTSeconds = 12.0 / 1024000.0;

class Computer {
public:
	static constexpr int kFixedBanks = 36;
	static constexpr int kErasableBanks = 8;

	Computer();

	// Loads a rope image in the common .bin layout (banks 2,3,0,1,4..43;
	// big-endian words, data in bits 15..1). Returns false on bad input.
	bool LoadRope(const uint8_t* data, size_t size);
	bool LoadRopeFile(const std::string& path);

	// Power-on: clears erasable memory, then GOJAM.
	void PowerOn();
	// Hardware restart (GOJAM): erasable memory is preserved.
	void Restart();

	// Moves the mission clock (TIME2/TIME1) forward without executing
	// code, like the V55 clock increment. Used when the simulation runs
	// faster than the emulator can keep up with.
	void AdvanceClock(double seconds);

	// Executes one instruction (plus any pending counter cycles).
	// Returns the number of memory cycles consumed.
	int Step();
	// Runs for the given amount of AGC time.
	void Run(double seconds);

	// --- I/O -----------------------------------------------------------
	uint16_t Channel(int ch) const;
	void SetInputChannel(int ch, uint16_t value);
	void SetInputBit(int ch, int bit, bool on);  // bit is 1-based
	void PressKey(int keycode);                  // KEYRUPT1
	void SetProceed(bool pressed);               // PRO key (ch 032 bit 14)
	void RequestInterrupt(Interrupt which);

	// Called whenever the program writes an output channel.
	std::function<void(int channel, uint16_t value)> onChannelWrite;

	// --- Counter cells (driven by spacecraft hardware models) -----------
	void CountUp(uint16_t cell);    // PINC
	void CountDown(uint16_t cell);  // MINC
	// Adds a signed number of PINC/MINC counts to a ones-complement
	// counter (PIPAs, RHC). Returns true if the counter overflowed.
	bool AddCounts(uint16_t cell, int counts);
	// Counter traffic that changes nothing but still takes memory cycles
	// (e.g. a CDU hunting back and forth): `mct` cycles stolen.
	void StealCycles(int mct);
	// Raw access to unswitched cells and counters (CDUs track gimbals in
	// two's complement and are set directly).
	uint16_t Cell(uint16_t cell) const { return fErasable[0][cell & 0377]; }
	void SetCell(uint16_t cell, uint16_t value)
		{ fErasable[0][cell & 0377] = value & 077777; }

	// --- Erasable memory (pad loads, telemetry) ---------------------------
	uint16_t ReadErasable(int bank, int offset) const
		{ return fErasable[bank & 7][offset & 0377]; }
	void WriteErasable(int bank, int offset, uint16_t value)
		{ fErasable[bank & 7][offset & 0377] = value & 077777; }

	// --- Peripheral interrupts -------------------------------------------------
	// A radar word arrives in RNRAD and RADARUPT is requested.
	void RadarData(uint16_t value);
	// MSFN uplink: one keycode, sent triply redundant into INLINK.
	void Uplink(int keycode);
	// Navigation-panel/LM discretes on channel 16 with KEYRUPT2.
	void NavKey(uint16_t bits);

	// --- Inspection ------------------------------------------------------
	uint16_t Erasable(int bank, int offset) const;
	uint16_t Fixed(int bank, int offset) const;
	uint16_t ReadAddress(uint16_t address);  // through current banking
	uint16_t Z() const { return fZ; }
	uint64_t Cycles() const { return fCycles; }
	double Time() const { return fCycles * kMCTSeconds; }
	bool RestartLight() const { return fRestartLight; }
	void ClearRestartLight() { fRestartLight = false; }
	uint32_t RestartCount() const { return fRestarts; }
	// Interrupts taken per vector since power-on (diagnostics).
	uint32_t InterruptCount(int which) const { return fRuptCount[which]; }

	// Hardware alarms (night watchman, TC trap, rupt lock). They force a
	// restart exactly like the real computer. Enabled by default.
	void SetAlarmsEnabled(bool on) { fAlarmsEnabled = on; }
	// Telemetry: the downlink system takes two words every 20 ms and
	// signals each pickup with DOWNRUPT. Enabled by default.
	void SetTelemetryEnabled(bool on) { fTelemetry = on; }

private:
	// Memory primitives.
	uint16_t* _Cell(uint16_t address);
	uint16_t _Read(uint16_t address);              // raw stored value
	uint16_t _Load(uint16_t address);              // 16-bit view
	uint16_t _LoadOperand(uint16_t address);       // + edit rewrite
	void _Store(uint16_t address, uint16_t value16);
	void _StoreEdited(uint16_t address, uint16_t value15);
	bool _Is16Bit(uint16_t address) const
		{ return address == REG_A || address == REG_Q; }

	uint16_t _ReadChannel(int ch);
	void _WriteChannel(int ch, uint16_t value16);

	void _Fetch();
	void _Execute(uint16_t instruction, bool extended);
	void _Counters();
	bool _InterruptsAllowed() const;
	void _TakeInterrupt(int which);
	void _Tick(int mct);
	void _StealCycles(int mct);
	void _Traps();
	void _CounterIncrement(uint16_t cell, int delta, bool* overflow);
	void _CounterOverflow(uint16_t address);
	void _Alarm();

	// Arithmetic helpers.
	void _DoubleAdd(uint16_t k);
	void _Multiply(uint16_t k);
	void _Divide(uint16_t k);

	uint16_t fErasable[kErasableBanks][256];
	uint16_t fFixed[kFixedBanks][1024];
	uint16_t fChannels[0200];

	uint16_t fZ;            // address of the instruction after B
	uint16_t fB;            // next instruction to execute
	uint16_t fEB, fFB;      // bank numbers
	uint16_t fIndex;        // pending index value
	bool fIndexPending;
	bool fIndexApplied;     // B was modified by INDEX (no rupt yet)
	bool fExtend;         // next instruction is an extracode
	bool fExtendPending;    // EXTEND executed (applies after fetch)
	bool fInhibit;          // INHINT in effect
	bool fInISR;            // between interrupt entry and RESUME
	bool fPendingRupt[RUPT_COUNT];
	uint32_t fRuptCount[RUPT_COUNT] = {};

	uint64_t fCycles;
	double fTimerAccum;     // seconds toward the next 10 ms scaler tick
	double fT6Accum;        // seconds toward the next TIME6 tick
	bool fT4Phase;
	bool fTelemetry;
	double fDownAccum;
	double fBudget;         // for Run()

	// Alarms.
	bool fAlarmsEnabled;
	bool fRestartLight;
	uint32_t fRestarts;
	double fNightWatchman;  // time since NEWJOB was last touched
	double fRuptLockTime;   // time spent continuously in an ISR
	double fNoRuptTime;     // time without any interrupt
	double fTCTrapTime;     // time executing only TC/TCF
	double fSinceTC;        // time since the last TC/TCF
	bool fNewJobTouched;
};

// Helpers shared with tests.
inline uint16_t SignExtend(uint16_t w15)
	{ return (w15 & 040000) ? (w15 | 0100000) : (w15 & 077777); }
inline uint16_t OverflowCorrect(uint16_t w16)
	{ return ((w16 & 0100000) >> 1) | (w16 & 037777); }
inline bool HasOverflow(uint16_t w16)
	{ return ((w16 >> 15) & 1) != ((w16 >> 14) & 1); }
inline uint16_t Add16(uint16_t a, uint16_t b)
{
	uint32_t s = uint32_t(a) + b;
	if (s > 0177777)
		s = (s + 1) & 0177777;
	return uint16_t(s);
}
inline uint16_t Add15(uint16_t a, uint16_t b)
{
	uint32_t s = uint32_t(a & 077777) + (b & 077777);
	if (s > 077777)
		s = (s + 1) & 077777;
	return uint16_t(s);
}
// Converts a 15-bit ones-complement word to a signed integer.
inline int ToInt15(uint16_t w)
	{ return (w & 040000) ? -int((~w) & 037777) : int(w & 037777); }
// Converts a signed magnitude (|v| <= 037777) to a 15-bit word.
inline uint16_t FromInt15(int v, bool negativeZero = false)
{
	if (v < 0 || (v == 0 && negativeZero))
		return uint16_t((~uint16_t(-v)) & 077777);
	return uint16_t(v & 037777);
}

}  // namespace agc

#endif
