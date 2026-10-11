// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Block II AGC CPU. Structure follows the hardware closely enough for the
// flight software: B holds the prefetched next instruction and Z points
// just past it, which is what interrupts, RESUME and INDEX depend on.

#include "AGC.h"

#include <cstdio>
#include <cstring>
#include <vector>

namespace agc {

namespace {

const uint16_t kNewJob = 067;

// Interrupt vectors, in priority order (index == Interrupt).
const uint16_t kVector[RUPT_COUNT] = {
	04004, 04010, 04014, 04020, 04024, 04030, 04034, 04040, 04044, 04050
};

inline bool IsPlusZero(uint16_t w16) { return w16 == 0; }
inline bool IsMinusZero(uint16_t w16) { return w16 == 0177777; }

// Magnitude of a 16-bit ones-complement value (overflow bit included).
inline uint32_t Magnitude16(uint16_t w16)
{
	return (w16 & 0100000) ? uint32_t((~w16) & 0177777) : w16;
}

}  // namespace


Computer::Computer()
	:
	onChannelWrite(nullptr)
{
	std::memset(fFixed, 0, sizeof(fFixed));
	PowerOn();
}


bool
Computer::LoadRope(const uint8_t* data, size_t size)
{
	if (size != size_t(kFixedBanks) * 1024 * 2)
		return false;
	for (int block = 0; block < kFixedBanks; block++) {
		int bank = block;
		if (block < 4)
			bank = (block + 2) % 4;     // file order is 2, 3, 0, 1, 4...
		for (int i = 0; i < 1024; i++) {
			size_t at = (size_t(block) * 1024 + i) * 2;
			uint16_t word = uint16_t(data[at] << 8 | data[at + 1]);
			fFixed[bank][i] = (word >> 1) & 077777;
		}
	}
	return true;
}


bool
Computer::LoadRopeFile(const std::string& path)
{
	FILE* file = std::fopen(path.c_str(), "rb");
	if (file == nullptr)
		return false;
	std::vector<uint8_t> data(kFixedBanks * 1024 * 2 + 1);
	size_t got = std::fread(data.data(), 1, data.size(), file);
	std::fclose(file);
	return LoadRope(data.data(), got);
}


void
Computer::PowerOn()
{
	std::memset(fErasable, 0, sizeof(fErasable));
	std::memset(fChannels, 0, sizeof(fChannels));
	// Discrete inputs are active-low; idle means all ones.
	for (int ch = 030; ch <= 033; ch++)
		fChannels[ch] = 077777;
	fCycles = 0;
	fTimerAccum = 0;
	fT6Accum = 0;
	fT4Phase = false;
	fTelemetry = true;
	fDownAccum = 0;
	fBudget = 0;
	fAlarmsEnabled = true;
	fRestarts = 0;
	Restart();
	fRestartLight = false;
}


void
Computer::Restart()
{
	// GOJAM: inhibit interrupts, clear output channels, go to 04000.
	for (int ch : {5, 6, 010, 011, 012, 013, 014, 034, 035})
		fChannels[ch] = 0;
	fChannels[7] = 0;
	fEB = fFB = 0;
	fIndex = 0;
	fIndexPending = false;
	fExtend = false;
	fExtendPending = false;
	fInhibit = true;
	fInISR = false;
	for (bool& r : fPendingRupt)
		r = false;
	fNightWatchman = 0;
	fRuptLockTime = 0;
	fNoRuptTime = 0;
	fTCTrapTime = 0;
	fSinceTC = 0;
	fNewJobTouched = false;
	fIndexApplied = false;
	fRestartLight = true;
	fRestarts++;
	fZ = 04000;
	_Fetch();
	if (onChannelWrite) {
		onChannelWrite(011, 0);
		onChannelWrite(013, 0);
	}
}


// ---------------------------------------------------------------------------
// Memory


uint16_t*
Computer::_Cell(uint16_t address)
{
	address &= 07777;
	if (address < 01400)
		return &fErasable[address >> 8][address & 0377];
	if (address < 02000)
		return &fErasable[fEB][address & 0377];
	if (address < 04000) {
		int bank = fFB;
		if (bank >= 030 && (fChannels[7] & 0100))
			bank += 010;
		if (bank >= kFixedBanks)
			return nullptr;
		return &fFixed[bank][address & 01777];
	}
	return &fFixed[(address < 06000) ? 2 : 3][address & 01777];
}


uint16_t
Computer::_Read(uint16_t address)
{
	address &= 07777;
	if (address == kNewJob)
		fNewJobTouched = true;
	switch (address) {
		case REG_EB:   return uint16_t(fEB << 8);
		case REG_FB:   return uint16_t(fFB << 10);
		case REG_Z:    return fZ;
		case REG_BB:   return uint16_t(fFB << 10 | fEB);
		case REG_ZERO: return 0;
	}
	uint16_t* cell = _Cell(address);
	return cell ? *cell : 0;
}


uint16_t
Computer::_Load(uint16_t address)
{
	uint16_t raw = _Read(address);
	return _Is16Bit(address & 07777) ? raw : SignExtend(raw);
}


uint16_t
Computer::_LoadOperand(uint16_t address)
{
	uint16_t value = _Load(address);
	// Erasable reads are destructive and rewritten through the editing
	// logic, so reading an editing register edits it again.
	address &= 07777;
	if (address >= REG_CYR && address <= REG_EDOP)
		_StoreEdited(address, _Read(address));
	return value;
}


void
Computer::_StoreEdited(uint16_t address, uint16_t v)
{
	v &= 077777;
	uint16_t edited = v;
	switch (address) {
		case REG_CYR: edited = uint16_t((v >> 1) | ((v & 1) << 14)); break;
		case REG_SR:  edited = uint16_t((v >> 1) | (v & 040000)); break;
		case REG_CYL: edited = uint16_t(((v << 1) & 077777) | (v >> 14)); break;
		case REG_EDOP: edited = uint16_t((v >> 7) & 0177); break;
	}
	fErasable[0][address] = edited;
}


void
Computer::_Store(uint16_t address, uint16_t value16)
{
	address &= 07777;
	if (address == kNewJob)
		fNewJobTouched = true;
	if (_Is16Bit(address)) {
		fErasable[0][address] = value16;
		return;
	}
	uint16_t v = OverflowCorrect(value16);
	switch (address) {
		case REG_EB:
			fEB = (v >> 8) & 7;
			return;
		case REG_FB:
			fFB = (v >> 10) & 037;
			return;
		case REG_BB:
			fFB = (v >> 10) & 037;
			fEB = v & 7;
			return;
		case REG_Z:
			fZ = v & 07777;
			return;
		case REG_ZERO:
			return;
		case REG_CYR: case REG_SR: case REG_CYL: case REG_EDOP:
			_StoreEdited(address, v);
			return;
	}
	if (address >= 02000)
		return;  // fixed memory is read-only
	*_Cell(address) = v;
}


// ---------------------------------------------------------------------------
// Channels


uint16_t
Computer::Channel(int ch) const
{
	ch &= 0777;
	if (ch == 1)
		return fErasable[0][REG_L];
	if (ch == 2)
		return fErasable[0][REG_Q];
	return ch < 0200 ? fChannels[ch] : 0;
}


uint16_t
Computer::_ReadChannel(int ch)
{
	ch &= 0777;
	if (ch == 1)
		return SignExtend(fErasable[0][REG_L]);
	if (ch == 2)
		return fErasable[0][REG_Q];
	if (ch == 7)
		return fChannels[7] & 0160;
	if (ch == 3 || ch == 4) {
		// Master scaler: 1/3200 s per count, 28 bits over two channels.
		uint64_t counts = uint64_t(fCycles * kMCTSeconds * 3200.0);
		return ch == 4 ? uint16_t(counts & 037777)
			: uint16_t((counts >> 14) & 037777);
	}
	return ch < 0200 ? SignExtend(fChannels[ch]) : 0;
}


void
Computer::_WriteChannel(int ch, uint16_t value16)
{
	ch &= 0777;
	if (ch == 1) {
		fErasable[0][REG_L] = OverflowCorrect(value16);
		return;
	}
	if (ch == 2) {
		fErasable[0][REG_Q] = value16;
		return;
	}
	if (ch >= 0200)
		return;
	uint16_t v = OverflowCorrect(value16);
	if (ch == 7)
		v &= 0160;
	// Input channels are driven by spacecraft hardware; the program cannot
	// change them. Writing channel 033 resets its warning latches (11-15).
	if ((ch >= 030 && ch <= 033) || ch == 015 || ch == 016) {
		if (ch == 033)
			fChannels[033] |= 076000;
		return;
	}
	if (ch == 011 && (v & 01000))
		fRestartLight = false;  // bit 10: caution reset
	fChannels[ch] = v;
	if (ch == 013)
		_Traps();
	if (onChannelWrite)
		onChannelWrite(ch, v);
}


void
Computer::SetInputChannel(int ch, uint16_t value)
{
	if (ch > 2 && ch < 0200)
	{
		fChannels[ch] = value & 077777;
		_Traps();
	}
}


void
Computer::SetInputBit(int ch, int bit, bool on)
{
	if (ch <= 2 || ch >= 0200 || bit < 1 || bit > 15)
		return;
	uint16_t mask = uint16_t(1 << (bit - 1));
	if (on)
		fChannels[ch] |= mask;
	else
		fChannels[ch] &= uint16_t(~mask);
	_Traps();
}


// Hand-controller interrupt (RUPT 10): the traps armed by channel 13
// bits 12-14 fire when their input bits go active (low), and disarm.
void
Computer::_Traps()
{
	uint16_t& ch13 = fChannels[013];
	if ((ch13 & 04000) && (fChannels[031] & 077) != 077) {
		ch13 &= uint16_t(~04000);
		fPendingRupt[RUPT_HAND] = true;
	}
	if ((ch13 & 010000) && (fChannels[031] & 07700) != 07700) {
		ch13 &= uint16_t(~010000);
		fPendingRupt[RUPT_HAND] = true;
	}
	if ((ch13 & 020000) && (fChannels[032] & 01777) != 01777) {
		ch13 &= uint16_t(~020000);
		fPendingRupt[RUPT_HAND] = true;
	}
}


void
Computer::PressKey(int keycode)
{
	fChannels[015] = uint16_t(keycode & 037);
	fPendingRupt[RUPT_KEY1] = true;
}


void
Computer::SetProceed(bool pressed)
{
	// Channel 032 bit 14 is active-low.
	SetInputBit(032, 14, !pressed);
}


void
Computer::RequestInterrupt(Interrupt which)
{
	fPendingRupt[which] = true;
}


// ---------------------------------------------------------------------------
// Counters


void
Computer::_CounterIncrement(uint16_t cell, int delta, bool* overflow)
{
	uint16_t value = SignExtend(fErasable[0][cell]);
	uint16_t sum = Add16(value, delta > 0 ? 1 : 0177776);
	*overflow = HasOverflow(sum);
	fErasable[0][cell] = OverflowCorrect(sum);
}


bool
Computer::AddCounts(uint16_t cell, int counts)
{
	bool any = false;
	while (counts != 0) {
		bool overflow;
		_CounterIncrement(cell, counts > 0 ? 1 : -1, &overflow);
		StealCycles(1);     // every increment is a memory cycle
		any = any || overflow;
		counts += counts > 0 ? -1 : 1;
	}
	return any;
}


void
Computer::StealCycles(int mct)
{
	// From outside Step(): the program loses this time.
	_StealCycles(mct);
	fBudget -= mct * kMCTSeconds;
}


void
Computer::RadarData(uint16_t value)
{
	fErasable[0][REG_RNRAD] = value & 077777;
	fPendingRupt[RUPT_RADAR] = true;
}


void
Computer::Uplink(int keycode)
{
	// Each uplinked word carries the 5-bit keycode three times: as is,
	// complemented, and as is (bits 15-11, 10-6, 5-1).
	uint16_t c = uint16_t(keycode & 037);
	fErasable[0][REG_INLINK] = uint16_t((c << 10) | ((~c & 037) << 5) | c);
	fPendingRupt[RUPT_UP] = true;
}


void
Computer::NavKey(uint16_t bits)
{
	fChannels[016] = bits & 077777;
	fPendingRupt[RUPT_KEY2] = true;
}


void
Computer::CountUp(uint16_t cell)
{
	bool overflow;
	_CounterIncrement(cell, 1, &overflow);
	_Tick(1);
}


void
Computer::CountDown(uint16_t cell)
{
	bool overflow;
	_CounterIncrement(cell, -1, &overflow);
	_Tick(1);
}


void
Computer::_Counters()
{
	// The 100 Hz scaler drives TIME1/TIME3/TIME5 together and TIME4 on the
	// opposite 5 ms phase. TIME6 runs at 1600 Hz when enabled.
	while (fTimerAccum >= 0.005) {
		fTimerAccum -= 0.005;
		fT4Phase = !fT4Phase;
		bool overflow;
		if (fT4Phase) {
			_CounterIncrement(REG_TIME1, 1, &overflow);
			if (overflow) {
				bool o2;
				_CounterIncrement(REG_TIME2, 1, &o2);
			}
			_CounterIncrement(REG_TIME3, 1, &overflow);
			if (overflow)
				fPendingRupt[RUPT_T3] = true;
			_CounterIncrement(REG_TIME5, 1, &overflow);
			if (overflow)
				fPendingRupt[RUPT_T5] = true;
			_StealCycles(3);
		} else {
			_CounterIncrement(REG_TIME4, 1, &overflow);
			if (overflow)
				fPendingRupt[RUPT_T4] = true;
			_StealCycles(1);
		}
	}
	while (fDownAccum >= 0.020) {
		fDownAccum -= 0.020;
		if (fTelemetry)
			fPendingRupt[RUPT_DOWN] = true;
	}
	while (fT6Accum >= 1.0 / 1600.0) {
		fT6Accum -= 1.0 / 1600.0;
		if (!(fChannels[013] & 040000))
			continue;
		uint16_t v = fErasable[0][REG_TIME6];
		if (v == 0 || v == 077777) {
			fPendingRupt[RUPT_T6] = true;
			fChannels[013] &= uint16_t(~040000);
		} else if (v & 040000) {
			fErasable[0][REG_TIME6] = Add15(v, 1);
		} else {
			fErasable[0][REG_TIME6] = Add15(v, 077776);
		}
		_StealCycles(1);
	}
}


// ---------------------------------------------------------------------------
// Execution


// Counter increments steal memory cycles from the program; they take real
// time, so the scalers advance through them like any other cycle.
void
Computer::_StealCycles(int mct)
{
	fCycles += mct;
	double dt = mct * kMCTSeconds;
	fTimerAccum += dt;
	fT6Accum += dt;
	fDownAccum += dt;
}


void
Computer::_Tick(int mct)
{
	fCycles += mct;
	double dt = mct * kMCTSeconds;
	fTimerAccum += dt;
	fT6Accum += dt;
	fDownAccum += dt;

	if (!fAlarmsEnabled)
		return;
	if (fNewJobTouched) {
		fNightWatchman = 0;
		fNewJobTouched = false;
	} else {
		fNightWatchman += dt;
	}
	if (fInISR)
		fRuptLockTime += dt;
	else
		fRuptLockTime = 0;
	fNoRuptTime += dt;
}


void
Computer::_Alarm()
{
	Restart();
}


void
Computer::_Fetch()
{
	fB = _Read(fZ) & 077777;
	fZ = (fZ + 1) & 07777;
	if (fIndexPending) {
		fB = Add15(fB, OverflowCorrect(fIndex));
		fIndexPending = false;
		fIndex = 0;
		fIndexApplied = true;
	} else {
		fIndexApplied = false;
	}
	fExtend = fExtendPending;
	fExtendPending = false;
}


bool
Computer::_InterruptsAllowed() const
{
	if (fInhibit || fInISR || fExtend || fIndexApplied)
		return false;
	if (HasOverflow(fErasable[0][REG_A]))
		return false;
	// Not just before INHINT, RELINT or EXTEND.
	if (fB == 3 || fB == 4 || fB == 6)
		return false;
	return true;
}


void
Computer::_TakeInterrupt(int which)
{
	fPendingRupt[which] = false;
	fRuptCount[which]++;
	fErasable[0][REG_ZRUPT] = fZ;
	fErasable[0][REG_BRUPT] = fB;
	fInISR = true;
	fNoRuptTime = 0;
	fZ = kVector[which];
	_Fetch();
	_Tick(2);
}


int
Computer::Step()
{
	uint64_t before = fCycles;

	_Counters();

	if (fAlarmsEnabled) {
		if (fNightWatchman > 1.92 || fRuptLockTime > 0.14
			|| fNoRuptTime > 0.14 || fTCTrapTime > 0.015) {
			_Alarm();
			return int(fCycles - before);
		}
	}

	if (_InterruptsAllowed()) {
		for (int i = 0; i < RUPT_COUNT; i++) {
			if (fPendingRupt[i]) {
				_TakeInterrupt(i);
				return int(fCycles - before);
			}
		}
	}

	uint16_t instruction = fB;
	bool extended = fExtend;
	fExtend = false;
	_Execute(instruction, extended);
	return int(fCycles - before);
}


void
Computer::Run(double seconds)
{
	fBudget += seconds;
	while (fBudget > 0) {
		int mct = Step();
		if (mct <= 0)
			mct = 1;
		fBudget -= mct * kMCTSeconds;
	}
}


void
Computer::AdvanceClock(double seconds)
{
	if (seconds <= 0)
		return;
	uint32_t t2 = fErasable[0][REG_TIME2] & 037777;
	uint32_t t1 = fErasable[0][REG_TIME1] & 037777;
	uint64_t total = uint64_t(t2) * 040000 + t1
		+ uint64_t(seconds * 100.0 + 0.5);
	fErasable[0][REG_TIME1] = uint16_t(total & 037777);
	fErasable[0][REG_TIME2] = uint16_t((total >> 14) & 037777);
}


uint16_t
Computer::ReadAddress(uint16_t address)
{
	return _Read(address);
}


uint16_t
Computer::Erasable(int bank, int offset) const
{
	return fErasable[bank & 7][offset & 0377];
}


uint16_t
Computer::Fixed(int bank, int offset) const
{
	return fFixed[bank % kFixedBanks][offset & 01777];
}


void
Computer::_Execute(uint16_t ins, bool extended)
{
	uint16_t& A = fErasable[0][REG_A];
	uint16_t& L = fErasable[0][REG_L];
	uint16_t& Q = fErasable[0][REG_Q];

	const int op = (ins >> 12) & 7;
	const int qc = (ins >> 10) & 3;
	const uint16_t k12 = ins & 07777;
	const uint16_t k10 = ins & 01777;
	int mct = 2;
	bool isTC = false;
	bool skipFetch = false;

	if (!extended) {
		switch (op) {
		case 0:
			isTC = true;
			mct = 1;
			if (k12 == 3) {             // RELINT
				fInhibit = false;
			} else if (k12 == 4) {      // INHINT
				fInhibit = true;
			} else if (k12 == 6) {      // EXTEND
				fExtendPending = true;
			} else {                     // TC
				uint16_t ret = fZ;
				fZ = k12;
				_Fetch();
				Q = ret;
				skipFetch = true;
			}
			break;
		case 1:
			if (qc == 0) {              // CCS
				uint16_t v = _LoadOperand(k10);
				uint32_t mag = Magnitude16(v);
				A = uint16_t(mag >= 1 ? mag - 1 : 0);
				if (v & 0100000) {
					fZ += IsMinusZero(v) ? 3 : 2;
				} else if (IsPlusZero(v)) {
					fZ += 1;
				}
				fZ &= 07777;
			} else {                     // TCF
				isTC = true;
				mct = 1;
				fZ = k12;
			}
			break;
		case 2:
			switch (qc) {
			case 0:                      // DAS
				_DoubleAdd(k10);
				mct = 3;
				break;
			case 1: {                    // LXCH
				uint16_t v = _Load(k10);
				_Store(k10, SignExtend(L));
				L = OverflowCorrect(v);
				break;
			}
			case 2: {                    // INCR
				uint16_t v = _Load(k10);
				uint16_t sum = Add16(v, 1);
				_Store(k10, sum);
				if (!_Is16Bit(k10) && HasOverflow(sum))
					_CounterOverflow(k10);
				break;
			}
			case 3: {                    // ADS
				uint16_t sum = Add16(A, _Load(k10));
				A = sum;
				_Store(k10, sum);
				break;
			}
			}
			break;
		case 3:                          // CA
			A = _LoadOperand(k12);
			break;
		case 4:                          // CS
			A = uint16_t(~_LoadOperand(k12));
			break;
		case 5:
			switch (qc) {
			case 0:
				if (k10 == REG_BRUPT) {  // RESUME
					fZ = fErasable[0][REG_ZRUPT] & 07777;
					fB = fErasable[0][REG_BRUPT] & 077777;
					fInISR = false;
					fIndexApplied = false;
					skipFetch = true;
				} else {                 // INDEX
					fIndex = _LoadOperand(k10);
					fIndexPending = true;
				}
				break;
			case 1: {                    // DXCH: low word first, then high
				uint16_t lo = k10;
				uint16_t hi = (k10 - 1) & 07777;
				uint16_t v = _Load(lo);
				_Store(lo, SignExtend(L));
				L = OverflowCorrect(v);
				v = _Load(hi);
				_Store(hi, A);
				A = v;
				mct = 3;
				break;
			}
			case 2:                      // TS
				if (k10 == REG_A) {
					// OVSK: skip on overflow; A is left untouched.
					if (HasOverflow(A))
						fZ = (fZ + 1) & 07777;
				} else {
					uint16_t a = A;
					_Store(k10, a);
					if (HasOverflow(a)) {
						A = (a & 0100000) ? 0177776 : 1;
						fZ = (fZ + 1) & 07777;
					}
				}
				break;
			case 3: {                    // XCH
				uint16_t v = _Load(k10);
				_Store(k10, A);
				A = v;
				break;
			}
			}
			break;
		case 6:                          // AD
			A = Add16(A, _LoadOperand(k12));
			break;
		case 7:                          // MASK
			A = A & _Load(k12);  // MASK does not re-edit
			break;
		}
	} else {
		switch (op) {
		case 0: {                        // I/O
			int pc = (ins >> 9) & 7;
			int ch = ins & 0777;
			switch (pc) {
			case 0: A = _ReadChannel(ch); break;                   // READ
			case 1: _WriteChannel(ch, A); break;                   // WRITE
			case 2: A = A & _ReadChannel(ch); break;               // RAND
			case 3: A = A & _ReadChannel(ch);                      // WAND
				_WriteChannel(ch, A); break;
			case 4: A = A | _ReadChannel(ch); break;               // ROR
			case 5: A = A | _ReadChannel(ch);                      // WOR
				_WriteChannel(ch, A); break;
			case 6: A = A ^ _ReadChannel(ch); break;               // RXOR
			case 7:                                                // EDRUPT
				fErasable[0][REG_ZRUPT] = fZ;
				fErasable[0][REG_BRUPT] = 0;
				fInISR = true;
				fZ = 0;
				mct = 3;
				break;
			}
			break;
		}
		case 1:
			if (qc == 0) {              // DV
				_Divide(k10);
				mct = 6;
			} else {                     // BZF
				isTC = true;
				if (A == 0 || A == 0177777) {
					fZ = k12;
					mct = 1;
				}
			}
			break;
		case 2:
			switch (qc) {
			case 0: {                    // MSU
				uint16_t a = OverflowCorrect(A);
				uint16_t k = _LoadOperand(k10) & 077777;
				if (_Is16Bit(k10))
					k = OverflowCorrect(_Load(k10));
				uint16_t d = uint16_t((a - k) & 077777);
				if (d & 040000)
					d = uint16_t((d - 1) & 077777);
				A = SignExtend(d);
				break;
			}
			case 1: {                    // QXCH
				uint16_t v = _Load(k10);
				_Store(k10, Q);
				Q = v;
				break;
			}
			case 2: {                    // AUG
				uint16_t v = _Load(k10);
				uint16_t sum = Add16(v, (v & 0100000) ? 0177776 : 1);
				_Store(k10, sum);
				if (!_Is16Bit(k10) && HasOverflow(sum))
					_CounterOverflow(k10);
				break;
			}
			case 3: {                    // DIM
				uint16_t v = _Load(k10);
				if (!(IsPlusZero(v) || IsMinusZero(v)))
					v = Add16(v, (v & 0100000) ? 1 : 0177776);
				_Store(k10, v);
				break;
			}
			}
			break;
		case 3: {                        // DCA
			uint16_t lo = k12;
			uint16_t hi = (k12 - 1) & 07777;
			L = OverflowCorrect(_LoadOperand(lo));
			A = _LoadOperand(hi);
			mct = 3;
			break;
		}
		case 4: {                        // DCS
			uint16_t lo = k12;
			uint16_t hi = (k12 - 1) & 07777;
			L = OverflowCorrect(uint16_t(~_LoadOperand(lo)));
			A = uint16_t(~_LoadOperand(hi));
			mct = 3;
			break;
		}
		case 5:                          // INDEX (extracode)
			fIndex = _LoadOperand(k12);
			fIndexPending = true;
			fExtendPending = true;
			break;
		case 6:
			if (qc == 0) {              // SU
				A = Add16(A, uint16_t(~_LoadOperand(k10)));
			} else {                     // BZMF
				isTC = true;
				if (A == 0 || (A & 0100000)) {
					fZ = k12;
					mct = 1;
				}
			}
			break;
		case 7:                          // MP
			_Multiply(k12);
			mct = 3;
			break;
		}
	}

	if (fAlarmsEnabled) {
		// TC trap: too long executing only TC/TCF, or none at all.
		if (isTC) {
			fTCTrapTime += mct * kMCTSeconds;
			fSinceTC = 0;
		} else {
			fTCTrapTime = 0;
			fSinceTC += mct * kMCTSeconds;
			if (fSinceTC > 0.015) {
				fSinceTC = 0;
				fTCTrapTime = 1.0;  // trips on the next Step()
			}
		}
	}

	_Tick(mct);
	if (!skipFetch)
		_Fetch();
}


void
Computer::_CounterOverflow(uint16_t address)
{
	switch (address & 07777) {
		case REG_TIME1: {
			bool o;
			_CounterIncrement(REG_TIME2, 1, &o);
			break;
		}
		case REG_TIME3: fPendingRupt[RUPT_T3] = true; break;
		case REG_TIME4: fPendingRupt[RUPT_T4] = true; break;
		case REG_TIME5: fPendingRupt[RUPT_T5] = true; break;
	}
}


void
Computer::_DoubleAdd(uint16_t k)
{
	uint16_t& A = fErasable[0][REG_A];
	uint16_t& L = fErasable[0][REG_L];
	uint16_t lo = k & 07777;
	uint16_t hi = (k - 1) & 07777;

	uint16_t low = Add16(SignExtend(L), _Load(lo));
	int carry = 0;
	if (HasOverflow(low))
		carry = (low & 0100000) ? -1 : 1;
	uint16_t high = Add16(A, _Load(hi));
	if (carry != 0)
		high = Add16(high, carry > 0 ? 1 : 0177776);

	if (hi == REG_A) {                 // DDOUBL
		L = OverflowCorrect(low);
		A = high;
		return;
	}
	_Store(lo, low);
	_Store(hi, high);
	if (HasOverflow(high))
		A = (high & 0100000) ? 0177776 : 1;
	else
		A = 0;
	L = 0;
}


void
Computer::_Multiply(uint16_t k)
{
	uint16_t& A = fErasable[0][REG_A];
	uint16_t& L = fErasable[0][REG_L];
	uint16_t a = OverflowCorrect(A);
	uint16_t b = OverflowCorrect(_Load(k));
	bool negA = a & 040000, negB = b & 040000;
	uint32_t ma = negA ? (~a & 037777) : a;
	uint32_t mb = negB ? (~b & 037777) : b;
	uint32_t p = ma * mb;
	bool neg = negA != negB;
	// A zero factor yields a zero whose sign is +0, unless the other
	// factor is also signed opposite; the hardware gives +0 when either
	// operand is +0 or -0 against a positive number.
	if (p == 0) {
		bool aZero = ma == 0, bZero = mb == 0;
		if (aZero && bZero)
			neg = negA != negB;
		else if (aZero)
			neg = negA != negB;
		else
			neg = negA != negB;
	}
	uint16_t hiMag = uint16_t(p >> 14), loMag = uint16_t(p & 037777);
	uint16_t hiW = neg ? uint16_t(~hiMag & 077777) : hiMag;
	uint16_t loW = neg ? uint16_t(~loMag & 077777) : loMag;
	A = SignExtend(hiW);
	L = loW;
}


void
Computer::_Divide(uint16_t k)
{
	uint16_t& A = fErasable[0][REG_A];
	uint16_t& L = fErasable[0][REG_L];
	uint16_t aw = OverflowCorrect(A);
	uint16_t lw = L & 077777;
	uint16_t dw = OverflowCorrect(_Load(k));

	int ia = ToInt15(aw), il = ToInt15(lw), id = ToInt15(dw);
	// The dividend's sign is A's, unless A is a zero; then it is L's.
	bool dividendNeg = (aw == 0 || aw == 077777)
		? (lw & 040000) != 0 : (aw & 040000) != 0;
	bool divisorNeg = (dw & 040000) != 0;

	int64_t dividend = int64_t(ia) * 16384 + il;
	uint64_t num = uint64_t(dividend < 0 ? -dividend : dividend);
	uint64_t den = uint64_t(id < 0 ? -id : id);

	uint16_t qMag, rMag;
	if (den == 0 || num >= den * 16384) {
		// Out of range: the hardware saturates the quotient.
		qMag = 037777;
		rMag = uint16_t(num & 037777);
		if (den != 0 && num == den * 16384) {
			qMag = 037777;
			rMag = uint16_t(den);
		}
	} else {
		qMag = uint16_t(num / den);
		rMag = uint16_t(num % den);
	}
	bool qNeg = dividendNeg != divisorNeg;
	A = SignExtend(qNeg ? uint16_t(~qMag & 077777) : qMag);
	L = dividendNeg ? uint16_t(~rMag & 077777) : rMag;
}

}  // namespace agc
