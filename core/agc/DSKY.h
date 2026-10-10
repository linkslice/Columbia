// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Decodes the AGC's DSKY output channels (010 relay words, 011 and 013
// lamps) into a display model a UI can draw.

#ifndef COLUMBIA_DSKY_H
#define COLUMBIA_DSKY_H

#include <cstdint>
#include <string>

namespace agc {

class Computer;

struct DSKYState {
	// Digits are ' ' (blank) or '0'..'9'.
	char prog[2] = {' ', ' '};
	char verb[2] = {' ', ' '};
	char noun[2] = {' ', ' '};
	char reg[3][5] = {{' ', ' ', ' ', ' ', ' '}, {' ', ' ', ' ', ' ', ' '},
		{' ', ' ', ' ', ' ', ' '}};
	char sign[3] = {' ', ' ', ' '};   // '+', '-' or ' '
	bool plus[3] = {}, minus[3] = {};

	// Lamps.
	bool compActy = false, uplinkActy = false, temp = false, keyRel = false;
	bool verbNounFlash = false, oprErr = false, noAtt = false;
	bool gimbalLock = false, progAlarm = false, tracker = false;
	bool restart = false, standby = false, alt = false, vel = false;
	bool prioDisp = false, noDap = false;

	std::string RegisterText(int r) const;   // e.g. "+12345"
};

class DSKY {
public:
	DSKY();
	// Feed every output-channel write from the computer.
	void ChannelWritten(int channel, uint16_t value);
	// Refresh lamps that the computer holds outside channel writes.
	void Sync(const Computer& computer);
	const DSKYState& State() const { return fState; }

private:
	static char _Digit(int code);
	void _Relay(uint16_t word);

	DSKYState fState;
};

}  // namespace agc

#endif
