// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "DSKY.h"

#include "AGC.h"

namespace agc {

DSKY::DSKY()
{
}


char
DSKY::_Digit(int code)
{
	switch (code & 037) {
		case 025: return '0';
		case 003: return '1';
		case 031: return '2';
		case 033: return '3';
		case 017: return '4';
		case 036: return '5';
		case 034: return '6';
		case 023: return '7';
		case 035: return '8';
		case 037: return '9';
	}
	return ' ';
}


void
DSKY::_Relay(uint16_t word)
{
	int relay = (word >> 11) & 017;
	bool b = (word >> 10) & 1;
	char c = _Digit((word >> 5) & 037);
	char d = _Digit(word & 037);
	DSKYState& s = fState;

	switch (relay) {
		case 11: s.prog[0] = c; s.prog[1] = d; break;
		case 10: s.verb[0] = c; s.verb[1] = d; break;
		case 9:  s.noun[0] = c; s.noun[1] = d; break;
		case 8:  s.reg[0][0] = d; break;
		case 7:  s.plus[0] = b; s.reg[0][1] = c; s.reg[0][2] = d; break;
		case 6:  s.minus[0] = b; s.reg[0][3] = c; s.reg[0][4] = d; break;
		case 5:  s.plus[1] = b; s.reg[1][0] = c; s.reg[1][1] = d; break;
		case 4:  s.minus[1] = b; s.reg[1][2] = c; s.reg[1][3] = d; break;
		case 3:  s.reg[1][4] = c; s.reg[2][0] = d; break;
		case 2:  s.plus[2] = b; s.reg[2][1] = c; s.reg[2][2] = d; break;
		case 1:  s.minus[2] = b; s.reg[2][3] = c; s.reg[2][4] = d; break;
		case 12:
			s.prioDisp = word & 01;
			s.noDap = word & 02;
			s.vel = word & 04;
			s.noAtt = word & 010;
			s.alt = word & 020;
			s.gimbalLock = word & 040;
			s.tracker = word & 0200;
			s.progAlarm = word & 0400;
			break;
	}
	for (int r = 0; r < 3; r++)
		s.sign[r] = s.minus[r] ? '-' : (s.plus[r] ? '+' : ' ');
}


void
DSKY::ChannelWritten(int channel, uint16_t value)
{
	if (channel == 010) {
		_Relay(value);
	} else if (channel == 011) {
		fState.compActy = value & 02;
		fState.uplinkActy = value & 04;
		fState.temp = value & 010;
		fState.keyRel = value & 020;
		fState.verbNounFlash = value & 040;
		fState.oprErr = value & 0100;
	}
}


void
DSKY::Sync(const Computer& computer)
{
	fState.restart = computer.RestartLight();
}


std::string
DSKYState::RegisterText(int r) const
{
	std::string text(1, sign[r]);
	text.append(reg[r], 5);
	return text;
}

}  // namespace agc
