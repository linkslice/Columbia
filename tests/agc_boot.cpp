// Columbia — boots a flight rope headless and drives the DSKY from a script.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: agc_boot <rope.bin> "<keys>" [settle-seconds]
// Keys: V N E(ENTR) C(CLR) R(RSET) P(PRO) K(KEY REL) + - 0-9, ' ' waits 1 s.

#include "../core/agc/AGC.h"
#include "../core/agc/DSKY.h"

#include <cstdio>
#include <cstdlib>
#include <string>

static void
Show(const agc::DSKYState& s, double t, const agc::Computer& c)
{
	std::printf("t=%6.1fs PROG %c%c VERB %c%c NOUN %c%c | R1 %s R2 %s R3 %s"
		" |%s%s%s%s%s%s restarts=%u\n", t,
		s.prog[0], s.prog[1], s.verb[0], s.verb[1], s.noun[0], s.noun[1],
		s.RegisterText(0).c_str(), s.RegisterText(1).c_str(),
		s.RegisterText(2).c_str(),
		s.compActy ? " COMP" : "", s.oprErr ? " OPR-ERR" : "",
		s.progAlarm ? " PROG" : "", s.keyRel ? " KEY-REL" : "",
		s.verbNounFlash ? " FLASH" : "", s.restart ? " RESTART" : "",
		c.RestartCount() - 1);
}

int
main(int argc, char** argv)
{
	if (argc < 2)
		return 2;
	std::string keys = argc > 2 ? argv[2] : "";
	double settle = argc > 3 ? std::atof(argv[3]) : 3.0;

	agc::Computer computer;
	agc::DSKY dsky;
	if (!computer.LoadRopeFile(argv[1]))
		return 2;
	computer.onChannelWrite = [&](int ch, uint16_t v) {
		dsky.ChannelWritten(ch, v);
	};
	computer.PowerOn();

	double t = 0;
	auto run = [&](double s) {
		computer.Run(s);
		t += s;
		dsky.Sync(computer);
	};
	run(settle);
	Show(dsky.State(), t, computer);
	for (char k : keys) {
		int code = -1;
		switch (k) {
			case 'V': code = agc::KEY_VERB; break;
			case 'N': code = agc::KEY_NOUN; break;
			case 'E': code = agc::KEY_ENTR; break;
			case 'C': code = agc::KEY_CLR; break;
			case 'R': code = agc::KEY_RSET; break;
			case 'K': code = agc::KEY_KEYREL; break;
			case '+': code = agc::KEY_PLUS; break;
			case '-': code = agc::KEY_MINUS; break;
			case 'P':
				computer.SetProceed(true);
				run(0.3);
				computer.SetProceed(false);
				break;
			case ' ':
				run(1.0);
				Show(dsky.State(), t, computer);
				continue;
			default:
				if (k >= '0' && k <= '9')
					code = k == '0' ? agc::KEY_0 : (k - '0');
		}
		if (code >= 0)
			computer.PressKey(code);
		run(0.25);
		if (k == 'E')
			Show(dsky.State(), t, computer);
	}
	run(2.0);
	Show(dsky.State(), t, computer);
	return 0;
}
