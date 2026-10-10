// Columbia — runs an AGC rope headless and reports what it puts on the DSKY.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Usage: agc_selftest <rope.bin> [seconds]
// With the Virtual AGC "Validation" rope, success is PROG 77.

#include "../core/agc/AGC.h"
#include "../core/agc/DSKY.h"

#include <cstdio>
#include <cstdlib>
#include <string>

int
main(int argc, char** argv)
{
	if (argc < 2) {
		std::fprintf(stderr, "usage: %s rope.bin [seconds]\n", argv[0]);
		return 2;
	}
	double limit = argc > 2 ? std::atof(argv[2]) : 60.0;

	agc::Computer computer;
	agc::DSKY dsky;
	if (!computer.LoadRopeFile(argv[1])) {
		std::fprintf(stderr, "cannot load %s\n", argv[1]);
		return 2;
	}
	std::string lastProg = "  ";
	int errors = 0;
	bool passed = false;
	computer.onChannelWrite = [&](int ch, uint16_t v) {
		dsky.ChannelWritten(ch, v);
	};
	computer.PowerOn();

	double t = 0;
	const double slice = 0.01;
	double proRelease = -1;
	while (t < limit) {
		computer.Run(slice);
		t += slice;
		dsky.Sync(computer);
		const agc::DSKYState& s = dsky.State();
		std::string prog(s.prog, 2);
		if (prog != lastProg) {
			lastProg = prog;
			if (prog == "77") {
				std::printf("%8.2fs PROG 77 — self-test PASSED\n", t);
				passed = true;
				break;
			}
		}
		if (s.oprErr && proRelease < 0) {
			if (prog != "00") {
				std::printf("%8.2fs FAIL test %s sub %c%c\n", t,
					prog.c_str(), s.noun[0], s.noun[1]);
				errors++;
			}
			computer.SetProceed(true);
			proRelease = t + 0.3;
		}
		if (proRelease >= 0 && t >= proRelease) {
			computer.SetProceed(false);
			proRelease = -1;
		}
	}
	std::printf("restarts: %u  errors: %d  AGC time %.2fs\n",
		computer.RestartCount() - 1, errors, computer.Time());
	return passed && errors == 0 ? 0 : 1;
}
