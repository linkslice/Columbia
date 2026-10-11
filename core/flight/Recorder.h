// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The flight recorder: attached to a FlightSim, it fills a Recording as
// the mission is flown, densely around the events and sparsely in the
// long coasts.

#ifndef COLUMBIA_RECORDER_H
#define COLUMBIA_RECORDER_H

#include "Recording.h"

namespace flight {

class FlightSim;

class FlightRecorder {
public:
	explicit FlightRecorder(FlightSim& sim);

	// Start listening (once).
	void Attach();
	// Close the recording: a last sample and the outcome.
	void Finish(const std::string& outcome, bool success);

	Recording& Data() { return fRec; }

	// Seconds between samples while something's happening, and in a
	// quiet coast.
	double busyInterval = 1.0;
	double quietInterval = 60.0;

private:
	void _Sample();
	void _Displays();
	bool _Busy() const;

	FlightSim& f;
	Recording fRec;
	bool fAttached = false;
	double fLastSample = -1e30, fLastDisplay = -1e30;
	int fLastPhase = -1;
	uint16_t fLastModules[REC_SLOTS] = {};
};

}  // namespace flight

#endif
