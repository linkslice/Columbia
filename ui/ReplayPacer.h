// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Pacing for a replay: real time where it matters (liftoff, the last
// minutes of a landing, docking, the parachutes), fast through the coasts,
// to fit a mission into a set running time. Pure C++ (no Haiku).

#ifndef COLUMBIA_UI_REPLAY_PACER_H
#define COLUMBIA_UI_REPLAY_PACER_H

#include "flight/Recording.h"

#include <vector>

namespace ui {

class ReplayPacer {
public:
	// `minutes`: wanted running time. Moments that need watching keep their
	// speed even if that makes it longer.
	void Build(const flight::Recording& recording, double minutes);

	double TotalSeconds() const { return fReal.empty() ? 0 : fReal.back(); }
	// Mission time (GET) after `seconds` of replay.
	double GETAt(double seconds) const;
	// Replay speed (simulated seconds per real second) there.
	double SpeedAt(double seconds) const;
	// Replay time at which mission time `get` is shown.
	double SecondsAt(double get) const;

private:
	std::vector<double> fGET;     // cell boundaries (mission time)
	std::vector<double> fReal;    // replay time at each boundary
	std::vector<double> fSpeed;   // speed in each cell
};

}  // namespace ui

#endif
