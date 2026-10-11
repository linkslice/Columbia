// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// A game controller as the hand controllers: the stick for rotation (roll
// and pitch, twist or a third axis for yaw), the hat for translation, a
// throttle axis for the LM's descent engine, buttons for PRO and +/-X.

#ifndef COLUMBIA_JOYSTICK_INPUT_H
#define COLUMBIA_JOYSTICK_INPUT_H

#include <String.h>

#include "sim/Math.h"

class BJoystick;

namespace columbia {

class Joystick {
public:
	Joystick();
	~Joystick();

	// Opens the first controller found. False if there's none.
	bool Open();
	bool IsOpen() const { return fStick != nullptr; }
	const char* Name() const { return fName.String(); }

	struct State {
		sim::Vec3 rotation;     // roll, pitch, yaw: -1..1
		int translation[3] = {0, 0, 0};
		double throttle = -1;   // 0..1, or -1 without a throttle axis
		bool proceed = false;   // button 3
	};
	bool Poll(State* state);

private:
	BJoystick* fStick = nullptr;
	BString fName;
	int fAxes = 0, fHats = 0, fButtons = 0;
};

}  // namespace columbia

#endif
