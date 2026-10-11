// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "JoystickInput.h"

#include <Joystick.h>

#include <cmath>
#include <vector>

namespace columbia {

namespace {

// -32768..32767 to -1..1 with a small dead zone.
double
Axis(int16 v)
{
	double x = v / 32767.0;
	if (std::fabs(x) < 0.08)
		return 0;
	return (x - (x > 0 ? 0.08 : -0.08)) / 0.92;
}

}  // namespace


Joystick::Joystick()
{
}


Joystick::~Joystick()
{
	delete fStick;
}


bool
Joystick::Open()
{
	delete fStick;
	fStick = nullptr;
	BJoystick* stick = new BJoystick;
	int32 count = stick->CountDevices();
	for (int32 i = 0; i < count; i++) {
		char name[B_OS_NAME_LENGTH];
		if (stick->GetDeviceName(i, name, sizeof(name)) != B_OK)
			continue;
		if (stick->Open(name, true) < B_OK)
			continue;
		fStick = stick;
		fName = name;
		fAxes = stick->CountAxes();
		fHats = stick->CountHats();
		fButtons = stick->CountButtons();
		return true;
	}
	delete stick;
	return false;
}


bool
Joystick::Poll(State* s)
{
	if (!fStick || fStick->Update() != B_OK)
		return false;
	std::vector<int16> axes(fAxes > 0 ? fAxes : 1, 0);
	if (fAxes > 0)
		fStick->GetAxisValues(axes.data());
	*s = State();
	if (fAxes >= 2) {
		s->rotation.x = Axis(axes[0]);          // roll: stick left/right
		s->rotation.y = -Axis(axes[1]);         // pitch: stick fore/aft
	}
	if (fAxes >= 3)
		s->rotation.z = Axis(axes[2]);          // yaw: twist
	if (fAxes >= 4)
		s->throttle = (1 - axes[3] / 32767.0) / 2;
	if (fHats > 0) {
		std::vector<uint8> hats(fHats, 0);
		fStick->GetHatValues(hats.data());
		// 1 up, 2 up-right, 3 right, ... 8 up-left; 0 centred.
		switch (hats[0]) {
			case 1: s->translation[2] = -1; break;
			case 2: s->translation[2] = -1; s->translation[1] = 1; break;
			case 3: s->translation[1] = 1; break;
			case 4: s->translation[2] = 1; s->translation[1] = 1; break;
			case 5: s->translation[2] = 1; break;
			case 6: s->translation[2] = 1; s->translation[1] = -1; break;
			case 7: s->translation[1] = -1; break;
			case 8: s->translation[2] = -1; s->translation[1] = -1; break;
		}
	}
	uint32 buttons = fStick->ButtonValues();
	if (buttons & 1)
		s->translation[0] = 1;                  // trigger: +X
	if (buttons & 2)
		s->translation[0] = -1;                 // thumb: -X
	s->proceed = (buttons & 4) != 0;
	return true;
}

}  // namespace columbia
