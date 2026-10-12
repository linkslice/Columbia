// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// A flight: the controls (the default), the window and the map as cards;
// the air-to-ground loop, the clocks, the time warp and the automatic crew
// down the side. Keyboard and joystick fly the hand controllers.

#ifndef COLUMBIA_FLIGHT_WINDOW_H
#define COLUMBIA_FLIGHT_WINDOW_H

#include <Window.h>

#include <memory>

#include "FlightSession.h"
#include "JoystickInput.h"

class BButton;
class BCardLayout;
class BCheckBox;
class BMenuItem;
class BMessageRunner;
class BSlider;
class BStringView;
class BTextView;

namespace columbia {

class DecisionWindow;
class MapView;
class PanelView;
class WindowView;

class FlightWindow : public BWindow {
public:
	FlightWindow(const FlightOptions& options);
	~FlightWindow() override;

	bool Started() const { return fStarted; }
	const std::string& StartError() const { return fError; }

	void MessageReceived(BMessage* message) override;
	void DispatchMessage(BMessage* message, BHandler* handler) override;
	bool QuitRequested() override;

private:
	void _BuildMenu(BView* parent);
	void _Pulse();
	void _ShowCard(int index);
	void _UpdateStatus();
	void _ApplyControllers();
	bool _HandleKey(BMessage* message, bool down);
	void _SetWarpFromSlider();
	void _FlightOver();
	void _SaveRecording(bool quiet);
	void _CheckDecision();

	FlightSession fSession;
	bool fStarted = false;
	std::string fError;

	BCardLayout* fCards = nullptr;
	PanelView* fPanel = nullptr;
	WindowView* fWindow = nullptr;
	MapView* fMap = nullptr;
	int fCard = 0;

	BTextView* fLoop = nullptr;
	BStringView* fClock = nullptr;
	BStringView* fPhase = nullptr;
	BStringView* fNext = nullptr;
	BStringView* fWarpLabel = nullptr;
	BStringView* fCrewLabel = nullptr;
	BSlider* fWarp = nullptr;
	BButton* fPause = nullptr;
	BCheckBox* fAutopilot = nullptr;
	BMenuItem* fPauseItem = nullptr;
	BMenuItem* fAutoItem = nullptr;
	BMenuItem* fCraftItems[3] = {};
	BMessageRunner* fRunner = nullptr;

	// Hand controllers from the keyboard (held keys) and the joystick.
	int fKeyRot[3] = {0, 0, 0};       // roll, pitch, yaw
	int fKeyTrans[3] = {0, 0, 0};
	int fThrottleSteps = 0;           // pending LM throttle clicks
	Joystick fJoystick;
	bool fProceedHeld = false;
	bool fOverShown = false;
	bool fRecordingSaved = false;
	DecisionWindow* fDecisionWindow = nullptr;
	std::string fDecisionId;
};

}  // namespace columbia

#endif
