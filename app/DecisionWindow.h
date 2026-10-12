// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Flight's call: the situation Mission Control is looking at, the options
// with what each costs, and which one was taken on the real flight. The
// clock stops until you choose; the choice goes back to the flight window
// as a message ('dcid' with "id" and "key").

#ifndef COLUMBIA_DECISION_WINDOW_H
#define COLUMBIA_DECISION_WINDOW_H

#include <Messenger.h>
#include <Window.h>

#include <string>

#include "flight/MissionDirector.h"

namespace columbia {

enum { kMsgDecided = 'dcid' };

class DecisionWindow : public BWindow {
public:
	DecisionWindow(const flight::Decision& decision, BMessenger target,
		bool showHistory);

	void MessageReceived(BMessage* message) override;
	bool QuitRequested() override;

private:
	std::string fId;
	BMessenger fTarget;
	bool fChosen = false;
};

}  // namespace columbia

#endif
