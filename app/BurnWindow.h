// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Editor for a single burn: label, engine, ignition time and the
// delta-v in the local orbital frame.

#ifndef COLUMBIA_BURN_WINDOW_H
#define COLUMBIA_BURN_WINDOW_H

#include <Messenger.h>
#include <Window.h>

#include "sim/Flight.h"

class BMenuField;
class BStringView;
class BTextControl;

enum {
	MSG_BURN_EDITED = 'brnE'
};

class BurnWindow : public BWindow {
public:
	// `burn.id == 0` means a new burn.
	BurnWindow(BMessenger target, const sim::Burn& burn, bool sivbAvailable,
		BPoint where);

	void MessageReceived(BMessage* message) override;

	static void AddToMessage(BMessage* message, const sim::Burn& burn);
	static bool FromMessage(const BMessage* message, sim::Burn* burn);

private:
	void _UpdateTotal();
	bool _Read(sim::Burn* burn, BString* error) const;

	BMessenger fTarget;
	sim::Burn fBurn;
	BTextControl* fLabel;
	BMenuField* fEngine;
	BTextControl* fTime;
	BTextControl* fPrograde;
	BTextControl* fNormal;
	BTextControl* fRadial;
	BStringView* fTotal;
	BStringView* fError;
};

#endif
