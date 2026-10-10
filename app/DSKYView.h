// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The Command Module DSKY: indicator lamps, the electroluminescent
// display and the 19-key keyboard, drawn to scale and wired to the AGC.

#ifndef COLUMBIA_DSKY_VIEW_H
#define COLUMBIA_DSKY_VIEW_H

#include <View.h>

#include "agc/DSKY.h"

namespace columbia {
class Mission;
}

class DSKYView : public BView {
public:
	explicit DSKYView(columbia::Mission* mission);

	void Draw(BRect updateRect) override;
	void MouseDown(BPoint where) override;
	void MouseUp(BPoint where) override;
	void KeyDown(const char* bytes, int32 numBytes) override;
	void KeyUp(const char* bytes, int32 numBytes) override;
	void AttachedToWindow() override;

	BSize MinSize() override;
	BSize MaxSize() override;
	BSize PreferredSize() override;

	// Called from the window's timer; redraws only when something changed.
	void Refresh();

	static const float kWidth;
	static const float kHeight;

private:
	struct KeyInfo {
		const char* top;
		const char* bottom;
		int code;           // AGC keycode, or -1 for PRO
		BRect frame;
	};

	void _LayoutKeys();
	int _KeyAt(BPoint where) const;
	void _Press(int index);
	void _Release();

	void _DrawLamps(BRect area);
	void _DrawDisplay(BRect area);
	void _DrawKeys(BRect area);
	void _DrawLamp(BRect frame, const char* line1, const char* line2,
		bool on, bool amber);
	void _DrawDigit(BRect cell, char digit, bool lit);
	void _DrawSign(BRect cell, char sign);
	void _DrawLabelPlate(BRect frame, const char* text, bool lit);

	columbia::Mission* fMission;
	agc::DSKYState fShown;
	bool fFlashPhase;
	bool fFirst;
	KeyInfo fKeys[19];
	int fPressed;
	int fKeyboardPressed;
};

#endif
