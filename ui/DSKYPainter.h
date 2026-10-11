// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Draws a DSKY (CM or LM) into any BView: the app's working DSKY, the
// screensaver's replay, the mission report. Drawn at 380 x 444 (or the
// upper 282 without the keyboard); callers scale with SetScale/SetOrigin.

#ifndef COLUMBIA_UI_DSKY_PAINTER_H
#define COLUMBIA_UI_DSKY_PAINTER_H

#include <View.h>

#include "agc/DSKY.h"

namespace ui {

struct DSKYKey {
	const char* top;
	const char* bottom;
	int code;           // AGC keycode, or -1 for PRO
	BRect frame;
};

class DSKYPainter {
public:
	static const float kWidth;
	static const float kHeight;          // with the keyboard
	static const float kDisplayHeight;   // lamps and display only

	explicit DSKYPainter(bool lm = false);

	void SetLM(bool lm) { fLM = lm; }
	// Draw at (0, 0). `flashOn` is the verb/noun flash phase; `pressed`
	// is the index of a key drawn down (-1 for none).
	void Draw(BView* view, const agc::DSKYState& state, bool flashOn,
		bool withKeys = true, int pressed = -1, int pressed2 = -1) const;

	const DSKYKey* Keys() const { return fKeys; }
	int KeyAt(BPoint where) const;

private:
	void _DrawLamps(BView* v, const agc::DSKYState& s, BRect area) const;
	void _DrawLamp(BView* v, BRect frame, const char* line1, const char* line2,
		bool on, bool amber) const;
	void _DrawDisplay(BView* v, const agc::DSKYState& s, bool flashOn,
		BRect area) const;
	void _DrawDigit(BView* v, BRect cell, char digit, bool lit) const;
	void _DrawSign(BView* v, BRect cell, char sign) const;
	void _DrawLabelPlate(BView* v, BRect frame, const char* text,
		bool lit) const;
	void _DrawKeys(BView* v, int pressed, int pressed2) const;

	bool fLM;
	DSKYKey fKeys[19];
};

}  // namespace ui

#endif
