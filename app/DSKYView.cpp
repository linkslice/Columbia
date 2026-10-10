// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "DSKYView.h"

#include <InterfaceDefs.h>
#include <OS.h>
#include <Window.h>

#include <cstring>

#include "Mission.h"
#include "agc/AGC.h"

const float DSKYView::kWidth = 380;
const float DSKYView::kHeight = 444;

namespace {

const rgb_color kBody = {124, 128, 130, 255};
const rgb_color kBodyDark = {92, 95, 97, 255};
const rgb_color kPanel = {30, 32, 33, 255};
const rgb_color kGlass = {12, 16, 13, 255};
const rgb_color kSegmentOn = {120, 255, 110, 255};
const rgb_color kSegmentGlow = {60, 150, 55, 255};
const rgb_color kSegmentOff = {22, 40, 24, 255};
const rgb_color kLampOff = {58, 58, 54, 255};
const rgb_color kLampOffText = {112, 112, 104, 255};
const rgb_color kLampWhite = {244, 242, 232, 255};
const rgb_color kLampAmber = {255, 180, 40, 255};
const rgb_color kLampText = {20, 20, 18, 255};
const rgb_color kKeyCap = {44, 45, 46, 255};
const rgb_color kKeyCapDown = {86, 88, 90, 255};
const rgb_color kKeyEdge = {14, 14, 14, 255};
const rgb_color kKeyText = {238, 238, 232, 255};
const rgb_color kPlateOn = {120, 255, 110, 255};

struct LampSpec {
	const char* line1;
	const char* line2;
	bool amber;
};

// Command Module DSKY lamp layout (left column, right column).
const LampSpec kLeftLamps[7] = {
	{"UPLINK", "ACTY", false}, {"NO ATT", nullptr, false},
	{"STBY", nullptr, false}, {"KEY REL", nullptr, false},
	{"OPR ERR", nullptr, false}, {nullptr, nullptr, false},
	{nullptr, nullptr, false}
};
const LampSpec kRightLamps[7] = {
	{"TEMP", nullptr, true}, {"GIMBAL", "LOCK", true},
	{"PROG", nullptr, true}, {"RESTART", nullptr, true},
	{"TRACKER", nullptr, true}, {nullptr, nullptr, true},
	{nullptr, nullptr, true}
};

// Seven-segment patterns: bits a b c d e f g.
uint8
Segments(char digit)
{
	switch (digit) {
		case '0': return 0x7e;
		case '1': return 0x30;
		case '2': return 0x6d;
		case '3': return 0x79;
		case '4': return 0x33;
		case '5': return 0x5b;
		case '6': return 0x5f;
		case '7': return 0x70;
		case '8': return 0x7f;
		case '9': return 0x7b;
	}
	return 0;
}

bool
SameDisplay(const agc::DSKYState& a, const agc::DSKYState& b)
{
	if (std::memcmp(a.prog, b.prog, 2) || std::memcmp(a.verb, b.verb, 2)
		|| std::memcmp(a.noun, b.noun, 2) || std::memcmp(a.reg, b.reg, 15)
		|| std::memcmp(a.sign, b.sign, 3))
		return false;
	return a.compActy == b.compActy && a.uplinkActy == b.uplinkActy
		&& a.temp == b.temp && a.keyRel == b.keyRel
		&& a.verbNounFlash == b.verbNounFlash && a.oprErr == b.oprErr
		&& a.noAtt == b.noAtt && a.gimbalLock == b.gimbalLock
		&& a.progAlarm == b.progAlarm && a.tracker == b.tracker
		&& a.restart == b.restart && a.standby == b.standby;
}

}  // namespace


DSKYView::DSKYView(columbia::Mission* mission)
	:
	BView("DSKY", B_WILL_DRAW | B_NAVIGABLE | B_FRAME_EVENTS),
	fMission(mission),
	fFlashPhase(true),
	fFirst(true),
	fPressed(-1),
	fKeyboardPressed(-1)
{
	SetViewColor(B_TRANSPARENT_COLOR);
	_LayoutKeys();
}


void
DSKYView::AttachedToWindow()
{
	BView::AttachedToWindow();
	SetLowColor(ui_color(B_PANEL_BACKGROUND_COLOR));
}


BSize
DSKYView::MinSize()
{
	return BSize(kWidth, kHeight);
}


BSize
DSKYView::MaxSize()
{
	return BSize(kWidth, kHeight);
}


BSize
DSKYView::PreferredSize()
{
	return BSize(kWidth, kHeight);
}


void
DSKYView::_LayoutKeys()
{
	struct Spec { const char* top; const char* bottom; int code; };
	const Spec specs[19] = {
		{"VERB", nullptr, agc::KEY_VERB}, {"NOUN", nullptr, agc::KEY_NOUN},
		{"+", nullptr, agc::KEY_PLUS}, {"7", nullptr, agc::KEY_7},
		{"8", nullptr, agc::KEY_8}, {"9", nullptr, agc::KEY_9},
		{"CLR", nullptr, agc::KEY_CLR},
		{"-", nullptr, agc::KEY_MINUS}, {"4", nullptr, agc::KEY_4},
		{"5", nullptr, agc::KEY_5}, {"6", nullptr, agc::KEY_6},
		{"PRO", nullptr, -1},
		{"0", nullptr, agc::KEY_0}, {"1", nullptr, agc::KEY_1},
		{"2", nullptr, agc::KEY_2}, {"3", nullptr, agc::KEY_3},
		{"KEY", "REL", agc::KEY_KEYREL},
		{"ENTR", nullptr, agc::KEY_ENTR}, {"RSET", nullptr, agc::KEY_RSET}
	};
	const float left = 12, top = 290, size = 46, gap = 5;
	auto cell = [&](float col, float row) {
		float x = left + col * (size + gap);
		float y = top + row * (size + gap);
		return BRect(x, y, x + size, y + size);
	};
	BRect frames[19] = {
		cell(0, 0.5f), cell(0, 1.5f),
		cell(1, 0), cell(2, 0), cell(3, 0), cell(4, 0), cell(5, 0),
		cell(1, 1), cell(2, 1), cell(3, 1), cell(4, 1), cell(5, 1),
		cell(1, 2), cell(2, 2), cell(3, 2), cell(4, 2), cell(5, 2),
		cell(6, 0.5f), cell(6, 1.5f)
	};
	for (int i = 0; i < 19; i++) {
		fKeys[i].top = specs[i].top;
		fKeys[i].bottom = specs[i].bottom;
		fKeys[i].code = specs[i].code;
		fKeys[i].frame = frames[i];
	}
}


void
DSKYView::Refresh()
{
	const agc::DSKYState& now = fMission->Display();
	bool phase = (system_time() / 320000) % 2 == 0;
	bool flashChanged = now.verbNounFlash && phase != fFlashPhase;
	if (fFirst || flashChanged || !SameDisplay(now, fShown)) {
		fShown = now;
		fFlashPhase = phase;
		fFirst = false;
		Invalidate();
	}
}


// ---------------------------------------------------------------------------
// Drawing


void
DSKYView::Draw(BRect)
{
	BRect bounds = Bounds();
	SetHighColor(ui_color(B_PANEL_BACKGROUND_COLOR));
	FillRect(bounds);

	BRect body(2, 2, kWidth - 3, kHeight - 3);
	SetHighColor(kBodyDark);
	FillRoundRect(body.OffsetByCopy(2, 2), 8, 8);
	SetHighColor(kBody);
	FillRoundRect(body, 8, 8);
	SetHighColor(tint_color(kBody, B_LIGHTEN_1_TINT));
	StrokeRoundRect(body, 8, 8);

	SetDrawingMode(B_OP_ALPHA);
	_DrawLamps(BRect(10, 10, 182, 272));
	_DrawDisplay(BRect(192, 10, kWidth - 10, 272));
	_DrawKeys(BRect(8, 284, kWidth - 8, kHeight - 8));
	SetDrawingMode(B_OP_COPY);
}


void
DSKYView::_DrawLamps(BRect area)
{
	SetHighColor(kPanel);
	FillRoundRect(area, 4, 4);
	const float w = (area.Width() - 18) / 2, h = 30, gap = 5.5f;
	bool lit[2][7] = {
		{fShown.uplinkActy, fShown.noAtt, fShown.standby, fShown.keyRel,
			fShown.oprErr, false, false},
		{fShown.temp, fShown.gimbalLock, fShown.progAlarm, fShown.restart,
			fShown.tracker, false, false}
	};
	for (int col = 0; col < 2; col++) {
		for (int row = 0; row < 7; row++) {
			float x = area.left + 6 + col * (w + 6);
			float y = area.top + 8 + row * (h + gap);
			const LampSpec& spec = col == 0 ? kLeftLamps[row] : kRightLamps[row];
			_DrawLamp(BRect(x, y, x + w, y + h), spec.line1, spec.line2,
				lit[col][row], spec.amber);
		}
	}
}


void
DSKYView::_DrawLamp(BRect frame, const char* line1, const char* line2,
	bool on, bool amber)
{
	rgb_color fill = on ? (amber ? kLampAmber : kLampWhite) : kLampOff;
	SetHighColor(fill);
	FillRoundRect(frame, 3, 3);
	SetHighColor(tint_color(fill, B_DARKEN_2_TINT));
	StrokeRoundRect(frame, 3, 3);
	if (line1 == nullptr)
		return;

	BFont font(be_bold_font);
	font.SetSize(9.5f);
	SetFont(&font);
	SetHighColor(on ? kLampText : kLampOffText);
	font_height fh;
	font.GetHeight(&fh);
	float lineHeight = fh.ascent + fh.descent;
	int lines = line2 ? 2 : 1;
	float y = frame.top + (frame.Height() - lines * lineHeight) / 2
		+ fh.ascent;
	for (int i = 0; i < lines; i++) {
		const char* text = i == 0 ? line1 : line2;
		float x = frame.left + (frame.Width() - StringWidth(text)) / 2;
		DrawString(text, BPoint(x, y));
		y += lineHeight;
	}
}


void
DSKYView::_DrawLabelPlate(BRect frame, const char* text, bool lit)
{
	SetHighColor(lit ? kPlateOn : kSegmentOff);
	FillRect(frame);
	BFont font(be_bold_font);
	font.SetSize(9);
	SetFont(&font);
	SetHighColor(lit ? kGlass : kSegmentGlow);
	font_height fh;
	font.GetHeight(&fh);
	float x = frame.left + (frame.Width() - StringWidth(text)) / 2;
	float y = frame.top + (frame.Height() + fh.ascent - fh.descent) / 2;
	DrawString(text, BPoint(x, y));
}


void
DSKYView::_DrawDigit(BRect cell, char digit, bool lit)
{
	uint8 bits = lit ? Segments(digit) : 0;
	const float t = 3.0f;              // segment thickness
	float l = cell.left + 2, r = cell.right - 2;
	float top = cell.top + 1, bottom = cell.bottom - 1;
	float mid = (top + bottom) / 2;

	auto horizontal = [&](float y, bool on) {
		BPoint p[6] = {
			BPoint(l + t / 2, y), BPoint(l + t, y - t / 2),
			BPoint(r - t, y - t / 2), BPoint(r - t / 2, y),
			BPoint(r - t, y + t / 2), BPoint(l + t, y + t / 2)
		};
		if (on) {
			SetHighColor(kSegmentGlow.red, kSegmentGlow.green,
				kSegmentGlow.blue, 90);
			BPoint g[6];
			for (int i = 0; i < 6; i++)
				g[i] = p[i] + BPoint(0, (i < 3 ? -1 : 1) * 1.0f);
			FillPolygon(g, 6);
		}
		SetHighColor(on ? kSegmentOn : kSegmentOff);
		FillPolygon(p, 6);
	};
	auto vertical = [&](float x, float y0, float y1, bool on) {
		BPoint p[6] = {
			BPoint(x, y0 + t / 2), BPoint(x + t / 2, y0 + t),
			BPoint(x + t / 2, y1 - t), BPoint(x, y1 - t / 2),
			BPoint(x - t / 2, y1 - t), BPoint(x - t / 2, y0 + t)
		};
		SetHighColor(on ? kSegmentOn : kSegmentOff);
		FillPolygon(p, 6);
	};
	horizontal(top + t / 2, bits & 0x40);         // a
	vertical(r - t / 2, top, mid, bits & 0x20);   // b
	vertical(r - t / 2, mid, bottom, bits & 0x10);// c
	horizontal(bottom - t / 2, bits & 0x08);      // d
	vertical(l + t / 2, mid, bottom, bits & 0x04);// e
	vertical(l + t / 2, top, mid, bits & 0x02);   // f
	horizontal(mid, bits & 0x01);                 // g
}


void
DSKYView::_DrawSign(BRect cell, char sign)
{
	const float t = 3.0f;
	float cx = (cell.left + cell.right) / 2, cy = (cell.top + cell.bottom) / 2;
	float half = cell.Width() / 2 - 2;
	SetHighColor(sign == '+' || sign == '-' ? kSegmentOn : kSegmentOff);
	FillRect(BRect(cx - half, cy - t / 2, cx + half, cy + t / 2));
	SetHighColor(sign == '+' ? kSegmentOn : kSegmentOff);
	FillRect(BRect(cx - t / 2, cy - half, cx + t / 2, cy - t / 2 - 1));
	FillRect(BRect(cx - t / 2, cy + t / 2 + 1, cx + t / 2, cy + half));
}


void
DSKYView::_DrawDisplay(BRect area)
{
	SetHighColor(kPanel);
	FillRoundRect(area, 4, 4);
	BRect glass = area.InsetByCopy(5, 5);
	SetHighColor(kGlass);
	FillRect(glass);

	float x0 = glass.left, y0 = glass.top;
	const float dw = 20, dh = 32;
	auto digit = [&](float x, float y, char c, bool lit = true) {
		_DrawDigit(BRect(x, y, x + dw, y + dh), c, lit && c != ' ');
	};

	// COMP ACTY and PROG.
	BRect comp(x0 + 6, y0 + 6, x0 + 58, y0 + 50);
	SetHighColor(fShown.compActy ? kPlateOn : kSegmentOff);
	FillRect(comp);
	BFont font(be_bold_font);
	font.SetSize(9);
	SetFont(&font);
	SetHighColor(fShown.compActy ? kGlass : kSegmentGlow);
	DrawString("COMP", BPoint(comp.left + (comp.Width() - StringWidth("COMP")) / 2,
		comp.top + 19));
	DrawString("ACTY", BPoint(comp.left + (comp.Width() - StringWidth("ACTY")) / 2,
		comp.top + 33));

	float right = x0 + 92;
	_DrawLabelPlate(BRect(right, y0 + 6, right + 60, y0 + 18), "PROG", true);
	digit(right + 6, y0 + 22, fShown.prog[0]);
	digit(right + 32, y0 + 22, fShown.prog[1]);

	// VERB and NOUN (these flash together when the computer wants input).
	bool vnLit = !fShown.verbNounFlash || fFlashPhase;
	_DrawLabelPlate(BRect(x0 + 2, y0 + 62, x0 + 62, y0 + 74), "VERB", true);
	digit(x0 + 8, y0 + 78, fShown.verb[0], vnLit);
	digit(x0 + 34, y0 + 78, fShown.verb[1], vnLit);
	_DrawLabelPlate(BRect(right, y0 + 62, right + 60, y0 + 74), "NOUN", true);
	digit(right + 6, y0 + 78, fShown.noun[0], vnLit);
	digit(right + 32, y0 + 78, fShown.noun[1], vnLit);

	// Registers R1-R3, each preceded by a separator bar.
	float y = y0 + 116;
	for (int r = 0; r < 3; r++) {
		SetHighColor(kSegmentOn.red, kSegmentOn.green, kSegmentOn.blue, 180);
		FillRect(BRect(x0 + 6, y, glass.right - 6, y + 1.5f));
		y += 5;
		_DrawSign(BRect(x0 + 4, y + 4, x0 + 22, y + dh - 4), fShown.sign[r]);
		for (int i = 0; i < 5; i++)
			digit(x0 + 26 + i * 26, y, fShown.reg[r][i]);
		y += dh + 4;
	}
}


void
DSKYView::_DrawKeys(BRect area)
{
	(void)area;
	BFont font(be_bold_font);
	for (int i = 0; i < 19; i++) {
		const KeyInfo& key = fKeys[i];
		bool down = i == fPressed || i == fKeyboardPressed;
		BRect f = key.frame;
		SetHighColor(kKeyEdge);
		FillRoundRect(f.OffsetByCopy(0, 2), 5, 5);
		SetHighColor(down ? kKeyCapDown : kKeyCap);
		FillRoundRect(down ? f.OffsetByCopy(0, 1) : f, 5, 5);
		SetHighColor(tint_color(kKeyCap, B_LIGHTEN_2_TINT));
		StrokeRoundRect(down ? f.OffsetByCopy(0, 1) : f, 5, 5);

		bool big = std::strlen(key.top) == 1;
		font.SetSize(big ? 19 : 11);
		SetFont(&font);
		SetHighColor(kKeyText);
		font_height fh;
		font.GetHeight(&fh);
		float lh = fh.ascent + fh.descent;
		int lines = key.bottom ? 2 : 1;
		float ty = f.top + (f.Height() - lines * lh) / 2 + fh.ascent
			+ (down ? 1 : 0);
		for (int l = 0; l < lines; l++) {
			const char* text = l == 0 ? key.top : key.bottom;
			DrawString(text, BPoint(f.left + (f.Width() - StringWidth(text)) / 2,
				ty));
			ty += lh;
		}
	}
}


// ---------------------------------------------------------------------------
// Input


int
DSKYView::_KeyAt(BPoint where) const
{
	for (int i = 0; i < 19; i++) {
		if (fKeys[i].frame.Contains(where))
			return i;
	}
	return -1;
}


void
DSKYView::_Press(int index)
{
	if (index < 0)
		return;
	if (fKeys[index].code < 0)
		fMission->SetProceed(true);
	else
		fMission->PressKey(fKeys[index].code);
}


void
DSKYView::_Release()
{
	fMission->SetProceed(false);
}


void
DSKYView::MouseDown(BPoint where)
{
	MakeFocus(true);
	int index = _KeyAt(where);
	if (index < 0)
		return;
	fPressed = index;
	SetMouseEventMask(B_POINTER_EVENTS, B_LOCK_WINDOW_FOCUS);
	_Press(index);
	Invalidate(fKeys[index].frame.InsetByCopy(-2, -3));
}


void
DSKYView::MouseUp(BPoint where)
{
	(void)where;
	if (fPressed >= 0) {
		if (fKeys[fPressed].code < 0)
			_Release();
		Invalidate(fKeys[fPressed].frame.InsetByCopy(-2, -3));
		fPressed = -1;
	}
}


void
DSKYView::KeyDown(const char* bytes, int32 numBytes)
{
	if (numBytes != 1) {
		BView::KeyDown(bytes, numBytes);
		return;
	}
	int index = -1;
	switch (bytes[0]) {
		case 'v': case 'V': index = 0; break;
		case 'n': case 'N': index = 1; break;
		case '+': case '=': index = 2; break;
		case '7': index = 3; break;
		case '8': index = 4; break;
		case '9': index = 5; break;
		case 'c': case 'C': case B_BACKSPACE: index = 6; break;
		case '-': index = 7; break;
		case '4': index = 8; break;
		case '5': index = 9; break;
		case '6': index = 10; break;
		case 'p': case 'P': index = 11; break;
		case '0': index = 12; break;
		case '1': index = 13; break;
		case '2': index = 14; break;
		case '3': index = 15; break;
		case 'k': case 'K': index = 16; break;
		case 'e': case 'E': case B_ENTER: index = 17; break;
		case 'r': case 'R': index = 18; break;
		default:
			BView::KeyDown(bytes, numBytes);
			return;
	}
	// Ignore auto-repeat while a key is held.
	if (fKeyboardPressed == index)
		return;
	fKeyboardPressed = index;
	_Press(index);
	Invalidate(fKeys[index].frame.InsetByCopy(-2, -3));
}


void
DSKYView::KeyUp(const char* bytes, int32 numBytes)
{
	(void)bytes;
	(void)numBytes;
	if (fKeyboardPressed >= 0) {
		if (fKeys[fKeyboardPressed].code < 0)
			_Release();
		Invalidate(fKeys[fKeyboardPressed].frame.InsetByCopy(-2, -3));
		fKeyboardPressed = -1;
	}
}
