// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "DSKYPainter.h"

#include <InterfaceDefs.h>

#include <cstring>

#include "agc/AGC.h"

namespace ui {

const float DSKYPainter::kWidth = 380;
const float DSKYPainter::kHeight = 444;
const float DSKYPainter::kDisplayHeight = 282;

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
};

// Lamp layouts (left column white, right column amber).
const LampSpec kCMLeft[7] = {
	{"UPLINK", "ACTY"}, {"NO ATT", nullptr}, {"STBY", nullptr},
	{"KEY REL", nullptr}, {"OPR ERR", nullptr}, {nullptr, nullptr},
	{nullptr, nullptr}
};
const LampSpec kCMRight[7] = {
	{"TEMP", nullptr}, {"GIMBAL", "LOCK"}, {"PROG", nullptr},
	{"RESTART", nullptr}, {"TRACKER", nullptr}, {nullptr, nullptr},
	{nullptr, nullptr}
};
const LampSpec kLMLeft[7] = {
	{"UPLINK", "ACTY"}, {"NO ATT", nullptr}, {"STBY", nullptr},
	{"KEY REL", nullptr}, {"OPR ERR", nullptr}, {"PRIO DISP", nullptr},
	{"NO DAP", nullptr}
};
const LampSpec kLMRight[7] = {
	{"TEMP", nullptr}, {"GIMBAL", "LOCK"}, {"PROG", nullptr},
	{"RESTART", nullptr}, {"TRACKER", nullptr}, {"ALT", nullptr},
	{"VEL", nullptr}
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

}  // namespace


DSKYPainter::DSKYPainter(bool lm)
	:
	fLM(lm)
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


int
DSKYPainter::KeyAt(BPoint where) const
{
	for (int i = 0; i < 19; i++)
		if (fKeys[i].frame.Contains(where))
			return i;
	return -1;
}


void
DSKYPainter::Draw(BView* v, const agc::DSKYState& s, bool flashOn,
	bool withKeys, int pressed, int pressed2) const
{
	float h = withKeys ? kHeight : kDisplayHeight;
	BRect body(2, 2, kWidth - 3, h - 3);
	v->SetDrawingMode(B_OP_COPY);
	v->SetHighColor(kBodyDark);
	v->FillRoundRect(body.OffsetByCopy(2, 2), 8, 8);
	v->SetHighColor(kBody);
	v->FillRoundRect(body, 8, 8);
	v->SetHighColor(tint_color(kBody, B_LIGHTEN_1_TINT));
	v->StrokeRoundRect(body, 8, 8);

	v->SetDrawingMode(B_OP_ALPHA);
	_DrawLamps(v, s, BRect(10, 10, 182, 272));
	_DrawDisplay(v, s, flashOn, BRect(192, 10, kWidth - 10, 272));
	if (withKeys)
		_DrawKeys(v, pressed, pressed2);
	v->SetDrawingMode(B_OP_COPY);
}


void
DSKYPainter::_DrawLamps(BView* v, const agc::DSKYState& s, BRect area) const
{
	v->SetHighColor(kPanel);
	v->FillRoundRect(area, 4, 4);
	const float w = (area.Width() - 18) / 2, h = 30, gap = 5.5f;
	bool lit[2][7] = {
		{s.uplinkActy, s.noAtt, s.standby, s.keyRel, s.oprErr,
			fLM && s.prioDisp, fLM && s.noDap},
		{s.temp, s.gimbalLock, s.progAlarm, s.restart, s.tracker,
			fLM && s.alt, fLM && s.vel}
	};
	for (int col = 0; col < 2; col++) {
		for (int row = 0; row < 7; row++) {
			float x = area.left + 6 + col * (w + 6);
			float y = area.top + 8 + row * (h + gap);
			const LampSpec& spec = col == 0
				? (fLM ? kLMLeft : kCMLeft)[row] : (fLM ? kLMRight : kCMRight)[row];
			_DrawLamp(v, BRect(x, y, x + w, y + h), spec.line1, spec.line2,
				lit[col][row], col == 1);
		}
	}
}


void
DSKYPainter::_DrawLamp(BView* v, BRect frame, const char* line1,
	const char* line2, bool on, bool amber) const
{
	rgb_color fill = on ? (amber ? kLampAmber : kLampWhite) : kLampOff;
	v->SetHighColor(fill);
	v->FillRoundRect(frame, 3, 3);
	v->SetHighColor(tint_color(fill, B_DARKEN_2_TINT));
	v->StrokeRoundRect(frame, 3, 3);
	if (line1 == nullptr)
		return;

	BFont font(be_bold_font);
	font.SetSize(9.5f);
	v->SetFont(&font);
	v->SetHighColor(on ? kLampText : kLampOffText);
	font_height fh;
	font.GetHeight(&fh);
	float lineHeight = fh.ascent + fh.descent;
	int lines = line2 ? 2 : 1;
	float y = frame.top + (frame.Height() - lines * lineHeight) / 2 + fh.ascent;
	for (int i = 0; i < lines; i++) {
		const char* text = i == 0 ? line1 : line2;
		float x = frame.left + (frame.Width() - v->StringWidth(text)) / 2;
		v->DrawString(text, BPoint(x, y));
		y += lineHeight;
	}
}


void
DSKYPainter::_DrawLabelPlate(BView* v, BRect frame, const char* text,
	bool lit) const
{
	v->SetHighColor(lit ? kPlateOn : kSegmentOff);
	v->FillRect(frame);
	BFont font(be_bold_font);
	font.SetSize(9);
	v->SetFont(&font);
	v->SetHighColor(lit ? kGlass : kSegmentGlow);
	font_height fh;
	font.GetHeight(&fh);
	float x = frame.left + (frame.Width() - v->StringWidth(text)) / 2;
	float y = frame.top + (frame.Height() + fh.ascent - fh.descent) / 2;
	v->DrawString(text, BPoint(x, y));
}


void
DSKYPainter::_DrawDigit(BView* v, BRect cell, char digit, bool lit) const
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
			v->SetHighColor(kSegmentGlow.red, kSegmentGlow.green,
				kSegmentGlow.blue, 90);
			BPoint g[6];
			for (int i = 0; i < 6; i++)
				g[i] = p[i] + BPoint(0, (i < 3 ? -1 : 1) * 1.0f);
			v->FillPolygon(g, 6);
		}
		v->SetHighColor(on ? kSegmentOn : kSegmentOff);
		v->FillPolygon(p, 6);
	};
	auto vertical = [&](float x, float y0, float y1, bool on) {
		BPoint p[6] = {
			BPoint(x, y0 + t / 2), BPoint(x + t / 2, y0 + t),
			BPoint(x + t / 2, y1 - t), BPoint(x, y1 - t / 2),
			BPoint(x - t / 2, y1 - t), BPoint(x - t / 2, y0 + t)
		};
		v->SetHighColor(on ? kSegmentOn : kSegmentOff);
		v->FillPolygon(p, 6);
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
DSKYPainter::_DrawSign(BView* v, BRect cell, char sign) const
{
	const float t = 3.0f;
	float cx = (cell.left + cell.right) / 2, cy = (cell.top + cell.bottom) / 2;
	float half = cell.Width() / 2 - 2;
	v->SetHighColor(sign == '+' || sign == '-' ? kSegmentOn : kSegmentOff);
	v->FillRect(BRect(cx - half, cy - t / 2, cx + half, cy + t / 2));
	v->SetHighColor(sign == '+' ? kSegmentOn : kSegmentOff);
	v->FillRect(BRect(cx - t / 2, cy - half, cx + t / 2, cy - t / 2 - 1));
	v->FillRect(BRect(cx - t / 2, cy + t / 2 + 1, cx + t / 2, cy + half));
}


void
DSKYPainter::_DrawDisplay(BView* v, const agc::DSKYState& s, bool flashOn,
	BRect area) const
{
	v->SetHighColor(kPanel);
	v->FillRoundRect(area, 4, 4);
	BRect glass = area.InsetByCopy(5, 5);
	v->SetHighColor(kGlass);
	v->FillRect(glass);

	float x0 = glass.left, y0 = glass.top;
	const float dw = 20, dh = 32;
	auto digit = [&](float x, float y, char c, bool lit = true) {
		_DrawDigit(v, BRect(x, y, x + dw, y + dh), c, lit && c != ' ');
	};

	// COMP ACTY and PROG.
	BRect comp(x0 + 6, y0 + 6, x0 + 58, y0 + 50);
	v->SetHighColor(s.compActy ? kPlateOn : kSegmentOff);
	v->FillRect(comp);
	BFont font(be_bold_font);
	font.SetSize(9);
	v->SetFont(&font);
	v->SetHighColor(s.compActy ? kGlass : kSegmentGlow);
	v->DrawString("COMP", BPoint(comp.left
		+ (comp.Width() - v->StringWidth("COMP")) / 2, comp.top + 19));
	v->DrawString("ACTY", BPoint(comp.left
		+ (comp.Width() - v->StringWidth("ACTY")) / 2, comp.top + 33));

	float right = x0 + 92;
	_DrawLabelPlate(v, BRect(right, y0 + 6, right + 60, y0 + 18), "PROG", true);
	digit(right + 6, y0 + 22, s.prog[0]);
	digit(right + 32, y0 + 22, s.prog[1]);

	// VERB and NOUN (these flash together when the computer wants input).
	bool vnLit = !s.verbNounFlash || flashOn;
	_DrawLabelPlate(v, BRect(x0 + 2, y0 + 62, x0 + 62, y0 + 74), "VERB", true);
	digit(x0 + 8, y0 + 78, s.verb[0], vnLit);
	digit(x0 + 34, y0 + 78, s.verb[1], vnLit);
	_DrawLabelPlate(v, BRect(right, y0 + 62, right + 60, y0 + 74), "NOUN", true);
	digit(right + 6, y0 + 78, s.noun[0], vnLit);
	digit(right + 32, y0 + 78, s.noun[1], vnLit);

	// Registers R1-R3, each preceded by a separator bar.
	float y = y0 + 116;
	for (int r = 0; r < 3; r++) {
		v->SetHighColor(kSegmentOn.red, kSegmentOn.green, kSegmentOn.blue, 180);
		v->FillRect(BRect(x0 + 6, y, glass.right - 6, y + 1.5f));
		y += 5;
		_DrawSign(v, BRect(x0 + 4, y + 4, x0 + 22, y + dh - 4), s.sign[r]);
		for (int i = 0; i < 5; i++)
			digit(x0 + 26 + i * 26, y, s.reg[r][i]);
		y += dh + 4;
	}
}


void
DSKYPainter::_DrawKeys(BView* v, int pressed, int pressed2) const
{
	BFont font(be_bold_font);
	for (int i = 0; i < 19; i++) {
		const DSKYKey& key = fKeys[i];
		bool down = i == pressed || i == pressed2;
		BRect f = key.frame;
		v->SetHighColor(kKeyEdge);
		v->FillRoundRect(f.OffsetByCopy(0, 2), 5, 5);
		v->SetHighColor(down ? kKeyCapDown : kKeyCap);
		v->FillRoundRect(down ? f.OffsetByCopy(0, 1) : f, 5, 5);
		v->SetHighColor(tint_color(kKeyCap, B_LIGHTEN_2_TINT));
		v->StrokeRoundRect(down ? f.OffsetByCopy(0, 1) : f, 5, 5);

		bool big = std::strlen(key.top) == 1;
		font.SetSize(big ? 19 : 11);
		v->SetFont(&font);
		v->SetHighColor(kKeyText);
		font_height fh;
		font.GetHeight(&fh);
		float lh = fh.ascent + fh.descent;
		int lines = key.bottom ? 2 : 1;
		float ty = f.top + (f.Height() - lines * lh) / 2 + fh.ascent
			+ (down ? 1 : 0);
		for (int l = 0; l < lines; l++) {
			const char* text = l == 0 ? key.top : key.bottom;
			v->DrawString(text, BPoint(f.left
				+ (f.Width() - v->StringWidth(text)) / 2, ty));
			ty += lh;
		}
	}
}

}  // namespace ui
