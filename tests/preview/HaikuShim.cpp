// Columbia — Cairo-backed stand-in for the Haiku drawing calls used by the
// custom views. Preview only; the app itself uses the real Interface Kit.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "shim/HaikuShim.h"

#include <algorithm>
#include <chrono>

static BFont sPlain(false), sBold(true), sFixed(false, true);
const BFont* be_plain_font = &sPlain;
const BFont* be_bold_font = &sBold;
const BFont* be_fixed_font = &sFixed;

bigtime_t
system_time()
{
	return std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

rgb_color
ui_color(color_which which)
{
	switch (which) {
		case B_PANEL_BACKGROUND_COLOR: return {216, 216, 216, 255};
		case B_PANEL_TEXT_COLOR: return {0, 0, 0, 255};
		case B_LIST_BACKGROUND_COLOR: return {255, 255, 255, 255};
		case B_LIST_SELECTED_BACKGROUND_COLOR: return {190, 190, 190, 255};
		default: return {0, 0, 0, 255};
	}
}

rgb_color
tint_color(rgb_color c, float tint)
{
	auto f = [tint](uint8 v) -> uint8 {
		if (tint < 1.0f) {
			// lighten
			double t = (1.0 - tint) / (1.0 - 0.385);
			return uint8(std::min(255.0, v + (255 - v) * t * 0.8));
		}
		double t = (tint - 1.0) / 0.7;
		return uint8(std::max(0.0, v * (1.0 - t * 0.6)));
	};
	return {f(c.red), f(c.green), f(c.blue), c.alpha};
}

BView::BView(const char*, uint32)
{
}

BView::BView(BRect frame, const char*, uint32, uint32)
	: fBounds(0, 0, frame.Width(), frame.Height())
{
}

void
BView::_Color()
{
	double a = fMode == B_OP_ALPHA ? fHigh.alpha / 255.0 : 1.0;
	cairo_set_source_rgba(fCairo, fHigh.red / 255.0, fHigh.green / 255.0,
		fHigh.blue / 255.0, a);
}

static void
SelectFont(cairo_t* cr, const BFont& font)
{
	cairo_select_font_face(cr, font.Fixed() ? "DejaVu Sans Mono" : "DejaVu Sans",
		CAIRO_FONT_SLANT_NORMAL,
		font.Bold() ? CAIRO_FONT_WEIGHT_BOLD : CAIRO_FONT_WEIGHT_NORMAL);
	cairo_set_font_size(cr, font.Size() * 0.92);
}

float
BView::StringWidth(const char* text) const
{
	static cairo_surface_t* scratch = cairo_image_surface_create(
		CAIRO_FORMAT_ARGB32, 4, 4);
	static cairo_t* cr = cairo_create(scratch);
	SelectFont(cr, fFont);
	cairo_text_extents_t e;
	cairo_text_extents(cr, text, &e);
	return float(e.x_advance);
}

void
BView::FillRect(BRect r)
{
	_Color();
	cairo_rectangle(fCairo, r.left, r.top, r.Width() + 1, r.Height() + 1);
	cairo_fill(fCairo);
}

void
BView::StrokeRect(BRect r)
{
	_Color();
	cairo_set_line_width(fCairo, fPen);
	cairo_rectangle(fCairo, r.left + 0.5, r.top + 0.5, r.Width(), r.Height());
	cairo_stroke(fCairo);
}

void
BView::_RoundRect(BRect r, float rad)
{
	double x = r.left, y = r.top, w = r.Width() + 1, h = r.Height() + 1;
	cairo_new_sub_path(fCairo);
	cairo_arc(fCairo, x + w - rad, y + rad, rad, -M_PI / 2, 0);
	cairo_arc(fCairo, x + w - rad, y + h - rad, rad, 0, M_PI / 2);
	cairo_arc(fCairo, x + rad, y + h - rad, rad, M_PI / 2, M_PI);
	cairo_arc(fCairo, x + rad, y + rad, rad, M_PI, 3 * M_PI / 2);
	cairo_close_path(fCairo);
}

void
BView::FillRoundRect(BRect r, float rx, float)
{
	_Color();
	_RoundRect(r, rx);
	cairo_fill(fCairo);
}

void
BView::StrokeRoundRect(BRect r, float rx, float)
{
	_Color();
	cairo_set_line_width(fCairo, fPen);
	_RoundRect(r.InsetByCopy(0.5f, 0.5f), rx);
	cairo_stroke(fCairo);
}

void
BView::FillEllipse(BPoint c, float rx, float ry)
{
	_Color();
	cairo_save(fCairo);
	cairo_translate(fCairo, c.x, c.y);
	cairo_scale(fCairo, std::max(rx, 0.01f), std::max(ry, 0.01f));
	cairo_arc(fCairo, 0, 0, 1, 0, 2 * M_PI);
	cairo_restore(fCairo);
	cairo_fill(fCairo);
}

void
BView::StrokeEllipse(BPoint c, float rx, float ry, pattern p)
{
	_Color();
	cairo_set_line_width(fCairo, fPen);
	if (p.data[0] != 0xff) {
		double dash[2] = {2, 2};
		cairo_set_dash(fCairo, dash, 2, 0);
	}
	cairo_save(fCairo);
	cairo_translate(fCairo, c.x, c.y);
	cairo_scale(fCairo, std::max(rx, 0.01f), std::max(ry, 0.01f));
	cairo_arc(fCairo, 0, 0, 1, 0, 2 * M_PI);
	cairo_restore(fCairo);
	cairo_stroke(fCairo);
	cairo_set_dash(fCairo, nullptr, 0, 0);
}

void
BView::FillPolygon(const BPoint* points, int32 count)
{
	_Color();
	cairo_move_to(fCairo, points[0].x, points[0].y);
	for (int32 i = 1; i < count; i++)
		cairo_line_to(fCairo, points[i].x, points[i].y);
	cairo_close_path(fCairo);
	cairo_fill(fCairo);
}

void
BView::StrokeLine(BPoint a, BPoint b)
{
	_Color();
	cairo_set_line_width(fCairo, fPen);
	cairo_move_to(fCairo, a.x + 0.5, a.y + 0.5);
	cairo_line_to(fCairo, b.x + 0.5, b.y + 0.5);
	cairo_stroke(fCairo);
}

void
BView::DrawString(const char* text, BPoint where)
{
	_Color();
	SelectFont(fCairo, fFont);
	cairo_move_to(fCairo, where.x, where.y);
	cairo_show_text(fCairo, text);
}

void
BView::AddLine(BPoint a, BPoint b, rgb_color color)
{
	rgb_color saved = fHigh;
	fHigh = color;
	StrokeLine(a, b);
	fHigh = saved;
}

void
BView::DrawBitmap(const BBitmap* bitmap, BRect, BRect)
{
	cairo_set_source_surface(fCairo, bitmap->Surface(), 0, 0);
	cairo_paint(fCairo);
}

BBitmap::BBitmap(BRect bounds, color_space, bool)
	: fBounds(bounds)
{
	fSurface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
		int(bounds.Width()) + 1, int(bounds.Height()) + 1);
	fCairo = cairo_create(fSurface);
}

BBitmap::~BBitmap()
{
	cairo_destroy(fCairo);
	cairo_surface_destroy(fSurface);
}

void
BBitmap::AddChild(BView* view)
{
	view->SetContext(fCairo);
	view->ResizeTo(fBounds.Width(), fBounds.Height());
}

void
BView::DrawBitmap(const BBitmap* bitmap, BPoint at)
{
	cairo_set_source_surface(fCairo, bitmap->Surface(), at.x, at.y);
	cairo_paint(fCairo);
}

void
BView::PushState()
{
	cairo_save(fCairo);
}

void
BView::PopState()
{
	cairo_restore(fCairo);
}

void
BView::SetOrigin(BPoint p)
{
	cairo_translate(fCairo, p.x, p.y);
}

void
BView::SetScale(float scale)
{
	cairo_scale(fCairo, scale, scale);
}

void
BView::ConstrainClippingRegion(BRegion* region)
{
	cairo_reset_clip(fCairo);
	if (region && region->IsSet()) {
		BRect r = region->Frame();
		cairo_rectangle(fCairo, r.left, r.top, r.Width() + 1, r.Height() + 1);
		cairo_clip(fCairo);
	}
}
