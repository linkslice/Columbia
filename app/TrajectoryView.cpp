// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "TrajectoryView.h"

#include <Bitmap.h>
#include <Message.h>
#include <Window.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "Mission.h"

using sim::Vec3;

namespace {

const rgb_color kBackground = {10, 13, 19, 255};
const rgb_color kEarth = {47, 111, 214, 255};
const rgb_color kEarthEdge = {140, 190, 255, 255};
const rgb_color kMoon = {170, 170, 166, 255};
const rgb_color kMoonEdge = {225, 225, 220, 255};
const rgb_color kOrbit = {60, 66, 80, 255};
const rgb_color kSOI = {44, 48, 60, 255};
const rgb_color kFlown = {232, 232, 228, 255};
const rgb_color kBurn = {255, 166, 48, 255};
const rgb_color kPredicted = {53, 208, 255, 255};
const rgb_color kPredictedBurn = {255, 120, 60, 255};
const rgb_color kMarker = {255, 216, 74, 255};
const rgb_color kText = {201, 209, 217, 255};
const rgb_color kDim = {120, 128, 140, 255};
const rgb_color kCraft = {255, 255, 255, 255};

const double kMinScale = 2e-8;   // pixels per metre
const double kMaxScale = 5e-3;

// Clips a segment to a rectangle (Liang-Barsky). Returns false if outside.
bool
Clip(double& x0, double& y0, double& x1, double& y1, double left, double top,
	double right, double bottom)
{
	double t0 = 0, t1 = 1, dx = x1 - x0, dy = y1 - y0;
	double p[4] = {-dx, dx, -dy, dy};
	double q[4] = {x0 - left, right - x0, y0 - top, bottom - y0};
	for (int i = 0; i < 4; i++) {
		if (p[i] == 0) {
			if (q[i] < 0)
				return false;
			continue;
		}
		double r = q[i] / p[i];
		if (p[i] < 0) {
			if (r > t1)
				return false;
			t0 = std::max(t0, r);
		} else {
			if (r < t0)
				return false;
			t1 = std::min(t1, r);
		}
	}
	double nx0 = x0 + t0 * dx, ny0 = y0 + t0 * dy;
	double nx1 = x0 + t1 * dx, ny1 = y0 + t1 * dy;
	x0 = nx0; y0 = ny0; x1 = nx1; y1 = ny1;
	return true;
}

std::string
Distance(double metres)
{
	char text[32];
	double km = metres / 1000;
	if (km >= 1000)
		std::snprintf(text, sizeof(text), "%.0f,%03.0f km",
			std::floor(km / 1000), std::fmod(km, 1000));
	else
		std::snprintf(text, sizeof(text), "%.0f km", km);
	return text;
}

}  // namespace


TrajectoryView::TrajectoryView(columbia::Mission* mission)
	:
	BView("trajectory", B_WILL_DRAW | B_FRAME_EVENTS | B_NAVIGABLE),
	fMission(mission),
	fBitmap(nullptr),
	fOffscreen(nullptr),
	fFrame(FRAME_ROTATING),
	fFollow(false),
	fScale(1e-6),
	fDragging(false),
	fNow(0),
	fFitted(false)
{
	SetViewColor(B_TRANSPARENT_COLOR);
}


TrajectoryView::~TrajectoryView()
{
	delete fBitmap;
}


void
TrajectoryView::AttachedToWindow()
{
	BView::AttachedToWindow();
	FrameResized(Bounds().Width(), Bounds().Height());
}


BSize
TrajectoryView::MinSize()
{
	return BSize(360, 300);
}


void
TrajectoryView::FrameResized(float width, float height)
{
	delete fBitmap;
	fBitmap = new BBitmap(BRect(0, 0, std::max(width, 1.0f),
		std::max(height, 1.0f)), B_RGB32, true);
	fOffscreen = new BView(fBitmap->Bounds(), "offscreen", B_FOLLOW_NONE,
		B_WILL_DRAW);
	fBitmap->AddChild(fOffscreen);
	// The first real size arrives after layout; fit to it once.
	if (!fFitted && width > 50 && height > 50) {
		fFitted = true;
		Fit();
		return;
	}
	_Render();
	Invalidate();
}


void
TrajectoryView::SetFrameMode(ViewFrame frame)
{
	fFrame = frame;
	Fit();
}


void
TrajectoryView::SetFollow(bool follow)
{
	fFollow = follow;
	Refresh();
}


void
TrajectoryView::Fit()
{
	BRect b = Bounds();
	double w = std::max(1.0f, b.Width()), h = std::max(1.0f, b.Height());
	const double D = sim::kMoonDistance;
	switch (fFrame) {
		case FRAME_ROTATING:
			fCenter = Vec3(D * 0.52, 0, 0);
			fScale = std::min(w / (D * 1.3), h / (D * 0.75));
			break;
		case FRAME_INERTIAL:
			fCenter = Vec3();
			fScale = std::min(w, h) / (2.3 * D);
			break;
		case FRAME_MOON:
		{
			// Close in on a lunar orbit; otherwise show the approach.
			const sim::VehicleState& s = fMission->State();
			double d = (s.r - fMission->Model().MoonPosition(s.t)).Length();
			double radius = d < 10000e3 ? std::max(4000e3, 2.4 * d) : 14000e3;
			fCenter = Vec3();
			fScale = std::min(w, h) / (2 * radius);
			break;
		}
	}
	Refresh();
}


void
TrajectoryView::ZoomBy(float factor, BPoint about)
{
	Vec3 anchor = _FromView(about);
	fScale = std::max(kMinScale, std::min(kMaxScale, fScale * factor));
	// Keep the point under the cursor fixed.
	BRect b = Bounds();
	fCenter.x = anchor.x - (about.x - b.Width() / 2) / fScale;
	fCenter.y = anchor.y + (about.y - b.Height() / 2) / fScale;
	Refresh();
}


Vec3
TrajectoryView::_ToFrame(const Vec3& r, double t) const
{
	const sim::Model& model = fMission->Model();
	switch (fFrame) {
		case FRAME_ROTATING: {
			double a = model.moonPhase0 + model.MoonRate() * t;
			double c = std::cos(a), s = std::sin(a);
			return Vec3(c * r.x + s * r.y, -s * r.x + c * r.y, r.z);
		}
		case FRAME_INERTIAL:
			return r;
		case FRAME_MOON:
			return r - model.MoonPosition(t);
	}
	return r;
}


BPoint
TrajectoryView::_ToView(const Vec3& p) const
{
	BRect b = Bounds();
	return BPoint(b.Width() / 2 + (p.x - fCenter.x) * fScale,
		b.Height() / 2 - (p.y - fCenter.y) * fScale);
}


Vec3
TrajectoryView::_FromView(BPoint where) const
{
	BRect b = Bounds();
	return Vec3(fCenter.x + (where.x - b.Width() / 2) / fScale,
		fCenter.y - (where.y - b.Height() / 2) / fScale, 0);
}


void
TrajectoryView::Refresh()
{
	if (fBitmap == nullptr)
		return;
	if (fFollow) {
		const sim::VehicleState& s = fMission->State();
		fCenter = _ToFrame(s.r, s.t);
	}
	_Render();
	Invalidate();
}


void
TrajectoryView::Draw(BRect updateRect)
{
	if (fBitmap != nullptr)
		DrawBitmap(fBitmap, updateRect, updateRect);
}


// ---------------------------------------------------------------------------
// Rendering


void
TrajectoryView::_DrawPath(BView* view, const std::vector<sim::Sample>& samples,
	rgb_color color, rgb_color burnColor, bool dashed)
{
	if (samples.size() < 2)
		return;
	BRect b = view->Bounds();
	const double margin = 40;
	const double left = -margin, top = -margin;
	const double right = b.Width() + margin, bottom = b.Height() + margin;
	const double cx = b.Width() / 2, cy = b.Height() / 2;

	double dashPhase = 0;
	view->BeginLineArray(int32(std::min<size_t>(samples.size(), 60000)));
	int32 added = 0;
	Vec3 prev = _ToFrame(samples[0].r, samples[0].t);
	for (size_t i = 1; i < samples.size(); i++) {
		Vec3 p = _ToFrame(samples[i].r, samples[i].t);
		double x0 = cx + (prev.x - fCenter.x) * fScale;
		double y0 = cy - (prev.y - fCenter.y) * fScale;
		double x1 = cx + (p.x - fCenter.x) * fScale;
		double y1 = cy - (p.y - fCenter.y) * fScale;
		prev = p;
		double length = std::hypot(x1 - x0, y1 - y0);
		bool draw = true;
		if (dashed) {
			draw = std::fmod(dashPhase, 10.0) < 6.0;
			dashPhase += length;
		}
		if (!draw || length < 0.05)
			continue;
		if (!Clip(x0, y0, x1, y1, left, top, right, bottom))
			continue;
		if (added >= 60000) {
			view->EndLineArray();
			view->BeginLineArray(60000);
			added = 0;
		}
		view->AddLine(BPoint(x0, y0), BPoint(x1, y1),
			samples[i].burning ? burnColor : color);
		added++;
	}
	view->EndLineArray();
}


void
TrajectoryView::_DrawBody(BView* view, const Vec3& center, double radius,
	rgb_color color, const char* label)
{
	BPoint c = _ToView(center);
	float r = std::max(3.0f, float(radius * fScale));
	BRect b = view->Bounds();
	if (c.x + r < -50 || c.x - r > b.Width() + 50 || c.y + r < -50
		|| c.y - r > b.Height() + 50)
		return;
	if (r > 20000)
		r = 20000;
	view->SetHighColor(color);
	view->FillEllipse(c, r, r);
	view->SetHighColor(tint_color(color, B_LIGHTEN_1_TINT));
	view->StrokeEllipse(c, r, r);
	if (label != nullptr) {
		// Below the body, out of the way of markers near its limb.
		float w = view->StringWidth(label);
		if (r > 40) {
			view->SetHighColor(kBackground);
			view->DrawString(label, BPoint(c.x - w / 2, c.y + 4));
		} else {
			view->SetHighColor(kText);
			view->DrawString(label, BPoint(c.x - w / 2, c.y + r + 15));
		}
	}
}


void
TrajectoryView::_DrawMarker(BView* view, const Vec3& p, rgb_color color,
	const char* label)
{
	BPoint c = _ToView(p);
	view->SetHighColor(color);
	BPoint diamond[4] = {c + BPoint(0, -4), c + BPoint(4, 0), c + BPoint(0, 4),
		c + BPoint(-4, 0)};
	view->FillPolygon(diamond, 4);
	if (label != nullptr) {
		view->SetHighColor(color);
		view->DrawString(label, c + BPoint(7, 4));
	}
}


void
TrajectoryView::_DrawScale(BView* view)
{
	BRect b = view->Bounds();
	// A round length close to 120 pixels.
	double target = 120 / fScale;
	double magnitude = std::pow(10, std::floor(std::log10(target)));
	double length = magnitude;
	for (double m : {1.0, 2.0, 5.0, 10.0}) {
		if (m * magnitude <= target)
			length = m * magnitude;
	}
	float px = float(length * fScale);
	float x = 14, y = b.Height() - 16;
	view->SetHighColor(kText);
	view->StrokeLine(BPoint(x, y), BPoint(x + px, y));
	view->StrokeLine(BPoint(x, y - 4), BPoint(x, y + 4));
	view->StrokeLine(BPoint(x + px, y - 4), BPoint(x + px, y + 4));
	std::string text = Distance(length);
	view->DrawString(text.c_str(), BPoint(x + px + 8, y + 4));
}


void
TrajectoryView::_DrawLegend(BView* view)
{
	BRect b = view->Bounds();
	enum Swatch { LINE, DASHED, DIAMOND };
	struct Entry { rgb_color color; const char* text; Swatch swatch; };
	const Entry entries[] = {
		{kFlown, "Flown", LINE},
		{kBurn, "Burn", LINE},
		{kPredicted, "Predicted", DASHED},
		{kMarker, "Events", DIAMOND},
	};
	float x = b.Width() - 104, y = 18;
	for (const Entry& e : entries) {
		view->SetHighColor(e.color);
		view->SetPenSize(2);
		switch (e.swatch) {
			case LINE:
				view->StrokeLine(BPoint(x, y - 4), BPoint(x + 16, y - 4));
				break;
			case DASHED:
				view->StrokeLine(BPoint(x, y - 4), BPoint(x + 5, y - 4));
				view->StrokeLine(BPoint(x + 10, y - 4), BPoint(x + 16, y - 4));
				break;
			case DIAMOND:
			{
				BPoint c(x + 8, y - 4);
				BPoint d[4] = {c + BPoint(0, -4), c + BPoint(4, 0),
					c + BPoint(0, 4), c + BPoint(-4, 0)};
				view->FillPolygon(d, 4);
				break;
			}
		}
		view->SetPenSize(1);
		view->SetHighColor(kText);
		view->DrawString(e.text, BPoint(x + 24, y));
		y += 15;
	}
}


void
TrajectoryView::_Render()
{
	if (fBitmap == nullptr || !fBitmap->Lock())
		return;
	BView* v = fOffscreen;
	BRect b = v->Bounds();
	const sim::Model& model = fMission->Model();
	const sim::VehicleState& state = fMission->State();
	const sim::Prediction& prediction = fMission->Prediction();
	fNow = state.t;

	v->SetDrawingMode(B_OP_COPY);
	v->SetHighColor(kBackground);
	v->FillRect(b);
	BFont font(be_plain_font);
	font.SetSize(10.5f);
	v->SetFont(&font);

	Vec3 earth = _ToFrame(Vec3(), fNow);
	Vec3 moon = _ToFrame(model.MoonPosition(fNow), fNow);

	// Reference circles.
	v->SetHighColor(kOrbit);
	if (fFrame == FRAME_INERTIAL) {
		BPoint c = _ToView(earth);
		float r = float(sim::kMoonDistance * fScale);
		if (r < 100000)
			v->StrokeEllipse(c, r, r);
	}
	{
		BPoint c = _ToView(moon);
		float r = float(sim::kMoonSOI * fScale);
		if (r > 6 && r < 100000) {
			v->SetHighColor(kSOI);
			v->SetPenSize(1);
			pattern dots = {{0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55}};
			v->StrokeEllipse(c, r, r, dots);
		}
	}

	// In the inertial view, show where the Moon will be at pericynthion.
	if (fFrame == FRAME_INERTIAL && prediction.hasPericynthion
		&& prediction.pericynthionTime > fNow + 600) {
		Vec3 ghost = model.MoonPosition(prediction.pericynthionTime);
		BPoint c = _ToView(ghost);
		float r = std::max(3.0f, float(sim::kRMoon * fScale));
		v->SetHighColor(kDim);
		v->StrokeEllipse(c, r, r);
		v->DrawString("Moon at arrival", c + BPoint(r + 4, r + 12));
	}

	_DrawBody(v, earth, sim::kREarth, kEarth, "Earth");
	_DrawBody(v, moon, sim::kRMoon, kMoon, "Moon");
	(void)kEarthEdge;
	(void)kMoonEdge;

	// Paths.
	_DrawPath(v, prediction.samples, kPredicted, kPredictedBurn, true);
	_DrawPath(v, fMission->FlownPath(), kFlown, kBurn, false);

	// Planned burns along the prediction.
	const std::vector<sim::Sample>& ps = prediction.samples;
	for (const sim::Burn& burn : fMission->Plan()) {
		if (state.Completed(burn.id) || burn.time < fNow)
			continue;
		auto it = std::lower_bound(ps.begin(), ps.end(), burn.time,
			[](const sim::Sample& s, double t) { return s.t < t; });
		if (it == ps.end())
			continue;
		_DrawMarker(v, _ToFrame(it->r, it->t), kBurn, burn.label.c_str());
	}
	if (prediction.hasPericynthion && prediction.pericynthionTime > fNow) {
		auto it = std::lower_bound(ps.begin(), ps.end(),
			prediction.pericynthionTime,
			[](const sim::Sample& s, double t) { return s.t < t; });
		if (it != ps.end()) {
			char text[64];
			std::snprintf(text, sizeof(text), "Pericynthion %.0f km",
				prediction.pericynthionAltitude / 1000);
			_DrawMarker(v, _ToFrame(it->r, it->t), kMarker, text);
		}
	}
	const sim::VehicleState& end = prediction.final;
	if (end.outcome == sim::Outcome::Splashdown
		|| end.outcome == sim::Outcome::SkipOut
		|| end.outcome == sim::Outcome::TooSteep
		|| end.outcome == sim::Outcome::LunarImpact) {
		char text[64];
		if (end.outcome == sim::Outcome::LunarImpact)
			std::snprintf(text, sizeof(text), "Impact");
		else
			std::snprintf(text, sizeof(text), "Entry %.1f\xc2\xb0",
				end.entryAngle);
		_DrawMarker(v, _ToFrame(end.r, end.t),
			end.outcome == sim::Outcome::Splashdown ? kMarker : kBurn, text);
	}

	// The spacecraft.
	BPoint craft = _ToView(_ToFrame(state.r, state.t));
	v->SetHighColor(kCraft);
	v->FillEllipse(craft, 3.5f, 3.5f);
	v->SetHighColor(kCraft.red, kCraft.green, kCraft.blue, 255);
	v->StrokeEllipse(craft, 7, 7);
	v->DrawString("CSM", craft + BPoint(9, -6));

	// Overlays.
	const char* names[] = {"Earth–Moon rotating frame",
		"Earth-centred inertial frame", "Moon-centred frame"};
	v->SetHighColor(kText);
	v->DrawString(names[fFrame], BPoint(12, 18));
	v->SetHighColor(kDim);
	v->DrawString("Wheel: zoom   Drag: pan   Double-click: fit",
		BPoint(12, 33));
	_DrawScale(v);
	_DrawLegend(v);

	v->Sync();
	fBitmap->Unlock();
}


// ---------------------------------------------------------------------------
// Input


void
TrajectoryView::MouseDown(BPoint where)
{
	MakeFocus(true);
	int32 clicks = 1;
	if (Window()->CurrentMessage() != nullptr)
		Window()->CurrentMessage()->FindInt32("clicks", &clicks);
	if (clicks >= 2) {
		Fit();
		return;
	}
	fDragging = true;
	fDragStart = where;
	fDragCenter = fCenter;
	SetMouseEventMask(B_POINTER_EVENTS, B_LOCK_WINDOW_FOCUS);
}


void
TrajectoryView::MouseMoved(BPoint where, uint32 transit,
	const BMessage* dragMessage)
{
	(void)transit;
	(void)dragMessage;
	if (!fDragging)
		return;
	if (fFollow)
		fFollow = false;
	fCenter.x = fDragCenter.x - (where.x - fDragStart.x) / fScale;
	fCenter.y = fDragCenter.y + (where.y - fDragStart.y) / fScale;
	_Render();
	Invalidate();
}


void
TrajectoryView::MouseUp(BPoint where)
{
	(void)where;
	fDragging = false;
}


void
TrajectoryView::MessageReceived(BMessage* message)
{
	if (message->what == B_MOUSE_WHEEL_CHANGED) {
		float dy = 0;
		if (message->FindFloat("be:wheel_delta_y", &dy) == B_OK && dy != 0) {
			BPoint where;
			uint32 buttons;
			GetMouse(&where, &buttons, false);
			if (!Bounds().Contains(where))
				where = BPoint(Bounds().Width() / 2, Bounds().Height() / 2);
			ZoomBy(dy < 0 ? 1.25f : 0.8f, fFollow
				? BPoint(Bounds().Width() / 2, Bounds().Height() / 2) : where);
		}
		return;
	}
	BView::MessageReceived(message);
}
