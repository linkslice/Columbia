// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "WindowView.h"

#include <Window.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "Drawing.h"
#include "FlightSession.h"
#include "flight/Recording.h"

namespace columbia {

using namespace flight;
using namespace ui;
using sim::Mat3;
using sim::Quat;
using sim::Vec3;
namespace world = sim::world;

namespace {

uint16_t
ModuleBits(const sim::Vehicle& v)
{
	uint16_t m = 0;
	for (const sim::Module& mod : v.Modules()) {
		const std::string& n = mod.name;
		if (n == "S-IC") m |= REC_SIC;
		else if (n == "S-II") m |= REC_SII;
		else if (n == "S-IVB") m |= REC_SIVB;
		else if (n == "S-IB") m |= REC_SIB;
		else if (n == "Service Module") m |= REC_SM;
		else if (n == "Command Module") m |= REC_CM;
		else if (n == "LM descent stage") m |= REC_LM_DESCENT;
		else if (n == "LM ascent stage") m |= REC_LM_ASCENT;
		else if (n == "Launch escape tower") m |= REC_TOWER;
	}
	return m;
}

// Line segments of a simple model of a stack (body frame, m): rings and
// longerons of each part.
void
StackModel(uint16_t modules, std::vector<std::pair<Vec3, Vec3>>* out)
{
	Part parts[24];
	float length;
	int n = BuildStack(modules, parts, &length);
	const int seg = 12;
	for (int i = 0; i < n; i++) {
		const Part& p = parts[i];
		float x0 = p.x0 - length / 2, x1 = p.x1 - length / 2;
		for (int k = 0; k < seg; k++) {
			double a0 = 2 * kPi * k / seg, a1 = 2 * kPi * (k + 1) / seg;
			Vec3 r00(x0, p.r0 * std::cos(a0), p.r0 * std::sin(a0));
			Vec3 r01(x0, p.r0 * std::cos(a1), p.r0 * std::sin(a1));
			Vec3 r10(x1, p.r1 * std::cos(a0), p.r1 * std::sin(a0));
			Vec3 r11(x1, p.r1 * std::cos(a1), p.r1 * std::sin(a1));
			out->push_back({r00, r01});
			out->push_back({r10, r11});
			if (k % 3 == 0)
				out->push_back({r00, r10});
		}
	}
}

// The LM: descent stage box with legs, ascent stage above (+X up).
void
LMModel(bool descent, std::vector<std::pair<Vec3, Vec3>>* out)
{
	auto box = [&](double x0, double x1, double h) {
		Vec3 c[8];
		for (int i = 0; i < 8; i++)
			c[i] = Vec3(i & 4 ? x1 : x0, i & 1 ? h : -h, i & 2 ? h : -h);
		const int e[12][2] = {{0, 1}, {2, 3}, {0, 2}, {1, 3}, {4, 5}, {6, 7},
			{4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
		for (auto& k : e)
			out->push_back({c[k[0]], c[k[1]]});
	};
	if (descent) {
		box(-2.6, -0.2, 2.1);
		for (int s = 0; s < 4; s++) {
			double a = kPi / 4 + s * kPi / 2;
			Vec3 top(-1.0, 2.6 * std::cos(a), 2.6 * std::sin(a));
			Vec3 foot(-3.4, 4.6 * std::cos(a), 4.6 * std::sin(a));
			out->push_back({top, foot});
		}
	}
	box(-0.2, 2.6, 1.9);
	box(2.6, 3.4, 0.9);
}

}  // namespace


WindowView::WindowView(FlightSession* session)
	:
	BView("window", B_WILL_DRAW | B_FRAME_EVENTS),
	fSession(session)
{
	SetViewColor(B_TRANSPARENT_COLOR);
	// A fixed sky: stars spread over the sphere, brighter ones fewer.
	uint32_t seed = 0x5eed1969;
	auto rnd = [&]() {
		seed = seed * 1664525u + 1013904223u;
		return (seed >> 8) / double(1 << 24);
	};
	for (int i = 0; i < 1600; i++) {
		double z = rnd() * 2 - 1, t = rnd() * 2 * kPi;
		double r = std::sqrt(1 - z * z);
		fStars.push_back(Vec3(r * std::cos(t), r * std::sin(t), z));
		double b = rnd();
		fStarSize.push_back(float(b > 0.985 ? 2.2 : b > 0.9 ? 1.4 : 0.8));
	}
}


void
WindowView::FrameResized(float, float)
{
	Invalidate();
}


void
WindowView::Refresh()
{
	if (!fSession || !fSession->Running())
		return;
	_Snapshot();
	Invalidate();
}


void
WindowView::MouseDown(BPoint where)
{
	int32 clicks = 1;
	if (Window() && Window()->CurrentMessage())
		Window()->CurrentMessage()->FindInt32("clicks", &clicks);
	if (clicks >= 2) {
		fLookYaw = fLookPitch = 0;
		Refresh();
		return;
	}
	fDragging = true;
	fDragFrom = where;
	SetMouseEventMask(B_POINTER_EVENTS, B_LOCK_WINDOW_FOCUS);
}


void
WindowView::MouseMoved(BPoint where, uint32, const BMessage*)
{
	if (!fDragging)
		return;
	fLookYaw = std::clamp(fLookYaw + (where.x - fDragFrom.x) * 0.15, -60.0, 60.0);
	fLookPitch = std::clamp(fLookPitch - (where.y - fDragFrom.y) * 0.15, -45.0, 45.0);
	fDragFrom = where;
	Invalidate();
}


void
WindowView::MouseUp(BPoint)
{
	fDragging = false;
}


void
WindowView::_Snapshot()
{
	std::lock_guard<std::recursive_mutex> g(fSession->Lock());
	FlightSim& f = fSession->Sim();
	bool lm = f.LMVehicle() && f.LGCPowered() && f.CrewInLM() > f.CrewInCM();
	fLM = lm;
	const sim::Vehicle& me = lm ? *f.LMVehicle() : f.CSMVehicle();
	fEye = me.position;
	fJD = f.JulianDate();
	fMoon = f.Moon().MoonPosition();
	fMission = f.Mission().number;
	// The window's line of sight in the body: the CM's rendezvous windows
	// look along +X (the docking axis); the LM commander's forward window
	// along +Z, 25 degrees below.
	Vec3 axis = lm ? Vec3(-std::sin(25 * kDeg), 0, std::cos(25 * kDeg))
		: Vec3(1, 0, 0);
	Vec3 upBody = lm ? Vec3(std::cos(25 * kDeg), 0, std::sin(25 * kDeg))
		: Vec3(0, 0, -1);
	// The look-around offset.
	Quat look = Quat::AxisAngle(upBody, -fLookYaw * kDeg)
		* Quat::AxisAngle(axis.Cross(upBody), -fLookPitch * kDeg);
	axis = look.Rotate(axis);
	upBody = look.Rotate(upBody);
	fFwd = me.attitude.Rotate(axis).Unit();
	fUp = me.attitude.Rotate(upBody).Unit();
	fRight = fFwd.Cross(fUp).Unit();
	fUp = fRight.Cross(fFwd);
	fNearMoon = (fEye - fMoon).Length() < 3 * world::kRMoon;
	fAltitude = fNearMoon ? f.Moon().Altitude(fEye)
		: fEye.Length() - world::kREarth;
	// Everything else in view.
	fOthers.clear();
	auto add = [&](const sim::Vehicle* v, const std::string& name) {
		if (!v || v == &me)
			return;
		Other o;
		o.r = v->position;
		o.q = v->attitude;
		o.modules = ModuleBits(*v);
		o.lm = (o.modules & (REC_LM_ASCENT | REC_LM_DESCENT))
			&& !(o.modules & (REC_SM | REC_CM | REC_SIVB));
		o.name = name;
		for (const sim::Module& m : v->Modules())
			for (const sim::Thruster& t : m.thrusters)
				if (t.kind == sim::THRUSTER_MAIN && (t.on || t.level > 0))
					o.engine = true;
		fOthers.push_back(o);
	};
	add(&f.CSMVehicle(), f.Mission().csmCallsign);
	add(f.LMVehicle(), f.Mission().lmCallsign);
	add(f.BoosterVehicle(), "S-IVB");
	fRange = -1;
	fTargetName.clear();
	for (const Other& o : fOthers) {
		double d = (o.r - fEye).Length();
		if (fRange < 0 || d < fRange) {
			fRange = d;
			fTargetName = o.name;
			const sim::Vehicle* tv = o.name == "S-IVB" ? f.BoosterVehicle()
				: o.name == f.Mission().lmCallsign ? f.LMVehicle() : &f.CSMVehicle();
			if (tv)
				fRate = (tv->velocity - me.velocity).Dot((tv->position - fEye).Unit());
		}
	}
	// The LPD angle the LGC shows in P64 (N64 R1: seconds left, angle).
	fLPDAngle = -1;
	if (lm) {
		const agc::DSKYState& d = f.LGCDSKY().State();
		if (d.prog[0] == '6' && d.prog[1] == '4' && d.noun[0] == '6'
				&& d.noun[1] == '4' && d.reg[0][3] != ' ') {
			int angle = (d.reg[0][3] - '0') * 10 + (d.reg[0][4] - '0');
			fLPDAngle = angle;
		}
	}
	// Craters: the mapped ones round the sites, and a field of small ones.
	if (fCraters.empty() && fNearMoon) {
		for (const auto& feat : f.Terrain().Features())
			if (feat.depthRatio > 0)
				fCraters.push_back({world::LatLonToVector(feat.lat, feat.lon,
					f.Mission().site.radius), feat.diameter});
	}
	if (fNearMoon) {
		const MissionSpec& m = f.Mission();
		fSiteUp = world::LatLonToVector(m.site.latitude, m.site.longitude, 1.0);
		fSiteRadius = m.site.radius;
	}
	fValid = true;
}


BPoint
WindowView::_Project(const Vec3& p, bool* front) const
{
	Vec3 d = p - fEye;
	double z = d.Dot(fFwd);
	*front = z > 1e-3;
	if (z < 1e-3)
		z = 1e-3;
	BRect b = Bounds();
	return BPoint(b.Width() / 2 + float(d.Dot(fRight) / z * fFocal),
		b.Height() / 2 - float(d.Dot(fUp) / z * fFocal));
}


void
WindowView::_DrawStars()
{
	SetDrawingMode(B_OP_ALPHA);
	BRect b = Bounds();
	for (size_t i = 0; i < fStars.size(); i++) {
		const Vec3& s = fStars[i];
		double z = s.Dot(fFwd);
		if (z < 0.2)
			continue;
		float x = b.Width() / 2 + float(s.Dot(fRight) / z * fFocal);
		float y = b.Height() / 2 - float(s.Dot(fUp) / z * fFocal);
		if (x < 0 || y < 0 || x > b.Width() || y > b.Height())
			continue;
		float r = fStarSize[i];
		SetHighColor(230, 232, 240, uint8(r > 2 ? 255 : r > 1 ? 200 : 130));
		FillEllipse(BPoint(x, y), r, r);
	}
	SetDrawingMode(B_OP_COPY);
}


void
WindowView::_DrawBody(bool moon)
{
	Vec3 center = moon ? fMoon : Vec3();
	double R = moon ? world::kRMoon : world::kREarth;
	Vec3 d = center - fEye;
	double dist = d.Length();
	// Near the Moon the ground's under us whatever the mean radius says.
	if (moon && fNearMoon)
		R = std::min(R, dist - std::max(1.0, fAltitude));
	if (dist <= R)
		return;
	double ang = std::asin(R / dist);
	if (ang > 55 * kDeg) {
		_DrawGround(center, R, dist, moon);
		return;
	}
	Vec3 c = d / dist;
	double cz = c.Dot(fFwd);
	double half = std::atan(std::hypot(Bounds().Width(), Bounds().Height()) / 2
		/ fFocal);
	if (std::acos(std::clamp(cz, -1.0, 1.0)) > half + ang)
		return;
	bool front;
	BPoint at = _Project(center, &front);
	if (!front && cz < 0)
		return;
	// The limb's apparent radius (small-angle near the middle of the view).
	float radius = float(std::tan(std::min(ang, 1.45)) * fFocal / std::max(0.2, cz));
	// The body-fixed view basis: toward us, our right, our up.
	Mat3 toFixed = (moon ? world::Ephemeris::MoonFixedToInertial(fJD)
		: world::Ephemeris::EarthFixedToInertial(fJD)).Transposed();
	Vec3 fwd = toFixed * (c * -1.0);
	Vec3 east = toFixed * fRight;
	east = (east - fwd * east.Dot(fwd)).Unit();
	Vec3 north = fwd.Cross(east);
	Globe g(at, radius, fwd, east, north);
	if (moon)
		DrawMoon(this, g, fJD, fMission);
	else
		DrawEarth(this, g, fJD);
}


void
WindowView::_DrawGround(const Vec3& center, double R, double dist, bool moon)
{
	// Close in: the ground fills the view below the horizon, which dips by
	// acos(R / distance). Column by column, find where the rays graze it.
	BRect b = Bounds();
	Vec3 up = (fEye - center).Unit();
	double dip = std::acos(std::clamp(R / dist, -1.0, 1.0));
	auto elevation = [&](float x, float y) {
		Vec3 d = fFwd + fRight * ((x - b.Width() / 2) / fFocal)
			- fUp * ((y - b.Height() / 2) / fFocal);
		return std::asin(std::clamp(d.Unit().Dot(up), -1.0, 1.0));
	};
	rgb_color ground = moon ? rgb_color{118, 116, 110, 255}
		: rgb_color{40, 70, 110, 255};
	const float step = 6;
	for (float x = 0; x <= b.Width(); x += step) {
		double top = elevation(x, -2 * b.Height());
		double bottom = elevation(x, 3 * b.Height());
		bool groundBelow = top > bottom;
		double lo = -2 * b.Height(), hi = 3 * b.Height();
		// Elevation runs monotonically down the column.
		for (int it = 0; it < 30; it++) {
			double mid = (lo + hi) / 2;
			bool above = elevation(x, float(mid)) > -dip;
			if (above == groundBelow)
				lo = mid;
			else
				hi = mid;
		}
		float hy = float((lo + hi) / 2);
		BRect col = groundBelow
			? BRect(x, std::max(0.0f, hy), x + step, b.Height())
			: BRect(x, 0, x + step, std::min(b.Height(), hy));
		if (col.Height() <= 0)
			continue;
		SetHighColor(ground);
		FillRect(col);
	}
	// A softer band toward the horizon (the low Sun's long shadows).
	(void)moon;
}


void
WindowView::_DrawSurface()
{
	// Low over the Moon: the craters in perspective on the ground, nearest
	// last.
	if (!fNearMoon || fAltitude > 150e3)
		return;
	Mat3 toInertial = world::Ephemeris::MoonFixedToInertial(fJD);
	struct Seen { BPoint at; float rx, ry; double dist; };
	std::vector<Seen> seen;
	Vec3 up = (fEye - fMoon).Unit();
	for (const Crater& c : fCraters) {
		Vec3 p = fMoon + toInertial * c.fixed;
		Vec3 d = p - fEye;
		double dist = d.Length();
		if (dist > 250e3)
			continue;
		bool front;
		BPoint at = _Project(p, &front);
		if (!front)
			continue;
		double z = d.Dot(fFwd);
		float rx = float(c.diameter / 2 / z * fFocal);
		if (rx < 0.6f)
			continue;
		// Foreshortened by the angle we see the ground at.
		Vec3 local = (p - fMoon).Unit();
		double sinE = std::fabs((d / dist).Dot(local));
		float ry = rx * float(std::max(0.08, sinE));
		(void)up;
		seen.push_back({at, rx, ry, dist});
	}
	// And the small ones everywhere, from a few octaves of hashed cells
	// laid out round the site (stable as we fly over them).
	{
		Mat3 toFixed = toInertial.Transposed();
		Vec3 eyeFixed = toFixed * (fEye - fMoon);
		Vec3 east = Vec3(0, 0, 1).Cross(fSiteUp).Unit();
		Vec3 north = fSiteUp.Cross(east);
		Vec3 rel = eyeFixed - fSiteUp * fSiteRadius;
		double ex = rel.Dot(east), ny = rel.Dot(north);
		for (double cell : {6.0, 40.0, 260.0, 1700.0}) {
			if (cell < fAltitude * 0.01 || cell > std::max(200.0, fAltitude) * 3)
				continue;
			long ci = long(std::floor(ex / cell)), cj = long(std::floor(ny / cell));
			const int N = 34;
			for (long i = ci - N; i <= ci + N; i++) {
				for (long j = cj - N; j <= cj + N; j++) {
					uint32_t h = uint32_t(i * 73856093L) ^ uint32_t(j * 19349663L)
						^ uint32_t(cell * 83492791.0);
					h = h * 2654435761u;
					if ((h >> 28) > 6)
						continue;          // about 40% of cells
					double fx = ((h >> 4) & 1023) / 1024.0;
					double fy = ((h >> 14) & 1023) / 1024.0;
					double diameter = cell * (0.15 + 0.6 * ((h >> 24) & 15) / 15.0);
					Vec3 local = fSiteUp * fSiteRadius + east * ((i + fx) * cell)
						+ north * ((j + fy) * cell);
					Vec3 p = fMoon + toInertial * (local.Unit() * fSiteRadius);
					Vec3 d = p - fEye;
					double dist = d.Length();
					bool front;
					BPoint at = _Project(p, &front);
					if (!front)
						continue;
					double z = d.Dot(fFwd);
					float rx = float(diameter / 2 / z * fFocal);
					if (rx < 0.8f || at.y < -50 || at.y > Bounds().Height() + 50
							|| at.x < -100 || at.x > Bounds().Width() + 100)
						continue;
					Vec3 lu = (p - fMoon).Unit();
					double sinE = std::fabs((d / dist).Dot(lu));
					seen.push_back({at, rx, rx * float(std::max(0.06, sinE)), dist});
				}
			}
		}
	}
	std::sort(seen.begin(), seen.end(),
		[](const Seen& a, const Seen& b) { return a.dist > b.dist; });
	for (const Seen& s : seen) {
		SetHighColor(96, 95, 90);
		FillEllipse(s.at, s.rx, s.ry);
		SetHighColor(160, 158, 150);
		FillEllipse(BPoint(s.at.x, s.at.y - s.ry * 0.25f), s.rx * 0.9f, s.ry * 0.55f);
		SetHighColor(70, 70, 66);
		FillEllipse(BPoint(s.at.x, s.at.y + s.ry * 0.1f), s.rx * 0.75f, s.ry * 0.6f);
	}
}


void
WindowView::_DrawOther(const Other& o)
{
	double dist = (o.r - fEye).Length();
	if (dist > 400e3)
		return;
	bool front;
	BPoint c = _Project(o.r, &front);
	if (!front)
		return;
	std::vector<std::pair<Vec3, Vec3>> lines;
	if (o.lm)
		LMModel((o.modules & REC_LM_DESCENT) != 0, &lines);
	else
		StackModel(o.modules, &lines);
	// Small: a point of light.
	double z = (o.r - fEye).Dot(fFwd);
	if (12 / z * fFocal < 3) {
		SetHighColor(240, 240, 230);
		FillEllipse(c, 2, 2);
	} else {
		BeginLineArray(int32(lines.size()));
		for (auto& l : lines) {
			bool f1, f2;
			BPoint a = _Project(o.r + o.q.Rotate(l.first), &f1);
			BPoint b = _Project(o.r + o.q.Rotate(l.second), &f2);
			if (f1 && f2)
				AddLine(a, b, o.lm ? rgb_color{214, 180, 96, 255}
					: rgb_color{220, 222, 228, 255});
		}
		EndLineArray();
	}
	char label[64];
	std::snprintf(label, sizeof(label), "%s  %.0f FT", o.name.c_str(),
		dist / 0.3048);
	if (dist > 3000)
		std::snprintf(label, sizeof(label), "%s  %.1f NMI", o.name.c_str(),
			dist / 1852);
	Text(this, label, c.x + 10, c.y - 10, 11, kCyan);
}


void
WindowView::_DrawOverlay()
{
	BRect b = Bounds();
	float cx = b.Width() / 2, cy = b.Height() / 2;
	SetDrawingMode(B_OP_ALPHA);
	if (!fLM) {
		// The COAS: a reticle on the docking axis.
		SetHighColor(255, 200, 80, 200);
		StrokeEllipse(BPoint(cx, cy), 60, 60);
		StrokeLine(BPoint(cx - 90, cy), BPoint(cx - 10, cy));
		StrokeLine(BPoint(cx + 10, cy), BPoint(cx + 90, cy));
		StrokeLine(BPoint(cx, cy - 90), BPoint(cx, cy - 10));
		StrokeLine(BPoint(cx, cy + 10), BPoint(cx, cy + 90));
		for (int k = -4; k <= 4; k++)
			if (k)
				StrokeLine(BPoint(cx - 4, cy + k * 15), BPoint(cx + 4, cy + k * 15));
	} else {
		// The LPD scale: degrees below the LM's +Z axis, on the window.
		SetHighColor(255, 255, 255, 170);
		for (int a = 0; a <= 80; a += 5) {
			double off = (a - 25 - fLookPitch * -1) * kDeg;
			float y = cy + float(std::tan(off) * fFocal);
			if (y < 0 || y > b.Height())
				continue;
			float w = a % 10 == 0 ? 26 : 12;
			StrokeLine(BPoint(cx - 140 - w, y), BPoint(cx - 140, y));
			StrokeLine(BPoint(cx + 140, y), BPoint(cx + 140 + w, y));
			if (a % 10 == 0) {
				char t[8];
				std::snprintf(t, sizeof(t), "%d", a);
				Text(this, t, cx - 172 - w, y + 4, 11, kWhite, false, 2);
			}
		}
		if (fLPDAngle >= 0) {
			double off = (fLPDAngle - 25 + fLookPitch) * kDeg;
			float y = cy + float(std::tan(off) * fFocal);
			SetHighColor(255, 176, 46, 230);
			SetPenSize(2);
			StrokeLine(BPoint(cx - 150, y), BPoint(cx + 150, y));
			SetPenSize(1);
			char t[24];
			std::snprintf(t, sizeof(t), "LPD %02.0f", fLPDAngle);
			Text(this, t, cx + 160, y + 4, 13, kAmber, true);
		}
	}
	SetDrawingMode(B_OP_COPY);
	// The window frame.
	SetHighColor(24, 26, 28);
	if (fLM) {
		// The LM's forward window: a canted triangle-ish shape.
		BPoint left[4] = {BPoint(0, 0), BPoint(b.Width() * 0.18f, 0),
			BPoint(0, b.Height() * 0.5f), BPoint(0, 0)};
		FillPolygon(left, 3);
		BPoint right[3] = {BPoint(b.Width(), 0), BPoint(b.Width() * 0.82f, 0),
			BPoint(b.Width(), b.Height() * 0.5f)};
		FillPolygon(right, 3);
		FillRect(BRect(0, b.Height() - 22, b.Width(), b.Height()));
	} else {
		FillRect(BRect(0, 0, b.Width(), 16));
		FillRect(BRect(0, b.Height() - 16, b.Width(), b.Height()));
	}
	// What we're looking at.
	char line[128];
	std::snprintf(line, sizeof(line), "%s WINDOW", fLM ? "LM FORWARD" : "CM RENDEZVOUS");
	Text(this, line, 14, 34, 12, kDim);
	if (fAltitude < 1e6) {
		std::snprintf(line, sizeof(line), "ALT %.0f FT", fAltitude / 0.3048);
		if (fAltitude > 30000)
			std::snprintf(line, sizeof(line), "ALT %.1f NMI", fAltitude / 1852);
		Text(this, line, b.Width() - 14, 34, 13, kGreen, true, 2);
	}
	if (fRange >= 0 && fRange < 50e3) {
		std::snprintf(line, sizeof(line), "%s RANGE %.0f FT  RATE %+.1f FT/S",
			fTargetName.c_str(), fRange / 0.3048, fRate / 0.3048);
		Text(this, line, b.Width() - 14, 52, 12, kCyan, false, 2);
	}
	if (fLookYaw != 0 || fLookPitch != 0)
		Text(this, "(drag to look around; double-click centres)", 14, 52, 10, kDim);
}


void
WindowView::Draw(BRect)
{
	BRect b = Bounds();
	SetHighColor(0, 0, 0);
	FillRect(b);
	if (!fValid) {
		Text(this, "No flight", b.Width() / 2, b.Height() / 2, 14, kDim, false, 1);
		return;
	}
	fFocal = b.Width() / 2 / std::tan(35 * kDeg);   // 70 degree field
	_DrawStars();
	// Farther body first.
	double dEarth = fEye.Length(), dMoon = (fEye - fMoon).Length();
	if (dEarth > dMoon) {
		_DrawBody(false);
		_DrawBody(true);
	} else {
		_DrawBody(true);
		_DrawBody(false);
	}
	_DrawSurface();
	std::vector<const Other*> order;
	for (const Other& o : fOthers)
		order.push_back(&o);
	std::sort(order.begin(), order.end(), [this](const Other* a, const Other* c) {
		return (a->r - fEye).Length() > (c->r - fEye).Length();
	});
	for (const Other* o : order)
		_DrawOther(*o);
	_DrawOverlay();
}

}  // namespace columbia
