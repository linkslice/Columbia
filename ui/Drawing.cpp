// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Drawing.h"

#include <InterfaceDefs.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "Coastline.h"
#include "flight/Recording.h"
#include "sim/World.h"

namespace ui {

using namespace flight;
using sim::Mat3;
using sim::Quat;
using sim::Vec3;
namespace world = sim::world;





rgb_color
Alpha(rgb_color c, double a)
{
	c.alpha = uint8(std::clamp(a, 0.0, 1.0) * 255);
	return c;
}

void
Text(BView* v, const char* s, float x, float y, float size, rgb_color c,
	bool bold, int align)
{
	BFont font(bold ? be_bold_font : be_fixed_font);
	font.SetSize(size);
	v->SetFont(&font);
	v->SetHighColor(c);
	float w = v->StringWidth(s);
	if (align == 1)
		x -= w / 2;
	else if (align == 2)
		x -= w;
	v->DrawString(s, BPoint(x, y));
}

std::string
FormatGET(double get)
{
	char b[32];
	long t = long(std::floor(std::fabs(get)));
	std::snprintf(b, sizeof(b), "%s%03ld:%02ld:%02ld", get < 0 ? "-" : "",
		t / 3600, t / 60 % 60, t % 60);
	return b;
}

std::string
Fmt(const char* format, double value)
{
	char b[64];
	std::snprintf(b, sizeof(b), format, value);
	return b;
}

void
PanelFrame(BView* v, BRect r, const char* title)
{
	v->SetHighColor(kPanelBack);
	v->FillRect(r);
	v->SetHighColor(kGridBright);
	v->StrokeRect(r);
	if (title) {
		Text(v, title, r.left + 10, r.top + 18, 12, kDim);
	}
}

// --- Spacecraft silhouettes ---------------------------------------------------

// A part of a stack along the body X axis: from x0 to x1 (m), radius r0
// at x0 and r1 at x1.

void
DrawParts(BView* v, BPoint at, float angle, float scale, const Part* parts,
	int count, float originX)
{
	// `angle`: direction of body +X on screen (radians, 0 = right,
	// counter-clockwise = up).
	float ca = std::cos(angle), sa = std::sin(angle);
	auto P = [&](float x, float y) {
		x -= originX;
		return BPoint(at.x + (x * ca - y * sa) * scale,
			at.y - (x * sa + y * ca) * scale);
	};
	for (int i = 0; i < count; i++) {
		const Part& p = parts[i];
		BPoint q[4] = {P(p.x0, -p.r0), P(p.x1, -p.r1), P(p.x1, p.r1),
			P(p.x0, p.r0)};
		v->SetHighColor(p.color);
		v->FillPolygon(q, 4);
		v->SetHighColor(tint_color(p.color, B_DARKEN_2_TINT));
		v->StrokeLine(q[0], q[1]);
		v->StrokeLine(q[2], q[3]);
	}
}

// The stack for a set of modules, CM apex toward +X. Returns the length.
int
BuildStack(uint16_t m, Part* out, float* length)
{
	const rgb_color white = {225, 225, 218, 255};
	const rgb_color black = {40, 40, 42, 255};
	const rgb_color metal = {180, 182, 188, 255};
	const rgb_color gold = {200, 160, 70, 255};
	const rgb_color lmGray = {150, 150, 140, 255};
	int n = 0;
	float x = 0;
	auto add = [&](float len, float r0, float r1, rgb_color c) {
		out[n++] = {x, x + len, r0, r1, c};
		x += len;
	};
	if (m & REC_SIC)
		add(42, 5.0f, 5.0f, white);
	if (m & REC_SIB)
		add(25, 3.3f, 3.3f, white);
	if (m & REC_SII) {
		if (m & REC_SIC)
			add(5.5f, 5.0f, 5.0f, black);
		add(19.4f, 5.0f, 5.0f, white);
		add(3.5f, 5.0f, 3.3f, white);
	}
	bool launch = (m & (REC_SIC | REC_SIB | REC_SII)) != 0;
	if (m & REC_SIVB) {
		add(17.8f, 3.3f, 3.3f, white);
		if (launch || (m & (REC_SM | REC_CM)) == 0)
			add(1.0f, 3.3f, 3.3f, black);  // the IU
	}
	if (launch && (m & REC_SM))
		add(8.5f, 3.3f, 1.95f, white);     // SLA with the LM inside
	if (!launch && (m & REC_SIVB) && (m & REC_LM_DESCENT) && !(m & REC_SM)) {
		// The S-IVB with the LM on top, panels gone.
		add(3.2f, 2.1f, 2.1f, gold);
		add(3.5f, 2.0f, 1.6f, lmGray);
	}
	bool lmBelow = !launch && (m & REC_LM_DESCENT) && (m & REC_SM)
		&& !(m & REC_SIVB);
	if (m & REC_SM) {
		if (!launch)
			add(2.8f, 0.9f, 0.6f, black);  // SPS nozzle
		add(7.5f, 1.95f, 1.95f, metal);
	}
	if (m & REC_CM)
		add(3.2f, 1.95f, 0.4f, metal);
	if (m & REC_TOWER)
		add(10.0f, 0.35f, 0.15f, {200, 60, 40, 255});
	if (lmBelow) {
		// Docked LM ahead of the CM apex (head to head).
		add(0.6f, 0.4f, 0.4f, metal);
		add(3.8f, 2.1f, 2.1f, lmGray);
		add(3.2f, 2.1f, 2.1f, gold);
	}
	*length = x;
	return n;
}

void
DrawStack(BView* v, BPoint at, float angle, float scale, uint16_t modules)
{
	Part parts[24];
	float length;
	int n = BuildStack(modules, parts, &length);
	if (n == 0)
		return;
	DrawParts(v, at, angle, scale, parts, n, length / 2);
}

// The LM standing on +X (descent stage with legs, ascent stage above).
void
DrawLM(BView* v, BPoint at, float angle, float scale, bool descent,
	bool flame)
{
	const rgb_color gold = {200, 160, 70, 255};
	const rgb_color gray = {160, 160, 150, 255};
	float ca = std::cos(angle), sa = std::sin(angle);
	auto P = [&](float x, float y) {
		return BPoint(at.x + (x * ca - y * sa) * scale,
			at.y - (x * sa + y * ca) * scale);
	};
	auto quad = [&](float x0, float x1, float y0, float y1, rgb_color c) {
		BPoint q[4] = {P(x0, y0), P(x1, y0), P(x1, y1), P(x0, y1)};
		v->SetHighColor(c);
		v->FillPolygon(q, 4);
	};
	float base = descent ? -3.4f : -0.4f;
	if (flame) {
		BPoint f[3] = {P(base, -0.7f), P(base, 0.7f),
			P(base - 3.0f - 1.5f * float(std::fmod(at.x * 0.37, 1.0)), 0)};
		v->SetHighColor(255, 200, 120, 200);
		v->SetDrawingMode(B_OP_ALPHA);
		v->FillPolygon(f, 3);
		v->SetDrawingMode(B_OP_COPY);
	}
	if (descent) {
		quad(-2.6f, -0.2f, -2.1f, 2.1f, gold);
		v->SetHighColor(gold);
		for (int s : {-1, 1}) {
			v->StrokeLine(P(-1.0f, 2.1f * s), P(-3.4f, 4.3f * s));
			v->StrokeLine(P(-2.4f, 2.1f * s), P(-3.4f, 4.3f * s));
			v->StrokeLine(P(-3.4f, 3.9f * s), P(-3.4f, 4.7f * s));
		}
	}
	quad(-0.2f, 2.6f, -2.0f, 1.8f, gray);
	quad(2.6f, 3.4f, -1.1f, 0.8f, gray);
	v->SetHighColor(30, 30, 34, 255);
	v->StrokeLine(P(1.4f, 1.0f), P(2.2f, 1.6f));
}

// --- Spheres in orthographic projection ------------------------------------


// Draws a polyline on the globe (fixed-frame points), skipping the far side.
void
GlobeLine(BView* v, const Globe& g, const std::vector<Vec3>& pts,
	rgb_color c)
{
	if (pts.size() < 2)
		return;
	v->SetDrawingMode(B_OP_ALPHA);
	v->BeginLineArray(int32(pts.size()));
	for (size_t i = 1; i < pts.size(); i++) {
		if (!g.Visible(pts[i - 1]) || !g.Visible(pts[i]))
			continue;
		v->AddLine(g.Project(pts[i - 1]), g.Project(pts[i]), c);
	}
	v->EndLineArray();
	v->SetDrawingMode(B_OP_COPY);
}

void
GlobeGrid(BView* v, const Globe& g, rgb_color c, int step)
{
	std::vector<Vec3> line;
	for (int lat = -90 + step; lat < 90; lat += step) {
		line.clear();
		for (int lon = -180; lon <= 180; lon += 5)
			line.push_back(world::LatLonToVector(lat, lon, 1.0));
		GlobeLine(v, g, line, c);
	}
	for (int lon = -180; lon < 180; lon += step) {
		line.clear();
		for (int lat = -90; lat <= 90; lat += 5)
			line.push_back(world::LatLonToVector(lat, lon, 1.0));
		GlobeLine(v, g, line, c);
	}
}

// The night side as seen: the visible arc of the terminator closed along
// the dark part of the limb, filled once.
void
GlobeNight(BView* v, const Globe& g, const Vec3& sunFixed, rgb_color c)
{
	Vec3 s = sunFixed.Unit();
	Vec3 a = s.Cross(Vec3(0, 0, 1));
	if (a.Length() < 1e-6)
		a = s.Cross(Vec3(1, 0, 0));
	a = a.Unit();
	Vec3 b = s.Cross(a);
	const int N = 180;
	std::vector<Vec3> ring(N);
	std::vector<bool> vis(N);
	int visible = 0;
	for (int i = 0; i < N; i++) {
		double t = 2 * kPi * i / N;
		ring[i] = a * std::cos(t) + b * std::sin(t);
		vis[i] = g.Visible(ring[i]);
		visible += vis[i];
	}
	bool antiVisible = g.Visible(-s);
	std::vector<BPoint> poly;
	auto limbAngle = [&](const Vec3& p) {
		return std::atan2(p.Dot(g.north), p.Dot(g.east));
	};
	auto limbPoint = [&](double th) {
		return BPoint(g.center.x + float(std::cos(th)) * g.radius,
			g.center.y - float(std::sin(th)) * g.radius);
	};
	if (visible == 0) {
		if (!antiVisible)
			return;
		for (int i = 0; i < 72; i++)
			poly.push_back(limbPoint(2 * kPi * i / 72));
	} else if (visible == N) {
		if (!antiVisible)
			return;     // (a thin crescent of night on the far rim; skip)
		for (int i = 0; i < N; i++)
			poly.push_back(g.Project(ring[i]));
	} else {
		// The visible run, starting just after a hidden sample.
		int start = 0;
		for (int i = 0; i < N; i++)
			if (vis[i] && !vis[(i + N - 1) % N]) {
				start = i;
				break;
			}
		int i = start;
		while (vis[i % N] && int(poly.size()) < N) {
			poly.push_back(g.Project(ring[i % N]));
			i++;
		}
		double th0 = limbAngle(ring[(i - 1) % N]);     // end of the arc
		double th1 = limbAngle(ring[start]);           // back to the start
		// Go round the limb from th0 to th1 the way that's dark.
		double sweep = th1 - th0;
		while (sweep <= 0)
			sweep += 2 * kPi;
		double mid = th0 + sweep / 2;
		Vec3 midP = g.east * std::cos(mid) + g.north * std::sin(mid);
		if (midP.Dot(s) > 0)
			sweep -= 2 * kPi;       // the other way
		const int L = 60;
		for (int k = 0; k <= L; k++)
			poly.push_back(limbPoint(th0 + sweep * k / L));
	}
	v->SetDrawingMode(B_OP_ALPHA);
	v->SetHighColor(c);
	v->FillPolygon(poly.data(), int32(poly.size()));
	v->SetDrawingMode(B_OP_COPY);
}


void
DrawEarth(BView* v, const Globe& g, double jd)
{
	v->SetHighColor(kOcean);
	v->FillEllipse(g.center, g.radius, g.radius);
	// Coastlines.
	std::vector<Vec3> line;
	for (int i = 0; i < kCoastLineCount; i++) {
		line.clear();
		for (int k = kCoastLineStarts[i]; k < kCoastLineStarts[i + 1]; k++)
			line.push_back(world::LatLonToVector(kCoastPoints[k][0] / 100.0,
				kCoastPoints[k][1] / 100.0, 1.0));
		GlobeLine(v, g, line, kLand);
	}
	GlobeGrid(v, g, Alpha(kGrid, 0.9), 30);
	Mat3 toFixed = world::Ephemeris::EarthFixedToInertial(jd).Transposed();
	Vec3 sun = toFixed * world::Ephemeris::SunPosition(jd);
	GlobeNight(v, g, sun, {0, 0, 0, 120});
	v->SetHighColor(kGridBright);
	v->StrokeEllipse(g.center, g.radius, g.radius);
}

namespace {

struct Mare { double lat, lon, radius; };
const Mare kMaria[] = {
	{32.8, -15.6, 17}, {28.0, 17.5, 10.5}, {8.5, 31.4, 13}, {17.0, 59.1, 8.5},
	{-7.8, 51.3, 12}, {-15.2, 35.5, 5.5}, {-21.3, -16.6, 11}, {-24.4, -38.6, 6.5},
	{18.4, -57.4, 20}, {-2, -40, 14}, {56.0, 1.4, 6}, {13.3, 3.6, 4},
	{-10.0, -23.1, 6}, {20, -30, 9},
	// The far side.
	{27.3, 147.9, 4}, {-19.4, -92.8, 8}, {-33.7, 163.5, 3}, {-1.3, 87.3, 5},
	{13.3, 86.1, 4}, {-38.9, 93.0, 6}, {-21.2, 128.9, 2.5}
};

struct SiteMark { int mission; double lat, lon; };
const SiteMark kSites[] = {
	{11, 0.674, 23.473}, {12, -3.012, -23.422}, {14, -3.645, -17.471},
	{15, 26.132, 3.634}, {16, -8.973, 15.500}, {17, 20.191, 30.772}
};

}  // namespace


void
DrawMoon(BView* v, const Globe& g, double jd, int mission)
{
	v->SetHighColor(kMoonGray);
	v->FillEllipse(g.center, g.radius, g.radius);
	// The maria, as dark patches.
	v->SetHighColor(kMare);
	for (const Mare& m : kMaria) {
		Vec3 c = world::LatLonToVector(m.lat, m.lon, 1.0);
		if (c.Dot(g.fwd) < -0.3)
			continue;
		Vec3 a = c.Cross(Vec3(0, 0, 1));
		if (a.Length() < 1e-6)
			a = Vec3(1, 0, 0);
		a = a.Unit();
		Vec3 b = c.Cross(a);
		double rr = m.radius * kDeg;
		std::vector<BPoint> poly;
		for (int i = 0; i < 36; i++) {
			double t = 2 * kPi * i / 36;
			Vec3 p = c * std::cos(rr) + (a * std::cos(t) + b * std::sin(t))
				* std::sin(rr);
			poly.push_back(g.Visible(p) ? g.Project(p) : g.Limb(p));
		}
		v->FillPolygon(poly.data(), int32(poly.size()));
	}
	GlobeGrid(v, g, Alpha({60, 60, 60, 255}, 1.0), 30);
	Mat3 toFixed = world::Ephemeris::MoonFixedToInertial(jd).Transposed();
	Vec3 moon = world::Ephemeris::MoonPosition(jd);
	Vec3 sun = toFixed * (world::Ephemeris::SunPosition(jd) - moon);
	GlobeNight(v, g, sun, {0, 0, 0, 150});
	// Apollo landing sites.
	for (const SiteMark& s : kSites) {
		Vec3 p = world::LatLonToVector(s.lat, s.lon, 1.0);
		if (!g.Visible(p))
			continue;
		BPoint q = g.Project(p);
		bool ours = s.mission == mission;
		v->SetHighColor(ours ? kAmber : kWhite);
		float k = ours ? 5 : 3;
		v->StrokeLine(BPoint(q.x - k, q.y), BPoint(q.x + k, q.y));
		v->StrokeLine(BPoint(q.x, q.y - k), BPoint(q.x, q.y + k));
		char label[8];
		std::snprintf(label, sizeof(label), "%d", s.mission);
		Text(v, label, q.x + k + 2, q.y - 2, ours ? 12 : 9,
			ours ? kAmber : kDim);
	}
	v->SetHighColor(kGridBright);
	v->StrokeEllipse(g.center, g.radius, g.radius);
}

// Scale: fit [x0, x1] x [y0, y1] (world) into r with margins, y up.

void
PlotGrid(BView* v, const Plot& p, double dx, double dy, const char* fx,
	const char* fy, double kx, double ky)
{
	v->SetHighColor(kGrid);
	for (double x = std::ceil(p.x0 / dx) * dx; x <= p.x1; x += dx) {
		v->StrokeLine(p.P(x, p.y0), p.P(x, p.y1));
		std::string s = Fmt(fx, x * kx);
		BPoint q = p.P(x, p.y0);
		Text(v, s.c_str(), q.x, q.y + 14, 10, kDim, false, 1);
		v->SetHighColor(kGrid);
	}
	for (double y = std::ceil(p.y0 / dy) * dy; y <= p.y1; y += dy) {
		v->StrokeLine(p.P(p.x0, y), p.P(p.x1, y));
		std::string s = Fmt(fy, y * ky);
		BPoint q = p.P(p.x0, y);
		Text(v, s.c_str(), q.x - 6, q.y + 4, 10, kDim, false, 2);
		v->SetHighColor(kGrid);
	}
	v->SetHighColor(kGridBright);
	v->StrokeRect(p.r);
}

double
AngleBetween(const Vec3& a, const Vec3& b)
{
	return std::atan2(a.Cross(b).Length(), a.Dot(b));
}

// The velocity of the lunar surface (or anything turning with the Moon)
// at `rel` from its centre.
Vec3
MoonSurfaceVelocity(double jd, const Vec3& rel)
{
	Vec3 axis = world::Ephemeris::MoonFixedToInertial(jd).Column(2);
	return (axis * (2 * kPi / (27.321661 * 86400))).Cross(rel);
}


}  // namespace ui
