// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Drawing shared by the replay, the map and the window views: the Mission
// Control palette, text, spacecraft silhouettes, and the Earth and Moon as
// globes (orthographic, with coastlines, maria, grids and the night side).

#ifndef COLUMBIA_UI_DRAWING_H
#define COLUMBIA_UI_DRAWING_H

#include <View.h>

#include "sim/Math.h"
#include "sim/World.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ui {

using sim::Vec3;

// --- The look: MOCR projection screens ------------------------------------------
const rgb_color kBack = {6, 9, 16, 255};
const rgb_color kPanelBack = {12, 17, 27, 255};
const rgb_color kGrid = {28, 44, 62, 255};
const rgb_color kGridBright = {48, 72, 98, 255};
const rgb_color kAmber = {255, 176, 46, 255};
const rgb_color kGreen = {110, 245, 120, 255};
const rgb_color kCyan = {96, 206, 255, 255};
const rgb_color kWhite = {226, 228, 220, 255};
const rgb_color kDim = {120, 132, 140, 255};
const rgb_color kRed = {255, 84, 70, 255};
const rgb_color kLand = {70, 120, 84, 255};
const rgb_color kOcean = {14, 30, 58, 255};
const rgb_color kMoonGray = {128, 126, 120, 255};
const rgb_color kMare = {78, 78, 76, 255};

const double kPi = 3.14159265358979323846;
const double kDeg = kPi / 180;

rgb_color Alpha(rgb_color c, double a);
// align: 0 left, 1 centred, 2 right. Fixed-width face unless bold.
void Text(BView* v, const char* s, float x, float y, float size, rgb_color c,
	bool bold = false, int align = 0);
std::string FormatGET(double get);
std::string Fmt(const char* format, double value);
void PanelFrame(BView* v, BRect r, const char* title);

// --- Spacecraft ----------------------------------------------------------------
// A part of a stack along the body X axis: from x0 to x1 (m), radius r0
// at x0 and r1 at x1.
struct Part {
	float x0, x1, r0, r1;
	rgb_color color;
};

// `angle`: screen direction of body +X (radians, 0 right, counter-
// clockwise up); `scale`: pixels per metre.
void DrawParts(BView* v, BPoint at, float angle, float scale, const Part* parts,
	int count, float originX);
// The parts of a stack of flight::RecModule bits, CM apex toward +X.
int BuildStack(uint16_t modules, Part* out, float* length);
void DrawStack(BView* v, BPoint at, float angle, float scale, uint16_t modules);
// The LM standing on +X.
void DrawLM(BView* v, BPoint at, float angle, float scale, bool descent,
	bool flame);

// --- Globes ------------------------------------------------------------------
struct Globe {
	BPoint center;
	float radius;
	Vec3 fwd, east, north;      // view basis, in the body-fixed frame

	// Seen from straight above a body-fixed direction (north up), or with
	// the view basis given (fwd toward the viewer, east right, north up).
	Globe(BPoint c, float r, const Vec3& f, const Vec3& e, const Vec3& n)
		:
		center(c), radius(r), fwd(f), east(e), north(n)
	{
	}
	Globe(BPoint c, float r, double latDeg, double lonDeg)
		:
		center(c), radius(r)
	{
		fwd = sim::world::LatLonToVector(latDeg, lonDeg, 1.0);
		Vec3 z(0, 0, 1);
		east = z.Cross(fwd);
		if (east.Length() < 1e-6)
			east = Vec3(0, 1, 0);
		east = east.Unit();
		north = fwd.Cross(east);
	}
	bool Visible(const Vec3& p) const { return p.Dot(fwd) > 0; }
	BPoint Project(const Vec3& p) const
	{
		Vec3 u = p.Unit();
		return BPoint(center.x + float(u.Dot(east)) * radius,
			center.y - float(u.Dot(north)) * radius);
	}
	// A point on the limb in the direction of a hidden point.
	BPoint Limb(const Vec3& p) const
	{
		double x = p.Dot(east), y = p.Dot(north);
		double l = std::hypot(x, y);
		if (l < 1e-9)
			return center;
		return BPoint(center.x + float(x / l) * radius,
			center.y - float(y / l) * radius);
	}
};

void GlobeLine(BView* v, const Globe& g, const std::vector<Vec3>& pts,
	rgb_color c);
void GlobeGrid(BView* v, const Globe& g, rgb_color c, int step);
void GlobeNight(BView* v, const Globe& g, const Vec3& sunFixed, rgb_color c);
void DrawEarth(BView* v, const Globe& g, double jd);
void DrawMoon(BView* v, const Globe& g, double jd, int mission);

// --- Plots -------------------------------------------------------------------
// Scale: fit [x0, x1] x [y0, y1] (world) into r, y up.
struct Plot {
	BRect r;
	double x0, x1, y0, y1;
	BPoint P(double x, double y) const
	{
		return BPoint(r.left + float((x - x0) / (x1 - x0)) * r.Width(),
			r.bottom - float((y - y0) / (y1 - y0)) * r.Height());
	}
	bool Inside(double x, double y) const
	{
		return x >= x0 && x <= x1 && y >= y0 && y <= y1;
	}
};

void PlotGrid(BView* v, const Plot& p, double dx, double dy, const char* fx,
	const char* fy, double kx = 1, double ky = 1);

double AngleBetween(const Vec3& a, const Vec3& b);
// The velocity of the Moon's surface (or anything turning with it) at
// `rel` from its centre.
Vec3 MoonSurfaceVelocity(double jd, const Vec3& rel);

}  // namespace ui

#endif
