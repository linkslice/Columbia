// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Draws a flight recording the way the Mission Operations Control Room
// walls showed a flight: the launch and entry profiles, the world map
// with the ground track, the Earth-Moon plot, the lunar landing plot and
// the rendezvous plot, with the DSKYs and the air-to-ground loop alongside.
// Used by the screensaver and the app's replay window.

#ifndef COLUMBIA_UI_REPLAY_RENDERER_H
#define COLUMBIA_UI_REPLAY_RENDERER_H

#include <View.h>

#include "flight/Recording.h"
#include "sim/Math.h"

#include <map>
#include <string>
#include <vector>

namespace ui {

class DSKYPainter;

enum class Scene {
	Title, Launch, EarthOrbit, Cislunar, MoonOrbit, Landing, Surface,
	Ascent, Rendezvous, Docking, Entry, Splashdown, Lost
};

class ReplayRenderer {
public:
	explicit ReplayRenderer(const flight::Recording& recording);
	~ReplayRenderer();

	bool showDSKY = true;
	bool showCaptions = true;
	bool showTitle = true;

	// Draw the whole frame. `get`: mission time shown; `speed`: replay
	// speed (for the readout); `clock`: replay seconds since the start
	// (fades, flashing).
	void Draw(BView* view, BRect bounds, double get, double speed,
		double clock);

	Scene SceneAt(double get) const;

private:
	struct Frame;      // per-draw context

	void _DrawTopBar(BView* v, const Frame& f, BRect r);
	void _DrawCaptions(BView* v, const Frame& f, BRect r);
	void _DrawSidePanel(BView* v, const Frame& f, BRect r);
	void _DrawReadouts(BView* v, const Frame& f, BRect r);

	void _DrawTitle(BView* v, const Frame& f, BRect r, double fade);
	void _DrawLaunch(BView* v, const Frame& f, BRect r);
	void _DrawEarthOrbit(BView* v, const Frame& f, BRect r);
	void _DrawCislunar(BView* v, const Frame& f, BRect r);
	void _DrawMoonOrbit(BView* v, const Frame& f, BRect r);
	void _DrawLanding(BView* v, const Frame& f, BRect r, bool ascent);
	void _DrawSurface(BView* v, const Frame& f, BRect r);
	void _DrawRendezvous(BView* v, const Frame& f, BRect r);
	void _DrawDocking(BView* v, const Frame& f, BRect r);
	void _DrawEntry(BView* v, const Frame& f, BRect r);
	void _DrawEnd(BView* v, const Frame& f, BRect r, bool lost);

	void _Precompute();
	int _Crewed(const flight::RecSample& s) const;

	const flight::Recording& fRec;
	DSKYPainter* fCM;
	DSKYPainter* fLM;
	// The crewed spacecraft's path in the Earth-Moon rotating frame (km).
	struct MapPoint { double get; float x, y; };
	std::vector<MapPoint> fMapPath;
	// Burns, for the map's markers.
	struct Marker { double get; std::string label; };
	std::vector<Marker> fMarkers;
	double fLiftoffGET = 0;
	sim::Vec3 fPadFixed;          // launch pad, Earth-fixed unit vector
	// Captions on screen: event index -> replay clock when it appeared.
	std::map<size_t, double> fCaptionShown;
	double fLastGET = -1e30;
};

}  // namespace ui

#endif
