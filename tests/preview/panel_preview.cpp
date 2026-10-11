// Columbia — renders the controls view (CM and LM panels) from a live
// flight session to PNG through the Cairo shim.
// Usage: panel_preview <rope-dir> <outdir>
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "FlightSession.h"
#include "PanelView.h"
#include "WindowView.h"

#include <chrono>
#include <cstdio>
#include <thread>

using namespace columbia;
using namespace flight;

static void
Render(PanelView& v, const std::string& path, float w, float h)
{
	cairo_surface_t* s = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, int(w), int(h));
	cairo_t* cr = cairo_create(s);
	v.SetContext(cr);
	v.ResizeTo(w - 1, h - 1);
	v.Refresh();
	v.Draw(v.Bounds());
	cairo_surface_write_to_png(s, path.c_str());
	cairo_destroy(cr);
	cairo_surface_destroy(s);
	std::printf("wrote %s\n", path.c_str());
}

int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";
	std::string out = argc > 2 ? argv[2] : ".";
	struct Case { StartPoint start; double until; const char* name; };
	const MissionSpec& m11 = *FindMission(11);
	Case cases[] = {
		{StartPoint::Launch, 90, "panel_cm_launch"},
		{StartPoint::PoweredDescent, m11.plan.pdi + 640, "panel_lm_descent"},
		{StartPoint::EarthOrbit, m11.plan.tli + 1380, "panel_cm_docking"},
	};
	for (const Case& c : cases) {
		FlightOptions o;
		o.mission = *FindMission(11);
		o.start = c.start;
		o.autopilot = true;
		o.difficulty = Difficulty::AsFlown;
		o.ropes = ropes;
		FlightSession s;
		std::string err;
		if (!s.Start(o, &err)) {
			std::printf("%s\n", err.c_str());
			return 1;
		}
		s.SetWarp(100000);
		auto t0 = std::chrono::steady_clock::now();
		while (true) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			std::lock_guard<std::recursive_mutex> g(s.Lock());
			if (s.Sim().GET() > c.until || s.Over())
				break;
			if (std::chrono::steady_clock::now() - t0 > std::chrono::minutes(6))
				break;
		}
		s.SetPaused(true);
		PanelView v(&s);
		v.rotation = sim::Vec3(0, 0.4, -0.3);
		v.translation[0] = 1;
		Render(v, out + "/" + c.name + ".png", 1280, 720);
		{
			WindowView w(&s);
			cairo_surface_t* surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 1280, 720);
			cairo_t* cr = cairo_create(surf);
			w.SetContext(cr);
			w.ResizeTo(1279, 719);
			w.Refresh();
			w.Draw(w.Bounds());
			std::string path = out + "/" + c.name + "_window.png";
			cairo_surface_write_to_png(surf, path.c_str());
			cairo_destroy(cr);
			cairo_surface_destroy(surf);
			std::printf("wrote %s\n", path.c_str());
		}
		s.Stop();
	}
	return 0;
}
