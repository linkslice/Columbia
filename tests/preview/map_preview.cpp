// Columbia — renders the map view from a live flight session to PNG through
// the Cairo shim (the README's screenshot).
// Usage: map_preview <rope-dir> <out.png> [mission] [GET hours]
// Flies from launch with the automatic crew to the given mission time.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "FlightSession.h"
#include "MapView.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

using namespace columbia;
using namespace flight;

int
main(int argc, char** argv)
{
	std::string ropes = argc > 1 ? argv[1] : "rope";
	std::string out = argc > 2 ? argv[2] : "map.png";
	int mission = argc > 3 ? std::atoi(argv[3]) : 11;
	double hours = argc > 4 ? std::atof(argv[4]) : 37;

	FlightOptions o;
	o.mission = *FindMission(mission);
	o.start = StartPoint::Launch;
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
		if (s.Sim().GET() > hours * 3600 || s.Over())
			break;
		if (std::chrono::steady_clock::now() - t0 > std::chrono::minutes(20))
			break;
	}
	s.SetWarp(1);
	s.SetPaused(true);

	MapView v(&s);
	cairo_surface_t* surf = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 1280, 800);
	cairo_t* cr = cairo_create(surf);
	v.SetContext(cr);
	v.ResizeTo(1279, 799);
	v.Refresh();
	v.Draw(v.Bounds());
	cairo_surface_write_to_png(surf, out.c_str());
	cairo_destroy(cr);
	cairo_surface_destroy(surf);
	std::printf("wrote %s at GET %.0f s\n", out.c_str(), s.Sim().GET());
	s.Stop();
	return 0;
}
