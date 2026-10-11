// Columbia — renders frames of a flight replay (the screensaver's drawing)
// to PNG through the Cairo shim.
// Usage: replay_preview <file.colrec> <outdir> [width height] [minutes]
//        [--at GET ...]   frames at those mission times (else one per scene)
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "ReplayPacer.h"
#include "ReplayRenderer.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace flight;

int
main(int argc, char** argv)
{
	if (argc < 3) {
		std::fprintf(stderr, "usage: %s file.colrec outdir [w h] [minutes] [--at GET...]\n",
			argv[0]);
		return 2;
	}
	Recording rec;
	std::string err;
	if (!rec.Load(argv[1], &err)) {
		std::fprintf(stderr, "%s\n", err.c_str());
		return 1;
	}
	std::string out = argv[2];
	float w = 1280, h = 720;
	double minutes = 25;
	std::vector<double> at;
	for (int i = 3; i < argc; i++) {
		if (std::strcmp(argv[i], "--at") == 0) {
			while (i + 1 < argc)
				at.push_back(std::atof(argv[++i]));
		} else if (i + 1 < argc && std::isdigit(argv[i][0]) && w == 1280) {
			w = std::atof(argv[i]);
			h = std::atof(argv[++i]);
		} else
			minutes = std::atof(argv[i]);
	}
	ui::ReplayPacer pacer;
	pacer.Build(rec, minutes);
	ui::ReplayRenderer renderer(rec);
	if (at.empty()) {
		// One frame in the middle of each stretch of a scene.
		ui::Scene last = ui::Scene::Title;
		double start = 0;
		for (double t = 0; t <= pacer.TotalSeconds(); t += 1) {
			ui::Scene s = renderer.SceneAt(pacer.GETAt(t));
			if (s != last || t + 1 > pacer.TotalSeconds()) {
				if (t - start > 2)
					at.push_back(pacer.GETAt((start + t) / 2));
				start = t;
				last = s;
			}
		}
	}
	int n = 0;
	for (double get : at) {
		cairo_surface_t* s = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
			int(w), int(h));
		cairo_t* cr = cairo_create(s);
		BView view(BRect(0, 0, w - 1, h - 1), "replay", 0, 0);
		view.SetContext(cr);
		double clock = pacer.SecondsAt(get);
		renderer.Draw(&view, view.Bounds(), get, pacer.SpeedAt(clock), clock + 20);
		char path[512];
		std::snprintf(path, sizeof(path), "%s/frame%02d.png", out.c_str(), n++);
		cairo_surface_write_to_png(s, path);
		cairo_destroy(cr);
		cairo_surface_destroy(s);
		std::printf("%s  GET %.0f  replay %.1f min  scene %d\n", path, get,
			clock / 60, int(renderer.SceneAt(get)));
	}
	return 0;
}
