// Columbia — renders the real DSKYView and TrajectoryView code to PNG files
// through the Cairo shim, so the drawing can be checked off-Haiku.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "DSKYView.h"
#include "TrajectoryView.h"
#include "Mission.h"

#include <cstdio>
#include <string>

using columbia::Mission;

static void
Run(Mission& m, double seconds)
{
	for (int i = 0; i < int(seconds * 30); i++)
		m.Update(1.0 / 30);
	m.TakeEvents();
}

template<class V>
static void
Render(V& view, float w, float h, const std::string& path)
{
	cairo_surface_t* s = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
		int(w) + 1, int(h) + 1);
	cairo_t* cr = cairo_create(s);
	view.SetContext(cr);
	view.Draw(view.Bounds());
	cairo_surface_write_to_png(s, path.c_str());
	cairo_destroy(cr);
	cairo_surface_destroy(s);
	std::printf("wrote %s\n", path.c_str());
}

int
main(int argc, char** argv)
{
	std::string out = argc > 1 ? argv[1] : ".";
	Mission m;
	Run(m, 8);

	DSKYView dsky(&m);
	dsky.ResizeTo(DSKYView::kWidth, DSKYView::kHeight);
	m.QueueKeys("V16N65E");
	Run(m, 4);
	dsky.Refresh();
	Render(dsky, DSKYView::kWidth, DSKYView::kHeight, out + "/dsky-clock.png");

	m.QueueKeys("V35E");
	Run(m, 2.5);
	dsky.Refresh();
	Render(dsky, DSKYView::kWidth, DSKYView::kHeight, out + "/dsky-lamptest.png");
	Run(m, 6);
	m.QueueKeys("RV37E00E");
	Run(m, 3);

	sim::PlanResult tli = m.PlanNow(Mission::PLAN_TLI);
	std::printf("TLI %s\n", tli.message.c_str());
	m.SetWarp(1000);
	while (m.Time() < tli.burn.time + 20 * 3600)
		m.Update(1.0 / 30);

	TrajectoryView traj(&m);
	traj.ResizeTo(900, 620);
	traj.AttachedToWindow();
	traj.Refresh();
	Render(traj, 900, 620, out + "/path-rotating.png");
	traj.SetFrameMode(FRAME_INERTIAL);
	Render(traj, 900, 620, out + "/path-inertial.png");

	m.SetWarp(10000);
	while (m.Time() < tli.burn.time + 60 * 3600)
		m.Update(1.0 / 30);
	sim::PlanResult loi = m.PlanNow(Mission::PLAN_LOI);
	std::printf("LOI %s\n", loi.message.c_str());
	while (m.Time() < loi.burn.time + 5 * 3600)
		m.Update(1.0 / 30);
	traj.SetFrameMode(FRAME_MOON);
	Render(traj, 900, 620, out + "/path-moon.png");
	return 0;
}
