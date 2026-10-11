// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "MapView.h"

#include <OS.h>

#include <mutex>

#include "FlightSession.h"
#include "ReplayRenderer.h"

namespace columbia {

MapView::MapView(FlightSession* session)
	:
	BView("map", B_WILL_DRAW | B_FRAME_EVENTS),
	fSession(session),
	fStart(system_time())
{
	SetViewColor(B_TRANSPARENT_COLOR);
}


MapView::~MapView()
{
}


void
MapView::FrameResized(float, float)
{
	Invalidate();
}


void
MapView::Refresh()
{
	Invalidate();
}


void
MapView::Draw(BRect)
{
	if (!fSession || !fSession->Running()) {
		SetHighColor(6, 9, 16);
		FillRect(Bounds());
		return;
	}
	std::lock_guard<std::recursive_mutex> g(fSession->Lock());
	if (!fRenderer) {
		fRenderer.reset(new ui::ReplayRenderer(fSession->Recorder().Data()));
		fRenderer->showTitle = false;
		fRenderer->showCaptions = false;
	}
	fRenderer->Refresh();
	double clock = (system_time() - fStart) / 1e6 + 20;
	fRenderer->Draw(this, Bounds(), fSession->Sim().GET(),
		std::min(fSession->Warp(), fSession->Sim().MaxWarp()), clock);
}

}  // namespace columbia
