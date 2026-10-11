// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The map: the Mission Control wall displays for the flight in progress
// (the replay renderer drawing the recording as it's being made).

#ifndef COLUMBIA_MAP_VIEW_H
#define COLUMBIA_MAP_VIEW_H

#include <View.h>

#include <memory>

namespace ui {
class ReplayRenderer;
}

namespace columbia {

class FlightSession;

class MapView : public BView {
public:
	explicit MapView(FlightSession* session);
	~MapView() override;

	void Draw(BRect updateRect) override;
	void FrameResized(float width, float height) override;
	void Refresh();

private:
	FlightSession* fSession;
	std::unique_ptr<ui::ReplayRenderer> fRenderer;
	bigtime_t fStart;
};

}  // namespace columbia

#endif
