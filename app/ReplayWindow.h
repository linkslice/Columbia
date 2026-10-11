// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Watching a flight recording: the Mission Control replay with play,
// pause, a scrubber and the replay length.

#ifndef COLUMBIA_REPLAY_WINDOW_H
#define COLUMBIA_REPLAY_WINDOW_H

#include <Window.h>

#include <memory>
#include <string>

#include "ReplayPacer.h"
#include "flight/Recording.h"

class BButton;
class BMessageRunner;
class BSlider;

namespace ui {
class ReplayRenderer;
}

namespace columbia {

class ReplayView;

class ReplayWindow : public BWindow {
public:
	explicit ReplayWindow(const std::string& path);
	~ReplayWindow() override;

	bool Loaded() const { return fLoaded; }
	void MessageReceived(BMessage* message) override;

private:
	void _Rebuild(int minutes);

	flight::Recording fRecording;
	bool fLoaded = false;
	ui::ReplayPacer fPacer;
	std::unique_ptr<ui::ReplayRenderer> fRenderer;
	ReplayView* fView = nullptr;
	BSlider* fScrub = nullptr;
	BSlider* fLength = nullptr;
	BButton* fPlay = nullptr;
	BMessageRunner* fRunner = nullptr;
	double fClock = 0;
	bigtime_t fLast = 0;
	bool fPlaying = true;
};

}  // namespace columbia

#endif
