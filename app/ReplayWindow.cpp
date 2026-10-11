// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "ReplayWindow.h"

#include <Bitmap.h>
#include <Button.h>
#include <LayoutBuilder.h>
#include <MessageRunner.h>
#include <Slider.h>
#include <String.h>
#include <View.h>

#include <algorithm>

#include "ReplayRenderer.h"

namespace columbia {

namespace {

enum {
	kMsgTick = 'tick',
	kMsgPlay = 'play',
	kMsgScrub = 'scrb',
	kMsgLength = 'lngt',
};

}  // namespace


// Draws the replay (double-buffered).
class ReplayView : public BView {
public:
	ReplayView()
		:
		BView("replay", B_WILL_DRAW | B_FRAME_EVENTS)
	{
		SetViewColor(B_TRANSPARENT_COLOR);
		SetExplicitMinSize(BSize(800, 450));
	}

	~ReplayView() override { delete fBuffer; }

	void FrameResized(float, float) override
	{
		delete fBuffer;
		fBuffer = nullptr;
		Invalidate();
	}

	void Draw(BRect) override
	{
		if (!renderer)
			return;
		BRect b = Bounds();
		if (!fBuffer) {
			fBuffer = new BBitmap(b, B_RGB32, true);
			fBufferView = new BView(b, "buffer", B_FOLLOW_NONE, B_WILL_DRAW);
			fBuffer->AddChild(fBufferView);
		}
		if (fBuffer->Lock()) {
			renderer->Draw(fBufferView, fBufferView->Bounds(), get, speed, clock);
			fBufferView->Sync();
			fBuffer->Unlock();
		}
		DrawBitmap(fBuffer, BPoint(0, 0));
	}

	ui::ReplayRenderer* renderer = nullptr;
	double get = 0, speed = 1, clock = 0;

private:
	BBitmap* fBuffer = nullptr;
	BView* fBufferView = nullptr;
};


ReplayWindow::ReplayWindow(const std::string& path)
	:
	BWindow(BRect(80, 80, 1360, 840), "Columbia replay", B_DOCUMENT_WINDOW,
		B_AUTO_UPDATE_SIZE_LIMITS)
{
	fLoaded = fRecording.Load(path);
	if (fLoaded) {
		BString title;
		title.SetToFormat("%s \xe2\x80\x94 replay", fRecording.header.missionName.c_str());
		SetTitle(title.String());
	}
	fView = new ReplayView();
	fPlay = new BButton("play", "Pause", new BMessage(kMsgPlay));
	fScrub = new BSlider("scrub", nullptr, new BMessage(kMsgScrub), 0, 1000,
		B_HORIZONTAL);
	fScrub->SetModificationMessage(new BMessage(kMsgScrub));
	fLength = new BSlider("length", "Replay length: 25 min", new BMessage(kMsgLength),
		5, 120, B_HORIZONTAL);
	fLength->SetValue(25);
	BLayoutBuilder::Group<>(this, B_VERTICAL, 0)
		.Add(fView, 10)
		.AddGroup(B_HORIZONTAL, B_USE_SMALL_SPACING)
			.SetInsets(B_USE_SMALL_SPACING)
			.Add(fPlay)
			.Add(fScrub, 4)
			.Add(fLength, 1)
		.End()
		.End();
	if (fLoaded) {
		fRenderer.reset(new ui::ReplayRenderer(fRecording));
		fView->renderer = fRenderer.get();
		_Rebuild(25);
	}
	BMessage tick(kMsgTick);
	fRunner = new BMessageRunner(BMessenger(this), &tick, 40000);
	fLast = system_time();
}


ReplayWindow::~ReplayWindow()
{
	delete fRunner;
	fView->renderer = nullptr;
}


void
ReplayWindow::_Rebuild(int minutes)
{
	double get = fPacer.TotalSeconds() > 0 ? fPacer.GETAt(fClock) : 0;
	fPacer.Build(fRecording, minutes);
	fClock = fPacer.SecondsAt(get);
	BString label;
	label.SetToFormat("Replay length: %d min", minutes);
	fLength->SetLabel(label.String());
}


void
ReplayWindow::MessageReceived(BMessage* message)
{
	switch (message->what) {
		case kMsgTick: {
			bigtime_t now = system_time();
			double dt = (now - fLast) / 1e6;
			fLast = now;
			if (!fLoaded)
				break;
			if (fPlaying)
				fClock = std::min(fClock + dt, fPacer.TotalSeconds());
			fView->get = fPacer.GETAt(fClock);
			fView->speed = fPlaying ? fPacer.SpeedAt(fClock) : 0;
			fView->clock = fClock + 10;
			fView->Invalidate();
			if (fPacer.TotalSeconds() > 0)
				fScrub->SetValue(int32(fClock / fPacer.TotalSeconds() * 1000));
			break;
		}
		case kMsgPlay:
			fPlaying = !fPlaying;
			fPlay->SetLabel(fPlaying ? "Pause" : "Play");
			if (fPlaying && fClock >= fPacer.TotalSeconds())
				fClock = 0;
			break;
		case kMsgScrub:
			fClock = fScrub->Value() / 1000.0 * fPacer.TotalSeconds();
			break;
		case kMsgLength:
			_Rebuild(fLength->Value());
			break;
		default:
			BWindow::MessageReceived(message);
	}
}

}  // namespace columbia
