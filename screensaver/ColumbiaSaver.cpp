// Columbia — the screensaver: Apollo missions replayed from flight
// recordings, on the Mission Control displays, with the real DSKYs and the
// air-to-ground loop.
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include <Bitmap.h>
#include <CheckBox.h>
#include <Directory.h>
#include <Entry.h>
#include <FindDirectory.h>
#include <LayoutBuilder.h>
#include <MenuField.h>
#include <MenuItem.h>
#include <Path.h>
#include <PopUpMenu.h>
#include <ScreenSaver.h>
#include <Slider.h>
#include <String.h>
#include <StringView.h>
#include <View.h>

#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ReplayPacer.h"
#include "ReplayRenderer.h"
#include "flight/Recording.h"

namespace {

const uint32 kMsgMission = 'mssn';
const uint32 kMsgLength = 'lngt';
const uint32 kMsgDSKY = 'dsky';
const uint32 kMsgCaptions = 'capt';
const uint32 kMsgRate = 'rate';

struct RecordingFile {
	std::string path;
	std::string name;     // "Apollo 11 (as flown)"
};

// Where recordings live: the user's own (made by the app), then the ones
// that came with the package.
std::vector<RecordingFile>
FindRecordings()
{
	std::vector<RecordingFile> found;
	const directory_which dirs[] = {
		B_USER_NONPACKAGED_DATA_DIRECTORY, B_USER_DATA_DIRECTORY,
		B_SYSTEM_NONPACKAGED_DATA_DIRECTORY, B_SYSTEM_DATA_DIRECTORY
	};
	for (directory_which which : dirs) {
		BPath path;
		if (find_directory(which, &path) != B_OK)
			continue;
		path.Append("Columbia/recordings");
		BDirectory dir(path.Path());
		if (dir.InitCheck() != B_OK)
			continue;
		BEntry entry;
		while (dir.GetNextEntry(&entry, true) == B_OK) {
			BPath p;
			entry.GetPath(&p);
			std::string s = p.Path();
			if (s.size() < 7 || s.substr(s.size() - 7) != ".colrec")
				continue;
			bool dup = false;
			for (const RecordingFile& r : found)
				if (r.path.substr(r.path.rfind('/')) == s.substr(s.rfind('/')))
					dup = true;
			if (dup)
				continue;
			RecordingFile r;
			r.path = s;
			r.name = p.Leaf();
			r.name = r.name.substr(0, r.name.size() - 7);
			found.push_back(r);
		}
	}
	std::sort(found.begin(), found.end(),
		[](const RecordingFile& a, const RecordingFile& b) { return a.name < b.name; });
	return found;
}


class ColumbiaSaver;

class ConfigView : public BView {
public:
	ConfigView(ColumbiaSaver* saver);
	void AttachedToWindow() override;
	void MessageReceived(BMessage* message) override;

private:
	ColumbiaSaver* fSaver;
	BPopUpMenu* fMissions;
	BSlider* fLength;
	BSlider* fRate;
	BCheckBox* fDSKY;
	BCheckBox* fCaptions;
};


class ColumbiaSaver : public BScreenSaver {
public:
	ColumbiaSaver(BMessage* archive, image_id image);
	~ColumbiaSaver() override;

	void StartConfig(BView* view) override;
	status_t StartSaver(BView* view, bool preview) override;
	void StopSaver() override;
	void Draw(BView* view, int32 frame) override;
	status_t SaveState(BMessage* into) const override;

	// Settings (the config view changes them).
	std::string mission;          // "" = shuffle
	int32 minutes = 25;
	int32 framesPerSecond = 20;
	bool showDSKY = true;
	bool showCaptions = true;

private:
	bool _Next();
	void _DrawEmpty(BView* view);

	std::vector<RecordingFile> fFiles;
	std::vector<size_t> fOrder;
	size_t fNext = 0;
	std::unique_ptr<flight::Recording> fRecording;
	std::unique_ptr<ui::ReplayRenderer> fRenderer;
	ui::ReplayPacer fPacer;
	bigtime_t fStart = 0;
	bool fPreview = false;
	BBitmap* fBuffer = nullptr;
	BView* fBufferView = nullptr;
	std::mt19937 fRandom;
};


// --- The saver ---------------------------------------------------------------------

ColumbiaSaver::ColumbiaSaver(BMessage* archive, image_id image)
	:
	BScreenSaver(archive, image),
	fRandom(uint32(system_time()))
{
	if (archive != nullptr) {
		const char* m;
		if (archive->FindString("mission", &m) == B_OK)
			mission = m;
		archive->FindInt32("minutes", &minutes);
		archive->FindInt32("fps", &framesPerSecond);
		archive->FindBool("dsky", &showDSKY);
		archive->FindBool("captions", &showCaptions);
	}
	minutes = std::clamp<int32>(minutes, 5, 120);
	framesPerSecond = std::clamp<int32>(framesPerSecond, 5, 60);
}


ColumbiaSaver::~ColumbiaSaver()
{
	delete fBuffer;
}


status_t
ColumbiaSaver::SaveState(BMessage* into) const
{
	into->AddString("mission", mission.c_str());
	into->AddInt32("minutes", minutes);
	into->AddInt32("fps", framesPerSecond);
	into->AddBool("dsky", showDSKY);
	into->AddBool("captions", showCaptions);
	return B_OK;
}


void
ColumbiaSaver::StartConfig(BView* view)
{
	ConfigView* config = new ConfigView(this);
	BLayoutBuilder::Group<>(view, B_VERTICAL, 0)
		.Add(config)
		.End();
}


status_t
ColumbiaSaver::StartSaver(BView* view, bool preview)
{
	fPreview = preview;
	fFiles = FindRecordings();
	fOrder.clear();
	for (size_t i = 0; i < fFiles.size(); i++)
		if (mission.empty() || fFiles[i].name == mission)
			fOrder.push_back(i);
	if (fOrder.empty())
		for (size_t i = 0; i < fFiles.size(); i++)
			fOrder.push_back(i);
	std::shuffle(fOrder.begin(), fOrder.end(), fRandom);
	fNext = 0;
	SetTickSize(bigtime_t(1000000 / (preview ? 5 : framesPerSecond)));
	BRect b = view->Bounds();
	delete fBuffer;
	fBuffer = new BBitmap(b, B_RGB32, true);
	fBufferView = new BView(b, "replay", B_FOLLOW_NONE, B_WILL_DRAW);
	fBuffer->AddChild(fBufferView);
	_Next();
	return B_OK;
}


void
ColumbiaSaver::StopSaver()
{
	fRenderer.reset();
	fRecording.reset();
}


bool
ColumbiaSaver::_Next()
{
	fRenderer.reset();
	fRecording.reset();
	for (size_t tries = 0; tries < fOrder.size(); tries++) {
		const RecordingFile& file = fFiles[fOrder[fNext % fOrder.size()]];
		fNext++;
		std::unique_ptr<flight::Recording> rec(new flight::Recording);
		if (!rec->Load(file.path))
			continue;
		fRecording = std::move(rec);
		fRenderer.reset(new ui::ReplayRenderer(*fRecording));
		fRenderer->showDSKY = showDSKY;
		fRenderer->showCaptions = showCaptions;
		fPacer.Build(*fRecording, minutes);
		fStart = system_time();
		return true;
	}
	return false;
}


void
ColumbiaSaver::_DrawEmpty(BView* v)
{
	BRect b = v->Bounds();
	v->SetHighColor(6, 9, 16);
	v->FillRect(b);
	BFont font(be_bold_font);
	font.SetSize(std::max(12.0f, b.Width() / 40));
	v->SetFont(&font);
	v->SetHighColor(255, 176, 46);
	const char* line1 = "Columbia: no flight recordings yet";
	const char* line2 = "Fly a mission in Columbia (or run record_mission) to make one.";
	v->DrawString(line1, BPoint((b.Width() - v->StringWidth(line1)) / 2,
		b.Height() / 2));
	font.SetSize(std::max(10.0f, b.Width() / 70));
	v->SetFont(&font);
	v->SetHighColor(160, 170, 180);
	v->DrawString(line2, BPoint((b.Width() - v->StringWidth(line2)) / 2,
		b.Height() / 2 + font.Size() * 2));
}


void
ColumbiaSaver::Draw(BView* view, int32 frame)
{
	(void)frame;
	if (!fBuffer || !fBuffer->Lock())
		return;
	if (!fRecording) {
		_DrawEmpty(fBufferView);
	} else {
		double t = (system_time() - fStart) / 1e6;
		if (fPreview) {
			// The thumbnail: hold on the most telling moment of the flight
			// (the landing, or the middle of it), without the panels.
			fRenderer->showDSKY = false;
			fRenderer->showCaptions = false;
			fRenderer->showTitle = false;
			double get = fPacer.GETAt(fPacer.TotalSeconds() * 0.6);
			for (const flight::RecSample& s : fRecording->samples)
				if (s.phase == 6 && s.altitude < 400) {   // powered descent
					get = s.get;
					break;
				}
			fRenderer->Draw(fBufferView, fBufferView->Bounds(), get, 1, t + 20);
		} else {
			if (t > fPacer.TotalSeconds() + 12) {
				_Next();
				t = 0;
			}
			if (fRecording) {
				double get = fPacer.GETAt(t);
				fRenderer->Draw(fBufferView, fBufferView->Bounds(), get,
					fPacer.SpeedAt(t), t);
			} else
				_DrawEmpty(fBufferView);
		}
	}
	fBufferView->Sync();
	fBuffer->Unlock();
	view->DrawBitmap(fBuffer, BPoint(0, 0));
}


// --- Settings -----------------------------------------------------------------

ConfigView::ConfigView(ColumbiaSaver* saver)
	:
	BView("Columbia settings", B_WILL_DRAW),
	fSaver(saver)
{
	SetViewUIColor(B_PANEL_BACKGROUND_COLOR);
	BStringView* title = new BStringView("title", "Columbia: Apollo flight replays");
	BFont font(be_bold_font);
	title->SetFont(&font);

	fMissions = new BPopUpMenu("missions");
	BMessage* shuffle = new BMessage(kMsgMission);
	shuffle->AddString("name", "");
	fMissions->AddItem(new BMenuItem("Shuffle all flights", shuffle));
	std::vector<RecordingFile> files = FindRecordings();
	if (!files.empty())
		fMissions->AddSeparatorItem();
	for (const RecordingFile& f : files) {
		BMessage* m = new BMessage(kMsgMission);
		m->AddString("name", f.name.c_str());
		BMenuItem* item = new BMenuItem(f.name.c_str(), m);
		item->SetMarked(f.name == saver->mission);
		fMissions->AddItem(item);
	}
	if (saver->mission.empty())
		fMissions->ItemAt(0)->SetMarked(true);
	BMenuField* field = new BMenuField("mission", "Flight:", fMissions);

	fLength = new BSlider("length", "Replay length: 25 minutes",
		new BMessage(kMsgLength), 5, 90, B_HORIZONTAL);
	fLength->SetValue(saver->minutes);
	fLength->SetModificationMessage(new BMessage(kMsgLength));
	fLength->SetLimitLabels("5 min", "90 min");
	fRate = new BSlider("rate", "Frames per second: 20",
		new BMessage(kMsgRate), 5, 60, B_HORIZONTAL);
	fRate->SetValue(saver->framesPerSecond);
	fRate->SetModificationMessage(new BMessage(kMsgRate));
	fRate->SetLimitLabels("5 (gentle on a Pi)", "60");
	fDSKY = new BCheckBox("dsky", "Show the DSKYs and readouts",
		new BMessage(kMsgDSKY));
	fDSKY->SetValue(saver->showDSKY ? B_CONTROL_ON : B_CONTROL_OFF);
	fCaptions = new BCheckBox("captions", "Show the air-to-ground loop",
		new BMessage(kMsgCaptions));
	fCaptions->SetValue(saver->showCaptions ? B_CONTROL_ON : B_CONTROL_OFF);

	BString count;
	count.SetToFormat("%d flight recording%s found. Fly or record more in "
		"Columbia.", int(files.size()), files.size() == 1 ? "" : "s");
	BStringView* info = new BStringView("info", count.String());

	BLayoutBuilder::Group<>(this, B_VERTICAL, B_USE_DEFAULT_SPACING)
		.SetInsets(B_USE_WINDOW_SPACING)
		.Add(title)
		.Add(field)
		.Add(fLength)
		.Add(fRate)
		.Add(fDSKY)
		.Add(fCaptions)
		.AddGlue()
		.Add(info)
		.End();
}


void
ConfigView::AttachedToWindow()
{
	BView::AttachedToWindow();
	fMissions->SetTargetForItems(this);
	fLength->SetTarget(this);
	fRate->SetTarget(this);
	fDSKY->SetTarget(this);
	fCaptions->SetTarget(this);
	BMessage m(kMsgLength);
	MessageReceived(&m);
	BMessage r(kMsgRate);
	MessageReceived(&r);
}


void
ConfigView::MessageReceived(BMessage* message)
{
	switch (message->what) {
		case kMsgMission: {
			const char* name;
			if (message->FindString("name", &name) == B_OK)
				fSaver->mission = name;
			break;
		}
		case kMsgLength: {
			fSaver->minutes = fLength->Value();
			BString label;
			label.SetToFormat("Replay length: %d minutes", int(fSaver->minutes));
			fLength->SetLabel(label.String());
			break;
		}
		case kMsgRate: {
			fSaver->framesPerSecond = fRate->Value();
			BString label;
			label.SetToFormat("Frames per second: %d", int(fSaver->framesPerSecond));
			fRate->SetLabel(label.String());
			break;
		}
		case kMsgDSKY:
			fSaver->showDSKY = fDSKY->Value() == B_CONTROL_ON;
			break;
		case kMsgCaptions:
			fSaver->showCaptions = fCaptions->Value() == B_CONTROL_ON;
			break;
		default:
			BView::MessageReceived(message);
	}
}

}  // namespace


extern "C" _EXPORT BScreenSaver*
instantiate_screen_saver(BMessage* message, image_id image)
{
	return new ColumbiaSaver(message, image);
}
