// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "FlightWindow.h"

#include <Alert.h>
#include <Application.h>
#include <Button.h>
#include <CardLayout.h>
#include <CheckBox.h>
#include <Directory.h>
#include <FindDirectory.h>
#include <LayoutBuilder.h>
#include <Menu.h>
#include <MenuBar.h>
#include <MenuItem.h>
#include <MessageRunner.h>
#include <Path.h>
#include <ScrollView.h>
#include <Slider.h>
#include <String.h>
#include <StringView.h>
#include <TextView.h>

#include <cmath>
#include <cstdio>
#include <ctime>
#include <mutex>

#include "DecisionWindow.h"
#include "MapView.h"
#include "PanelView.h"
#include "WindowView.h"

namespace columbia {

using namespace flight;

namespace {

enum {
	kMsgPulse = 'puls',
	kMsgCard = 'card',
	kMsgWarp = 'warp',
	kMsgRealTime = 'rtim',
	kMsgPause = 'paus',
	kMsgAutopilot = 'auto',
	kMsgCraft = 'crft',
	kMsgSave = 'save',
	kMsgReport = 'rprt',
	kMsgFaster = 'fast',
	kMsgSlower = 'slow',
	kMsgKeys = 'keys',
};

std::string
Clock(double t)
{
	char b[32];
	long s = long(std::floor(std::fabs(t)));
	std::snprintf(b, sizeof(b), "%s%03ld:%02ld:%02ld", t < 0 ? "-" : "",
		s / 3600, s / 60 % 60, s % 60);
	return b;
}

double
WarpFromSlider(int32 v)
{
	return std::pow(10.0, v / 20.0);
}

int32
SliderFromWarp(double w)
{
	return int32(std::lround(std::log10(std::max(1.0, w)) * 20));
}

}  // namespace


FlightWindow::FlightWindow(const FlightOptions& options)
	:
	BWindow(BRect(60, 60, 1500, 940), options.mission.name.c_str(),
		B_DOCUMENT_WINDOW, B_AUTO_UPDATE_SIZE_LIMITS)
{
	fStarted = fSession.Start(options, &fError);

	fPanel = new PanelView(&fSession);
	fWindow = new WindowView(&fSession);
	fMap = new MapView(&fSession);

	BView* cardHolder = new BView("cards", 0);
	fCards = new BCardLayout();
	cardHolder->SetLayout(fCards);
	fCards->AddView(fPanel);
	fCards->AddView(fWindow);
	fCards->AddView(fMap);
	fCards->SetVisibleItem(int32(0));
	cardHolder->SetExplicitMinSize(BSize(640, 360));

	// The side: clocks, the loop, time, crew.
	fClock = new BStringView("clock", "GET 000:00:00");
	BFont big(be_bold_font);
	big.SetSize(20);
	fClock->SetFont(&big);
	fPhase = new BStringView("phase", "");
	fNext = new BStringView("next", "");
	fLoop = new BTextView("loop");
	fLoop->MakeEditable(false);
	fLoop->SetWordWrap(true);
	fLoop->SetStylable(true);
	BScrollView* loopScroll = new BScrollView("loopScroll", fLoop, 0, false, true);
	loopScroll->SetExplicitMinSize(BSize(300, 200));

	fWarp = new BSlider("warp", "Time warp", new BMessage(kMsgWarp), 0, 100,
		B_HORIZONTAL);
	fWarp->SetModificationMessage(new BMessage(kMsgWarp));
	fWarp->SetLimitLabels("Real time", "100,000\xc3\x97");
	fWarp->SetValue(0);
	fWarp->SetEnabled(!options.realTimeOnly);
	fWarpLabel = new BStringView("warpLabel", "1\xc3\x97");
	BButton* realTime = new BButton("realtime", "Real time",
		new BMessage(kMsgRealTime));
	fPause = new BButton("pause", "Pause", new BMessage(kMsgPause));
	fAutopilot = new BCheckBox("autopilot", "Automatic crew flies",
		new BMessage(kMsgAutopilot));
	fAutopilot->SetValue(options.autopilot ? B_CONTROL_ON : B_CONTROL_OFF);
	fCrewLabel = new BStringView("crewLabel", "");

	BButton* bPanel = new BButton("bPanel", "Controls", new BMessage(kMsgCard));
	bPanel->Message()->AddInt32("card", 0);
	BButton* bWindow = new BButton("bWindow", "Window", new BMessage(kMsgCard));
	bWindow->Message()->AddInt32("card", 1);
	BButton* bMap = new BButton("bMap", "Map", new BMessage(kMsgCard));
	bMap->Message()->AddInt32("card", 2);

	BView* side = new BView("side", 0);
	BLayoutBuilder::Group<>(side, B_VERTICAL, B_USE_SMALL_SPACING)
		.SetInsets(B_USE_SMALL_SPACING)
		.Add(fClock)
		.Add(fPhase)
		.Add(fNext)
		.AddGroup(B_HORIZONTAL, B_USE_SMALL_SPACING)
			.Add(bPanel)
			.Add(bWindow)
			.Add(bMap)
		.End()
		.Add(new BStringView("loopLabel", "Air-to-ground"))
		.Add(loopScroll, 10)
		.Add(fWarp)
		.AddGroup(B_HORIZONTAL, B_USE_SMALL_SPACING)
			.Add(fWarpLabel)
			.AddGlue()
			.Add(realTime)
			.Add(fPause)
		.End()
		.Add(fAutopilot)
		.Add(fCrewLabel)
		.End();
	side->SetExplicitMaxSize(BSize(380, B_SIZE_UNLIMITED));

	BLayoutBuilder::Group<>(this, B_VERTICAL, 0)
		.Add(new BMenuBar("menu"))
		.AddGroup(B_HORIZONTAL, 0)
			.Add(cardHolder, 3)
			.Add(side, 1)
		.End()
		.End();
	_BuildMenu(nullptr);

	fJoystick.Open();
	BMessage pulse(kMsgPulse);
	fRunner = new BMessageRunner(BMessenger(this), &pulse, 50000);
	fPanel->MakeFocus(true);
}


FlightWindow::~FlightWindow()
{
	delete fRunner;
	if (fDecisionWindow && fDecisionWindow->Lock())
		fDecisionWindow->Quit();
	fSession.Stop();
}


void
FlightWindow::_CheckDecision()
{
	// Flight's call: put it up (once), with the clock stopped.
	flight::Decision decision;
	{
		std::lock_guard<std::recursive_mutex> g(fSession.Lock());
		const flight::Decision* d = fSession.Director().PendingDecision();
		if (!d || d->id == fDecisionId)
			return;
		decision = *d;
	}
	fDecisionId = decision.id;
	fDecisionWindow = new DecisionWindow(decision, BMessenger(this), true);
	fDecisionWindow->CenterIn(Frame());
	fDecisionWindow->Show();
}


void
FlightWindow::_BuildMenu(BView*)
{
	BMenuBar* bar = dynamic_cast<BMenuBar*>(FindView("menu"));
	if (!bar)
		return;
	BMenu* flight = new BMenu("Flight");
	fPauseItem = new BMenuItem("Pause", new BMessage(kMsgPause), ' ', 0);
	flight->AddItem(fPauseItem);
	flight->AddItem(new BMenuItem("Faster", new BMessage(kMsgFaster), '.'));
	flight->AddItem(new BMenuItem("Slower", new BMessage(kMsgSlower), ','));
	flight->AddItem(new BMenuItem("Real time", new BMessage(kMsgRealTime), '1'));
	flight->AddSeparatorItem();
	fAutoItem = new BMenuItem("Automatic crew", new BMessage(kMsgAutopilot), 'A');
	fAutoItem->SetMarked(fSession.Autopilot());
	flight->AddItem(fAutoItem);
	flight->AddSeparatorItem();
	flight->AddItem(new BMenuItem("Save flight recording",
		new BMessage(kMsgSave), 'S'));
	flight->AddSeparatorItem();
	flight->AddItem(new BMenuItem("Close", new BMessage(B_QUIT_REQUESTED), 'W'));
	bar->AddItem(flight);

	BMenu* view = new BMenu("View");
	const char* names[3] = {"Controls", "Out the window", "Map"};
	for (int i = 0; i < 3; i++) {
		BMessage* m = new BMessage(kMsgCard);
		m->AddInt32("card", i);
		view->AddItem(new BMenuItem(names[i], m, char('1' + i), B_OPTION_KEY));
	}
	view->AddSeparatorItem();
	const char* crafts[3] = {"Follow the crew", "Command Module", "Lunar Module"};
	for (int i = 0; i < 3; i++) {
		BMessage* m = new BMessage(kMsgCraft);
		m->AddInt32("craft", i);
		fCraftItems[i] = new BMenuItem(crafts[i], m);
		view->AddItem(fCraftItems[i]);
	}
	fCraftItems[0]->SetMarked(true);
	bar->AddItem(view);

	BMenu* help = new BMenu("Help");
	help->AddItem(new BMenuItem("Keys and controllers\xe2\x80\xa6",
		new BMessage(kMsgKeys)));
	bar->AddItem(help);
}


bool
FlightWindow::QuitRequested()
{
	if (fSession.Running() && !fSession.Over() && !fRecordingSaved)
		_SaveRecording(true);   // keep what was flown for the replays
	fSession.Stop();
	be_app->PostMessage('flcl');
	return true;
}


void
FlightWindow::_ShowCard(int index)
{
	fCard = index;
	fCards->SetVisibleItem(int32(index));
	if (index == 0)
		fPanel->MakeFocus(true);
	else if (index == 1)
		fWindow->MakeFocus(true);
	else
		fMap->MakeFocus(true);
}


void
FlightWindow::MessageReceived(BMessage* message)
{
	switch (message->what) {
		case kMsgPulse:
			_Pulse();
			break;
		case kMsgDecided: {
			const char* id = nullptr;
			const char* key = nullptr;
			if (message->FindString("id", &id) == B_OK
					&& message->FindString("key", &key) == B_OK)
				fSession.Decide(id, key);
			fDecisionWindow = nullptr;
			fDecisionId.clear();
			break;
		}
		case kMsgCard: {
			int32 card;
			if (message->FindInt32("card", &card) == B_OK)
				_ShowCard(card);
			break;
		}
		case kMsgWarp:
			_SetWarpFromSlider();
			break;
		case kMsgFaster:
			fWarp->SetValue(std::min(100, fWarp->Value() + 10));
			_SetWarpFromSlider();
			break;
		case kMsgSlower:
			fWarp->SetValue(std::max(0, fWarp->Value() - 10));
			_SetWarpFromSlider();
			break;
		case kMsgRealTime:
			fWarp->SetValue(0);
			_SetWarpFromSlider();
			break;
		case kMsgPause: {
			bool paused = !fSession.Paused();
			fSession.SetPaused(paused);
			fPause->SetLabel(paused ? "Resume" : "Pause");
			fPauseItem->SetMarked(paused);
			break;
		}
		case kMsgAutopilot: {
			void* source = nullptr;
			message->FindPointer("source", &source);
			bool on = source == fAutopilot
				? fAutopilot->Value() == B_CONTROL_ON : !fSession.Autopilot();
			fSession.SetAutopilot(on);
			fAutopilot->SetValue(on ? B_CONTROL_ON : B_CONTROL_OFF);
			fAutoItem->SetMarked(on);
			break;
		}
		case kMsgCraft: {
			int32 craft;
			if (message->FindInt32("craft", &craft) == B_OK) {
				fPanel->SetCraft(craft);
				for (int i = 0; i < 3; i++)
					fCraftItems[i]->SetMarked(i == craft);
			}
			break;
		}
		case kMsgSave:
			_SaveRecording(false);
			break;
		case kMsgReport: {
			int32 which;
			if (message->FindInt32("which", &which) == B_OK && which == 0)
				_SaveRecording(false);
			break;
		}
		case kMsgKeys: {
			BAlert* alert = new BAlert("Keys",
				"Hand controllers\n"
				"  Rotation: \xe2\x86\x91 \xe2\x86\x93 pitch, \xe2\x86\x90 \xe2\x86\x92 roll, [ ] yaw\n"
				"  Translation: Shift + \xe2\x86\x91 \xe2\x86\x93 up/down, "
				"Shift + \xe2\x86\x90 \xe2\x86\x92 left/right,\n"
				"    Page Up / Page Down forward / aft\n"
				"  LM throttle (manual): Shift + Page Up / Page Down\n\n"
				"DSKY (on the controls): V N + - 0-9 E (ENTR) R (RSET) C (CLR)\n"
				"  K (KEY REL) P (PRO)\n\n"
				"Time: . faster, , slower, Space pause, Tab next view\n\n"
				"A joystick works as the hand controllers: stick for roll and "
				"pitch, twist for yaw, the hat to translate, trigger and thumb "
				"for +X and -X, button 3 for PRO, the throttle lever for the "
				"LM's descent engine.",
				"OK");
			alert->Go(nullptr);
			break;
		}
		default:
			BWindow::MessageReceived(message);
	}
}


void
FlightWindow::_SetWarpFromSlider()
{
	double w = WarpFromSlider(fWarp->Value());
	fSession.SetWarp(w);
}


bool
FlightWindow::_HandleKey(BMessage* message, bool down)
{
	const char* bytes;
	if (message->FindString("bytes", &bytes) != B_OK)
		return false;
	int32 modifiers = 0;
	message->FindInt32("modifiers", &modifiers);
	bool shift = modifiers & B_SHIFT_KEY;
	int v = down ? 1 : 0;
	switch (bytes[0]) {
		case B_UP_ARROW:
			if (shift) fKeyTrans[2] = down ? -1 : 0;    // up: -Z (toward the crew's head)
			else fKeyRot[1] = v;
			return true;
		case B_DOWN_ARROW:
			if (shift) fKeyTrans[2] = down ? 1 : 0;
			else fKeyRot[1] = -v;
			return true;
		case B_LEFT_ARROW:
			if (shift) fKeyTrans[1] = down ? -1 : 0;
			else fKeyRot[0] = -v;
			return true;
		case B_RIGHT_ARROW:
			if (shift) fKeyTrans[1] = down ? 1 : 0;
			else fKeyRot[0] = v;
			return true;
		case '[':
			fKeyRot[2] = -v;
			return true;
		case ']':
			fKeyRot[2] = v;
			return true;
		case B_PAGE_UP:
			if (shift) {
				if (down)
					fThrottleSteps++;
			} else
				fKeyTrans[0] = v;
			return true;
		case B_PAGE_DOWN:
			if (shift) {
				if (down)
					fThrottleSteps--;
			} else
				fKeyTrans[0] = -v;
			return true;
		case B_TAB:
			if (down)
				_ShowCard((fCard + 1) % 3);
			return true;
		case ' ':
			if (down && !message->HasInt32("be:key_repeat")) {
				BMessage m(kMsgPause);
				MessageReceived(&m);
			}
			return true;
		case '.':
			if (down)
				PostMessage(kMsgFaster);
			return true;
		case ',':
			if (down)
				PostMessage(kMsgSlower);
			return true;
	}
	return false;
}


void
FlightWindow::DispatchMessage(BMessage* message, BHandler* handler)
{
	if (message->what == B_KEY_DOWN || message->what == B_UNMAPPED_KEY_DOWN) {
		if (_HandleKey(message, true))
			return;
	} else if (message->what == B_KEY_UP || message->what == B_UNMAPPED_KEY_UP) {
		if (_HandleKey(message, false))
			return;
	}
	BWindow::DispatchMessage(message, handler);
}


void
FlightWindow::_ApplyControllers()
{
	Joystick::State js;
	bool stick = fJoystick.IsOpen() && fJoystick.Poll(&js);
	// Roll, pitch, yaw requested, -1..1.
	sim::Vec3 rot(fKeyRot[0], fKeyRot[1], fKeyRot[2]);
	int trans[3] = {fKeyTrans[0], fKeyTrans[1], fKeyTrans[2]};
	if (stick) {
		if (rot.Length() == 0)
			rot = js.rotation;
		for (int i = 0; i < 3; i++)
			if (trans[i] == 0)
				trans[i] = js.translation[i];
	}
	std::lock_guard<std::recursive_mutex> g(fSession.Lock());
	FlightSim& f = fSession.Sim();
	// While the automatic crew flies, the controllers are theirs.
	bool hands = !fSession.Autopilot();
	if (fPanel->ShowingLM() && f.LMPGNCS()) {
		gnc::LMPanel& p = f.LMPGNCS()->panel;
		if (hands) {
			// LM ACA: x yaw, y pitch, z roll.
			p.aca = sim::Vec3(rot.z, rot.y, rot.x);
			for (int i = 0; i < 3; i++)
				p.thc[i] = trans[i];
		}
		if (fThrottleSteps != 0) {
			p.manualThrottle = std::max(0.0, std::min(1.0,
				p.manualThrottle + 0.05 * fThrottleSteps));
			fThrottleSteps = 0;
		}
		if (stick && js.throttle >= 0 && !p.autoThrottle)
			p.manualThrottle = js.throttle;
		fPanel->rotation = sim::Vec3(rot.z, rot.y, rot.x);
	} else {
		gnc::CMPanel& p = f.CMPGNCS().panel;
		if (hands) {
			// CM RHC: x roll, y pitch, z yaw.
			p.rhc = rot;
			for (int i = 0; i < 3; i++)
				p.thc[i] = trans[i];
		}
		fPanel->rotation = rot;
	}
	for (int i = 0; i < 3; i++)
		fPanel->translation[i] = trans[i];
	if (stick && js.proceed != fProceedHeld) {
		fProceedHeld = js.proceed;
		if (fPanel->ShowingLM())
			f.ProceedLGC(js.proceed);
		else
			f.ProceedCMC(js.proceed);
	}
}


void
FlightWindow::_Pulse()
{
	if (!fStarted)
		return;
	_ApplyControllers();
	_CheckDecision();
	switch (fCard) {
		case 0: fPanel->Refresh(); break;
		case 1: fWindow->Refresh(); break;
		case 2: fMap->Refresh(); break;
	}
	// The loop.
	std::deque<LogEntry> log;
	fSession.TakeLog(&log);
	for (const LogEntry& e : log) {
		if (e.kind == LogEntry::Info)
			continue;
		BString line;
		const char* who = e.kind == LogEntry::Capcom ? "CAPCOM: "
			: e.kind == LogEntry::Crew ? "Crew: "
			: e.kind == LogEntry::Warning ? "MASTER ALARM: "
			: e.kind == LogEntry::Caution ? "Caution: " : "";
		line.SetToFormat("%s  %s%s\n", Clock(e.get).c_str(), who, e.text.c_str());
		rgb_color c = e.kind == LogEntry::Capcom ? rgb_color{170, 100, 0, 255}
			: e.kind == LogEntry::Crew ? rgb_color{0, 120, 30, 255}
			: e.kind == LogEntry::Warning ? rgb_color{200, 20, 10, 255}
			: ui_color(B_DOCUMENT_TEXT_COLOR);
		text_run_array runs;
		runs.count = 1;
		runs.runs[0].offset = 0;
		runs.runs[0].font = *be_plain_font;
		runs.runs[0].color = c;
		fLoop->Insert(fLoop->TextLength(), line.String(), line.Length(), &runs);
		fLoop->ScrollToOffset(fLoop->TextLength());
	}
	_UpdateStatus();
	if (fSession.Over() && !fOverShown)
		_FlightOver();
}


void
FlightWindow::_UpdateStatus()
{
	std::lock_guard<std::recursive_mutex> g(fSession.Lock());
	FlightSim& f = fSession.Sim();
	BString s;
	s.SetToFormat("GET %s", Clock(f.GET()).c_str());
	fClock->SetText(s.String());
	fPhase->SetText(PhaseName(f.CurrentPhase()));
	std::string next = fSession.Director().NextEvent();
	double nextGET = fSession.Director().NextEventGET();
	if (!next.empty() && nextGET > 0) {
		s.SetToFormat("Next: %s  (T%s)", next.c_str(),
			Clock(f.GET() - nextGET).c_str());
		fNext->SetText(s.String());
	} else
		fNext->SetText(next.c_str());
	double warp = std::min(fSession.Warp(), f.MaxWarp());
	if (fSession.DrillPending()) {
		s.SetToFormat("Setting up the drill (%.0f\xc3\x97)", warp);
		fNext->SetText(fSession.Drill()->name);
	} else if (fSession.Paused())
		s = "Paused";
	else if (warp < fSession.Warp() - 0.5)
		s.SetToFormat("%.0f\xc3\x97 (held to %.0f\xc3\x97 for the computers)",
			fSession.Warp(), warp);
	else
		s.SetToFormat("%.0f\xc3\x97", warp);
	fWarpLabel->SetText(s.String());
	bool settingUp = fSession.DrillPending();
	if (fWarp->IsEnabled() == settingUp) {
		fWarp->SetEnabled(!settingUp);
		fAutopilot->SetEnabled(!settingUp);
		if (!settingUp)
			fWarp->SetValue(0);
	}
	s.SetToFormat("Crew %.0f%%  \xc2\xb7  in the CM %d, in the LM %d",
		f.crew.AverageHealth(), f.CrewInCM(), f.CrewInLM());
	fCrewLabel->SetText(s.String());
	if (fSession.Autopilot() != (fAutopilot->Value() == B_CONTROL_ON))
		fAutopilot->SetValue(fSession.Autopilot() ? B_CONTROL_ON : B_CONTROL_OFF);
}


void
FlightWindow::_FlightOver()
{
	fOverShown = true;
	std::string outcome = fSession.Outcome();
	bool lost;
	{
		std::lock_guard<std::recursive_mutex> g(fSession.Lock());
		lost = fSession.Sim().Lost();
	}
	BString text;
	text.SetToFormat("%s\n\n%s\n\nSave the flight recording? It will replay in "
		"the Columbia screensaver and the replay window.",
		lost ? "Mission lost" : "Splashdown", outcome.c_str());
	BAlert* alert = new BAlert(lost ? "Mission lost" : "Splashdown",
		text.String(), "Save recording", "Not now");
	BMessage* m = new BMessage(kMsgReport);
	alert->Go(new BInvoker(m, this));
}


void
FlightWindow::_SaveRecording(bool quiet)
{
	BPath path;
	if (find_directory(B_USER_NONPACKAGED_DATA_DIRECTORY, &path) != B_OK)
		return;
	path.Append("Columbia/recordings");
	create_directory(path.Path(), 0755);
	char name[128];
	std::time_t now = std::time(nullptr);
	std::tm tm = *std::localtime(&now);
	std::string mission = fSession.Options().mission.name;
	for (char& c : mission)
		if (c == ' ')
			c = '-';
	std::snprintf(name, sizeof(name), "%s-%04d%02d%02d-%02d%02d.colrec",
		mission.c_str(), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min);
	path.Append(name);
	bool ok = fSession.SaveRecording(path.Path());
	fRecordingSaved = ok;
	if (!quiet) {
		BString text;
		text.SetToFormat(ok ? "Saved the flight recording to\n%s"
			: "Couldn't save the flight recording to\n%s", path.Path());
		(new BAlert("Recording", text.String(), "OK"))->Go(nullptr);
	}
}

}  // namespace columbia
