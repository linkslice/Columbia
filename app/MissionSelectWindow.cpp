// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "MissionSelectWindow.h"

#include <Alert.h>
#include <Application.h>
#include <Button.h>
#include <CheckBox.h>
#include <Directory.h>
#include <Entry.h>
#include <FindDirectory.h>
#include <LayoutBuilder.h>
#include <ListView.h>
#include <MenuField.h>
#include <MenuItem.h>
#include <Path.h>
#include <PopUpMenu.h>
#include <ScrollView.h>
#include <String.h>
#include <StringView.h>
#include <TextControl.h>
#include <TextView.h>

#include <cstdio>
#include <cstdlib>

#include "FlightWindow.h"
#include "ReplayWindow.h"
#include "flight/Contingency.h"
#include "flight/Drill.h"
#include "flight/MissionSpec.h"

namespace columbia {

using namespace flight;

namespace {

enum {
	kMsgSelect = 'msel',
	kMsgFly = 'fly ',
	kMsgDifficulty = 'diff',
	kMsgReplay = 'rply',
	kMsgStart = 'strt',
};

const struct {
	StartPoint start;
	const char* name;
} kStarts[] = {
	{StartPoint::Launch, "On the pad (T\xe2\x88\x92" "10 minutes)"},
	{StartPoint::EarthOrbit, "Earth parking orbit, before TLI"},
	{StartPoint::TranslunarCoast, "Translunar coast, LM extracted"},
	{StartPoint::LunarOrbit, "Lunar orbit, docked"},
	{StartPoint::PoweredDescent, "Ten minutes before powered descent"},
	{StartPoint::LunarSurface, "On the lunar surface"},
	{StartPoint::LunarAscent, "Ten minutes before lunar liftoff"},
	{StartPoint::TransearthCoast, "Transearth coast"},
	{StartPoint::Entry, "Half an hour before entry"},
};

const Difficulty kDifficulties[] = {
	Difficulty::Nominal, Difficulty::AsFlown, Difficulty::Challenging,
	Difficulty::Hard, Difficulty::Nightmare
};

bool
StartFits(const MissionSpec& m, StartPoint s)
{
	const FlightPlan& p = m.plan;
	switch (s) {
		case StartPoint::Launch: return true;
		case StartPoint::EarthOrbit: return true;
		case StartPoint::TranslunarCoast:
		case StartPoint::TransearthCoast: return p.tli > 0;
		case StartPoint::LunarOrbit: return p.loi > 0;
		case StartPoint::PoweredDescent:
		case StartPoint::LunarSurface:
		case StartPoint::LunarAscent: return p.landing > 0;
		case StartPoint::Entry: return p.entry > 0;
	}
	return false;
}

}  // namespace


MissionSelectWindow::MissionSelectWindow(const std::string& ropes)
	:
	BWindow(BRect(100, 100, 960, 700), "Columbia: choose a flight",
		B_TITLED_WINDOW, B_AUTO_UPDATE_SIZE_LIMITS),
	fRopes(ropes)
{
	fList = new BListView("missions");
	for (const MissionSpec& m : ApolloMissions()) {
		BString item;
		item.SetToFormat("%s  \xe2\x80\x94  %s", m.name.c_str(), m.motto.c_str());
		fList->AddItem(new BStringItem(item.String()));
	}
	fList->AddItem(new BStringItem("A mission of your own\xe2\x80\xa6"));
	fList->SetSelectionMessage(new BMessage(kMsgSelect));
	BScrollView* listScroll = new BScrollView("listScroll", fList, 0, false, true);

	fBrief = new BTextView("brief");
	fBrief->MakeEditable(false);
	fBrief->SetWordWrap(true);
	BScrollView* briefScroll = new BScrollView("briefScroll", fBrief, 0, false, true);
	briefScroll->SetExplicitMinSize(BSize(380, 220));

	fStarts = new BPopUpMenu("starts");
	BMenuField* startField = new BMenuField("start", "Start:", fStarts);

	fDifficulty = new BPopUpMenu("difficulty");
	for (Difficulty d : kDifficulties) {
		BMessage* m = new BMessage(kMsgDifficulty);
		m->AddInt32("difficulty", int32(d));
		fDifficulty->AddItem(new BMenuItem(DifficultyName(d), m));
	}
	fDifficulty->ItemAt(1)->SetMarked(true);
	BMenuField* diffField = new BMenuField("difficulty", "Difficulty:", fDifficulty);
	fDifficultyBlurb = new BStringView("blurb", DifficultyBlurb(Difficulty::AsFlown));

	fWatch = new BCheckBox("watch", "Let the automatic crew fly it (watch)",
		nullptr);
	fRealTime = new BCheckBox("realtime", "Real time only (no time warp)", nullptr);
	fDecide = new BCheckBox("decide", "Make Flight's calls yourself (aborts, "
		"burns, go/no-go)", nullptr);
	fDecide->SetValue(B_CONTROL_ON);

	fSites = new BPopUpMenu("sites");
	for (const LandingSite& s : CandidateSites())
		fSites->AddItem(new BMenuItem(s.name.c_str(), nullptr));
	fSites->ItemAt(0)->SetMarked(true);
	fSiteField = new BMenuField("site", "Landing site:", fSites);
	fDate = new BTextControl("date", "Launch date:", "1973-12-06", nullptr);
	fName = new BTextControl("name", "Mission name:", "Apollo 18", nullptr);

	fRecordings = new BPopUpMenu("recordings");
	BMenuField* recField = new BMenuField("recordings", "Replay:", fRecordings);
	BButton* replay = new BButton("replay", "Watch", new BMessage(kMsgReplay));
	fFly = new BButton("fly", "Fly", new BMessage(kMsgFly));
	fFly->MakeDefault(true);

	BLayoutBuilder::Group<>(this, B_HORIZONTAL, B_USE_DEFAULT_SPACING)
		.SetInsets(B_USE_WINDOW_SPACING)
		.Add(listScroll, 1)
		.AddGroup(B_VERTICAL, B_USE_DEFAULT_SPACING, 2)
			.Add(briefScroll, 3)
			.AddGrid(B_USE_SMALL_SPACING, B_USE_SMALL_SPACING)
				.AddMenuField(startField, 0, 0)
				.AddMenuField(diffField, 0, 1)
				.AddMenuField(fSiteField, 0, 2)
				.AddTextControl(fDate, 0, 3)
				.AddTextControl(fName, 0, 4)
			.End()
			.Add(fDifficultyBlurb)
			.Add(fWatch)
			.Add(fRealTime)
			.Add(fDecide)
			.AddGlue()
			.AddGroup(B_HORIZONTAL, B_USE_SMALL_SPACING)
				.Add(recField)
				.Add(replay)
				.AddGlue()
				.Add(fFly)
			.End()
		.End()
		.End();

	fList->Select(4);   // Apollo 11
	_FillRecordings();
	_ShowMission();
}


bool
MissionSelectWindow::QuitRequested()
{
	be_app->PostMessage(B_QUIT_REQUESTED);
	return true;
}


int
MissionSelectWindow::_Selected() const
{
	int32 i = fList->CurrentSelection();
	if (i < 0)
		return -1;
	const std::vector<MissionSpec>& all = ApolloMissions();
	if (i < int32(all.size()))
		return all[i].number;
	return 0;
}


void
MissionSelectWindow::_ShowMission()
{
	int n = _Selected();
	bool custom = n == 0;
	fSiteField->SetEnabled(custom);
	fDate->SetEnabled(custom);
	fName->SetEnabled(custom);
	const MissionSpec* m = n > 0 ? FindMission(n) : FindMission(17);
	if (!m)
		return;
	BString text;
	if (custom) {
		text = "A mission of your own on the J-mission hardware: pick a landing "
			"site (the ones flown and the ones studied), a launch date and a "
			"name. Mission Control plans the trajectory; the contingencies "
			"come from the difficulty you choose.";
	} else {
		text.SetToFormat("%s\n%s\n\nCommander %s, Command Module Pilot %s, "
			"Lunar Module Pilot %s\n%s%s%s\nFlight software: %s%s%s\n\n%s",
			m->name.c_str(), m->motto.c_str(), m->commander.c_str(),
			m->cmPilot.c_str(), m->lmPilot.c_str(),
			m->csmCallsign.c_str(), m->lmCallsign.empty() ? "" : " and ",
			m->lmCallsign.c_str(), m->cmcProgram.c_str(),
			m->lgcProgram.empty() ? "" : " / ", m->lgcProgram.c_str(),
			m->summary.c_str());
		if (!m->historical.empty()) {
			text << "\n\nAs flown:";
			for (const std::string& id : m->historical) {
				const ContingencySpec* c = FindContingency(id);
				if (c)
					text << "\n  \xe2\x80\xa2 " << c->title.c_str();
			}
		}
	}
	fBrief->SetText(text.String());
	_FillStarts();
}


void
MissionSelectWindow::_FillStarts()
{
	int n = _Selected();
	const MissionSpec* m = n > 0 ? FindMission(n) : FindMission(17);
	while (BMenuItem* item = fStarts->RemoveItem(int32(0)))
		delete item;
	if (!m)
		return;
	for (const auto& s : kStarts) {
		if (!StartFits(*m, s.start))
			continue;
		BMessage* msg = new BMessage(kMsgStart);
		msg->AddInt32("start", int32(s.start));
		fStarts->AddItem(new BMenuItem(s.name, msg));
	}
	// Training drills: the crew flies there, then hands over.
	bool separated = false;
	for (const DrillSpec& d : Drills()) {
		if (!DrillAvailable(d, *m))
			continue;
		if (!separated) {
			fStarts->AddSeparatorItem();
			separated = true;
		}
		BMessage* msg = new BMessage(kMsgStart);
		msg->AddInt32("start", int32(d.from));
		msg->AddInt32("drill", int32(d.id));
		BString label;
		label.SetToFormat("Drill: %s", d.name);
		fStarts->AddItem(new BMenuItem(label.String(), msg));
	}
	if (fStarts->CountItems() > 0)
		fStarts->ItemAt(0)->SetMarked(true);
}


void
MissionSelectWindow::_FillRecordings()
{
	while (BMenuItem* item = fRecordings->RemoveItem(int32(0)))
		delete item;
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
		BEntry entry;
		while (dir.GetNextEntry(&entry, true) == B_OK) {
			BPath p;
			entry.GetPath(&p);
			BString leaf = p.Leaf();
			if (!leaf.EndsWith(".colrec"))
				continue;
			leaf.RemoveLast(".colrec");
			BMessage* m = new BMessage(kMsgReplay);
			m->AddString("path", p.Path());
			fRecordings->AddItem(new BMenuItem(leaf.String(), m));
		}
	}
	if (fRecordings->CountItems() > 0)
		fRecordings->ItemAt(0)->SetMarked(true);
}


void
MissionSelectWindow::_Fly()
{
	int n = _Selected();
	if (n < 0)
		return;
	FlightOptions o;
	if (n > 0)
		o.mission = *FindMission(n);
	else {
		int y = 1973, mo = 12, d = 6;
		std::sscanf(fDate->Text(), "%d-%d-%d", &y, &mo, &d);
		int32 site = fSites->IndexOf(fSites->FindMarked());
		o.mission = CustomMission(fName->Text(),
			CandidateSites()[site < 0 ? 0 : site], y, mo, d);
	}
	BMenuItem* start = fStarts->FindMarked();
	int32 s = 0;
	if (start && start->Message())
		start->Message()->FindInt32("start", &s);
	o.start = StartPoint(s);
	int32 drill = 0;
	if (start && start->Message()
			&& start->Message()->FindInt32("drill", &drill) == B_OK)
		o.drill = DrillId(drill);
	int32 diff = int32(Difficulty::AsFlown);
	if (BMenuItem* item = fDifficulty->FindMarked())
		item->Message()->FindInt32("difficulty", &diff);
	o.difficulty = Difficulty(diff);
	o.autopilot = fWatch->Value() == B_CONTROL_ON;
	o.realTimeOnly = fRealTime->Value() == B_CONTROL_ON;
	o.askDecisions = fDecide->Value() == B_CONTROL_ON;
	o.ropes = fRopes;
	FlightWindow* w = new FlightWindow(o);
	if (!w->Started()) {
		BString text;
		text.SetToFormat("Couldn't start the flight: %s", w->StartError().c_str());
		w->Quit();
		(new BAlert("Columbia", text.String(), "OK"))->Go(nullptr);
		return;
	}
	w->CenterOnScreen();
	w->Show();
}


void
MissionSelectWindow::_Replay()
{
	BMenuItem* item = fRecordings->FindMarked();
	const char* path;
	if (!item || !item->Message()
			|| item->Message()->FindString("path", &path) != B_OK) {
		(new BAlert("Columbia", "No flight recordings yet. Fly a mission (or "
			"let the automatic crew fly one) and save its recording at the "
			"end.", "OK"))->Go(nullptr);
		return;
	}
	ReplayWindow* w = new ReplayWindow(path);
	if (!w->Loaded()) {
		w->Quit();
		(new BAlert("Columbia", "That recording couldn't be read.", "OK"))->Go(nullptr);
		return;
	}
	w->CenterOnScreen();
	w->Show();
}


void
MissionSelectWindow::MessageReceived(BMessage* message)
{
	switch (message->what) {
		case kMsgSelect:
			_ShowMission();
			break;
		case kMsgFly:
			_Fly();
			break;
		case kMsgDifficulty: {
			int32 d;
			if (message->FindInt32("difficulty", &d) == B_OK)
				fDifficultyBlurb->SetText(DifficultyBlurb(Difficulty(d)));
			break;
		}
		case kMsgReplay:
			if (message->HasString("path"))
				break;      // a menu choice
			_Replay();
			break;
		case kMsgStart: {
			// A drill: its brief in place of the mission's.
			int32 drill;
			if (message->FindInt32("drill", &drill) == B_OK) {
				if (const DrillSpec* d = FindDrill(DrillId(drill))) {
					BString text;
					text.SetToFormat("Drill: %s\n\n%s\n\nThe automatic crew flies "
						"the mission up to this moment as fast as the simulation "
						"will go, then hands you the controls.", d->name, d->brief);
					fBrief->SetText(text.String());
				}
			}
			break;
		}
		case 'flcl':
			_FillRecordings();
			break;
		default:
			BWindow::MessageReceived(message);
	}
}

}  // namespace columbia
