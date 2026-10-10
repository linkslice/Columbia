// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "MainWindow.h"

#include <Alert.h>
#include <Application.h>
#include <Box.h>
#include <Button.h>
#include <CheckBox.h>
#include <Entry.h>
#include <FilePanel.h>
#include <GroupView.h>
#include <LayoutBuilder.h>
#include <ListItem.h>
#include <ListView.h>
#include <Menu.h>
#include <MenuBar.h>
#include <MenuField.h>
#include <MenuItem.h>
#include <MessageRunner.h>
#include <Path.h>
#include <PopUpMenu.h>
#include <ScrollView.h>
#include <SplitView.h>
#include <String.h>
#include <StringView.h>

#include <cmath>
#include <cstdio>

#include "BurnWindow.h"
#include "DSKYView.h"
#include "TrajectoryView.h"

using columbia::Mission;

namespace {

enum {
	MSG_TICK = 'tick',
	MSG_NEW_FLIGHT = 'newF',
	MSG_OPEN_PLAN = 'opnP',
	MSG_SAVE_PLAN = 'savP',
	MSG_PAUSE = 'paus',
	MSG_FASTER = 'fast',
	MSG_SLOWER = 'slow',
	MSG_AUTO_SLOW = 'auto',
	MSG_ADD_BURN = 'addB',
	MSG_EDIT_BURN = 'edtB',
	MSG_DELETE_BURN = 'delB',
	MSG_PLAN_SELECTED = 'selB',
	MSG_PLAN_TLI = 'pTLI',
	MSG_PLAN_LOI = 'pLOI',
	MSG_PLAN_TEI = 'pTEI',
	MSG_PLAN_MCC = 'pMCC',
	MSG_PLAN_DONE = 'pDON',
	MSG_CLEAR_PLAN = 'clrP',
	MSG_FRAME = 'fram',
	MSG_FOLLOW = 'folw',
	MSG_FIT = 'fitV',
	MSG_KEYS = 'keys',
	MSG_ABOUT = 'abou'
};

const bigtime_t kTickInterval = 33333;   // 30 Hz

std::string
Thousands(double value)
{
	char digits[32];
	std::snprintf(digits, sizeof(digits), "%.0f", std::fabs(value));
	std::string text(digits), out;
	int n = int(text.size());
	for (int i = 0; i < n; i++) {
		out += text[i];
		if ((n - i - 1) % 3 == 0 && i != n - 1)
			out += ',';
	}
	return (value < 0 ? "-" : "") + out;
}

std::string
Altitude(double metres)
{
	if (std::fabs(metres) < 1e6)
		return Thousands(metres / 1000.0) + "."
			+ std::to_string(int(std::fabs(std::fmod(metres, 1000.0)) / 100))
			+ " km";
	return Thousands(metres / 1000.0) + " km";
}


// One row of the flight plan.
class BurnItem : public BListItem {
public:
	BurnItem(const sim::Burn& burn, bool done, bool active)
		: fBurn(burn), fDone(done), fActive(active) {}

	void DrawItem(BView* owner, BRect frame, bool complete) override
	{
		rgb_color background = IsSelected()
			? ui_color(B_LIST_SELECTED_BACKGROUND_COLOR)
			: ui_color(B_LIST_BACKGROUND_COLOR);
		rgb_color text = IsSelected()
			? ui_color(B_LIST_SELECTED_ITEM_TEXT_COLOR)
			: ui_color(B_LIST_ITEM_TEXT_COLOR);
		if (fDone && !IsSelected())
			text = tint_color(text, B_LIGHTEN_1_TINT);
		owner->SetHighColor(background);
		owner->FillRect(frame);
		(void)complete;
		if (fActive) {
			owner->SetHighColor(255, 166, 48);
			owner->FillRect(BRect(frame.left, frame.top, frame.left + 3,
				frame.bottom));
		}
		owner->SetHighColor(text);
		font_height fh;
		owner->GetFontHeight(&fh);
		float y = frame.top + fh.ascent + 2;
		const char* status = fActive ? "\xe2\x96\xb6" : (fDone ? "\xe2\x9c\x93" : " ");
		owner->DrawString(status, BPoint(frame.left + 6, y));
		owner->DrawString(fBurn.label.c_str(), BPoint(frame.left + 22, y));
		owner->DrawString(fBurn.engine == sim::ENGINE_SIVB ? "S-IVB" : "SPS",
			BPoint(frame.left + 82, y));
		owner->DrawString(sim::FormatGET(fBurn.time).c_str(),
			BPoint(frame.left + 130, y));
		char dv[32];
		std::snprintf(dv, sizeof(dv), "%.1f m/s", fBurn.DeltaV());
		owner->DrawString(dv, BPoint(frame.right - owner->StringWidth(dv) - 6,
			y));
	}

	void Update(BView* owner, const BFont* font) override
	{
		BListItem::Update(owner, font);
		font_height fh;
		font->GetHeight(&fh);
		SetHeight(std::ceil(fh.ascent + fh.descent + fh.leading) + 4);
	}

	const sim::Burn& Burn() const { return fBurn; }
	bool Done() const { return fDone; }

private:
	sim::Burn fBurn;
	bool fDone;
	bool fActive;
};


struct PlanJob {
	BMessenger target;
	Mission::PlanRequest request;
};

int32
PlanThread(void* data)
{
	PlanJob* job = static_cast<PlanJob*>(data);
	sim::PlanResult result = Mission::Solve(job->request);
	BMessage done(MSG_PLAN_DONE);
	BurnWindow::AddToMessage(&done, result.burn);
	done.AddBool("ok", result.ok);
	done.AddString("message", result.message.c_str());
	job->target.SendMessage(&done);
	delete job;
	return 0;
}

}  // namespace


MainWindow::MainWindow()
	:
	BWindow(BRect(40, 40, 1300, 780), "Columbia — Apollo flight simulator",
		B_TITLED_WINDOW, B_AUTO_UPDATE_SIZE_LIMITS | B_QUIT_ON_WINDOW_CLOSE),
	fRunner(nullptr),
	fLastTick(system_time()),
	fTickCount(0),
	fPlanning(false),
	fPausedForPlanning(false),
	fOpenPanel(nullptr),
	fSavePanel(nullptr)
{
	BMenuBar* menuBar = _BuildMenu();

	fDSKY = new DSKYView(&fMission);
	fTrajectory = new TrajectoryView(&fMission);

	// Flight status.
	fClock = new BStringView("clock", "");
	BFont big(be_bold_font);
	big.SetSize(big.Size() * 1.25f);
	fClock->SetFont(&big);
	fWhere = new BStringView("where", "");
	fOrbit = new BStringView("orbit", "");
	fPropellant = new BStringView("propellant", "");
	fNext = new BStringView("next", "");

	fPauseButton = new BButton("Pause", new BMessage(MSG_PAUSE));
	fSlowerButton = new BButton("\xc2\xab Slower", new BMessage(MSG_SLOWER));
	fFasterButton = new BButton("Faster \xc2\xbb", new BMessage(MSG_FASTER));
	fWarpLabel = new BStringView("warp", "1\xc3\x97");
	fWarpLabel->SetAlignment(B_ALIGN_CENTER);
	fWarpLabel->SetExplicitMinSize(BSize(70, B_SIZE_UNSET));

	BBox* flightBox = new BBox("flight");
	flightBox->SetLabel("Flight");
	BLayoutBuilder::Group<>(flightBox, B_VERTICAL, B_USE_HALF_ITEM_SPACING)
		.SetInsets(B_USE_DEFAULT_SPACING, B_USE_BIG_SPACING,
			B_USE_DEFAULT_SPACING, B_USE_DEFAULT_SPACING)
		.Add(fClock)
		.Add(fWhere)
		.Add(fOrbit)
		.Add(fPropellant)
		.Add(fNext)
		.AddGroup(B_HORIZONTAL)
			.Add(fSlowerButton)
			.Add(fWarpLabel)
			.Add(fFasterButton)
			.AddGlue()
			.Add(fPauseButton)
		.End();

	// Flight plan.
	fPlanList = new BListView("plan");
	fPlanList->SetSelectionMessage(new BMessage(MSG_PLAN_SELECTED));
	fPlanList->SetInvocationMessage(new BMessage(MSG_EDIT_BURN));
	BScrollView* planScroll = new BScrollView("planScroll", fPlanList, 0,
		false, true);
	planScroll->SetExplicitMinSize(BSize(B_SIZE_UNSET, 96));

	BButton* addButton = new BButton("Add\xe2\x80\xa6", new BMessage(MSG_ADD_BURN));
	fEditButton = new BButton("Edit\xe2\x80\xa6", new BMessage(MSG_EDIT_BURN));
	fDeleteButton = new BButton("Delete", new BMessage(MSG_DELETE_BURN));
	fEditButton->SetEnabled(false);
	fDeleteButton->SetEnabled(false);

	fPlanButtons[0] = new BButton("TLI", new BMessage(MSG_PLAN_TLI));
	fPlanButtons[1] = new BButton("MCC", new BMessage(MSG_PLAN_MCC));
	fPlanButtons[2] = new BButton("LOI", new BMessage(MSG_PLAN_LOI));
	fPlanButtons[3] = new BButton("TEI", new BMessage(MSG_PLAN_TEI));
	fPlanButtons[0]->SetToolTip("Free-return translunar injection with the "
		"S-IVB (pericynthion 110 km, safe entry if nothing else is done)");
	fPlanButtons[1]->SetToolTip("Mid-course correction: trims pericynthion "
		"on the way out, or the entry angle on the way home");
	fPlanButtons[2]->SetToolTip("Lunar orbit insertion at the next "
		"pericynthion");
	fPlanButtons[3]->SetToolTip("Trans-earth injection from lunar orbit");

	fPredictMoon = new BStringView("predictMoon", "");
	fPredictEarth = new BStringView("predictEarth", "");
	fNotice = new BStringView("notice", "");

	BStringView* helpers = new BStringView("helpers", "Plan with the RTCC:");
	BBox* planBox = new BBox("planBox");
	planBox->SetLabel("Flight plan");
	BLayoutBuilder::Group<>(planBox, B_VERTICAL, B_USE_HALF_ITEM_SPACING)
		.SetInsets(B_USE_DEFAULT_SPACING, B_USE_BIG_SPACING,
			B_USE_DEFAULT_SPACING, B_USE_DEFAULT_SPACING)
		.Add(planScroll)
		.AddGroup(B_HORIZONTAL)
			.Add(addButton)
			.Add(fEditButton)
			.Add(fDeleteButton)
			.AddGlue()
		.End()
		.AddGroup(B_HORIZONTAL, B_USE_HALF_ITEM_SPACING)
			.Add(helpers)
			.Add(fPlanButtons[0])
			.Add(fPlanButtons[1])
			.Add(fPlanButtons[2])
			.Add(fPlanButtons[3])
		.End()
		.Add(fPredictMoon)
		.Add(fPredictEarth)
		.Add(fNotice);

	// Trajectory controls.
	BPopUpMenu* frames = new BPopUpMenu("frame");
	const char* frameNames[] = {"Earth–Moon rotating", "Earth inertial",
		"Moon-centred"};
	for (int i = 0; i < 3; i++) {
		BMessage* m = new BMessage(MSG_FRAME);
		m->AddInt32("frame", i);
		BMenuItem* item = new BMenuItem(frameNames[i], m);
		if (i == 0)
			item->SetMarked(true);
		frames->AddItem(item);
	}
	fFrameField = new BMenuField("View:", frames);
	fFollowBox = new BCheckBox("Follow spacecraft", new BMessage(MSG_FOLLOW));
	BButton* fitButton = new BButton("Fit", new BMessage(MSG_FIT));

	fLog = new BListView("log");
	BScrollView* logScroll = new BScrollView("logScroll", fLog, 0, false,
		true);
	logScroll->SetExplicitMinSize(BSize(160, 90));

	BGroupView* trajectoryGroup = new BGroupView(B_VERTICAL, 0);
	BLayoutBuilder::Group<>(trajectoryGroup)
		.AddGroup(B_HORIZONTAL)
			.SetInsets(B_USE_HALF_ITEM_INSETS)
			.Add(fFrameField)
			.Add(fFollowBox)
			.AddGlue()
			.Add(fitButton)
		.End()
		.Add(fTrajectory);

	// Status lines change constantly; fix their size so the layout (and
	// the window) never jumps around as the text changes.
	for (BStringView* label : {fClock, fWhere, fOrbit, fPropellant, fNext,
			fPredictMoon, fPredictEarth, fNotice}) {
		label->SetExplicitMinSize(BSize(60, B_SIZE_UNSET));
		label->SetExplicitMaxSize(BSize(B_SIZE_UNLIMITED, B_SIZE_UNSET));
	}
	flightBox->SetExplicitMaxSize(BSize(DSKYView::kWidth, B_SIZE_UNSET));

	// Left: the DSKY and the flight status. Right: the path, with the
	// flight plan and the event log beneath it.
	BLayoutBuilder::Group<>(this, B_VERTICAL, 0)
		.Add(menuBar)
		.AddGroup(B_HORIZONTAL, B_USE_DEFAULT_SPACING)
			.SetInsets(B_USE_WINDOW_SPACING)
			.AddGroup(B_VERTICAL, B_USE_DEFAULT_SPACING, 0)
				.Add(fDSKY)
				.Add(flightBox)
				.AddGlue()
			.End()
			.AddSplit(B_VERTICAL, B_USE_HALF_ITEM_SPACING, 1)
				.Add(trajectoryGroup, 3)
				.AddGroup(B_HORIZONTAL, B_USE_DEFAULT_SPACING, 1)
					.Add(planBox, 3)
					.Add(logScroll, 2)
				.End()
			.End()
		.End();

	_UpdateWarp();
	_UpdatePlanList();
	_DrainEvents();
	fDSKY->MakeFocus(true);

	BMessage tick(MSG_TICK);
	fRunner = new BMessageRunner(BMessenger(this), &tick, kTickInterval);
}


MainWindow::~MainWindow()
{
	delete fRunner;
	delete fOpenPanel;
	delete fSavePanel;
}


BMenuBar*
MainWindow::_BuildMenu()
{
	BMenuBar* bar = new BMenuBar("menu");

	BMenu* file = new BMenu("Flight");
	file->AddItem(new BMenuItem("New flight", new BMessage(MSG_NEW_FLIGHT),
		'N'));
	file->AddSeparatorItem();
	file->AddItem(new BMenuItem("Open flight plan\xe2\x80\xa6",
		new BMessage(MSG_OPEN_PLAN), 'O'));
	file->AddItem(new BMenuItem("Save flight plan\xe2\x80\xa6",
		new BMessage(MSG_SAVE_PLAN), 'S'));
	file->AddSeparatorItem();
	file->AddItem(new BMenuItem("About Columbia\xe2\x80\xa6",
		new BMessage(MSG_ABOUT)));
	file->AddItem(new BMenuItem("Quit", new BMessage(B_QUIT_REQUESTED), 'Q'));
	bar->AddItem(file);

	BMenu* sim = new BMenu("Simulation");
	fPauseItem = new BMenuItem("Pause", new BMessage(MSG_PAUSE), 'P');
	sim->AddItem(fPauseItem);
	sim->AddItem(new BMenuItem("Faster", new BMessage(MSG_FASTER), '.'));
	sim->AddItem(new BMenuItem("Slower", new BMessage(MSG_SLOWER), ','));
	sim->AddSeparatorItem();
	fAutoSlowItem = new BMenuItem("Slow down before burns",
		new BMessage(MSG_AUTO_SLOW));
	fAutoSlowItem->SetMarked(true);
	sim->AddItem(fAutoSlowItem);
	sim->AddItem(new BMenuItem("Clear future burns",
		new BMessage(MSG_CLEAR_PLAN)));
	bar->AddItem(sim);

	struct KeySequence { const char* name; const char* keys; };
	const KeySequence sequences[] = {
		{"Fresh start (V36)", "V36E"},
		{"Program 00 (V37 E 00 E)", "V37E00E"},
		{"Mission clock (V16 N65)", "V16N65E"},
		{"Program alarm codes (V05 N09)", "V05N09E"},
		{"Lamp test (V35)", "V35E"},
		{"Reset (RSET)", "R"},
	};
	BMenu* computer = new BMenu("Computer");
	for (const KeySequence& s : sequences) {
		BMessage* m = new BMessage(MSG_KEYS);
		m->AddString("keys", s.keys);
		computer->AddItem(new BMenuItem(s.name, m));
	}
	bar->AddItem(computer);

	return bar;
}


bool
MainWindow::QuitRequested()
{
	be_app->PostMessage(B_QUIT_REQUESTED);
	return true;
}


// ---------------------------------------------------------------------------
// Messages


void
MainWindow::MessageReceived(BMessage* message)
{
	switch (message->what) {
		case MSG_TICK:
			_Tick();
			break;

		case MSG_NEW_FLIGHT:
			_NewFlight();
			break;

		case MSG_PAUSE:
			if (fPlanning)
				break;
			fMission.SetPaused(!fMission.Paused());
			fPauseButton->SetLabel(fMission.Paused() ? "Run" : "Pause");
			fPauseItem->SetMarked(fMission.Paused());
			break;
		case MSG_FASTER:
			_ChangeWarp(1);
			break;
		case MSG_SLOWER:
			_ChangeWarp(-1);
			break;
		case MSG_AUTO_SLOW:
			fMission.SetAutoSlow(!fMission.AutoSlow());
			fAutoSlowItem->SetMarked(fMission.AutoSlow());
			break;

		case MSG_PLAN_SELECTED:
		{
			BurnItem* item = dynamic_cast<BurnItem*>(
				fPlanList->ItemAt(fPlanList->CurrentSelection()));
			bool editable = item != nullptr
				&& fMission.Editable(item->Burn().id);
			fEditButton->SetEnabled(editable);
			fDeleteButton->SetEnabled(editable);
			break;
		}
		case MSG_ADD_BURN:
			_EditBurn(true);
			break;
		case MSG_EDIT_BURN:
			_EditBurn(false);
			break;
		case MSG_DELETE_BURN:
		{
			BurnItem* item = dynamic_cast<BurnItem*>(
				fPlanList->ItemAt(fPlanList->CurrentSelection()));
			if (item != nullptr && fMission.RemoveBurn(item->Burn().id))
				_UpdatePlanList();
			break;
		}
		case MSG_CLEAR_PLAN:
			fMission.ClearFuturePlan();
			_UpdatePlanList();
			break;
		case MSG_BURN_EDITED:
		{
			sim::Burn burn;
			if (!BurnWindow::FromMessage(message, &burn))
				break;
			std::string error;
			bool ok = burn.id == 0 ? fMission.AddBurn(burn, &error)
				: fMission.ReplaceBurn(burn, &error);
			if (!ok)
				_Notice(error, true);
			_UpdatePlanList();
			break;
		}

		case MSG_PLAN_TLI:
			_StartPlanning(Mission::PLAN_TLI, "TLI");
			break;
		case MSG_PLAN_LOI:
			_StartPlanning(Mission::PLAN_LOI, "LOI");
			break;
		case MSG_PLAN_TEI:
			_StartPlanning(Mission::PLAN_TEI, "TEI");
			break;
		case MSG_PLAN_MCC:
			_StartPlanning(Mission::PLAN_MCC, "a course correction");
			break;
		case MSG_PLAN_DONE:
			_PlanFinished(message);
			break;

		case MSG_FRAME:
		{
			int32 frame;
			if (message->FindInt32("frame", &frame) == B_OK)
				fTrajectory->SetFrameMode(ViewFrame(frame));
			break;
		}
		case MSG_FOLLOW:
			fTrajectory->SetFollow(fFollowBox->Value() == B_CONTROL_ON);
			break;
		case MSG_FIT:
			fFollowBox->SetValue(B_CONTROL_OFF);
			fTrajectory->SetFollow(false);
			fTrajectory->Fit();
			break;

		case MSG_KEYS:
		{
			const char* keys;
			if (message->FindString("keys", &keys) == B_OK)
				fMission.QueueKeys(keys);
			fDSKY->MakeFocus(true);
			break;
		}

		case MSG_OPEN_PLAN:
		{
			entry_ref ref;
			if (message->FindRef("refs", &ref) == B_OK) {
				BPath path(&ref);
				std::string error;
				if (!fMission.LoadPlan(path.Path(), &error))
					_Notice(error, true);
				else
					_Notice(std::string("Loaded ") + ref.name, false);
				_UpdatePlanList();
				break;
			}
			if (fOpenPanel == nullptr) {
				BMessenger self(this);
				fOpenPanel = new BFilePanel(B_OPEN_PANEL, &self, nullptr,
					B_FILE_NODE, false, new BMessage(MSG_OPEN_PLAN));
			}
			fOpenPanel->Show();
			break;
		}
		case MSG_SAVE_PLAN:
		{
			if (fSavePanel == nullptr) {
				BMessenger self(this);
				fSavePanel = new BFilePanel(B_SAVE_PANEL, &self, nullptr,
					B_FILE_NODE, false);
				fSavePanel->SetSaveText("Flight plan.columbia");
			}
			fSavePanel->Show();
			break;
		}
		case B_SAVE_REQUESTED:
		{
			entry_ref directory;
			const char* name;
			if (message->FindRef("directory", &directory) != B_OK
				|| message->FindString("name", &name) != B_OK)
				break;
			BPath path(&directory);
			path.Append(name);
			std::string error;
			if (fMission.SavePlan(path.Path(), &error))
				_Notice(std::string("Saved ") + name, false);
			else
				_Notice(error, true);
			break;
		}

		case MSG_ABOUT:
		{
			BAlert* alert = new BAlert("About Columbia",
				"Columbia\nAn Apollo flight simulator for Haiku\n\n"
				"The DSKY is driven by the real Apollo 11 Command Module "
				"software, Comanche 055 (Colossus 2A), running on a Block II "
				"AGC emulator written for Columbia. The rope was assembled "
				"from the Virtual AGC transcription, which matches the "
				"octal listing of the flight rope bit for bit.\n\n"
				"Flight dynamics: Earth and Moon gravity, the S-IVB and the "
				"SPS with finite burns.\n\nMIT License.", "Fly");
			alert->Go(nullptr);
			break;
		}

		default:
			BWindow::MessageReceived(message);
	}
}


// ---------------------------------------------------------------------------
// Simulation


void
MainWindow::_Tick()
{
	bigtime_t now = system_time();
	double dt = std::min(0.1, (now - fLastTick) / 1e6);
	fLastTick = now;
	fMission.Update(dt);
	fTickCount++;

	fDSKY->Refresh();
	if (fTickCount % 3 == 0)
		fTrajectory->Refresh();
	if (fTickCount % 6 == 0) {
		_UpdateStatus();
		_UpdatePlanList();
		_UpdateWarp();
	}
	_DrainEvents();
}


void
MainWindow::_UpdateWarp()
{
	BString text;
	if (fMission.Warp() >= 1000)
		text.SetToFormat("%sk\xc3\x97",
			Thousands(fMission.Warp() / 1000).c_str());
	else
		text.SetToFormat("%.0f\xc3\x97", fMission.Warp());
	if (text != fWarpLabel->Text())
		fWarpLabel->SetText(text.String());
}


void
MainWindow::_ChangeWarp(int direction)
{
	const std::vector<double>& levels = Mission::WarpLevels();
	size_t index = 0;
	for (size_t i = 0; i < levels.size(); i++) {
		if (levels[i] <= fMission.Warp() + 1e-9)
			index = i;
	}
	if (direction > 0 && index + 1 < levels.size())
		index++;
	else if (direction < 0 && index > 0)
		index--;
	fMission.SetWarp(levels[index]);
	_UpdateWarp();
}


void
MainWindow::_UpdateStatus()
{
	const sim::VehicleState& s = fMission.State();
	sim::OrbitSummary o = sim::Summarize(fMission.Model(), s);

	BString text;
	text.SetToFormat("GET %s%s", sim::FormatGET(s.t).c_str(),
		fMission.Paused() ? "  (paused)" : "");
	fClock->SetText(text.String());

	text.SetToFormat("%s \xe2\x80\x94 altitude %s, %s m/s",
		o.aroundMoon ? "Moon" : "Earth", Altitude(o.altitude).c_str(),
		Thousands(o.speed).c_str());
	fWhere->SetText(text.String());

	if (o.apoapsis > 0)
		text.SetToFormat("%s %s \xc2\xb7 %s %s",
			o.aroundMoon ? "Perilune" : "Perigee", Altitude(o.periapsis).c_str(),
			o.aroundMoon ? "apolune" : "apogee", Altitude(o.apoapsis).c_str());
	else
		text.SetToFormat("Escape trajectory \xc2\xb7 %s %s",
			o.aroundMoon ? "perilune" : "perigee", Altitude(o.periapsis).c_str());
	fOrbit->SetText(text.String());

	if (s.sivbAttached)
		text.SetToFormat("Propellant: S-IVB %s kg \xc2\xb7 SPS %s kg",
			Thousands(s.sivbPropellant).c_str(),
			Thousands(s.spsPropellant).c_str());
	else
		text.SetToFormat("Propellant: SPS %s kg (S-IVB jettisoned)",
			Thousands(s.spsPropellant).c_str());
	fPropellant->SetText(text.String());

	if (fMission.Finished()) {
		fNext->SetText(sim::OutcomeText(s.outcome));
	} else if (s.activeBurn != 0) {
		std::string label = "Burn";
		for (const sim::Burn& b : fMission.Plan())
			if (b.id == s.activeBurn)
				label = b.label;
		text.SetToFormat("BURNING %s: %.0f of %.0f m/s", label.c_str(),
			s.burnDone, s.burnTarget);
		fNext->SetText(text.String());
	} else {
		const sim::Burn* next = nullptr;
		for (const sim::Burn& b : fMission.Plan())
			if (!s.Completed(b.id) && (next == nullptr || b.time < next->time))
				next = &b;
		if (next != nullptr)
			text.SetToFormat("Next: %s at %s (in %s)", next->label.c_str(),
				sim::FormatGET(next->time).c_str(),
				sim::FormatGET(next->time - s.t).c_str());
		else
			text = "No burns planned";
		fNext->SetText(text.String());
	}

	// Prediction summary.
	const sim::Prediction& p = fMission.Prediction();
	if (p.hasPericynthion && p.pericynthionTime > s.t)
		text.SetToFormat("Predicted pericynthion %s at %s",
			Altitude(p.pericynthionAltitude).c_str(),
			sim::FormatGET(p.pericynthionTime).c_str());
	else if (p.final.outcome == sim::Outcome::LunarImpact)
		text = "Predicted: lunar impact!";
	else if (o.aroundMoon)
		text = "In the Moon's sphere of influence";
	else
		text = "No lunar encounter predicted";
	fPredictMoon->SetText(text.String());

	rgb_color good = {30, 140, 50, 255};
	rgb_color bad = {200, 50, 30, 255};
	switch (p.final.outcome) {
		case sim::Outcome::Splashdown:
			text.SetToFormat("Entry at %s: %.1f km/s, %.2f\xc2\xb0 \xe2\x80\x94 "
				"in the corridor", sim::FormatGET(p.final.t).c_str(),
				p.final.entrySpeed / 1000, p.final.entryAngle);
			fPredictEarth->SetHighColor(good);
			break;
		case sim::Outcome::SkipOut:
		case sim::Outcome::TooSteep:
			text.SetToFormat("Entry at %s: %.2f\xc2\xb0 \xe2\x80\x94 %s",
				sim::FormatGET(p.final.t).c_str(), p.final.entryAngle,
				p.final.outcome == sim::Outcome::SkipOut ? "too shallow"
					: "too steep");
			fPredictEarth->SetHighColor(bad);
			break;
		case sim::Outcome::LunarImpact:
			text = "No return to Earth";
			fPredictEarth->SetHighColor(bad);
			break;
		default:
			text = "No Earth entry in the next 12 days";
			fPredictEarth->SetHighColor(ui_color(B_PANEL_TEXT_COLOR));
	}
	fPredictEarth->SetText(text.String());
	fPredictEarth->Invalidate();
}


void
MainWindow::_UpdatePlanList()
{
	const sim::VehicleState& s = fMission.State();
	std::string signature;
	for (const sim::Burn& b : fMission.Plan()) {
		char part[160];
		std::snprintf(part, sizeof(part), "%u:%s:%d:%.3f:%.3f:%d:%d|", b.id,
			b.label.c_str(), b.engine, b.time, b.DeltaV(),
			s.Completed(b.id), s.activeBurn == b.id);
		signature += part;
	}
	if (signature == fPlanSignature)
		return;
	fPlanSignature = signature;

	uint32 selected = 0;
	if (BurnItem* item = dynamic_cast<BurnItem*>(
			fPlanList->ItemAt(fPlanList->CurrentSelection())))
		selected = item->Burn().id;
	while (BListItem* item = fPlanList->RemoveItem(int32(0)))
		delete item;
	for (const sim::Burn& b : fMission.Plan()) {
		fPlanList->AddItem(new BurnItem(b,
			s.Completed(b.id) && s.activeBurn != b.id, s.activeBurn == b.id));
		if (b.id == selected)
			fPlanList->Select(fPlanList->CountItems() - 1);
	}
	if (selected == 0 || fPlanList->CurrentSelection() < 0) {
		fEditButton->SetEnabled(false);
		fDeleteButton->SetEnabled(false);
	}
}


void
MainWindow::_DrainEvents()
{
	std::vector<std::string> events = fMission.TakeEvents();
	if (events.empty())
		return;
	for (const std::string& e : events)
		fLog->AddItem(new BStringItem(e.c_str()));
	while (fLog->CountItems() > 500)
		delete fLog->RemoveItem(int32(0));
	float bottom = fLog->ItemFrame(fLog->CountItems() - 1).bottom;
	fLog->ScrollTo(0, std::max(0.0f, bottom - fLog->Bounds().Height()));
}


void
MainWindow::_Notice(const std::string& text, bool problem)
{
	fNotice->SetHighColor(problem ? rgb_color{200, 50, 30, 255}
		: ui_color(B_PANEL_TEXT_COLOR));
	fNotice->SetText(text.c_str());
	fNotice->Invalidate();
}


void
MainWindow::_NewFlight()
{
	if (fPlanning)
		return;
	fMission.Reset();
	while (BListItem* item = fLog->RemoveItem(int32(0)))
		delete item;
	fPlanSignature.clear();
	_UpdatePlanList();
	_DrainEvents();
	_UpdateWarp();
	fPauseButton->SetLabel("Pause");
	fPauseItem->SetMarked(false);
	_Notice("", false);
	fTrajectory->Fit();
}


void
MainWindow::_EditBurn(bool create)
{
	sim::Burn burn;
	if (create) {
		burn.id = 0;
		burn.time = fMission.Time() + 30 * 60;
		burn.engine = fMission.State().sivbAttached ? sim::ENGINE_SIVB
			: sim::ENGINE_SPS;
		burn.label = "Burn";
		burn.prograde = 10;
	} else {
		BurnItem* item = dynamic_cast<BurnItem*>(
			fPlanList->ItemAt(fPlanList->CurrentSelection()));
		if (item == nullptr || !fMission.Editable(item->Burn().id))
			return;
		burn = item->Burn();
	}
	BPoint where = Frame().LeftTop() + BPoint(120, 160);
	BurnWindow* window = new BurnWindow(BMessenger(this), burn,
		fMission.State().sivbAttached, where);
	window->Show();
}


void
MainWindow::_StartPlanning(Mission::PlanKind kind, const char* name)
{
	if (fPlanning || fMission.Finished())
		return;
	fPlanning = true;
	fPausedForPlanning = !fMission.Paused();
	fMission.SetPaused(true);
	for (BButton* b : fPlanButtons)
		b->SetEnabled(false);
	_Notice(std::string("Planning ") + name + "\xe2\x80\xa6", false);

	PlanJob* job = new PlanJob;
	job->target = BMessenger(this);
	job->request = fMission.MakeRequest(kind);
	thread_id thread = spawn_thread(PlanThread, "planner",
		B_NORMAL_PRIORITY, job);
	if (thread < 0 || resume_thread(thread) != B_OK) {
		delete job;
		fPlanning = false;
		_Notice("Could not start the planner.", true);
	}
}


void
MainWindow::_PlanFinished(BMessage* message)
{
	sim::PlanResult result;
	const char* text = "";
	message->FindBool("ok", &result.ok);
	message->FindString("message", &text);
	result.message = text;
	BurnWindow::FromMessage(message, &result.burn);
	fMission.Adopt(result);
	_Notice(result.message, !result.ok);

	fPlanning = false;
	if (fPausedForPlanning)
		fMission.SetPaused(false);
	for (BButton* b : fPlanButtons)
		b->SetEnabled(true);
	fLastTick = system_time();
	_UpdatePlanList();
	_UpdateStatus();
}
