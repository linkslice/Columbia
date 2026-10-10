// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#ifndef COLUMBIA_MAIN_WINDOW_H
#define COLUMBIA_MAIN_WINDOW_H

#include <Window.h>

#include <string>

#include "Mission.h"

class BButton;
class BCheckBox;
class BFilePanel;
class BListView;
class BMenuField;
class BMenuBar;
class BMenuItem;
class BMessageRunner;
class BStringView;
class DSKYView;
class TrajectoryView;

class MainWindow : public BWindow {
public:
	MainWindow();
	~MainWindow() override;

	void MessageReceived(BMessage* message) override;
	bool QuitRequested() override;

private:
	BMenuBar* _BuildMenu();
	void _Tick();
	void _UpdateStatus();
	void _UpdatePlanList();
	void _UpdateWarp();
	void _DrainEvents();
	void _StartPlanning(columbia::Mission::PlanKind kind, const char* name);
	void _PlanFinished(BMessage* message);
	void _EditBurn(bool create);
	void _Notice(const std::string& text, bool problem);
	void _ChangeWarp(int direction);
	void _NewFlight();

	columbia::Mission fMission;

	DSKYView* fDSKY;
	TrajectoryView* fTrajectory;
	BListView* fPlanList;
	BListView* fLog;

	BStringView* fClock;
	BStringView* fWhere;
	BStringView* fOrbit;
	BStringView* fPropellant;
	BStringView* fNext;
	BStringView* fPredictMoon;
	BStringView* fPredictEarth;
	BStringView* fNotice;

	BButton* fPauseButton;
	BButton* fSlowerButton;
	BButton* fFasterButton;
	BStringView* fWarpLabel;
	BButton* fEditButton;
	BButton* fDeleteButton;
	BButton* fPlanButtons[4];
	BMenuField* fFrameField;
	BCheckBox* fFollowBox;
	BMenuItem* fPauseItem;
	BMenuItem* fAutoSlowItem;

	BMessageRunner* fRunner;
	bigtime_t fLastTick;
	uint32 fTickCount;
	std::string fPlanSignature;
	bool fPlanning;
	bool fPausedForPlanning;

	BFilePanel* fOpenPanel;
	BFilePanel* fSavePanel;
};

#endif
