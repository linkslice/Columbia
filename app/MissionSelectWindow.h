// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Choosing a flight: Apollo 7 to 17 or a mission of your own, where to
// start, how hard, and whether you fly it or watch the crew do it. Also
// opens the flight recordings for replay.

#ifndef COLUMBIA_MISSION_SELECT_WINDOW_H
#define COLUMBIA_MISSION_SELECT_WINDOW_H

#include <Window.h>

#include <string>

class BButton;
class BCheckBox;
class BListView;
class BMenuField;
class BPopUpMenu;
class BStringView;
class BTextControl;
class BTextView;

namespace columbia {

class MissionSelectWindow : public BWindow {
public:
	explicit MissionSelectWindow(const std::string& ropes);

	void MessageReceived(BMessage* message) override;
	bool QuitRequested() override;

private:
	void _ShowMission();
	void _Fly();
	void _FillStarts();
	void _FillRecordings();
	void _Replay();
	int _Selected() const;      // mission number, 0 custom, -1 none

	std::string fRopes;
	BListView* fList;
	BTextView* fBrief;
	BPopUpMenu* fStarts;
	BPopUpMenu* fDifficulty;
	BStringView* fDifficultyBlurb;
	BCheckBox* fWatch;
	BCheckBox* fRealTime;
	BPopUpMenu* fSites;
	BMenuField* fSiteField;
	BTextControl* fDate;
	BTextControl* fName;
	BPopUpMenu* fRecordings;
	BButton* fFly;
};

}  // namespace columbia

#endif
