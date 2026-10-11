// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include <Alert.h>
#include <Application.h>
#include <Directory.h>
#include <Entry.h>
#include <FindDirectory.h>
#include <Path.h>
#include <Roster.h>

#include <string>

#include "MissionSelectWindow.h"

static const char* kSignature = "application/x-vnd.linkslice-Columbia";

namespace {

// The flight software: next to the application, or installed as data.
std::string
FindRopes()
{
	app_info info;
	if (be_app->GetAppInfo(&info) == B_OK) {
		BEntry entry(&info.ref);
		BPath path;
		if (entry.GetPath(&path) == B_OK && path.GetParent(&path) == B_OK) {
			path.Append("rope");
			if (BEntry(path.Path()).Exists())
				return path.Path();
		}
	}
	const directory_which dirs[] = {
		B_USER_NONPACKAGED_DATA_DIRECTORY, B_USER_DATA_DIRECTORY,
		B_SYSTEM_NONPACKAGED_DATA_DIRECTORY, B_SYSTEM_DATA_DIRECTORY
	};
	for (directory_which which : dirs) {
		BPath path;
		if (find_directory(which, &path) != B_OK)
			continue;
		path.Append("Columbia/rope");
		if (BEntry(path.Path()).Exists())
			return path.Path();
	}
	return "rope";
}

}  // namespace


class ColumbiaApp : public BApplication {
public:
	ColumbiaApp() : BApplication(kSignature) {}

	void ReadyToRun() override
	{
		fSelect = new columbia::MissionSelectWindow(FindRopes());
		fSelect->CenterOnScreen();
		fSelect->Show();
	}

	void MessageReceived(BMessage* message) override
	{
		if (message->what == 'flcl') {
			// A flight closed: its recording may be new.
			if (fSelect)
				fSelect->PostMessage(message);
			return;
		}
		BApplication::MessageReceived(message);
	}

private:
	columbia::MissionSelectWindow* fSelect = nullptr;
};


int
main()
{
	ColumbiaApp app;
	app.Run();
	return 0;
}
