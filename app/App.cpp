// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include <Application.h>

#include "MainWindow.h"

static const char* kSignature = "application/x-vnd.linkslice-Columbia";

class ColumbiaApp : public BApplication {
public:
	ColumbiaApp() : BApplication(kSignature) {}

	void ReadyToRun() override
	{
		MainWindow* window = new MainWindow();
		window->CenterOnScreen();
		window->Show();
	}
};

int
main()
{
	ColumbiaApp app;
	app.Run();
	return 0;
}
