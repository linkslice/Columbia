// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "DecisionWindow.h"

#include <Button.h>
#include <Font.h>
#include <GroupView.h>
#include <LayoutBuilder.h>
#include <Messenger.h>
#include <StringView.h>
#include <TextView.h>

namespace columbia {

namespace {

enum { kMsgOption = 'dopt' };

// A read-only block of text that wraps to the window's width.
BTextView*
Paragraph(const std::string& text, bool bold = false)
{
	BTextView* view = new BTextView("text");
	view->SetText(text.c_str());
	view->MakeEditable(false);
	view->MakeSelectable(false);
	view->SetWordWrap(true);
	view->SetViewUIColor(B_PANEL_BACKGROUND_COLOR);
	if (bold) {
		BFont font(be_bold_font);
		view->SetFontAndColor(&font);
	}
	view->SetExplicitMinSize(BSize(460, B_SIZE_UNSET));
	return view;
}

}  // namespace


DecisionWindow::DecisionWindow(const flight::Decision& decision,
	BMessenger target, bool showHistory)
	:
	BWindow(BRect(0, 0, 520, 300), ("FLIGHT: " + decision.title).c_str(),
		B_TITLED_WINDOW, B_NOT_ZOOMABLE | B_NOT_MINIMIZABLE | B_NOT_CLOSABLE
			| B_AUTO_UPDATE_SIZE_LIMITS | B_ASYNCHRONOUS_CONTROLS),
	fId(decision.id),
	fTarget(target)
{
	BStringView* header = new BStringView("header", decision.title.c_str());
	BFont big(be_bold_font);
	big.SetSize(big.Size() * 1.4);
	header->SetFont(&big);
	BStringView* clock = new BStringView("clock",
		"The clock is stopped. Mission Control is waiting on Flight.");

	BGroupLayout* options = BLayoutBuilder::Group<>(B_VERTICAL, B_USE_SMALL_SPACING);
	for (const flight::DecisionOption& o : decision.options) {
		BMessage* msg = new BMessage(kMsgOption);
		msg->AddString("key", o.key.c_str());
		std::string label = o.label;
		if (showHistory && o.historical)
			label += "  (as flown)";
		BButton* button = new BButton(o.key.c_str(), label.c_str(), msg);
		BLayoutBuilder::Group<>(options)
			.AddGroup(B_VERTICAL, 2)
				.Add(button)
				.Add(Paragraph(o.detail))
			.End();
	}

	BLayoutBuilder::Group<>(this, B_VERTICAL, B_USE_DEFAULT_SPACING)
		.SetInsets(B_USE_WINDOW_INSETS)
		.Add(header)
		.Add(Paragraph(decision.situation))
		.Add(clock)
		.Add(options->View());
}


void
DecisionWindow::MessageReceived(BMessage* message)
{
	if (message->what == kMsgOption) {
		const char* key = nullptr;
		if (message->FindString("key", &key) == B_OK && !fChosen) {
			fChosen = true;
			BMessage decided(kMsgDecided);
			decided.AddString("id", fId.c_str());
			decided.AddString("key", key);
			fTarget.SendMessage(&decided);
			Quit();
		}
		return;
	}
	BWindow::MessageReceived(message);
}


bool
DecisionWindow::QuitRequested()
{
	// Only by choosing (the flight window closes it with the flight).
	return fChosen;
}

}  // namespace columbia
