// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "BurnWindow.h"

#include <Button.h>
#include <LayoutBuilder.h>
#include <Menu.h>
#include <MenuField.h>
#include <MenuItem.h>
#include <PopUpMenu.h>
#include <String.h>
#include <StringView.h>
#include <TextControl.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

enum {
	MSG_OK = 'bwOK',
	MSG_CANCEL = 'bwCN',
	MSG_CHANGED = 'bwCH',
	MSG_ENGINE = 'bwEN'
};

bool
ParseNumber(const char* text, double* value)
{
	char* end = nullptr;
	*value = std::strtod(text, &end);
	while (end != nullptr && *end == ' ')
		end++;
	return end != text && end != nullptr && *end == '\0';
}

BString
Number(double value)
{
	BString text;
	text.SetToFormat("%.2f", value);
	return text;
}

}  // namespace


BurnWindow::BurnWindow(BMessenger target, const sim::Burn& burn,
	bool sivbAvailable, BPoint where)
	:
	BWindow(BRect(where, where + BPoint(360, 260)),
		burn.id == 0 ? "New burn" : "Edit burn", B_FLOATING_WINDOW_LOOK,
		B_MODAL_APP_WINDOW_FEEL,
		B_NOT_ZOOMABLE | B_NOT_RESIZABLE | B_AUTO_UPDATE_SIZE_LIMITS
			| B_CLOSE_ON_ESCAPE),
	fTarget(target),
	fBurn(burn)
{
	fLabel = new BTextControl("Label:", burn.label.c_str(), nullptr);

	BPopUpMenu* engines = new BPopUpMenu("engine");
	BMessage* sivb = new BMessage(MSG_ENGINE);
	sivb->AddInt32("engine", sim::ENGINE_SIVB);
	BMessage* sps = new BMessage(MSG_ENGINE);
	sps->AddInt32("engine", sim::ENGINE_SPS);
	BMenuItem* sivbItem = new BMenuItem("S-IVB (J-2, 1000 kN)", sivb);
	BMenuItem* spsItem = new BMenuItem("SPS (91 kN)", sps);
	sivbItem->SetEnabled(sivbAvailable);
	engines->AddItem(sivbItem);
	engines->AddItem(spsItem);
	(burn.engine == sim::ENGINE_SIVB && sivbAvailable ? sivbItem : spsItem)
		->SetMarked(true);
	if (!sivbAvailable)
		fBurn.engine = sim::ENGINE_SPS;
	fEngine = new BMenuField("Engine:", engines);

	fTime = new BTextControl("Ignition (GET):",
		sim::FormatGET(burn.time).c_str(), new BMessage(MSG_CHANGED));
	fPrograde = new BTextControl("Prograde (m/s):",
		Number(burn.prograde).String(), new BMessage(MSG_CHANGED));
	fNormal = new BTextControl("Normal (m/s):", Number(burn.normal).String(),
		new BMessage(MSG_CHANGED));
	fRadial = new BTextControl("Radial out (m/s):",
		Number(burn.radial).String(), new BMessage(MSG_CHANGED));
	for (BTextControl* c : {fTime, fPrograde, fNormal, fRadial})
		c->SetModificationMessage(new BMessage(MSG_CHANGED));

	fTotal = new BStringView("total", "");
	fError = new BStringView("error", "");
	fError->SetHighColor(200, 40, 30);

	BButton* ok = new BButton(burn.id == 0 ? "Add" : "Apply",
		new BMessage(MSG_OK));
	BButton* cancel = new BButton("Cancel", new BMessage(MSG_CANCEL));

	BLayoutBuilder::Group<>(this, B_VERTICAL)
		.SetInsets(B_USE_WINDOW_SPACING)
		.AddGrid()
			.AddTextControl(fLabel, 0, 0)
			.AddMenuField(fEngine, 0, 1)
			.AddTextControl(fTime, 0, 2)
			.AddTextControl(fPrograde, 0, 3)
			.AddTextControl(fNormal, 0, 4)
			.AddTextControl(fRadial, 0, 5)
		.End()
		.Add(fTotal)
		.Add(fError)
		.AddGroup(B_HORIZONTAL)
			.AddGlue()
			.Add(cancel)
			.Add(ok)
		.End();

	SetDefaultButton(ok);
	_UpdateTotal();
	fTime->MakeFocus(true);
}


bool
BurnWindow::_Read(sim::Burn* burn, BString* error) const
{
	*burn = fBurn;
	burn->label = fLabel->Text();
	if (!sim::ParseGET(fTime->Text(), &burn->time)) {
		*error = "Ignition time must look like 075:43:10.";
		return false;
	}
	if (!ParseNumber(fPrograde->Text(), &burn->prograde)
		|| !ParseNumber(fNormal->Text(), &burn->normal)
		|| !ParseNumber(fRadial->Text(), &burn->radial)) {
		*error = "Delta-v components must be numbers.";
		return false;
	}
	if (burn->DeltaV() <= 0) {
		*error = "The burn needs some delta-v.";
		return false;
	}
	return true;
}


void
BurnWindow::_UpdateTotal()
{
	sim::Burn burn;
	BString error;
	if (_Read(&burn, &error)) {
		const sim::EngineSpec& e = sim::Engine(burn.engine);
		BString text;
		text.SetToFormat("Total %.1f m/s on the %s", burn.DeltaV(), e.name);
		fTotal->SetText(text.String());
		fError->SetText("");
	} else {
		fTotal->SetText("");
		fError->SetText(error.String());
	}
}


void
BurnWindow::MessageReceived(BMessage* message)
{
	switch (message->what) {
		case MSG_CHANGED:
			_UpdateTotal();
			break;
		case MSG_ENGINE:
		{
			int32 engine;
			if (message->FindInt32("engine", &engine) == B_OK)
				fBurn.engine = sim::EngineId(engine);
			_UpdateTotal();
			break;
		}
		case MSG_OK:
		{
			sim::Burn burn;
			BString error;
			if (!_Read(&burn, &error)) {
				fError->SetText(error.String());
				break;
			}
			BMessage edited(MSG_BURN_EDITED);
			AddToMessage(&edited, burn);
			fTarget.SendMessage(&edited);
			Quit();
			break;
		}
		case MSG_CANCEL:
			Quit();
			break;
		default:
			BWindow::MessageReceived(message);
	}
}


void
BurnWindow::AddToMessage(BMessage* message, const sim::Burn& burn)
{
	message->AddUInt32("id", burn.id);
	message->AddString("label", burn.label.c_str());
	message->AddInt32("engine", burn.engine);
	message->AddDouble("time", burn.time);
	message->AddDouble("prograde", burn.prograde);
	message->AddDouble("normal", burn.normal);
	message->AddDouble("radial", burn.radial);
}


bool
BurnWindow::FromMessage(const BMessage* message, sim::Burn* burn)
{
	const char* label;
	int32 engine;
	if (message->FindUInt32("id", &burn->id) != B_OK
		|| message->FindString("label", &label) != B_OK
		|| message->FindInt32("engine", &engine) != B_OK
		|| message->FindDouble("time", &burn->time) != B_OK
		|| message->FindDouble("prograde", &burn->prograde) != B_OK
		|| message->FindDouble("normal", &burn->normal) != B_OK
		|| message->FindDouble("radial", &burn->radial) != B_OK)
		return false;
	burn->label = label;
	burn->engine = sim::EngineId(engine);
	return true;
}
