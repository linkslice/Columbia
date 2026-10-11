// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// The controls view (the default): the spacecraft's main panel as the
// crew flies it. The FDAI ball, the DSKY you can key, the switches that
// matter (engine arming, abort, SCE to AUX, the RCS quads, the pyros), the
// consumables gauges and the caution-and-warning lights. Command Module or
// Lunar Module, following the crew.

#ifndef COLUMBIA_PANEL_VIEW_H
#define COLUMBIA_PANEL_VIEW_H

#include <View.h>

#include "DSKYPainter.h"
#include "agc/DSKY.h"
#include "sim/Math.h"

#include <functional>
#include <string>
#include <vector>

namespace columbia {

class FlightSession;

class PanelView : public BView {
public:
	explicit PanelView(FlightSession* session);

	void Draw(BRect updateRect) override;
	void MouseDown(BPoint where) override;
	void MouseUp(BPoint where) override;
	void KeyDown(const char* bytes, int32 numBytes) override;
	void KeyUp(const char* bytes, int32 numBytes) override;
	void FrameResized(float width, float height) override;

	// Pull the state from the simulation (call from the window's pulse).
	void Refresh();

	// Which panel: 0 follows the crew, 1 the CM, 2 the LM.
	void SetCraft(int craft);
	int Craft() const { return fCraft; }
	bool ShowingLM() const { return fLM; }

	// Logical size the panel is drawn at (scaled to the view).
	static const float kWidth;
	static const float kHeight;

	// Hand controller deflections to draw (set by the window).
	sim::Vec3 rotation;
	int translation[3] = {0, 0, 0};

private:
	struct Control {
		const char* label;
		std::vector<const char*> positions;   // empty: a pushbutton
		BRect frame;                          // logical
		bool guarded = false;
		std::function<int()> get;             // position shown (or lit)
		std::function<void(int)> set;         // to this position / pressed
	};
	struct Gauge {
		const char* label;
		double value = 0;      // 0..1 of full scale
		std::string text;
		bool warn = false;
	};
	struct Lamp {
		const char* label;
		bool on = false;
		bool warning = false;  // red (warning) or amber (caution)
	};

	void _BuildControls();
	void _Snapshot();
	BPoint _ToLogical(BPoint view) const;
	float _Scale() const;

	void _DrawFDAI(BRect r);
	void _DrawTimers(BRect r);
	void _DrawLamps(BRect r);
	void _DrawDSKY(BRect r);
	void _DrawControllers(BRect r);
	void _DrawControl(const Control& c);
	void _DrawGauges(BRect r);

	FlightSession* fSession;
	int fCraft = 0;
	bool fLM = false;
	ui::DSKYPainter fPainter;
	std::vector<Control> fControls;
	bool fControlsLM = false;
	int fPressedKey = -1, fKeyboardKey = -1;
	int fPressedControl = -1;

	// The snapshot drawn.
	agc::DSKYState fDisplay;
	sim::Vec3 fGimbals, fRates;
	bool fIMUFailed = false;
	double fGET = 0, fNextGET = -1;
	std::string fNextName;
	std::string fPhase;
	std::vector<Gauge> fGauges;
	std::vector<Lamp> fLamps;
	bool fMasterAlarm = false;
	std::vector<int> fPositions;          // per control, from get()
	double fThrottle = 0;
	bool fThrottleAuto = true;
	std::string fCraftName;
};

}  // namespace columbia

#endif
