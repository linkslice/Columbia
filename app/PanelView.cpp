// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "PanelView.h"

#include <InterfaceDefs.h>
#include <Window.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>

#include "FlightSession.h"
#include "agc/AGC.h"

namespace columbia {

using namespace flight;
using sim::Vec3;

const float PanelView::kWidth = 1280;
const float PanelView::kHeight = 720;

namespace {

const rgb_color kPanelGray = {74, 78, 82, 255};
const rgb_color kPanelDark = {46, 49, 52, 255};
const rgb_color kPanelLine = {112, 118, 124, 255};
const rgb_color kLabel = {232, 232, 224, 255};
const rgb_color kLabelDim = {170, 172, 168, 255};
const rgb_color kAmberLamp = {255, 176, 46, 255};
const rgb_color kRedLamp = {255, 70, 56, 255};
const rgb_color kLampOff = {58, 56, 52, 255};
const rgb_color kGreenDigits = {120, 255, 110, 255};
const rgb_color kBlack = {10, 10, 12, 255};
const double kDeg = 3.14159265358979323846 / 180;

void
Label(BView* v, const char* s, float x, float y, float size, rgb_color c,
	int align = 1, bool bold = true)
{
	BFont font(bold ? be_bold_font : be_plain_font);
	font.SetSize(size);
	v->SetFont(&font);
	v->SetHighColor(c);
	float w = v->StringWidth(s);
	if (align == 1)
		x -= w / 2;
	else if (align == 2)
		x -= w;
	v->DrawString(s, BPoint(x, y));
}

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
TankFraction(const sim::Vehicle* v, const char* module, int tank)
{
	if (!v)
		return 0;
	const sim::Module* m = v->FindModule(module);
	if (!m || int(m->tanks.size()) <= tank || m->tanks[tank].capacity <= 0)
		return 0;
	return m->tanks[tank].mass / m->tanks[tank].capacity;
}

double
TankRange(const sim::Vehicle* v, const char* module, int from, int to)
{
	if (!v)
		return 0;
	const sim::Module* m = v->FindModule(module);
	if (!m)
		return 0;
	double mass = 0, cap = 0;
	for (int i = from; i <= to && i < int(m->tanks.size()); i++) {
		mass += m->tanks[i].mass;
		cap += m->tanks[i].capacity;
	}
	return cap > 0 ? mass / cap : 0;
}

// DSKY key from the keyboard (when the panel has focus).
int
KeyIndex(char c)
{
	switch (c) {
		case 'v': case 'V': return 0;
		case 'n': case 'N': return 1;
		case '+': case '=': return 2;
		case '7': return 3;
		case '8': return 4;
		case '9': return 5;
		case 'c': case 'C': case B_BACKSPACE: return 6;
		case '-': return 7;
		case '4': return 8;
		case '5': return 9;
		case '6': return 10;
		case 'p': case 'P': return 11;
		case '0': return 12;
		case '1': return 13;
		case '2': return 14;
		case '3': return 15;
		case 'k': case 'K': return 16;
		case 'e': case 'E': case B_ENTER: return 17;
		case 'r': case 'R': return 18;
	}
	return -1;
}

}  // namespace


PanelView::PanelView(FlightSession* session)
	:
	BView("panel", B_WILL_DRAW | B_NAVIGABLE | B_FRAME_EVENTS),
	fSession(session)
{
	SetViewColor(B_TRANSPARENT_COLOR);
	_BuildControls();
}


void
PanelView::SetCraft(int craft)
{
	fCraft = craft;
	Refresh();
}


float
PanelView::_Scale() const
{
	BRect b = Bounds();
	return std::min((b.Width() + 1) / kWidth, (b.Height() + 1) / kHeight);
}


BPoint
PanelView::_ToLogical(BPoint p) const
{
	float s = _Scale();
	BRect b = Bounds();
	float ox = (b.Width() + 1 - kWidth * s) / 2;
	float oy = (b.Height() + 1 - kHeight * s) / 2;
	return BPoint((p.x - ox) / s, (p.y - oy) / s);
}


void
PanelView::FrameResized(float, float)
{
	Invalidate();
}


// --- The switches ----------------------------------------------------------------

void
PanelView::_BuildControls()
{
	fControls.clear();
	fControlsLM = fLM;
	FlightSession* s = fSession;
	auto sim = [s]() -> FlightSim& { return s->Sim(); };
	// Grid: three columns of switches on the right.
	const float x0 = 748, y0 = 34, w = 168, h = 66, gap = 6;
	int n = 0;
	auto at = [&](void) {
		int col = n % 3, row = n / 3;
		n++;
		return BRect(x0 + col * (w + gap), y0 + row * (h + gap),
			x0 + col * (w + gap) + w, y0 + row * (h + gap) + h);
	};
	auto toggle = [&](const char* label, std::vector<const char*> pos,
		std::function<int()> get, std::function<void(int)> set) {
		Control c;
		c.label = label;
		c.positions = pos;
		c.frame = at();
		c.get = get;
		c.set = set;
		fControls.push_back(c);
	};
	auto push = [&](const char* label, bool guarded, std::function<int()> lit,
		std::function<void(int)> act) {
		Control c;
		c.label = label;
		c.frame = at();
		c.guarded = guarded;
		c.get = lit;
		c.set = act;
		fControls.push_back(c);
	};
	if (!fLM) {
		auto pan = [s]() -> gnc::CMPanel& { return s->Sim().CMPGNCS().panel; };
		toggle("SC CONT", {"SCS", "CMC"},
			[pan]() { return pan().scContCMC ? 1 : 0; },
			[pan](int p) { pan().scContCMC = p == 1; });
		toggle("CMC MODE", {"FREE", "HOLD", "AUTO"},
			[pan]() { return int(pan().mode); },
			[pan](int p) { pan().mode = gnc::CMPanel::Mode(p); });
		toggle("\xce\x94V THRUST", {"OFF", "NORMAL"},
			[pan]() { return pan().spsArmed ? 1 : 0; },
			[pan](int p) { pan().spsArmed = p == 1; });
		toggle("SPS BANKS", {"B ONLY", "A + B"},
			[pan]() { return pan().spsBankA ? 1 : 0; },
			[pan](int p) { pan().spsBankA = p == 1; });
		toggle("ROT CONTR", {"RATE", "MIN IMP"},
			[pan]() { return pan().minimumImpulse ? 1 : 0; },
			[pan](int p) { pan().minimumImpulse = p == 1; });
		toggle("TVC GIMBALS", {"OFF", "ON"},
			[pan]() { return pan().tvcEnable ? 1 : 0; },
			[pan](int p) { pan().tvcEnable = p == 1; });
		toggle("SCE POWER", {"NORM", "AUX"},
			[sim]() { return sim().csmSystems.sceAux ? 1 : 0; },
			[sim](int p) { if (p == 1) sim().SCEToAux(); });
		push("FUEL CELLS\nRESET", false,
			[sim]() { return sim().csmSystems.FuelCellsOnline() < 3 ? 1 : 0; },
			[sim](int) { sim().ResetFuelCells(); });
		push("TOWER\nJETT", true,
			[]() { return 0; }, [sim](int) { sim().JettisonTower(); });
		push("CSM/LV\nSEP", true,
			[sim]() { return sim().BoosterAttached() ? 0 : 1; },
			[sim](int) { sim().SeparateCSMFromBooster(); });
		push("LM\nEXTRACT", false, []() { return 0; },
			[sim](int) { sim().ExtractLM(); });
		push("LM\nACTIVATE", false,
			[sim]() { return sim().LGCPowered() ? 1 : 0; },
			[sim](int) { sim().TransferCrew(2); sim().PowerUpLGC(); });
		push("CREW TO\nCM", false, []() { return 0; },
			[sim](int) { sim().TransferCrew(0); });
		push("UNDOCK", true,
			[sim]() { return sim().LMDocked() ? 1 : 0; },
			[sim](int) { sim().UndockLM(); });
		push("LM\nJETT", true, []() { return 0; },
			[sim](int) { sim().JettisonLM(); });
		push("CM/SM\nSEP", true,
			[sim]() { return sim().SMAttached() ? 0 : 1; },
			[sim](int) { sim().SeparateCMFromSM(); });
		push("DROGUE\nDEPLOY", true,
			[sim]() { return sim().DroguesOut() ? 1 : 0; },
			[sim](int) { sim().DeployDrogues(); });
		push("MAIN\nDEPLOY", true,
			[sim]() { return sim().MainsOut() ? 1 : 0; },
			[sim](int) { sim().DeployMains(); });
	} else {
		auto pan = [s]() -> gnc::LMPanel& { return s->Sim().LMPGNCS()->panel; };
		auto pg = [s]() { return s->Sim().LMPGNCS(); };
		toggle("GUID CONT", {"AGS", "PGNS"},
			[pan]() { return pan().pgnsControl ? 1 : 0; },
			[pan](int p) { pan().pgnsControl = p == 1; });
		toggle("MODE CONTROL", {"OFF", "ATT HOLD", "AUTO"},
			[pan]() { return int(pan().mode); },
			[pan](int p) { pan().mode = gnc::LMPanel::Mode(p); });
		toggle("ENGINE ARM", {"OFF", "DES", "ASC"},
			[pan]() {
				return pan().engineArm == gnc::LMPanel::ARM_DESCENT ? 1
					: pan().engineArm == gnc::LMPanel::ARM_ASCENT ? 2 : 0;
			},
			[pan](int p) {
				pan().engineArm = p == 1 ? gnc::LMPanel::ARM_DESCENT
					: p == 2 ? gnc::LMPanel::ARM_ASCENT : gnc::LMPanel::ARM_OFF;
			});
		toggle("THROTTLE", {"MAN", "AUTO"},
			[pan]() { return pan().autoThrottle ? 1 : 0; },
			[pan](int p) { pan().autoThrottle = p == 1; });
		toggle("ACA", {"RATE", "PULSE"},
			[pan]() { return pan().acaPulse ? 1 : 0; },
			[pan](int p) { pan().acaPulse = p == 1; });
		toggle("ASC FEED", {"CLOSE", "OPEN"},
			[pan]() { return pan().ascentFeed ? 1 : 0; },
			[pan](int p) { pan().ascentFeed = p == 1; });
		push("ABORT", true, [pan]() { return pan().abort ? 1 : 0; },
			[pan](int) { pan().abort = !pan().abort; });
		push("ABORT\nSTAGE", true, [pan]() { return pan().abortStage ? 1 : 0; },
			[pan, sim](int) {
				pan().abortStage = true;
				sim().StageLM();
			});
		push("ENGINE\nSTOP", false, [pan]() { return pan().engineStop ? 1 : 0; },
			[pan](int) { pan().engineStop = !pan().engineStop; });
		push("ENGINE\nSTART", false, [pan]() { return pan().engineStart ? 1 : 0; },
			[pan](int) { pan().engineStart = !pan().engineStart; });
		push("ROD\nFASTER", false, []() { return 0; },
			[pg](int) { pg()->RateOfDescentClick(-1); });
		push("ROD\nSLOWER", false, []() { return 0; },
			[pg](int) { pg()->RateOfDescentClick(1); });
		push("ENG ARM\nBREAKER", false,
			[pg]() { return pg()->ascentArmBreakerOpen ? 1 : 0; },
			[sim](int) { sim().PenInBreaker(); });
		push("LDG RADAR\nBREAKER", false, [pg]() { return pg()->lrFailed ? 1 : 0; },
			[pg](int) { pg()->lrFailed = false; });
		toggle("RNDZ RADAR", {"SLEW", "LGC"},
			[pg]() { return pg()->rrCDUHunting ? 0 : 1; },
			[sim](int p) { if (p == 1) sim().RRModeToLGC(); });
		push("UNDOCK", true, [sim]() { return sim().LMDocked() ? 1 : 0; },
			[sim](int) { sim().UndockLM(); });
		push("CREW TO\nCM", false, []() { return 0; },
			[sim](int) { sim().TransferCrew(0); });
		toggle("RCS QUADS", {"ISOL", "OPEN"},
			[pan]() {
				for (bool b : pan().thrusterPairOff)
					if (b)
						return 0;
				return 1;
			},
			[pan](int p) {
				for (bool& b : pan().thrusterPairOff)
					b = p == 0;
			});
	}
	fPositions.assign(fControls.size(), 0);
}


// --- State ----------------------------------------------------------------------------

void
PanelView::Refresh()
{
	if (!fSession || !fSession->Running())
		return;
	_Snapshot();
	Invalidate();
}


void
PanelView::_Snapshot()
{
	std::lock_guard<std::recursive_mutex> g(fSession->Lock());
	FlightSim& f = fSession->Sim();
	bool lm = fCraft == 2 || (fCraft == 0 && f.LMPGNCS() && f.LGCPowered()
		&& f.CrewInLM() > f.CrewInCM());
	if (lm && !f.LMPGNCS())
		lm = false;
	fLM = lm;
	fPainter.SetLM(lm);
	if (fControlsLM != fLM)
		_BuildControls();
	fCraftName = lm ? f.Mission().lmCallsign : f.Mission().csmCallsign;
	fDisplay = lm ? f.LGCDSKY().State() : f.CMCDSKY().State();
	fGET = f.GET();
	fPhase = PhaseName(f.CurrentPhase());
	fNextName = fSession->Director().NextEvent();
	fNextGET = fSession->Director().NextEventGET();
	const sim::Vehicle* craft = lm ? f.LMVehicle() : &f.CSMVehicle();
	if (lm && !craft)
		craft = &f.CSMVehicle();
	if (lm) {
		gnc::LMPGNCS* pg = f.LMPGNCS();
		fGimbals = pg->imu.Gimbals(craft->attitude);
		fIMUFailed = pg->imu.failed;
		fThrottle = pg->panel.manualThrottle;
		fThrottleAuto = pg->panel.autoThrottle;
	} else {
		fGimbals = f.CMPGNCS().imu.Gimbals(craft->attitude);
		fIMUFailed = f.CMPGNCS().imu.failed;
	}
	fRates = craft->rates;
	for (size_t i = 0; i < fControls.size(); i++)
		fPositions[i] = fControls[i].get ? fControls[i].get() : 0;

	// Gauges and lamps.
	fGauges.clear();
	fLamps.clear();
	auto gauge = [&](const char* label, double frac, const std::string& text,
		bool warn) {
		Gauge gg;
		gg.label = label;
		gg.value = std::clamp(frac, 0.0, 1.0);
		gg.text = text;
		gg.warn = warn;
		fGauges.push_back(gg);
	};
	char b[32];
	if (!lm) {
		const sim::Vehicle* csm = &f.CSMVehicle();
		double sps = TankFraction(csm, "Service Module", 0);
		std::snprintf(b, sizeof(b), "%.0f%%", sps * 100);
		gauge("SPS", sps, b, sps < 0.05);
		double rcs = TankRange(csm, "Service Module", 1, 4);
		std::snprintf(b, sizeof(b), "%.0f%%", rcs * 100);
		gauge("SM RCS", rcs, b, rcs < 0.15);
		const CSMSystems& cs = f.csmSystems;
		for (int i = 0; i < 2; i++) {
			double q = cs.o2[i].capacity > 0 ? cs.o2[i].quantity / cs.o2[i].capacity : 0;
			std::snprintf(b, sizeof(b), "%.0f psi", cs.o2[i].pressure);
			gauge(i == 0 ? "O2 TK 1" : "O2 TK 2", q, b, cs.o2[i].pressure < 800);
		}
		double h2 = (cs.h2[0].capacity + cs.h2[1].capacity) > 0
			? cs.TotalH2() / (cs.h2[0].capacity + cs.h2[1].capacity) : 0;
		std::snprintf(b, sizeof(b), "%.0f%%", h2 * 100);
		gauge("H2", h2, b, h2 < 0.1);
		std::snprintf(b, sizeof(b), "%.1f V", std::min(cs.mainBusA, cs.mainBusB));
		gauge("MAIN BUS", (std::min(cs.mainBusA, cs.mainBusB) - 20) / 12, b,
			cs.MainBusUndervolt());
		std::snprintf(b, sizeof(b), "%.1f mm", f.cmCabin.co2);
		gauge("CO2 PP", f.cmCabin.co2 / 15, b, f.cmCabin.co2 > 7.6);
		std::snprintf(b, sizeof(b), "%.1f psi", f.cmCabin.pressure);
		gauge("CABIN", f.cmCabin.pressure / 6, b, f.cmCabin.pressure < 4.2);

		fLamps = {
			{"FC 1", !cs.fuelCell[0].online, false},
			{"FC 2", !cs.fuelCell[1].online, false},
			{"FC 3", !cs.fuelCell[2].online, false},
			{"CRYO PRESS", cs.o2[0].pressure < 800 || cs.o2[1].pressure < 800, false},
			{"MAIN BUS A", cs.mainBusA < 26.25, true},
			{"MAIN BUS B", cs.mainBusB < 26.25, true},
			{"CO2 PP HI", f.cmCabin.co2 > 7.6, true},
			{"CABIN PRESS", f.cmCabin.pressure < 4.2, true},
			{"SPS PRESS", f.CMPGNCS().spsFailed, false},
			{"ISS", fIMUFailed, false},
			{"CMC", fDisplay.progAlarm || fDisplay.restart, false},
			{"SPS THRUST", f.CMPGNCS().spsThrustShortA, false},
		};
	} else {
		const sim::Vehicle* lmv = f.LMVehicle();
		double dps = TankFraction(lmv, "LM descent stage", 0);
		std::snprintf(b, sizeof(b), "%.1f%%", dps * 100);
		gauge("DPS", dps, b, lmv && lmv->FindModule("LM descent stage") && dps < 0.056);
		double aps = TankFraction(lmv, "LM ascent stage", 0);
		std::snprintf(b, sizeof(b), "%.0f%%", aps * 100);
		gauge("APS", aps, b, aps < 0.1);
		double ra = TankFraction(lmv, "LM ascent stage", 1);
		std::snprintf(b, sizeof(b), "%.0f%%", ra * 100);
		gauge("RCS A", ra, b, ra < 0.15);
		double rb = TankFraction(lmv, "LM ascent stage", 2);
		std::snprintf(b, sizeof(b), "%.0f%%", rb * 100);
		gauge("RCS B", rb, b, rb < 0.15);
		const LMSystems& ls = f.lmSystems;
		std::snprintf(b, sizeof(b), "%.1f kg", ls.OxygenKg());
		gauge("O2", ls.OxygenKg() / 24.2, b, ls.OxygenKg() < 2);
		std::snprintf(b, sizeof(b), "%.0f kg", ls.WaterKg());
		gauge("WATER", ls.WaterKg() / 190, b, ls.WaterKg() < 10);
		std::snprintf(b, sizeof(b), "%.0f A h", ls.AmpHours());
		gauge("BATTERY", ls.AmpHours() / 2252, b, ls.AmpHours() < 100);
		std::snprintf(b, sizeof(b), "%.1f mm", f.lmCabin.co2);
		gauge("CO2 PP", f.lmCabin.co2 / 15, b, f.lmCabin.co2 > 7.6);
		gnc::LMPGNCS* pg = f.LMPGNCS();
		bool descent = lmv && lmv->FindModule("LM descent stage");
		fLamps = {
			{"DES QTY", descent && dps < 0.056 && dps > 0, false},
			{"ASC PRESS", aps < 0.05 && !descent, true},
			{"RCS A REG", ra < 0.05, false},
			{"RCS B REG", rb < 0.05, false},
			{"ISS", fIMUFailed, true},
			{"LGC", fDisplay.progAlarm || fDisplay.restart, false},
			{"CO2 QTY", f.lmCabin.co2 > 7.6, true},
			{"CABIN", f.lmCabin.pressure < 4.2, true},
			{"BATTERY", ls.AmpHours() < 100, false},
			{"ENG ARM", pg && pg->ascentArmBreakerOpen, false},
			{"LDG RADAR", pg && (pg->lrFailed || (!pg->LRRangeGood()
				&& f.CurrentPhase() == Phase::PoweredDescent)), false},
			{"ABORT", pg && pg->panel.abort, true},
		};
	}
	// The crew.
	double health = f.crew.AverageHealth();
	std::snprintf(b, sizeof(b), "%.0f", health);
	gauge("CREW", health / 100, b, health < 40);
	fMasterAlarm = false;
	for (const Lamp& l : fLamps)
		if (l.on && l.warning)
			fMasterAlarm = true;
}


// --- Drawing -----------------------------------------------------------------------

void
PanelView::Draw(BRect)
{
	BRect b = Bounds();
	SetHighColor(30, 32, 34);
	FillRect(b);
	float s = _Scale();
	float ox = (b.Width() + 1 - kWidth * s) / 2;
	float oy = (b.Height() + 1 - kHeight * s) / 2;
	PushState();
	SetOrigin(BPoint(ox, oy));
	SetScale(s);
	SetHighColor(kPanelGray);
	FillRect(BRect(0, 0, kWidth - 1, kHeight - 1));
	SetHighColor(kPanelLine);
	StrokeRect(BRect(0, 0, kWidth - 1, kHeight - 1));
	Label(this, (fCraftName + (fLM ? "  —  LUNAR MODULE" : "  —  COMMAND MODULE")).c_str(),
		14, 22, 13, kLabelDim, 0);

	_DrawFDAI(BRect(14, 34, 330, 350));
	_DrawTimers(BRect(14, 360, 330, 450));
	_DrawLamps(BRect(14, 460, 330, 706));
	_DrawDSKY(BRect(346, 34, 726, 478));
	_DrawControllers(BRect(346, 490, 726, 706));
	for (size_t i = 0; i < fControls.size(); i++)
		_DrawControl(fControls[i]);
	_DrawGauges(BRect(748, 472, 1266, 706));
	PopState();
}


void
PanelView::_DrawFDAI(BRect r)
{
	SetHighColor(kPanelDark);
	FillRoundRect(r, 8, 8);
	BPoint c(r.left + r.Width() / 2, r.top + r.Height() / 2 + 6);
	float R = r.Width() * 0.38f;
	// The ball: pitch moves the horizon, roll turns it (gimbal angles:
	// x outer = roll, y inner = pitch, z middle = yaw).
	double roll = fGimbals.x, pitch = fGimbals.y, yaw = fGimbals.z;
	double ca = std::cos(-roll), sa = std::sin(-roll);
	auto rot = [&](float x, float y) {
		return BPoint(c.x + float(x * ca - y * sa), c.y + float(x * sa + y * ca));
	};
	// Sky (white) and ground (black) halves as polygons, the horizon offset
	// by pitch (in ball radii).
	double p = std::remainder(pitch, 2 * 3.14159265358979);
	float off = float(std::sin(p)) * R;
	const int N = 64;
	SetHighColor(kBlack);
	FillEllipse(c, R, R);
	// The sky side of the horizon: a circular segment (convex), swept from
	// the point opposite it.
	{
		bool upright = std::cos(p) >= 0;
		std::vector<BPoint> poly;
		double start = upright ? 3.14159265358979 / 2 : 3 * 3.14159265358979 / 2;
		for (int i = 0; i <= N; i++) {
			double t = start + 2 * 3.14159265358979 * i / N;
			float x = float(std::cos(t)) * R, y = float(std::sin(t)) * R;
			bool sky = upright ? y < off : y > off;
			if (sky)
				poly.push_back(rot(x, y));
		}
		// Close along the horizon chord.
		float hx = std::sqrt(std::max(0.0f, R * R - off * off));
		if (!poly.empty() && std::fabs(off) < R) {
			// From the bottom (upright) the sweep reaches the sky on the
			// left first; from the top (inverted), on the right.
			std::vector<BPoint> closed;
			closed.push_back(rot(upright ? -hx : hx, off));
			closed.insert(closed.end(), poly.begin(), poly.end());
			closed.push_back(rot(upright ? hx : -hx, off));
			poly.swap(closed);
		}
		SetHighColor(225, 222, 210);
		if (poly.size() >= 3)
			FillPolygon(poly.data(), int32(poly.size()));
		else if (upright ? off > R : off < -R)
			FillEllipse(c, R, R);
	}
	// Pitch ladder every 30 degrees.
	for (int k = -150; k <= 180; k += 30) {
		double d = std::remainder(pitch - k * kDeg, 2 * 3.14159265358979);
		if (std::fabs(d) > 1.2)
			continue;
		float y = float(std::sin(d)) * R;
		float half = std::sqrt(std::max(0.0f, R * R - y * y)) * 0.6f;
		bool sky = std::fabs(std::remainder(k * kDeg, 2 * 3.14159265358979)) < 1.6;
		SetHighColor(sky ? kBlack : kLabel);
		StrokeLine(rot(-half, y), rot(half, y));
		char lbl[8];
		std::snprintf(lbl, sizeof(lbl), "%d", (k + 360) % 360);
		BPoint lp = rot(half + 4, y + 4);
		Label(this, lbl, lp.x, lp.y, 9, sky ? kBlack : kLabel, 0, false);
	}
	SetHighColor(kPanelLine);
	SetPenSize(3);
	StrokeEllipse(c, R + 2, R + 2);
	SetPenSize(1);
	// The fixed aircraft symbol.
	SetHighColor(kAmberLamp);
	SetPenSize(3);
	StrokeLine(BPoint(c.x - R * 0.45f, c.y), BPoint(c.x - R * 0.12f, c.y));
	StrokeLine(BPoint(c.x + R * 0.12f, c.y), BPoint(c.x + R * 0.45f, c.y));
	SetPenSize(1);
	FillEllipse(c, 3, 3);
	// Rate needles: roll (top), pitch (right), yaw (bottom), +/-5 deg/s.
	auto needle = [&](double rate, bool horizontal, float pos) {
		float d = float(std::clamp(rate / kDeg / 5, -1.0, 1.0)) * R * 0.8f;
		SetHighColor(kAmberLamp);
		if (horizontal)
			FillRect(BRect(c.x + d - 2, pos - 8, c.x + d + 2, pos + 8));
		else
			FillRect(BRect(pos - 8, c.y + d - 2, pos + 8, c.y + d + 2));
	};
	needle(fRates.x, true, r.top + 18);
	needle(fRates.y, false, r.right - 14);
	needle(fRates.z, true, r.bottom - 10);
	char text[64];
	std::snprintf(text, sizeof(text), "R %05.1f  P %05.1f  Y %05.1f",
		std::fmod(roll / kDeg + 360, 360), std::fmod(pitch / kDeg + 360, 360),
		std::fmod(yaw / kDeg + 360, 360));
	Label(this, text, c.x, r.bottom - 22, 11, kLabel);
	if (fIMUFailed)
		Label(this, "NO ATT", c.x, c.y - R * 0.5f, 18, kRedLamp);
	Label(this, "FDAI", r.left + 10, r.top + 16, 10, kLabelDim, 0);
}


void
PanelView::_DrawTimers(BRect r)
{
	SetHighColor(kPanelDark);
	FillRoundRect(r, 6, 6);
	Label(this, "GET", r.left + 12, r.top + 22, 10, kLabelDim, 0);
	Label(this, Clock(fGET).c_str(), r.right - 12, r.top + 26, 22, kGreenDigits, 2);
	if (!fNextName.empty() && fNextGET > 0) {
		std::string lbl = "NEXT  " + fNextName;
		Label(this, lbl.c_str(), r.left + 12, r.top + 56, 10, kLabelDim, 0);
		Label(this, ("T" + Clock(fGET - fNextGET)).c_str(), r.right - 12,
			r.top + 60, 16, kAmberLamp, 2);
	}
	Label(this, fPhase.c_str(), r.left + 12, r.bottom - 8, 11, kLabel, 0);
}


void
PanelView::_DrawLamps(BRect r)
{
	SetHighColor(kPanelDark);
	FillRoundRect(r, 6, 6);
	Label(this, "CAUTION & WARNING", r.left + 10, r.top + 16, 10, kLabelDim, 0);
	// Master alarm.
	BRect ma(r.right - 108, r.top + 4, r.right - 8, r.top + 26);
	bool lit = fMasterAlarm && (system_time() / 400000) % 2 == 0;
	SetHighColor(lit ? kRedLamp : kLampOff);
	FillRoundRect(ma, 3, 3);
	Label(this, "MASTER ALARM", ma.left + ma.Width() / 2, ma.top + 15, 9,
		lit ? kBlack : kLabelDim);
	const int cols = 3;
	float w = (r.Width() - 16 - (cols - 1) * 6) / cols, h = 34;
	for (size_t i = 0; i < fLamps.size(); i++) {
		int col = int(i) % cols, row = int(i) / cols;
		BRect f(r.left + 8 + col * (w + 6), r.top + 34 + row * (h + 6),
			r.left + 8 + col * (w + 6) + w, r.top + 34 + row * (h + 6) + h);
		const Lamp& l = fLamps[i];
		SetHighColor(l.on ? (l.warning ? kRedLamp : kAmberLamp) : kLampOff);
		FillRoundRect(f, 3, 3);
		Label(this, l.label, f.left + f.Width() / 2, f.top + 21, 10,
			l.on ? kBlack : kLabelDim);
	}
}


void
PanelView::_DrawDSKY(BRect r)
{
	PushState();
	SetOrigin(r.LeftTop());
	bool flash = (system_time() / 320000) % 2 == 0;
	fPainter.Draw(this, fDisplay, flash, true, fPressedKey, fKeyboardKey);
	PopState();
}


void
PanelView::_DrawControllers(BRect r)
{
	SetHighColor(kPanelDark);
	FillRoundRect(r, 6, 6);
	Label(this, fLM ? "ACA" : "RHC", r.left + 70, r.top + 18, 10, kLabelDim);
	Label(this, fLM ? "TTCA" : "THC", r.left + 200, r.top + 18, 10, kLabelDim);
	// Rotation: a stick top view (pitch/yaw) and roll arc.
	BPoint c1(r.left + 70, r.top + 100);
	SetHighColor(kBlack);
	FillEllipse(c1, 50, 50);
	SetHighColor(kPanelLine);
	StrokeEllipse(c1, 50, 50);
	StrokeLine(BPoint(c1.x - 50, c1.y), BPoint(c1.x + 50, c1.y));
	StrokeLine(BPoint(c1.x, c1.y - 50), BPoint(c1.x, c1.y + 50));
	// CM: x roll, y pitch, z yaw. LM: x yaw, y pitch, z roll.
	double pitchIn = rotation.y, yawIn = fLM ? rotation.x : rotation.z;
	double rollIn = fLM ? rotation.z : rotation.x;
	BPoint knob(c1.x + float(yawIn) * 40, c1.y - float(pitchIn) * 40);
	SetHighColor(kAmberLamp);
	FillEllipse(knob, 8, 8);
	SetHighColor(kLabel);
	float ra = float(rollIn) * 1.2f;
	StrokeLine(BPoint(c1.x, c1.y), BPoint(c1.x + std::sin(ra) * 48,
		c1.y - std::cos(ra) * 48));
	Label(this, "\xe2\x86\x91\xe2\x86\x93 pitch  \xe2\x86\x90\xe2\x86\x92 roll  [ ] yaw", c1.x,
		r.bottom - 10, 9, kLabelDim);
	// Translation.
	BPoint c2(r.left + 200, r.top + 100);
	SetHighColor(kBlack);
	FillRect(BRect(c2.x - 50, c2.y - 50, c2.x + 50, c2.y + 50));
	SetHighColor(kPanelLine);
	StrokeRect(BRect(c2.x - 50, c2.y - 50, c2.x + 50, c2.y + 50));
	const char* axes[3] = {"+X", "+Y", "+Z"};
	for (int i = 0; i < 3; i++) {
		BRect cell(c2.x - 46, c2.y - 44 + i * 30, c2.x + 46, c2.y - 20 + i * 30);
		SetHighColor(translation[i] ? kAmberLamp : kPanelDark);
		FillRect(cell);
		char t[16];
		std::snprintf(t, sizeof(t), "%s %s", axes[i],
			translation[i] > 0 ? "FWD" : translation[i] < 0 ? "AFT" : "--");
		Label(this, t, cell.left + cell.Width() / 2, cell.top + 17, 11,
			translation[i] ? kBlack : kLabelDim);
	}
	Label(this, "Shift + arrows, PgUp/PgDn", c2.x, r.bottom - 10, 9, kLabelDim);
	// The LM throttle.
	if (fLM) {
		BRect t(r.right - 60, r.top + 30, r.right - 30, r.bottom - 30);
		SetHighColor(kBlack);
		FillRect(t);
		float y = t.bottom - float(fThrottle) * t.Height();
		SetHighColor(fThrottleAuto ? kLabelDim : kAmberLamp);
		FillRect(BRect(t.left - 6, y - 4, t.right + 6, y + 4));
		Label(this, "THR", t.left + t.Width() / 2, r.top + 18, 10, kLabelDim);
		Label(this, fThrottleAuto ? "AUTO" : "MAN", t.left + t.Width() / 2,
			r.bottom - 10, 9, kLabelDim);
	}
}


void
PanelView::_DrawControl(const Control& c)
{
	size_t index = size_t(&c - &fControls[0]);
	int pos = index < fPositions.size() ? fPositions[index] : 0;
	BRect f = c.frame;
	SetHighColor(kPanelDark);
	FillRoundRect(f, 5, 5);
	// The label (one or two lines).
	std::string label = c.label;
	size_t nl = label.find('\n');
	if (c.positions.empty()) {
		// A pushbutton (with a striped guard when it's a pyro).
		BRect btn(f.left + 10, f.top + 8, f.right - 10, f.bottom - 8);
		bool lit = pos != 0;
		bool down = int(index) == fPressedControl;
		SetHighColor(lit ? kAmberLamp : rgb_color{96, 98, 100, 255});
		FillRoundRect(down ? btn.OffsetByCopy(0, 1) : btn, 4, 4);
		if (c.guarded) {
			SetHighColor(200, 40, 30);
			SetPenSize(2);
			StrokeRoundRect(btn.InsetByCopy(-3, -3), 5, 5);
			SetPenSize(1);
		}
		rgb_color tc = lit ? kBlack : kLabel;
		if (nl == std::string::npos)
			Label(this, label.c_str(), btn.left + btn.Width() / 2,
				btn.top + btn.Height() / 2 + 5, 12, tc);
		else {
			Label(this, label.substr(0, nl).c_str(), btn.left + btn.Width() / 2,
				btn.top + btn.Height() / 2 - 2, 11, tc);
			Label(this, label.substr(nl + 1).c_str(), btn.left + btn.Width() / 2,
				btn.top + btn.Height() / 2 + 12, 11, tc);
		}
		return;
	}
	// A toggle: the lever and its positions.
	Label(this, label.c_str(), f.left + f.Width() / 2, f.top + 15, 11, kLabel);
	int n = int(c.positions.size());
	float spanL = f.left + 18, spanR = f.right - 18;
	float y = f.top + 40;
	for (int i = 0; i < n; i++) {
		float x = n == 1 ? (spanL + spanR) / 2 : spanL + (spanR - spanL) * i / (n - 1);
		bool on = i == pos;
		Label(this, c.positions[i], x, f.bottom - 6, 9,
			on ? kAmberLamp : kLabelDim);
	}
	float lx = n == 1 ? (spanL + spanR) / 2
		: spanL + (spanR - spanL) * std::clamp(pos, 0, n - 1) / (n - 1);
	SetHighColor(30, 30, 32);
	FillEllipse(BPoint((spanL + spanR) / 2, y), 9, 9);
	SetHighColor(210, 210, 214);
	SetPenSize(5);
	StrokeLine(BPoint((spanL + spanR) / 2, y), BPoint(lx, y - 10));
	SetPenSize(1);
	FillEllipse(BPoint(lx, y - 10), 5, 5);
}


void
PanelView::_DrawGauges(BRect r)
{
	SetHighColor(kPanelDark);
	FillRoundRect(r, 6, 6);
	int n = int(fGauges.size());
	if (n == 0)
		return;
	float w = (r.Width() - 16) / n;
	for (int i = 0; i < n; i++) {
		const Gauge& g = fGauges[i];
		float cx = r.left + 8 + w * i + w / 2;
		BRect bar(cx - 10, r.top + 30, cx + 10, r.bottom - 44);
		SetHighColor(kBlack);
		FillRect(bar);
		float top = bar.bottom - float(g.value) * bar.Height();
		SetHighColor(g.warn ? kRedLamp : kGreenDigits);
		FillRect(BRect(bar.left + 2, top, bar.right - 2, bar.bottom - 1));
		SetHighColor(kPanelLine);
		StrokeRect(bar);
		Label(this, g.label, cx, r.top + 20, 10, kLabel);
		Label(this, g.text.c_str(), cx, r.bottom - 26, 10,
			g.warn ? kRedLamp : kLabelDim, 1, false);
	}
}


// --- Input ----------------------------------------------------------------------

void
PanelView::MouseDown(BPoint where)
{
	MakeFocus(true);
	BPoint p = _ToLogical(where);
	// The DSKY.
	BPoint d(p.x - 346, p.y - 34);
	int key = fPainter.KeyAt(d);
	if (key >= 0 && fSession) {
		fPressedKey = key;
		std::lock_guard<std::recursive_mutex> g(fSession->Lock());
		int code = fPainter.Keys()[key].code;
		if (code < 0) {
			if (fLM)
				fSession->Sim().ProceedLGC(true);
			else
				fSession->Sim().ProceedCMC(true);
		} else if (fLM)
			fSession->Sim().PressLGC(code);
		else
			fSession->Sim().PressCMC(code);
		SetMouseEventMask(B_POINTER_EVENTS, B_LOCK_WINDOW_FOCUS);
		Invalidate();
		return;
	}
	for (size_t i = 0; i < fControls.size(); i++) {
		const Control& c = fControls[i];
		if (!c.frame.Contains(p) || !fSession)
			continue;
		std::lock_guard<std::recursive_mutex> g(fSession->Lock());
		if (c.positions.empty()) {
			fPressedControl = int(i);
			c.set(1);
		} else {
			// Click left or right of the lever's pivot to move it a notch.
			int n = int(c.positions.size());
			int pos = fPositions[i];
			float mid = c.frame.left + c.frame.Width() / 2;
			int next = p.x < mid ? std::max(0, pos - 1) : std::min(n - 1, pos + 1);
			if (next == pos)
				next = p.x < mid ? n - 1 : 0;
			c.set(next);
		}
		fPositions[i] = c.get ? c.get() : 0;
		SetMouseEventMask(B_POINTER_EVENTS, B_LOCK_WINDOW_FOCUS);
		Invalidate();
		return;
	}
}


void
PanelView::MouseUp(BPoint)
{
	if (fPressedKey >= 0 && fSession
			&& fPainter.Keys()[fPressedKey].code < 0) {
		std::lock_guard<std::recursive_mutex> g(fSession->Lock());
		if (fLM)
			fSession->Sim().ProceedLGC(false);
		else
			fSession->Sim().ProceedCMC(false);
	}
	fPressedKey = -1;
	fPressedControl = -1;
	Invalidate();
}


void
PanelView::KeyDown(const char* bytes, int32 numBytes)
{
	int index = numBytes == 1 ? KeyIndex(bytes[0]) : -1;
	if (index < 0 || !fSession) {
		BView::KeyDown(bytes, numBytes);
		return;
	}
	if (fKeyboardKey == index)
		return;
	fKeyboardKey = index;
	std::lock_guard<std::recursive_mutex> g(fSession->Lock());
	int code = fPainter.Keys()[index].code;
	if (code < 0) {
		if (fLM)
			fSession->Sim().ProceedLGC(true);
		else
			fSession->Sim().ProceedCMC(true);
	} else if (fLM)
		fSession->Sim().PressLGC(code);
	else
		fSession->Sim().PressCMC(code);
	Invalidate();
}


void
PanelView::KeyUp(const char*, int32)
{
	if (fKeyboardKey >= 0 && fSession
			&& fPainter.Keys()[fKeyboardKey].code < 0) {
		std::lock_guard<std::recursive_mutex> g(fSession->Lock());
		if (fLM)
			fSession->Sim().ProceedLGC(false);
		else
			fSession->Sim().ProceedCMC(false);
	}
	fKeyboardKey = -1;
	Invalidate();
}

}  // namespace columbia
