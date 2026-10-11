// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "ReplayRenderer.h"

#include <InterfaceDefs.h>
#include <Region.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "DSKYPainter.h"
#include "Drawing.h"
#include "sim/World.h"

namespace ui {

using namespace flight;
using sim::Mat3;
using sim::Quat;
using sim::Vec3;
namespace world = sim::world;

namespace {

enum {
	PH_PRELAUNCH, PH_BOOST, PH_PARKING, PH_TLI, PH_TLC, PH_LUNAR_ORBIT,
	PH_DESCENT, PH_SURFACE, PH_ASCENT, PH_RENDEZVOUS, PH_TEC, PH_ENTRY,
	PH_RECOVERY, PH_LOST
};
enum { LOG_INFO, LOG_CAPCOM, LOG_CREW, LOG_CAUTION, LOG_WARNING, LOG_EVENT };

}  // namespace


// --- Per-frame context ---------------------------------------------------------

struct ReplayRenderer::Frame {
	RecSample s;
	double get = 0;
	double speed = 1;
	double clock = 0;
	double jd = 0;
	int crewed = REC_CSM;
	Scene scene = Scene::Title;
	bool flash = true;
};


ReplayRenderer::ReplayRenderer(const Recording& recording)
	:
	fRec(recording),
	fCM(new DSKYPainter(false)),
	fLM(new DSKYPainter(true))
{
	_Precompute();
}


ReplayRenderer::~ReplayRenderer()
{
	delete fCM;
	delete fLM;
}


int
ReplayRenderer::_Crewed(const RecSample& s) const
{
	switch (s.phase) {
		case PH_DESCENT:
		case PH_SURFACE:
		case PH_ASCENT:
			if (s.veh[REC_LM].Present())
				return REC_LM;
			break;
		case PH_RENDEZVOUS:
			// Two crew in the LM until it docks.
			if (s.veh[REC_LM].Present())
				return REC_LM;
			break;
		default:
			break;
	}
	return REC_CSM;
}


void
ReplayRenderer::Refresh()
{
	// The path is cheap to redo, but not every frame.
	if (fRec.samples.size() >= fPrecomputed + 30
			|| (fRec.samples.size() != fPrecomputed && fPrecomputed < 30))
		_Precompute();
}


void
ReplayRenderer::_Precompute()
{
	fPrecomputed = fRec.samples.size();
	fMapPath.clear();
	fMarkers.clear();
	for (const RecSample& s : fRec.samples) {
		const RecVehicle& v = s.veh[REC_CSM];
		if (!v.Present())
			continue;
		Vec3 ux = s.moon.Unit();
		Vec3 uz = s.moon.Cross(s.moonV).Unit();
		Vec3 uy = uz.Cross(ux);
		MapPoint p;
		p.get = s.get;
		p.x = float(v.r.Dot(ux) / 1000);
		p.y = float(v.r.Dot(uy) / 1000);
		if (!fMapPath.empty() && s.get - fMapPath.back().get < 120)
			continue;
		fMapPath.push_back(p);
	}
	// Markers for the big burns, from the PADs and events.
	const char* names[] = {"TLI", "MCC-1", "MCC-2", "MCC-3", "MCC-4", "LOI-1",
		"LOI-2", "TEI", "MCC-5", "MCC-6", "MCC-7"};
	for (const RecEvent& e : fRec.events) {
		for (const char* n : names) {
			std::string key = std::string(n) + ": ignition";
			if (e.text.find(key) != std::string::npos
					|| (std::strcmp(n, "TLI") == 0
						&& e.text.find("TLI complete") != std::string::npos))
				fMarkers.push_back({e.get, n});
		}
		if (e.text == "Liftoff")
			fLiftoffGET = e.get;
	}
	// The pad, Earth-fixed.
	for (const RecSample& s : fRec.samples) {
		if (!s.veh[REC_CSM].Present())
			continue;
		double jd = fRec.header.getZeroJD + s.get / 86400;
		fPadFixed = (world::Ephemeris::EarthFixedToInertial(jd).Transposed()
			* s.veh[REC_CSM].r).Unit();
		break;
	}
}


Scene
ReplayRenderer::SceneAt(double get) const
{
	RecSample s = fRec.At(get);
	const RecVehicle& csm = s.veh[REC_CSM];
	auto near = [&](int k, double range) {
		const RecVehicle& o = s.veh[k];
		return csm.Present() && o.Present() && (o.r - csm.r).Length() < range;
	};
	switch (s.phase) {
		case PH_PRELAUNCH:
		case PH_BOOST:
			return Scene::Launch;
		case PH_PARKING:
		case PH_TLI:
			if (near(REC_BOOSTER, 300) || near(REC_LM, 300))
				return Scene::Docking;
			return get < 1200 ? Scene::Launch : Scene::EarthOrbit;
		case PH_TLC:
		case PH_TEC: {
			if (near(REC_BOOSTER, 300) || near(REC_LM, 300))
				return Scene::Docking;
			double moonAlt = (csm.r - s.moon).Length() - world::kRMoon;
			if (moonAlt < 12000e3)
				return Scene::MoonOrbit;
			double earthAlt = csm.r.Length() - world::kREarth;
			if (earthAlt < 9000e3)
				return Scene::EarthOrbit;
			return Scene::Cislunar;
		}
		case PH_LUNAR_ORBIT:
			if (near(REC_LM, 300))
				return Scene::Docking;
			return Scene::MoonOrbit;
		case PH_DESCENT:
			return Scene::Landing;
		case PH_SURFACE:
			return Scene::Surface;
		case PH_ASCENT:
			return Scene::Ascent;
		case PH_RENDEZVOUS:
			return near(REC_LM, 400) ? Scene::Docking : Scene::Rendezvous;
		case PH_ENTRY:
			return Scene::Entry;
		case PH_RECOVERY:
			return Scene::Splashdown;
		case PH_LOST:
			return Scene::Lost;
	}
	return Scene::Cislunar;
}


// --- The frame ---------------------------------------------------------------------

void
ReplayRenderer::Draw(BView* v, BRect bounds, double get, double speed,
	double clock)
{
	Frame f;
	f.get = get;
	f.speed = speed;
	f.clock = clock;
	f.s = fRec.At(get);
	f.jd = fRec.header.getZeroJD + get / 86400;
	f.crewed = _Crewed(f.s);
	f.scene = SceneAt(get);
	f.flash = std::fmod(clock, 0.64) < 0.32;
	if (get < fLastGET - 1)
		fCaptionShown.clear();      // started over
	fLastGET = get;

	v->SetDrawingMode(B_OP_COPY);
	v->SetHighColor(kBack);
	v->FillRect(bounds);

	float W = bounds.Width(), H = bounds.Height();
	float top = std::max(28.0f, H * 0.055f);
	float captionH = showCaptions ? std::max(64.0f, H * 0.14f) : 0;
	float sideW = showDSKY ? std::clamp(W * 0.27f, 200.0f, 420.0f) : 0;
	BRect topBar(bounds.left, bounds.top, bounds.right, bounds.top + top);
	BRect main(bounds.left + 8, topBar.bottom + 6,
		bounds.right - sideW - (showDSKY ? 14 : 8),
		bounds.bottom - captionH - 8);
	BRect side(main.right + 6, main.top, bounds.right - 8, main.bottom);
	BRect captions(bounds.left + 8, main.bottom + 6, bounds.right - 8,
		bounds.bottom - 6);

	_DrawTopBar(v, f, topBar);
	// Scenes draw freely; what spills over is cut at the panel's edge.
	BRegion clip(main);
	v->ConstrainClippingRegion(&clip);
	switch (f.scene) {
		case Scene::Title:
		case Scene::Launch: _DrawLaunch(v, f, main); break;
		case Scene::EarthOrbit: _DrawEarthOrbit(v, f, main); break;
		case Scene::Cislunar: _DrawCislunar(v, f, main); break;
		case Scene::MoonOrbit: _DrawMoonOrbit(v, f, main); break;
		case Scene::Landing: _DrawLanding(v, f, main, false); break;
		case Scene::Surface: _DrawSurface(v, f, main); break;
		case Scene::Ascent: _DrawLanding(v, f, main, true); break;
		case Scene::Rendezvous: _DrawRendezvous(v, f, main); break;
		case Scene::Docking: _DrawDocking(v, f, main); break;
		case Scene::Entry: _DrawEntry(v, f, main); break;
		case Scene::Splashdown: _DrawEnd(v, f, main, false); break;
		case Scene::Lost: _DrawEnd(v, f, main, true); break;
	}
	v->ConstrainClippingRegion(nullptr);
	if (showDSKY && side.Width() > 100)
		_DrawSidePanel(v, f, side);
	if (showCaptions)
		_DrawCaptions(v, f, captions);
	if (showTitle && clock < 9) {
		double fade = clock < 6 ? 1.0 : 1.0 - (clock - 6) / 3;
		_DrawTitle(v, f, bounds, fade);
	}
}


void
ReplayRenderer::_DrawTopBar(BView* v, const Frame& f, BRect r)
{
	v->SetHighColor(kPanelBack);
	v->FillRect(r);
	v->SetHighColor(kGridBright);
	v->StrokeLine(BPoint(r.left, r.bottom), BPoint(r.right, r.bottom));
	float size = std::clamp(r.Height() * 0.5f, 12.0f, 22.0f);
	float y = r.top + r.Height() / 2 + size * 0.38f;
	const RecHeader& h = fRec.header;
	std::string who = h.missionName;
	if (!h.csmCallsign.empty() && h.csmCallsign != h.missionName)
		who += "   " + h.csmCallsign;
	if (!h.lmCallsign.empty())
		who += " / " + h.lmCallsign;
	Text(v, who.c_str(), r.left + 12, y, size, kWhite, true);
	Text(v, RecPhaseName(f.s.phase), r.left + r.Width() / 2, y, size, kAmber,
		true, 1);
	std::string t = "GET " + FormatGET(f.get);
	Text(v, t.c_str(), r.right - 12, y, size, kGreen, true, 2);
	std::string sp = f.speed < 1.5 ? std::string("REAL TIME")
		: Fmt("x%.0f", f.speed);
	BFont font(be_bold_font);
	font.SetSize(size);
	v->SetFont(&font);
	float tw = v->StringWidth(t.c_str());
	Text(v, sp.c_str(), r.right - 24 - tw, y, size * 0.7f,
		f.speed < 1.5 ? kGreen : kDim, false, 2);
}


void
ReplayRenderer::_DrawCaptions(BView* v, const Frame& f, BRect r)
{
	v->SetHighColor(kPanelBack);
	v->FillRect(r);
	v->SetHighColor(kGridBright);
	v->StrokeRect(r);
	// The newest lines that have happened, faded by how long they've been up.
	std::vector<size_t> shown;
	for (size_t i = 0; i < fRec.events.size(); i++) {
		const RecEvent& e = fRec.events[i];
		if (e.get > f.get)
			break;
		if (e.kind == LOG_INFO)
			continue;
		if (e.text.rfind("Phase: ", 0) == 0)
			continue;
		shown.push_back(i);
	}
	float size = std::clamp(r.Height() / 5.2f, 11.0f, 18.0f);
	size_t lines = std::max<size_t>(1, size_t((r.Height() - 8) / (size * 1.35f)));
	if (shown.size() > lines)
		shown.erase(shown.begin(), shown.end() - long(lines));
	float y = r.top + size + 6;
	for (size_t idx : shown) {
		auto it = fCaptionShown.find(idx);
		if (it == fCaptionShown.end())
			it = fCaptionShown.emplace(idx, f.clock).first;
		double age = f.clock - it->second;
		const RecEvent& e = fRec.events[idx];
		double alpha = age < 0.4 ? age / 0.4 : age > 25 ? 0.35 : 1.0;
		rgb_color c = e.kind == LOG_CAPCOM ? kAmber
			: e.kind == LOG_CREW ? kGreen
			: e.kind == LOG_WARNING ? kRed
			: e.kind == LOG_CAUTION ? kAmber : kWhite;
		std::string who = e.kind == LOG_CAPCOM ? "CAPCOM"
			: e.kind == LOG_CREW ? "CREW" : e.kind == LOG_WARNING ? "MASTER ALARM"
			: e.kind == LOG_CAUTION ? "CAUTION" : "";
		std::string stamp = FormatGET(e.get);
		Text(v, stamp.c_str(), r.left + 10, y, size * 0.85f, Alpha(kDim, alpha));
		float x = r.left + 10 + size * 6.2f;
		if (!who.empty()) {
			Text(v, who.c_str(), x, y, size * 0.85f, Alpha(c, alpha), true);
			x += size * (who.size() * 0.62f + 1);
		}
		std::string text = e.text;
		// Fit the width.
		BFont font(be_fixed_font);
		font.SetSize(size);
		v->SetFont(&font);
		while (text.size() > 8 && x + v->StringWidth(text.c_str()) > r.right - 8)
			text = text.substr(0, text.size() - 4) + "...";
		Text(v, text.c_str(), x, y, size, Alpha(c, alpha));
		y += size * 1.35f;
	}
}


void
ReplayRenderer::_DrawSidePanel(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, nullptr);
	// The DSKY of the spacecraft that matters now: the LM's in its own
	// phases (both when there's room), the CM's otherwise.
	bool lmPhase = f.crewed == REC_LM;
	const RecDisplay* cmc = fRec.DisplayAt(false, f.get);
	const RecDisplay* lgc = fRec.DisplayAt(true, f.get);
	auto toState = [](const RecDisplay* d) {
		agc::DSKYState s;
		if (!d)
			return s;
		std::memcpy(s.prog, d->prog, 2);
		std::memcpy(s.verb, d->verb, 2);
		std::memcpy(s.noun, d->noun, 2);
		std::memcpy(s.reg, d->reg, sizeof(s.reg));
		std::memcpy(s.sign, d->sign, 3);
		uint32_t l = d->lamps;
		s.compActy = l & LAMP_COMP_ACTY;
		s.uplinkActy = l & LAMP_UPLINK;
		s.temp = l & LAMP_TEMP;
		s.keyRel = l & LAMP_KEY_REL;
		s.verbNounFlash = l & LAMP_FLASH;
		s.oprErr = l & LAMP_OPR_ERR;
		s.noAtt = l & LAMP_NO_ATT;
		s.gimbalLock = l & LAMP_GIMBAL_LOCK;
		s.progAlarm = l & LAMP_PROG;
		s.tracker = l & LAMP_TRACKER;
		s.restart = l & LAMP_RESTART;
		s.standby = l & LAMP_STBY;
		s.alt = l & LAMP_ALT;
		s.vel = l & LAMP_VEL;
		s.prioDisp = l & LAMP_PRIO_DISP;
		s.noDap = l & LAMP_NO_DAP;
		return s;
	};
	float inner = r.Width() - 20;
	float scale = inner / DSKYPainter::kWidth;
	float dskyH = DSKYPainter::kDisplayHeight * scale;
	float y = r.top + 10;
	auto draw = [&](DSKYPainter* p, const RecDisplay* d, const char* label) {
		Text(v, label, r.left + 10, y + 12, 11, kDim, true);
		y += 18;
		v->PushState();
		v->SetOrigin(BPoint(r.left + 10, y));
		v->SetScale(scale);
		p->Draw(v, toState(d), f.flash, false);
		v->PopState();
		y += dskyH + 10;
	};
	bool both = lgc && r.Height() > 2 * dskyH + 220 && f.s.veh[REC_LM].Present()
		&& f.s.phase >= PH_LUNAR_ORBIT && f.s.phase <= PH_RENDEZVOUS;
	if (lmPhase || both)
		draw(fLM, lgc, ("LGC  " + fRec.header.lmCallsign).c_str());
	if (!lmPhase || both)
		draw(fCM, cmc, ("CMC  " + fRec.header.csmCallsign).c_str());
	_DrawReadouts(v, f, BRect(r.left + 10, y, r.right - 10, r.bottom - 10));
}


void
ReplayRenderer::_DrawReadouts(BView* v, const Frame& f, BRect r)
{
	if (r.Height() < 40)
		return;
	const RecVehicle& c = f.s.veh[f.crewed];
	bool moon = f.s.phase >= PH_LUNAR_ORBIT && f.s.phase <= PH_RENDEZVOUS;
	if (f.s.phase == PH_TLC || f.s.phase == PH_TEC) {
		moon = (c.r - f.s.moon).Length() < world::kMoonSOI;
	}
	struct Row { std::string label, value; };
	std::vector<Row> rows;
	double alt = f.s.altitude;
	if (f.scene == Scene::Landing || f.scene == Scene::Ascent
			|| f.scene == Scene::Surface) {
		rows.push_back({"ALT", Fmt("%.0f FT", alt / 0.3048)});
		Vec3 up = (c.r - f.s.moon).Unit();
		Vec3 rel = c.v - f.s.moonV - MoonSurfaceVelocity(f.jd, c.r - f.s.moon);
		rows.push_back({"ALT RATE", Fmt("%+.1f FT/S", rel.Dot(up) / 0.3048)});
		rows.push_back({"HORIZ VEL",
			Fmt("%.1f FT/S", (rel - up * rel.Dot(up)).Length() / 0.3048)});
	} else if (alt < 1e6) {
		rows.push_back({"ALT", Fmt("%.1f NMI", alt / 1852)});
		rows.push_back({"VEL", Fmt("%.0f FT/S", f.s.speed / 0.3048)});
	} else {
		rows.push_back({moon ? "FROM MOON" : "FROM EARTH",
			Fmt("%.0f NMI", alt / 1852)});
		rows.push_back({"VEL", Fmt("%.0f FT/S", f.s.speed / 0.3048)});
	}
	if (f.scene != Scene::Landing && f.scene != Scene::Surface) {
		double toMoon = ((c.r - f.s.moon).Length() - world::kRMoon) / 1852;
		double toEarth = (c.r.Length() - world::kREarth) / 1852;
		if (alt >= 1e6)
			rows.push_back({moon ? "FROM EARTH" : "FROM MOON",
				Fmt("%.0f NMI", moon ? toEarth : toMoon)});
	}
	const RecVehicle& csm = f.s.veh[REC_CSM];
	const RecVehicle& lm = f.s.veh[REC_LM];
	if (csm.Present() && lm.Present()) {
		double range = (lm.r - csm.r).Length();
		if (range < 1e6)
			rows.push_back({"CSM-LM RANGE", range < 3000 ? Fmt("%.0f FT", range / 0.3048)
				: Fmt("%.1f NMI", range / 1852)});
	}
	float size = std::clamp(r.Width() / 22.0f, 10.0f, 16.0f);
	float y = r.top + size + 4;
	for (const Row& row : rows) {
		if (y > r.bottom)
			break;
		Text(v, row.label.c_str(), r.left, y, size * 0.8f, kDim);
		Text(v, row.value.c_str(), r.right, y, size, kGreen, true, 2);
		y += size * 1.6f;
	}
}


void
ReplayRenderer::_DrawTitle(BView* v, const Frame& f, BRect r, double fade)
{
	(void)f;
	v->SetDrawingMode(B_OP_ALPHA);
	v->SetHighColor(0, 0, 0, uint8(200 * fade));
	v->FillRect(r);
	const RecHeader& h = fRec.header;
	float cx = r.left + r.Width() / 2, cy = r.top + r.Height() / 2;
	float big = std::clamp(r.Width() / 16, 20.0f, 72.0f);
	Text(v, h.missionName.c_str(), cx, cy - big * 0.3f, big,
		Alpha(kWhite, fade), true, 1);
	std::string crew = h.csmCallsign;
	if (!h.lmCallsign.empty())
		crew += "  ·  " + h.lmCallsign;
	Text(v, crew.c_str(), cx, cy + big * 0.5f, big * 0.4f, Alpha(kAmber, fade),
		true, 1);
	std::string date = world::FormatDate(h.getZeroJD);
	Text(v, ("Launch " + date).c_str(), cx, cy + big * 1.1f, big * 0.28f,
		Alpha(kDim, fade), false, 1);
	if (!h.siteName.empty() && h.missionNumber != 7 && h.missionNumber != 9) {
		Text(v, h.siteName.c_str(), cx, cy + big * 1.5f, big * 0.28f,
			Alpha(kDim, fade), false, 1);
	}
	if (!h.difficulty.empty())
		Text(v, (h.difficulty + " — a flight flown by the real Apollo guidance"
			" computer software").c_str(), cx, cy + big * 1.9f, big * 0.24f,
			Alpha(kDim, fade), false, 1);
	v->SetDrawingMode(B_OP_COPY);
}


// --- Launch ----------------------------------------------------------------------

void
ReplayRenderer::_DrawLaunch(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, "LAUNCH PHASE — ALTITUDE vs RANGE");
	// Downrange from the pad over the Earth, altitude above it.
	auto ground = [&](const RecSample& s, double* down, double* alt) {
		const RecVehicle& c = s.veh[REC_CSM];
		double jd = fRec.header.getZeroJD + s.get / 86400;
		Vec3 fixed = world::Ephemeris::EarthFixedToInertial(jd).Transposed() * c.r;
		*down = AngleBetween(fPadFixed, fixed) * world::kREarth / 1000;
		*alt = (c.r.Length() - world::kREarth) / 1000;
	};
	double maxDown = 2400, maxAlt = 200;
	Plot p{r.InsetByCopy(60, 34), 0, maxDown, 0, maxAlt};
	p.r.bottom -= 6;
	PlotGrid(v, p, 400, 50, "%.0f", "%.0f");
	Text(v, "KM DOWNRANGE", p.r.right, p.r.bottom + 28, 10, kDim, false, 2);
	Text(v, "KM", p.r.left - 6, p.r.top - 6, 10, kDim, false, 2);
	// The flown path.
	BPoint last;
	bool have = false;
	v->SetPenSize(2);
	for (const RecSample& s : fRec.samples) {
		if (s.get > f.get)
			break;
		if (s.get < -60 || !s.veh[REC_CSM].Present())
			continue;
		double d, a;
		ground(s, &d, &a);
		if (!p.Inside(d, a))
			continue;
		BPoint q = p.P(d, a);
		if (have) {
			v->SetHighColor(kAmber);
			v->StrokeLine(last, q);
		}
		last = q;
		have = true;
	}
	v->SetPenSize(1);
	// Event labels along the path.
	for (const RecEvent& e : fRec.events) {
		if (e.get > f.get || e.get < -1 || e.get > 1200)
			continue;
		if (e.kind != LOG_EVENT || e.text.rfind("Phase: ", 0) == 0
				|| e.text == "Liftoff" || e.text.find("Ignition") == 0)
			continue;
		RecSample s = fRec.At(e.get);
		if (!s.veh[REC_CSM].Present())
			continue;
		double d, a;
		ground(s, &d, &a);
		if (!p.Inside(d, a))
			continue;
		BPoint q = p.P(d, a);
		v->SetHighColor(kWhite);
		v->FillEllipse(q, 2.5f, 2.5f);
		std::string t = e.text.substr(0, 28);
		Text(v, t.c_str(), q.x + 6, q.y + 14, 10, kWhite);
	}
	// The vehicle.
	const RecVehicle& c = f.s.veh[REC_CSM];
	if (c.Present()) {
		double d, a;
		ground(f.s, &d, &a);
		BPoint q = p.P(std::clamp(d, 0.0, maxDown), std::clamp(a, 0.0, maxAlt));
		// Body X in the trajectory plane.
		Vec3 up = c.r.Unit();
		Vec3 vel = c.v - Vec3(-world::kEarthRate * c.r.y, world::kEarthRate * c.r.x, 0);
		Vec3 downDir = (vel - up * vel.Dot(up));
		if (downDir.Length() < 1)
			downDir = up.Cross(Vec3(0, 0, 1)).Cross(up) * -1.0;
		downDir = downDir.Unit();
		Vec3 x = c.q.Rotate(Vec3(1, 0, 0));
		float angle = float(std::atan2(x.Dot(up), x.Dot(downDir)));
		// The plot isn't to scale: keep the rocket's look.
		Part tmp[24];
		float stackLen = 1;
		BuildStack(c.modules, tmp, &stackLen);
		float scale = std::clamp(p.r.Height() * 0.16f / std::max(stackLen, 10.0f),
			0.3f, 6.0f);
		DrawStack(v, BPoint(q.x, q.y - 4), angle, scale, c.modules);
		if (c.flags & REC_ENGINE) {
			float ca = std::cos(angle), sa = std::sin(angle);
			Part parts[24];
			float len;
			BuildStack(c.modules, parts, &len);
			BPoint tail(q.x - ca * len / 2 * scale, q.y - 4 + sa * len / 2 * scale);
			BPoint flame[3] = {
				BPoint(tail.x - sa * 4 * scale, tail.y - ca * 4 * scale),
				BPoint(tail.x + sa * 4 * scale, tail.y + ca * 4 * scale),
				BPoint(tail.x - ca * (18 + 6 * float(f.flash)) * scale,
					tail.y + sa * (18 + 6 * float(f.flash)) * scale)};
			v->SetDrawingMode(B_OP_ALPHA);
			v->SetHighColor(255, 190, 90, 200);
			v->FillPolygon(flame, 3);
			v->SetDrawingMode(B_OP_COPY);
		}
	}
	// Countdown before liftoff.
	if (f.get < 0) {
		std::string t = Fmt("T-%.0f", std::ceil(-f.get));
		float size = std::clamp(r.Height() / 6, 24.0f, 96.0f);
		Text(v, t.c_str(), r.left + r.Width() / 2, r.top + r.Height() / 2, size,
			kAmber, true, 1);
	}
}


// --- Earth orbit: the world map --------------------------------------------------

void
ReplayRenderer::_DrawEarthOrbit(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, "GROUND TRACK");
	const RecVehicle& c = f.s.veh[f.crewed];
	if (!c.Present())
		return;
	Mat3 toFixed = world::Ephemeris::EarthFixedToInertial(f.jd).Transposed();
	Vec3 fixed = toFixed * c.r;
	double lat, lon, rad;
	world::VectorToLatLon(fixed, &lat, &lon, &rad);
	// Look from a little above the spacecraft, the globe filling the view.
	float radius = std::min(r.Width(), r.Height()) * 0.44f;
	BPoint center(r.left + r.Width() / 2, r.top + r.Height() / 2 + 8);
	Globe g(center, radius, std::clamp(lat, -60.0, 60.0), lon);
	DrawEarth(v, g, f.jd);
	// The ground track: the last orbit and a half and what's coming.
	std::vector<Vec3> past, future;
	for (const RecSample& s : fRec.samples) {
		if (s.get < f.get - 8000 || s.get > f.get + 3000)
			continue;
		const RecVehicle& vv = s.veh[f.crewed];
		if (!vv.Present())
			continue;
		double jd = fRec.header.getZeroJD + s.get / 86400;
		Vec3 p = (world::Ephemeris::EarthFixedToInertial(jd).Transposed()
			* vv.r).Unit();
		(s.get <= f.get ? past : future).push_back(p);
	}
	if (!past.empty())
		future.insert(future.begin(), past.back());
	GlobeLine(v, g, future, Alpha(kAmber, 0.35));
	v->SetPenSize(2);
	GlobeLine(v, g, past, kAmber);
	v->SetPenSize(1);
	Vec3 here = fixed.Unit();
	if (g.Visible(here)) {
		BPoint q = g.Project(here);
		v->SetHighColor(kWhite);
		v->FillEllipse(q, 4, 4);
		v->SetHighColor(kAmber);
		v->StrokeEllipse(q, 9, 9);
	}
	std::string pos = Fmt("%.1f", std::fabs(lat)) + (lat >= 0 ? "N " : "S ")
		+ Fmt("%.1f", std::fabs(lon)) + (lon >= 0 ? "E" : "W");
	Text(v, pos.c_str(), r.right - 10, r.top + 18, 12, kGreen, false, 2);
	// The rest of the stack, if it's close.
	const RecVehicle& b = f.s.veh[REC_BOOSTER];
	if (b.Present() && f.crewed == REC_CSM) {
		double range = (b.r - c.r).Length();
		if (range < 1e6)
			Text(v, ("S-IVB " + Fmt("%.1f NMI", range / 1852)).c_str(),
				r.right - 10, r.top + 34, 11, kDim, false, 2);
	}
}


// --- Translunar and transearth: the Earth-Moon plot ---------------------------------

void
ReplayRenderer::_DrawCislunar(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, "EARTH-MOON — ROTATING FRAME");
	// Earth on the left, the Moon on the right, the path drawn in the
	// frame that turns with the Moon.
	double margin = 60000;
	double x0 = -margin - 20000, x1 = 384400 + margin;
	double half = (x1 - x0) / 2 * r.Height() / r.Width();
	Plot p{r.InsetByCopy(16, 30), x0, x1, -half, half};
	// Grid rings round the Earth.
	v->SetHighColor(kGrid);
	for (double k = 100000; k <= 400000; k += 100000) {
		BPoint c = p.P(0, 0);
		float rx = float(k / (x1 - x0) * p.r.Width());
		float ry = float(k / (2 * half) * p.r.Height());
		v->StrokeEllipse(c, rx, ry);
		Text(v, Fmt("%.0fk km", k / 1000).c_str(), c.x + rx + 3, c.y - 3, 9, kDim);
		v->SetHighColor(kGrid);
	}
	// Earth and Moon (exaggerated, labelled).
	BPoint e = p.P(0, 0), m = p.P(384400, 0);
	float er = std::max(6.0f, float(6378.0 / (x1 - x0) * p.r.Width()) * 3);
	float mr = std::max(4.0f, float(1737.0 / (x1 - x0) * p.r.Width()) * 3);
	v->SetHighColor(kOcean);
	v->FillEllipse(e, er, er);
	v->SetHighColor(kCyan);
	v->StrokeEllipse(e, er, er);
	v->SetHighColor(kMoonGray);
	v->FillEllipse(m, mr, mr);
	Text(v, "EARTH", e.x, e.y + er + 14, 10, kDim, false, 1);
	Text(v, "MOON", m.x, m.y + mr + 14, 10, kDim, false, 1);
	// The path: all of it faint, the part flown bright.
	BPoint last;
	bool have = false;
	for (const MapPoint& q : fMapPath) {
		BPoint s = p.P(q.x, q.y);
		if (have) {
			v->SetHighColor(q.get <= f.get ? kAmber : Alpha(kAmber, 0.18));
			if (q.get > f.get)
				v->SetDrawingMode(B_OP_ALPHA);
			v->StrokeLine(last, s);
			v->SetDrawingMode(B_OP_COPY);
		}
		last = s;
		have = true;
	}
	// Burn markers.
	for (const Marker& mk : fMarkers) {
		if (mk.get > f.get)
			continue;
		RecSample s = fRec.At(mk.get);
		const RecVehicle& c = s.veh[REC_CSM];
		if (!c.Present())
			continue;
		Vec3 ux = s.moon.Unit(), uz = s.moon.Cross(s.moonV).Unit();
		Vec3 uy = uz.Cross(ux);
		BPoint q = p.P(c.r.Dot(ux) / 1000, c.r.Dot(uy) / 1000);
		v->SetHighColor(kCyan);
		v->StrokeEllipse(q, 4, 4);
		Text(v, mk.label.c_str(), q.x + 6, q.y - 6, 10, kCyan);
	}
	// The spacecraft now.
	const RecVehicle& c = f.s.veh[REC_CSM];
	if (c.Present()) {
		Vec3 ux = f.s.moon.Unit(), uz = f.s.moon.Cross(f.s.moonV).Unit();
		Vec3 uy = uz.Cross(ux);
		BPoint q = p.P(c.r.Dot(ux) / 1000, c.r.Dot(uy) / 1000);
		v->SetHighColor(kWhite);
		v->FillEllipse(q, 4, 4);
		v->SetHighColor(kAmber);
		v->StrokeEllipse(q, 8 + 2 * float(f.flash), 8 + 2 * float(f.flash));
	}
	// Elapsed and to go.
	double fromEarth = c.Present() ? (c.r.Length() - world::kREarth) / 1852 : 0;
	double fromMoon = c.Present()
		? ((c.r - f.s.moon).Length() - world::kRMoon) / 1852 : 0;
	Text(v, ("EARTH " + Fmt("%.0f NMI", fromEarth)).c_str(), r.left + 12,
		r.bottom - 12, 12, kGreen);
	Text(v, ("MOON " + Fmt("%.0f NMI", fromMoon)).c_str(), r.right - 12,
		r.bottom - 12, 12, kGreen, false, 2);
}


// --- Lunar orbit -----------------------------------------------------------------------

void
ReplayRenderer::_DrawMoonOrbit(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, "LUNAR ORBIT");
	Mat3 toFixed = world::Ephemeris::MoonFixedToInertial(f.jd).Transposed();
	const RecVehicle& c = f.s.veh[f.crewed];
	if (!c.Present())
		return;
	Vec3 fixed = toFixed * (c.r - f.s.moon);
	double lat, lon, rad;
	world::VectorToLatLon(fixed, &lat, &lon, &rad);
	// From the far side of the approach, look at the near side with the
	// spacecraft in view; in orbit, follow it.
	float radius = std::min(r.Width(), r.Height()) * 0.40f;
	BPoint center(r.left + r.Width() / 2, r.top + r.Height() / 2 + 8);
	double alt = rad - world::kRMoon;
	double viewLat = std::clamp(lat * 0.5, -30.0, 30.0);
	double viewLon = alt < 500e3 ? lon : 0;
	Globe g(center, radius, viewLat, viewLon);
	DrawMoon(v, g, f.jd, fRec.header.missionNumber);
	// Orbit tracks (CSM amber, LM cyan) over the last two hours.
	for (int k : {REC_CSM, REC_LM}) {
		std::vector<Vec3> past;
		for (const RecSample& s : fRec.samples) {
			if (s.get < f.get - 7200 || s.get > f.get)
				continue;
			const RecVehicle& vv = s.veh[k];
			if (!vv.Present())
				continue;
			double jd = fRec.header.getZeroJD + s.get / 86400;
			Vec3 q = world::Ephemeris::MoonFixedToInertial(jd).Transposed()
				* (vv.r - s.moon);
			if (q.Length() > world::kRMoon * 3)
				continue;
			past.push_back(q.Unit());
		}
		v->SetPenSize(2);
		GlobeLine(v, g, past, k == REC_CSM ? kAmber : kCyan);
		v->SetPenSize(1);
	}
	for (int k : {REC_CSM, REC_LM}) {
		const RecVehicle& vv = f.s.veh[k];
		if (!vv.Present())
			continue;
		Vec3 q = toFixed * (vv.r - f.s.moon);
		double h = q.Length() - world::kRMoon;
		// Off the globe (on the approach): draw at its true scale.
		Vec3 u = q.Unit();
		BPoint at;
		if (h > 400e3) {
			double s = q.Length() / world::kRMoon;
			at = BPoint(center.x + float(u.Dot(g.east) * s) * radius,
				center.y - float(u.Dot(g.north) * s) * radius);
		} else if (g.Visible(u))
			at = g.Project(u);
		else
			continue;
		v->SetHighColor(kWhite);
		v->FillEllipse(at, 4, 4);
		v->SetHighColor(k == REC_CSM ? kAmber : kCyan);
		v->StrokeEllipse(at, 8, 8);
		std::string name = k == REC_CSM ? fRec.header.csmCallsign
			: fRec.header.lmCallsign;
		Text(v, name.c_str(), at.x + 11, at.y - 8, 11,
			k == REC_CSM ? kAmber : kCyan);
	}
	Text(v, fRec.header.siteName.c_str(), r.right - 10, r.top + 18, 12, kAmber,
		false, 2);
}


// --- Powered descent and ascent: the landing plot -----------------------------------

void
ReplayRenderer::_DrawLanding(BView* v, const Frame& f, BRect r, bool ascent)
{
	PanelFrame(v, r, ascent ? "LUNAR ASCENT — ALTITUDE vs RANGE"
		: "POWERED DESCENT — ALTITUDE vs RANGE TO GO");
	const RecHeader& h = fRec.header;
	auto siteAt = [&](const RecSample& s) {
		double jd = h.getZeroJD + s.get / 86400;
		return world::Ephemeris::MoonFixedToInertial(jd)
			* world::LatLonToVector(h.siteLatitude, h.siteLongitude, h.siteRadius);
	};
	// Range (km, signed: + before the site) and altitude (km).
	auto where = [&](const RecSample& s, double* range, double* alt) {
		const RecVehicle& lm = s.veh[REC_LM];
		Vec3 rel = lm.r - s.moon;
		Vec3 site = siteAt(s);
		double ang = AngleBetween(rel, site);
		Vec3 vel = lm.v - s.moonV;
		double sign = (site - rel).Dot(vel) >= 0 ? 1 : -1;
		*range = sign * ang * h.siteRadius / 1000;
		*alt = (rel.Length() - h.siteRadius) / 1000;
		if (ascent)
			*range = -*range;
	};
	double maxRange = ascent ? 300 : 500, maxAlt = ascent ? 20 : 16;
	BRect plotR = r.InsetByCopy(60, 34);
	plotR.bottom -= 6;
	// The close-up below 1,000 m: a second plot to scale.
	bool close = !ascent && f.s.altitude < 1500 && f.s.veh[REC_LM].Present();
	BRect closeR;
	if (close) {
		closeR = plotR;
		closeR.left = plotR.left + plotR.Width() * 0.52f;
		plotR.right = closeR.left - 50;
	}
	Plot p{plotR, ascent ? 0 : -maxRange, ascent ? maxRange : 10, 0, maxAlt};
	if (ascent)
		p = Plot{plotR, -10, maxRange, 0, maxAlt};
	else
		p = Plot{plotR, -10, maxRange, 0, maxAlt};
	// Range to go runs right to left toward the site at zero.
	auto P = [&](double range, double alt) {
		return ascent ? p.P(range, alt) : p.P(maxRange - range - 10, alt);
	};
	v->SetHighColor(kGrid);
	for (double x = 0; x <= maxRange; x += 100) {
		BPoint a = P(x, 0), b = P(x, maxAlt);
		v->SetHighColor(kGrid);
		v->StrokeLine(a, b);
		Text(v, Fmt("%.0f", x).c_str(), a.x, a.y + 14, 10, kDim, false, 1);
	}
	for (double y = 0; y <= maxAlt; y += 4) {
		BPoint a = P(-10, y), b = P(maxRange, y);
		v->SetHighColor(kGrid);
		v->StrokeLine(a, b);
		Text(v, Fmt("%.0f", y).c_str(), std::min(a.x, b.x) - 6, a.y + 4, 10, kDim,
			false, 2);
	}
	v->SetHighColor(kGridBright);
	v->StrokeRect(p.r);
	Text(v, ascent ? "KM DOWNRANGE" : "KM TO GO", p.r.right, p.r.bottom + 28, 10,
		kDim, false, 2);
	// The site.
	BPoint site = P(0, 0);
	v->SetHighColor(kAmber);
	v->FillEllipse(site, 4, 4);
	// The path.
	BPoint last;
	bool have = false;
	v->SetPenSize(2);
	for (const RecSample& s : fRec.samples) {
		if (s.get > f.get)
			break;
		if (s.phase != (ascent ? PH_ASCENT : PH_DESCENT)
				&& !(ascent && s.phase == PH_RENDEZVOUS))
			continue;
		if (!s.veh[REC_LM].Present())
			continue;
		double rg, al;
		where(s, &rg, &al);
		if (rg < -10 || rg > maxRange || al > maxAlt)
			continue;
		BPoint q = P(rg, al);
		if (have) {
			v->SetHighColor(kCyan);
			v->StrokeLine(last, q);
		}
		last = q;
		have = true;
	}
	v->SetPenSize(1);
	const RecVehicle& lm = f.s.veh[REC_LM];
	if (lm.Present()) {
		double rg, al;
		where(f.s, &rg, &al);
		BPoint q = P(std::clamp(rg, -10.0, maxRange), std::clamp(al, 0.0, maxAlt));
		Vec3 up = (lm.r - f.s.moon).Unit();
		Vec3 vel = lm.v - f.s.moonV;
		Vec3 dir = vel - up * vel.Dot(up);
		if (dir.Length() < 0.1)
			dir = up.Cross(Vec3(0, 0, 1));
		dir = dir.Unit();
		Vec3 x = lm.q.Rotate(Vec3(1, 0, 0));
		// Screen: downrange is to the left in the descent plot.
		float angle = float(std::atan2(x.Dot(up), (ascent ? 1 : -1) * x.Dot(dir)));
		DrawLM(v, q, angle, 2.6f, (lm.modules & REC_LM_DESCENT) != 0,
			(lm.flags & REC_ENGINE) != 0);
	}
	if (!close)
		return;

	// Close-up: the last 1,500 m to scale with the surface and the LM.
	PanelFrame(v, closeR, "FINAL APPROACH");
	double rg, al;
	where(f.s, &rg, &al);
	double altM = f.s.altitude;
	double spanM = std::max(60.0, std::min(1500.0, altM * 2.2 + 40));
	float k = float(closeR.Height() * 0.8 / spanM);
	float groundY = closeR.bottom - closeR.Height() * 0.12f;
	BPoint lmAt(closeR.left + closeR.Width() * 0.55f, groundY - float(altM) * k);
	// Surface with a few craters, drifting by with the horizontal speed.
	Vec3 up = (lm.r - f.s.moon).Unit();
	Vec3 vel = lm.v - f.s.moonV - MoonSurfaceVelocity(f.jd, lm.r - f.s.moon);
	double horiz = (vel - up * vel.Dot(up)).Length();
	double vert = vel.Dot(up);
	double drift = std::fmod(rg * 1000 * k, 400.0);
	v->SetHighColor(kMoonGray);
	v->FillRect(BRect(closeR.left + 1, groundY, closeR.right - 1, closeR.bottom - 1));
	v->SetHighColor(kMare);
	{
		BRegion inner(closeR.InsetByCopy(1, 1));
		v->ConstrainClippingRegion(&inner);
		for (int i = -1; i < 12; i++) {
			float x = closeR.left + float(std::fmod(i * 97.0 + drift + 4000, 400.0))
				* closeR.Width() / 400;
			float radius = float(1.5 + (i * 37 % 7)) * k;     // craters 2-8 m
			v->FillEllipse(BPoint(x, groundY + 3 + (i % 3) * 4), radius,
				std::max(1.0f, radius / 5));
		}
	}
	// Dust below 30 m.
	if (altM < 30 && (lm.flags & REC_ENGINE)) {
		v->SetDrawingMode(B_OP_ALPHA);
		v->SetHighColor(200, 195, 180, uint8(120 * (1 - altM / 30)));
		float w = float(40 + (30 - altM) * 4);
		v->FillEllipse(BPoint(lmAt.x, groundY), w, 6);
		v->SetDrawingMode(B_OP_COPY);
	}
	Vec3 x = lm.q.Rotate(Vec3(1, 0, 0));
	Vec3 dir = horiz > 0.1 ? ((vel - up * vel.Dot(up)) / horiz) : up.Cross(Vec3(0, 0, 1)).Unit();
	float angle = float(std::atan2(x.Dot(up), -x.Dot(dir)));
	DrawLM(v, lmAt, angle, std::max(3.0f, std::min(14.0f, k)),
		(lm.modules & REC_LM_DESCENT) != 0, (lm.flags & REC_ENGINE) != 0);
	{
		BRegion outer(r);
		v->ConstrainClippingRegion(&outer);
	}
	// The cross-pointer numbers.
	float size = std::clamp(closeR.Width() / 16, 11.0f, 20.0f);
	Text(v, Fmt("%.0f FT", altM / 0.3048).c_str(), closeR.left + 12,
		closeR.top + 22 + size, size, kGreen, true);
	Text(v, Fmt("%+.1f FT/S DOWN", vert / 0.3048).c_str(), closeR.left + 12,
		closeR.top + 26 + 2 * size, size * 0.8f, kGreen);
	Text(v, Fmt("%.1f FT/S FWD", horiz / 0.3048).c_str(), closeR.left + 12,
		closeR.top + 30 + 3 * size, size * 0.8f, kGreen);
	if (altM < 25 && f.flash && (lm.flags & REC_ENGINE))
		Text(v, "CONTACT", closeR.right - 12, closeR.top + 22 + size, size,
			kAmber, true, 2);
}


// --- On the surface -----------------------------------------------------------------

void
ReplayRenderer::_DrawSurface(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, nullptr);
	const RecHeader& h = fRec.header;
	// Black sky, the Earth over the horizon, grey ground, the LM.
	v->SetHighColor(0, 0, 0, 255);
	v->FillRect(r.InsetByCopy(1, 1));
	float horizon = r.top + r.Height() * 0.62f;
	v->SetHighColor(kMoonGray);
	v->FillRect(BRect(r.left + 1, horizon, r.right - 1, r.bottom - 1));
	v->SetHighColor(150, 148, 140, 255);
	for (int i = 0; i < 14; i++) {
		float x = r.left + float((i * 151) % int(std::max(1.0f, r.Width())));
		float y = horizon + 10 + float((i * 67) % int(std::max(1.0f,
			r.Height() * 0.35f)));
		float w = 8 + float(i * 13 % 30);
		v->SetHighColor(i % 2 ? kMare : tint_color(kMoonGray, B_LIGHTEN_1_TINT));
		v->FillEllipse(BPoint(x, y), w, w * 0.22f);
	}
	// The Earth, a small blue-white crescent-ish disk.
	BPoint earth(r.left + r.Width() * 0.78f, r.top + r.Height() * 0.2f);
	float er = std::max(10.0f, r.Height() * 0.05f);
	Globe g(earth, er, 0, 0);
	DrawEarth(v, g, f.jd);
	float scale = std::clamp(r.Height() / 40.0f, 4.0f, 20.0f);
	DrawLM(v, BPoint(r.left + r.Width() * 0.42f, horizon + scale * 3.4f),
		float(kPi / 2), scale, (f.s.veh[REC_LM].modules & REC_LM_DESCENT) != 0,
		false);
	// The flag beside it.
	float fx = r.left + r.Width() * 0.42f + scale * 7;
	v->SetHighColor(kWhite);
	v->StrokeLine(BPoint(fx, horizon + scale), BPoint(fx, horizon - scale * 4));
	v->SetHighColor(200, 40, 40, 255);
	v->FillRect(BRect(fx, horizon - scale * 4, fx + scale * 2.4f,
		horizon - scale * 2.6f));
	v->SetHighColor(30, 50, 140, 255);
	v->FillRect(BRect(fx, horizon - scale * 4, fx + scale, horizon - scale * 3.3f));
	// Time on the surface.
	double landed = 0;
	for (const RecSample& s : fRec.samples)
		if (s.phase == PH_SURFACE) {
			landed = s.get;
			break;
		}
	float size = std::clamp(r.Height() / 18, 12.0f, 28.0f);
	Text(v, h.siteName.c_str(), r.left + 16, r.top + 16 + size, size, kWhite, true);
	Text(v, ("ON THE SURFACE " + FormatGET(f.get - landed)).c_str(), r.left + 16,
		r.top + 22 + 2 * size, size * 0.7f, kAmber);
}


// --- Rendezvous: the relative-motion plot ------------------------------------------------

void
ReplayRenderer::_DrawRendezvous(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, "RENDEZVOUS — LM RELATIVE TO CSM (LVLH)");
	const RecVehicle& csm = f.s.veh[REC_CSM];
	const RecVehicle& lm = f.s.veh[REC_LM];
	if (!csm.Present() || !lm.Present())
		return;
	// The CSM's local frame: x behind/ahead along the orbit, y up.
	auto rel = [&](const RecSample& s, double* x, double* y) {
		const RecVehicle& c = s.veh[REC_CSM];
		const RecVehicle& l = s.veh[REC_LM];
		Vec3 rc = c.r - s.moon, vc = c.v - s.moonV;
		Vec3 up = rc.Unit();
		Vec3 hn = rc.Cross(vc).Unit();
		Vec3 along = hn.Cross(up);
		Vec3 d = (l.r - s.moon) - rc;
		// Curvilinear along-track: the angle around the Moon.
		double ang = std::atan2(hn.Dot(rc.Cross(l.r - s.moon)), rc.Dot(l.r - s.moon));
		*x = ang * rc.Length() / 1000;
		*y = ((l.r - s.moon).Length() - rc.Length()) / 1000;
		(void)d;
		(void)along;
	};
	// Extent from the whole rendezvous so the plot doesn't jump.
	double xmin = -5, xmax = 5, ymin = -5, ymax = 2;
	for (const RecSample& s : fRec.samples) {
		if (s.phase != PH_RENDEZVOUS && s.phase != PH_ASCENT)
			continue;
		if (!s.veh[REC_CSM].Present() || !s.veh[REC_LM].Present())
			continue;
		double x, y;
		rel(s, &x, &y);
		if (std::fabs(x) > 600)
			continue;
		xmin = std::min(xmin, x);
		xmax = std::max(xmax, x);
		ymin = std::min(ymin, y);
		ymax = std::max(ymax, y);
	}
	double padx = (xmax - xmin) * 0.08, pady = (ymax - ymin) * 0.15;
	Plot p{r.InsetByCopy(60, 34), xmin - padx, xmax + padx, ymin - pady,
		ymax + pady};
	p.r.bottom -= 6;
	double gx = std::pow(10, std::floor(std::log10(std::max(1.0, (p.x1 - p.x0) / 4))));
	double gy = std::pow(10, std::floor(std::log10(std::max(0.5, (p.y1 - p.y0) / 3))));
	PlotGrid(v, p, gx, gy, "%.0f", "%.0f");
	Text(v, "KM AHEAD OF THE CSM", p.r.right, p.r.bottom + 28, 10, kDim, false, 2);
	Text(v, "KM ABOVE", p.r.left - 6, p.r.top - 6, 10, kDim, false, 2);
	BPoint c0 = p.P(0, 0);
	v->SetHighColor(kAmber);
	v->FillEllipse(c0, 5, 5);
	Text(v, fRec.header.csmCallsign.c_str(), c0.x + 8, c0.y - 8, 11, kAmber);
	BPoint last;
	bool have = false;
	v->SetPenSize(2);
	for (const RecSample& s : fRec.samples) {
		if (s.get > f.get)
			break;
		if (s.phase != PH_RENDEZVOUS && s.phase != PH_ASCENT)
			continue;
		if (!s.veh[REC_CSM].Present() || !s.veh[REC_LM].Present())
			continue;
		double x, y;
		rel(s, &x, &y);
		if (!p.Inside(x, y)) {
			have = false;
			continue;
		}
		BPoint q = p.P(x, y);
		if (have) {
			v->SetHighColor(kCyan);
			v->StrokeLine(last, q);
		}
		last = q;
		have = true;
	}
	v->SetPenSize(1);
	double x, y;
	rel(f.s, &x, &y);
	if (p.Inside(x, y)) {
		BPoint q = p.P(x, y);
		v->SetHighColor(kWhite);
		v->FillEllipse(q, 4, 4);
		v->SetHighColor(kCyan);
		v->StrokeEllipse(q, 8, 8);
		Text(v, fRec.header.lmCallsign.c_str(), q.x + 10, q.y + 14, 11, kCyan);
	}
	double range = (lm.r - csm.r).Length();
	double rate = (lm.v - csm.v).Dot((lm.r - csm.r).Unit());
	Text(v, ("RANGE " + Fmt("%.2f NMI", range / 1852) + "   RATE "
		+ Fmt("%+.1f FT/S", rate / 0.3048)).c_str(), r.left + 12, r.bottom - 12,
		12, kGreen);
}


// --- Docking: side view to scale ----------------------------------------------------------

void
ReplayRenderer::_DrawDocking(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, "DOCKING");
	const RecVehicle& csm = f.s.veh[REC_CSM];
	int otherSlot = f.s.veh[REC_LM].Present() ? REC_LM : REC_BOOSTER;
	const RecVehicle& other = f.s.veh[otherSlot];
	if (!csm.Present() || !other.Present())
		return;
	// View plane: the CSM's X axis horizontal, the offset to the target
	// (perpendicular to it) vertical.
	Vec3 x = csm.q.Rotate(Vec3(1, 0, 0));
	Vec3 d = other.r - csm.r;
	Vec3 perp = d - x * d.Dot(x);
	Vec3 yAxis = perp.Length() > 0.05 ? perp.Unit()
		: csm.q.Rotate(Vec3(0, 0, 1));
	double range = d.Length();
	double span = std::clamp(range * 1.6 + 30, 40.0, 600.0);
	float k = float(r.Width() * 0.8 / span);
	BPoint mid(r.left + r.Width() / 2, r.top + r.Height() / 2 + 10);
	auto S = [&](const Vec3& p) {
		Vec3 q = p - (csm.r + other.r) * 0.5;
		return BPoint(mid.x + float(q.Dot(x)) * k, mid.y - float(q.Dot(yAxis)) * k);
	};
	auto angleOf = [&](const Quat& q) {
		Vec3 bx = q.Rotate(Vec3(1, 0, 0));
		return float(std::atan2(bx.Dot(yAxis), bx.Dot(x)));
	};
	// Stars for the sense of motion.
	v->SetHighColor(90, 96, 110, 255);
	for (int i = 0; i < 40; i++) {
		float sx = r.left + float((i * 211) % int(std::max(1.0f, r.Width())));
		float sy = r.top + 24 + float((i * 97) % int(std::max(1.0f, r.Height() - 30)));
		v->FillRect(BRect(sx, sy, sx + 1, sy + 1));
	}
	if (otherSlot == REC_LM)
		DrawLM(v, S(other.r), angleOf(other.q), k,
			(other.modules & REC_LM_DESCENT) != 0, (other.flags & REC_ENGINE) != 0);
	else
		DrawStack(v, S(other.r), angleOf(other.q), k, other.modules);
	DrawStack(v, S(csm.r), angleOf(csm.q), k, csm.modules);
	{
		BPoint a = S(csm.r), b = S(other.r);
		std::string otherName = otherSlot == REC_LM ? fRec.header.lmCallsign
			: std::string("S-IVB");
		Text(v, fRec.header.csmCallsign.c_str(), a.x, a.y + 8 + 3 * k, 11, kAmber,
			false, 1);
		Text(v, otherName.c_str(), b.x, b.y - 8 - 5 * k, 11, kCyan, false, 1);
	}
	if (csm.flags & REC_RCS) {
		v->SetHighColor(240, 240, 255, 255);
		BPoint q = S(csm.r);
		v->FillEllipse(q, 2, 2);
	}
	double rate = (other.v - csm.v).Dot(d.Unit());
	float size = std::clamp(r.Height() / 22, 11.0f, 18.0f);
	Text(v, ("RANGE " + Fmt("%.1f FT", range / 0.3048)).c_str(), r.left + 12,
		r.bottom - 14 - size, size, kGreen, true);
	Text(v, ("RATE " + Fmt("%+.2f FT/S", rate / 0.3048)).c_str(), r.left + 12,
		r.bottom - 12, size * 0.85f, kGreen);
}


// --- Entry ------------------------------------------------------------------------------

void
ReplayRenderer::_DrawEntry(BView* v, const Frame& f, BRect r)
{
	PanelFrame(v, r, "ENTRY — ALTITUDE vs RANGE TO SPLASHDOWN");
	const RecHeader& h = fRec.header;
	auto where = [&](const RecSample& s, double* range, double* alt) {
		const RecVehicle& c = s.veh[REC_CSM];
		double jd = h.getZeroJD + s.get / 86400;
		Vec3 fixed = world::Ephemeris::EarthFixedToInertial(jd).Transposed() * c.r;
		Vec3 target = world::LatLonToVector(h.splashLatitude, h.splashLongitude, 1.0);
		*range = AngleBetween(fixed, target) * world::kREarth / 1852;
		*alt = (c.r.Length() - world::kREarth) / 0.3048 / 1000;
	};
	double maxRange = 1800, maxAlt = 400;
	Plot p{r.InsetByCopy(60, 34), 0, maxRange, 0, maxAlt};
	p.r.bottom -= 6;
	auto P = [&](double range, double alt) { return p.P(maxRange - range, alt); };
	for (double x = 0; x <= maxRange; x += 300) {
		BPoint a = P(x, 0), b = P(x, maxAlt);
		v->SetHighColor(kGrid);
		v->StrokeLine(a, b);
		Text(v, Fmt("%.0f", x).c_str(), a.x, a.y + 14, 10, kDim, false, 1);
	}
	for (double y = 0; y <= maxAlt; y += 100) {
		BPoint a = P(maxRange, y), b = P(0, y);
		v->SetHighColor(kGrid);
		v->StrokeLine(a, b);
		Text(v, Fmt("%.0fK FT", y).c_str(), a.x - 6, a.y + 4, 10, kDim, false, 2);
	}
	v->SetHighColor(kGridBright);
	v->StrokeRect(p.r);
	Text(v, "NMI TO GO", p.r.right, p.r.bottom + 28, 10, kDim, false, 2);
	BPoint ship = P(0, 0);
	v->SetHighColor(kAmber);
	v->FillEllipse(ship, 4, 4);
	Text(v, "RECOVERY SHIP", ship.x - 6, ship.y - 8, 10, kAmber, false, 2);
	BPoint last;
	bool have = false;
	v->SetPenSize(2);
	for (const RecSample& s : fRec.samples) {
		if (s.get > f.get)
			break;
		if (s.phase != PH_ENTRY && s.phase != PH_RECOVERY)
			continue;
		double rg, al;
		where(s, &rg, &al);
		if (rg > maxRange || al > maxAlt)
			continue;
		BPoint q = P(rg, al);
		if (have) {
			v->SetHighColor(kAmber);
			v->StrokeLine(last, q);
		}
		last = q;
		have = true;
	}
	v->SetPenSize(1);
	const RecVehicle& c = f.s.veh[REC_CSM];
	if (!c.Present())
		return;
	double rg, al;
	where(f.s, &rg, &al);
	BPoint q = P(std::clamp(rg, 0.0, maxRange), std::clamp(al, 0.0, maxAlt));
	// The CM, heat shield forward, with the plasma glow at speed.
	double speed = c.v.Length();
	if (f.s.altitude < 100e3 && speed > 3000) {
		v->SetDrawingMode(B_OP_ALPHA);
		float glow = float(std::min(1.0, (speed - 3000) / 6000));
		v->SetHighColor(255, 140, 60, uint8(160 * glow));
		v->FillEllipse(q, 14 * glow + 4, 9 * glow + 3);
		v->SetDrawingMode(B_OP_COPY);
	}
	DrawStack(v, q, float(kPi), 4, c.modules);
	if (c.flags & (REC_DROGUES | REC_MAINS)) {
		bool mains = c.flags & REC_MAINS;
		v->SetHighColor(mains ? rgb_color{230, 120, 60, 255} : kWhite);
		for (int i = -1; i <= 1; i += mains ? 1 : 2) {
			BPoint cp(q.x + i * 12, q.y - 34);
			v->FillEllipse(cp, mains ? 10 : 5, mains ? 6 : 3);
			v->StrokeLine(q, cp);
		}
	}
	float size = std::clamp(r.Height() / 22, 11.0f, 18.0f);
	Text(v, Fmt("%.0f FT/S", speed / 0.3048).c_str(), r.left + 12, r.bottom - 14 - size,
		size, kGreen, true);
	Text(v, Fmt("%.0f NMI TO GO", rg).c_str(), r.left + 12, r.bottom - 12,
		size * 0.85f, kGreen);
}


// --- The end -----------------------------------------------------------------------------

void
ReplayRenderer::_DrawEnd(BView* v, const Frame& f, BRect r, bool lost)
{
	PanelFrame(v, r, nullptr);
	const RecHeader& h = fRec.header;
	float cx = r.left + r.Width() / 2, cy = r.top + r.Height() * 0.4f;
	float big = std::clamp(r.Width() / 14, 20.0f, 64.0f);
	if (!lost) {
		// The ocean, the CM under three mains.
		v->SetHighColor(kOcean);
		v->FillRect(BRect(r.left + 1, r.top + r.Height() * 0.72f, r.right - 1,
			r.bottom - 1));
		BPoint cm(cx, r.top + r.Height() * 0.72f - 6);
		DrawStack(v, cm, float(kPi / 2), 5, REC_CM);
		Text(v, "SPLASHDOWN", cx, cy, big, kWhite, true, 1);
	} else
		Text(v, "MISSION LOST", cx, cy, big, kRed, true, 1);
	Text(v, h.outcome.c_str(), cx, cy + big * 0.8f, big * 0.32f, kAmber, false, 1);
	Text(v, ("Mission elapsed time " + FormatGET(f.get)).c_str(), cx,
		cy + big * 1.3f, big * 0.28f, kDim, false, 1);
	float y = cy + big * 1.9f;
	for (const std::string& c : h.contingencies) {
		Text(v, ("Worked through: " + c).c_str(), cx, y, big * 0.26f, kDim, false, 1);
		y += big * 0.4f;
	}
}

}  // namespace ui
