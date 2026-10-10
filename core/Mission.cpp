// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Mission.h"

#include "agc/Ropes.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace columbia {

namespace {

const double kMaxComputerRate = 50.0;  // AGC runs at most 50x real time
const double kKeyInterval = 0.20;      // AGC seconds between queued keys
const double kSlowLead = 60.0;         // seconds before a burn
const double kSlowWarp = 10.0;
const double kPredictionHorizon = 12 * 86400.0;

}  // namespace


Mission::Mission()
	:
	fPredictionDirty(true),
	fPredictionAge(0),
	fWarp(1),
	fRequestedWarp(1),
	fPaused(false),
	fAutoSlow(true),
	fLastActiveBurn(0),
	fSlowedFor(0),
	fKeyDelay(0)
{
	fComputer.LoadRope(agc::kComanche055Rope, agc::kComanche055RopeSize);
	fComputer.onChannelWrite = [this](int ch, uint16_t value) {
		fDSKY.ChannelWritten(ch, value);
	};
	Reset();
}


const std::vector<double>&
Mission::WarpLevels()
{
	static const std::vector<double> levels = {1, 2, 5, 10, 50, 100, 500,
		1000, 5000, 10000, 50000};
	return levels;
}


void
Mission::Reset()
{
	fModel = sim::Model();
	// The Moon starts 60 degrees ahead of the spacecraft, giving a TLI
	// window within the first orbit or two.
	fModel.moonPhase0 = 60.0 * sim::kPi / 180.0;
	fState = sim::ParkingOrbit(fModel);
	fPlan.clear();
	fFlown.clear();
	fFlown.push_back({fState.t, fState.r, fState.v, false});
	fPredictionDirty = true;
	fPredictionAge = 0;
	fWarp = fRequestedWarp = 1;
	fPaused = false;
	fLastActiveBurn = 0;
	fSlowedFor = 0;
	fEvents.clear();
	fKeyQueue.clear();
	fKeyDelay = 0;

	fComputer.PowerOn();
	// Let the computer come up, then fresh-start it into P00, the way a
	// pad-loaded computer would be found in Earth orbit.
	fComputer.Run(2.0);
	QueueKeys("V36E");
	fKeyQueue.push_back(-2);   // pause
	QueueKeys("V37E00E");
	_Log("Earth parking orbit, 185 km. Guidance computer: Comanche 055.");
}


void
Mission::SetWarp(double warp)
{
	fWarp = fRequestedWarp = std::max(1.0, std::min(50000.0, warp));
}


bool
Mission::Finished() const
{
	return fState.outcome != sim::Outcome::None
		&& fState.outcome != sim::Outcome::OutOfPropellant;
}


void
Mission::Update(double realSeconds)
{
	if (fPaused || realSeconds <= 0)
		return;
	if (Finished()) {
		_RunComputer(realSeconds, realSeconds);
		return;
	}
	double dt = realSeconds * fWarp;

	// Slow down ahead of the next burn so it can be watched.
	if (fAutoSlow && fState.activeBurn == 0) {
		double next = 1e300;
		const sim::Burn* nextBurn = nullptr;
		for (const sim::Burn& b : fPlan) {
			if (!fState.Completed(b.id) && b.time < next) {
				next = b.time;
				nextBurn = &b;
			}
		}
		double slowAt = next - kSlowLead;
		if (nextBurn != nullptr && fWarp > kSlowWarp
			&& fState.t + dt > slowAt && fSlowedFor != nextBurn->id) {
			fSlowedFor = nextBurn->id;   // once; the user may speed up again
			dt = std::max(0.0, slowAt - fState.t);
			fWarp = kSlowWarp;
			_Log("Slowing to 10x for " + nextBurn->label + " at "
				+ sim::FormatGET(nextBurn->time) + ".");
		}
	}

	sim::VehicleState before = fState;
	sim::Propagator prop(fModel);
	prop.onPropellantDepleted = [this](const sim::Burn&, double achieved) {
		char text[96];
		std::snprintf(text, sizeof(text),
			"Propellant depleted — burn cut off after %.1f m/s.", achieved);
		_Log(text);
	};
	std::vector<sim::Sample> more;
	prop.Advance(fState, fPlan, fState.t + dt, &more, 60.0);
	_RecordFlown(more);

	// Events. A burn may start and finish within one tick at high warp.
	if (before.sivbAttached && !fState.sivbAttached)
		_Log("S-IVB separation.");
	auto cutoff = [this]() {
		char text[96];
		std::snprintf(text, sizeof(text), "Cutoff. S-IVB %.0f kg, SPS %.0f kg"
			" remaining.", fState.sivbAttached ? fState.sivbPropellant : 0.0,
			fState.spsPropellant);
		_Log(text);
		fWarp = fRequestedWarp;
		fPredictionDirty = true;
	};
	if (fLastActiveBurn != 0 && fState.activeBurn != fLastActiveBurn)
		cutoff();
	for (uint32_t id : fState.completed) {
		if (before.Completed(id))
			continue;
		for (const sim::Burn& b : fPlan) {
			if (b.id != id)
				continue;
			char text[128];
			std::snprintf(text, sizeof(text), "%s ignition (%s, %.1f m/s).",
				b.label.c_str(), sim::Engine(b.engine).name, b.DeltaV());
			_Log(text, std::max(b.time, before.t));
			if (fState.activeBurn != id)
				cutoff();
		}
	}
	fLastActiveBurn = fState.activeBurn;
	bool wasNear = fModel.NearMoon(before.r, before.t);
	bool isNear = fModel.NearMoon(fState.r, fState.t);
	if (!wasNear && isNear)
		_Log("Entering the Moon's sphere of influence.");
	else if (wasNear && !isNear)
		_Log("Leaving the Moon's sphere of influence.");
	if (fState.outcome != before.outcome) {
		std::string text = sim::OutcomeText(fState.outcome);
		if (fState.entrySpeed > 0) {
			char extra[96];
			std::snprintf(extra, sizeof(extra), " (%.0f m/s, %.2f°)",
				fState.entrySpeed, fState.entryAngle);
			text += extra;
		}
		_Log(text + " at " + sim::FormatGET(fState.t) + ".");
		fPredictionDirty = true;
	}

	_RunComputer(fState.t - before.t, realSeconds);
	fPredictionAge += realSeconds;
}


void
Mission::_RunComputer(double flightSeconds, double realSeconds)
{
	double budget = std::min(flightSeconds, realSeconds * kMaxComputerRate);
	double done = 0;
	const double slice = 0.05;
	while (done < budget) {
		double step = std::min(slice, budget - done);
		if (fKeyDelay <= 0 && !fKeyQueue.empty()) {
			int key = fKeyQueue.front();
			fKeyQueue.pop_front();
			if (key == -1) {
				fComputer.SetProceed(true);
				fKeyQueue.push_front(-3);   // release
				fKeyDelay = 0.3;
			} else if (key == -3) {
				fComputer.SetProceed(false);
				fKeyDelay = kKeyInterval;
			} else if (key == -2) {
				fKeyDelay = 1.0;
			} else {
				fComputer.PressKey(key);
				fKeyDelay = kKeyInterval;
			}
		}
		fComputer.Run(step);
		fKeyDelay -= step;
		done += step;
	}
	if (flightSeconds > budget)
		fComputer.AdvanceClock(flightSeconds - budget);
}


void
Mission::_RecordFlown(const std::vector<sim::Sample>& more)
{
	for (const sim::Sample& s : more) {
		const sim::Sample& last = fFlown.back();
		// Keep points that are far enough apart to matter on screen.
		if (s.burning || last.burning || s.t - last.t >= 60.0
			|| (s.r - last.r).Length() > 200e3)
			fFlown.push_back(s);
	}
	// Bound memory on very long flights by thinning older points.
	if (fFlown.size() > 40000) {
		std::vector<sim::Sample> thinned;
		thinned.reserve(fFlown.size() / 2 + 1);
		for (size_t i = 0; i < fFlown.size(); i++) {
			if (i % 2 == 0 || fFlown[i].burning || i + 1 == fFlown.size())
				thinned.push_back(fFlown[i]);
		}
		fFlown.swap(thinned);
	}
}


const sim::Prediction&
Mission::Prediction()
{
	if (fPredictionDirty || fPredictionAge > 2.0) {
		fPrediction = sim::Predict(fModel, fState, fPlan, kPredictionHorizon);
		fPredictionDirty = false;
		fPredictionAge = 0;
	}
	return fPrediction;
}


// ---------------------------------------------------------------------------
// Flight plan


bool
Mission::Editable(uint32_t id) const
{
	return !fState.Completed(id) && fState.activeBurn != id;
}


uint32_t
Mission::NewBurnId() const
{
	uint32_t id = 1;
	for (const sim::Burn& b : fPlan)
		id = std::max(id, b.id + 1);
	return id;
}


void
Mission::_SortPlan()
{
	std::stable_sort(fPlan.begin(), fPlan.end(),
		[](const sim::Burn& a, const sim::Burn& b) { return a.time < b.time; });
	fPredictionDirty = true;
}


bool
Mission::AddBurn(sim::Burn burn, std::string* error)
{
	if (burn.time < fState.t + 1.0) {
		if (error)
			*error = "The ignition time has already passed.";
		return false;
	}
	if (burn.engine == sim::ENGINE_SIVB && !fState.sivbAttached) {
		if (error)
			*error = "The S-IVB has been jettisoned.";
		return false;
	}
	burn.id = NewBurnId();
	if (burn.label.empty())
		burn.label = burn.engine == sim::ENGINE_SIVB ? "S-IVB" : "SPS";
	fPlan.push_back(burn);
	_SortPlan();
	return true;
}


bool
Mission::ReplaceBurn(const sim::Burn& burn, std::string* error)
{
	if (!Editable(burn.id)) {
		if (error)
			*error = "That burn has already been executed.";
		return false;
	}
	if (burn.time < fState.t + 1.0) {
		if (error)
			*error = "The ignition time has already passed.";
		return false;
	}
	for (sim::Burn& b : fPlan) {
		if (b.id == burn.id) {
			b = burn;
			_SortPlan();
			return true;
		}
	}
	if (error)
		*error = "No such burn.";
	return false;
}


bool
Mission::RemoveBurn(uint32_t id)
{
	if (!Editable(id))
		return false;
	auto it = std::remove_if(fPlan.begin(), fPlan.end(),
		[id](const sim::Burn& b) { return b.id == id; });
	if (it == fPlan.end())
		return false;
	fPlan.erase(it, fPlan.end());
	fPredictionDirty = true;
	return true;
}


void
Mission::ClearFuturePlan()
{
	auto it = std::remove_if(fPlan.begin(), fPlan.end(),
		[this](const sim::Burn& b) { return Editable(b.id); });
	fPlan.erase(it, fPlan.end());
	fPredictionDirty = true;
}


Mission::PlanRequest
Mission::MakeRequest(PlanKind kind)
{
	PlanRequest request;
	request.kind = kind;
	request.model = fModel;
	request.state = fState;
	request.plan = fPlan;
	request.outbound = false;
	if (kind == PLAN_MCC) {
		// Outbound if the Moon is still ahead (or LOI is planned),
		// otherwise correct the entry.
		const sim::Prediction& p = Prediction();
		request.outbound = p.hasPericynthion && p.pericynthionTime > fState.t
			&& !fModel.NearMoon(fState.r, fState.t);
		for (const sim::Burn& b : fPlan)
			if (!fState.Completed(b.id) && b.label == "LOI")
				request.outbound = true;
	}
	return request;
}


sim::PlanResult
Mission::Solve(const PlanRequest& request)
{
	sim::Planner planner(request.model, request.state, request.plan);
	switch (request.kind) {
		case PLAN_TLI:
			return planner.TransLunarInjection();
		case PLAN_LOI:
			return planner.LunarOrbitInsertion();
		case PLAN_TEI:
			return planner.TransEarthInjection();
		case PLAN_MCC:
		{
			double at = request.state.t + 15 * 60.0;
			sim::PlanResult result = request.outbound
				? planner.PericynthionCorrection(at)
				: planner.EntryCorrection(at);
			if (result.ok && result.burn.DeltaV() < 0.3) {
				char text[96];
				std::snprintf(text, sizeof(text),
					"No correction needed (%.2f m/s).", result.burn.DeltaV());
				result.ok = false;
				result.message = text;
			}
			return result;
		}
	}
	return sim::PlanResult();
}


bool
Mission::Adopt(sim::PlanResult& result)
{
	if (!result.ok)
		return false;
	std::string error;
	if (!AddBurn(result.burn, &error)) {
		result.ok = false;
		result.message = error;
		return false;
	}
	char text[160];
	std::snprintf(text, sizeof(text), "Planned %s at %s, %.1f m/s. %s",
		result.burn.label.c_str(), sim::FormatGET(result.burn.time).c_str(),
		result.burn.DeltaV(), result.message.c_str());
	_Log(text);
	return true;
}


sim::PlanResult
Mission::PlanNow(PlanKind kind)
{
	sim::PlanResult result = Solve(MakeRequest(kind));
	Adopt(result);
	return result;
}


bool
Mission::SavePlan(const std::string& path, std::string* error) const
{
	std::ofstream out(path);
	if (!out) {
		if (error)
			*error = "Cannot write " + path;
		return false;
	}
	out << "# Columbia flight plan\n";
	out << "# burn <engine> <GET hhh:mm:ss> <prograde> <normal> <radial>"
		" <label>\n";
	out << "columbia-plan 1\n";
	for (const sim::Burn& b : fPlan) {
		char line[256];
		std::snprintf(line, sizeof(line), "burn %s %s %.3f %.3f %.3f %s\n",
			b.engine == sim::ENGINE_SIVB ? "SIVB" : "SPS",
			sim::FormatGET(b.time).c_str(), b.prograde, b.normal, b.radial,
			b.label.c_str());
		out << line;
	}
	return bool(out);
}


bool
Mission::LoadPlan(const std::string& path, std::string* error)
{
	std::ifstream in(path);
	if (!in) {
		if (error)
			*error = "Cannot read " + path;
		return false;
	}
	std::vector<sim::Burn> burns;
	std::string line;
	bool header = false;
	int lineNumber = 0;
	while (std::getline(in, line)) {
		lineNumber++;
		if (line.empty() || line[0] == '#')
			continue;
		std::istringstream fields(line);
		std::string word;
		fields >> word;
		if (word == "columbia-plan") {
			header = true;
			continue;
		}
		if (word != "burn") {
			if (error)
				*error = "Unrecognised line " + std::to_string(lineNumber);
			return false;
		}
		std::string engine, get;
		sim::Burn b;
		fields >> engine >> get >> b.prograde >> b.normal >> b.radial;
		std::getline(fields >> std::ws, b.label);
		if (!fields.eof() && fields.fail()) {
			if (error)
				*error = "Malformed burn on line " + std::to_string(lineNumber);
			return false;
		}
		b.engine = engine == "SIVB" ? sim::ENGINE_SIVB : sim::ENGINE_SPS;
		if (!sim::ParseGET(get, &b.time)) {
			if (error)
				*error = "Bad time on line " + std::to_string(lineNumber);
			return false;
		}
		burns.push_back(b);
	}
	if (!header) {
		if (error)
			*error = "Not a Columbia flight plan.";
		return false;
	}
	ClearFuturePlan();
	int skipped = 0;
	for (const sim::Burn& b : burns) {
		if (!AddBurn(b))
			skipped++;
	}
	if (skipped > 0)
		_Log(std::to_string(skipped) + " burn(s) in the plan were in the past"
			" or impossible and were skipped.");
	return true;
}


// ---------------------------------------------------------------------------
// Computer


const agc::DSKYState&
Mission::Display()
{
	fDSKY.Sync(fComputer);
	return fDSKY.State();
}


void
Mission::PressKey(int keycode)
{
	fKeyQueue.push_back(keycode);
}


void
Mission::QueueKeys(const std::string& keys)
{
	for (char k : keys) {
		int code = -1;
		switch (k) {
			case 'V': code = agc::KEY_VERB; break;
			case 'N': code = agc::KEY_NOUN; break;
			case 'E': code = agc::KEY_ENTR; break;
			case 'C': code = agc::KEY_CLR; break;
			case 'R': code = agc::KEY_RSET; break;
			case 'K': code = agc::KEY_KEYREL; break;
			case '+': code = agc::KEY_PLUS; break;
			case '-': code = agc::KEY_MINUS; break;
			case 'P': fKeyQueue.push_back(-1); continue;
			default:
				if (k >= '0' && k <= '9')
					code = k == '0' ? agc::KEY_0 : (k - '0');
		}
		if (code >= 0)
			fKeyQueue.push_back(code);
	}
}


// ---------------------------------------------------------------------------
// Events


void
Mission::_Log(const std::string& text, double t)
{
	fEvents.push_back(sim::FormatGET(t < 0 ? fState.t : t) + "  " + text);
}


std::vector<std::string>
Mission::TakeEvents()
{
	std::vector<std::string> events;
	events.swap(fEvents);
	return events;
}

}  // namespace columbia
