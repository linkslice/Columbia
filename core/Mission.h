// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Ties the flight dynamics, the flight plan and the guidance computer
// together. UI-agnostic: the Haiku front end drives it from a timer.

#ifndef COLUMBIA_MISSION_H
#define COLUMBIA_MISSION_H

#include "agc/AGC.h"
#include "agc/DSKY.h"
#include "sim/Flight.h"
#include "sim/Planner.h"

#include <deque>
#include <string>
#include <vector>

namespace columbia {

class Mission {
public:
	Mission();

	// New flight: parking orbit at GET 0, guidance computer powered up and
	// given a fresh start (V36E, then V37E00E into P00).
	void Reset();

	// Advances by `realSeconds` of wall time scaled by the warp factor.
	void Update(double realSeconds);

	// --- Time ----------------------------------------------------------------
	double Time() const { return fState.t; }
	double Warp() const { return fWarp; }
	void SetWarp(double warp);
	bool Paused() const { return fPaused; }
	void SetPaused(bool paused) { fPaused = paused; }
	// Drop to a slow warp shortly before each burn.
	bool AutoSlow() const { return fAutoSlow; }
	void SetAutoSlow(bool on) { fAutoSlow = on; }
	static const std::vector<double>& WarpLevels();

	// --- Flight ----------------------------------------------------------------
	const sim::Model& Model() const { return fModel; }
	const sim::VehicleState& State() const { return fState; }
	const std::vector<sim::Sample>& FlownPath() const { return fFlown; }
	const sim::Prediction& Prediction();
	bool Finished() const;

	// --- Flight plan --------------------------------------------------------------
	const sim::FlightPlan& Plan() const { return fPlan; }
	// Burns that are complete or in progress cannot be changed.
	bool Editable(uint32_t id) const;
	uint32_t NewBurnId() const;
	bool AddBurn(sim::Burn burn, std::string* error = nullptr);
	bool ReplaceBurn(const sim::Burn& burn, std::string* error = nullptr);
	bool RemoveBurn(uint32_t id);
	void ClearFuturePlan();

	// Planning runs from a snapshot so a UI can do it on a worker thread:
	// MakeRequest() (cheap) -> Solve() (pure, slow) -> Adopt().
	enum PlanKind { PLAN_TLI, PLAN_LOI, PLAN_TEI, PLAN_MCC };
	struct PlanRequest {
		PlanKind kind;
		sim::Model model;
		sim::VehicleState state;
		sim::FlightPlan plan;
		bool outbound;   // for PLAN_MCC
	};
	PlanRequest MakeRequest(PlanKind kind);
	static sim::PlanResult Solve(const PlanRequest& request);
	bool Adopt(sim::PlanResult& result);
	sim::PlanResult PlanNow(PlanKind kind);   // all three, synchronously

	bool SavePlan(const std::string& path, std::string* error) const;
	bool LoadPlan(const std::string& path, std::string* error);

	// --- Guidance computer --------------------------------------------------------
	agc::Computer& Computer() { return fComputer; }
	const agc::DSKYState& Display();
	void PressKey(int keycode);       // queued, so keys are never lost
	void SetProceed(bool pressed) { fComputer.SetProceed(pressed); }
	void QueueKeys(const std::string& keys);   // "V37E00E"

	// --- Event log ----------------------------------------------------------------
	std::vector<std::string> TakeEvents();

private:
	void _Log(const std::string& text, double t = -1);
	void _SortPlan();
	void _RecordFlown(const std::vector<sim::Sample>& more);
	void _RunComputer(double flightSeconds, double realSeconds);

	sim::Model fModel;
	sim::VehicleState fState;
	sim::FlightPlan fPlan;
	std::vector<sim::Sample> fFlown;

	sim::Prediction fPrediction;
	bool fPredictionDirty;
	double fPredictionAge;

	double fWarp;
	double fRequestedWarp;
	bool fPaused;
	bool fAutoSlow;
	uint32_t fLastActiveBurn;
	uint32_t fSlowedFor;

	agc::Computer fComputer;
	agc::DSKY fDSKY;
	std::deque<int> fKeyQueue;      // keycodes; -1 = PRO
	double fKeyDelay;

	std::vector<std::string> fEvents;
};

}  // namespace columbia

#endif
