// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "ReplayPacer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ui {

using namespace flight;

namespace {

const double kCell = 5;          // seconds of mission time per pacing cell
const double kRamp = 2.5;        // speed may change this much per real second

// flight::Phase values (Recording stores them as numbers).
enum {
	PH_PRELAUNCH, PH_BOOST, PH_PARKING, PH_TLI, PH_TLC, PH_LUNAR_ORBIT,
	PH_DESCENT, PH_SURFACE, PH_ASCENT, PH_RENDEZVOUS, PH_TEC, PH_ENTRY,
	PH_RECOVERY, PH_LOST
};

// LogEntry::Kind values.
enum { LOG_INFO, LOG_CAPCOM, LOG_CREW, LOG_CAUTION, LOG_WARNING, LOG_EVENT };

double
LimitFor(const RecSample& s)
{
	double limit = std::numeric_limits<double>::infinity();
	bool engine = false;
	for (const RecVehicle& v : s.veh)
		if (v.Present() && (v.flags & REC_ENGINE))
			engine = true;
	if (engine)
		limit = std::min(limit, 8.0);
	switch (s.phase) {
		case PH_BOOST:
			limit = std::min(limit, 5.0);
			break;
		case PH_DESCENT:
			limit = std::min(limit, s.altitude < 3000 ? 2.0
				: s.altitude < 15000 ? 5.0 : 10.0);
			break;
		case PH_ASCENT:
			limit = std::min(limit, 6.0);
			break;
		case PH_ENTRY:
			limit = std::min(limit, s.altitude < 12000 ? 3.0 : 8.0);
			break;
		case PH_RECOVERY:
		case PH_LOST:
			limit = std::min(limit, 3.0);
			break;
		default:
			break;
	}
	// Close to another vehicle: docking and the end of a rendezvous.
	const RecVehicle& csm = s.veh[REC_CSM];
	for (int k : {REC_LM, REC_BOOSTER}) {
		const RecVehicle& o = s.veh[k];
		if (!csm.Present() || !o.Present())
			continue;
		double range = (o.r - csm.r).Length();
		if (range < 150)
			limit = std::min(limit, 3.0);
		else if (range < 2000)
			limit = std::min(limit, 15.0);
	}
	return limit;
}

}  // namespace


void
ReplayPacer::Build(const Recording& rec, double minutes)
{
	fGET.clear();
	fReal.clear();
	fSpeed.clear();
	if (rec.samples.size() < 2)
		return;
	double start = rec.StartGET(), end = rec.EndGET();
	size_t n = size_t(std::ceil((end - start) / kCell));
	n = std::max<size_t>(n, 1);
	std::vector<double> limit(n, std::numeric_limits<double>::infinity());
	// Moments shown at their own speed whatever the running time: the
	// countdown and liftoff, the last 200 m of a landing, the last 30 m of
	// a docking, the parachutes.
	std::vector<double> fixed(n, std::numeric_limits<double>::infinity());
	auto cellOf = [&](double get) {
		double c = (get - start) / kCell;
		return size_t(std::clamp(c, 0.0, double(n - 1)));
	};
	auto limitRange = [&](double from, double to, double speed) {
		size_t a = cellOf(from), b = cellOf(to);
		for (size_t i = a; i <= b; i++)
			limit[i] = std::min(limit[i], speed);
	};
	// From the samples.
	for (size_t i = 0; i + 1 < rec.samples.size(); i++) {
		const RecSample& s = rec.samples[i];
		double l = LimitFor(s);
		if (std::isfinite(l))
			limitRange(s.get, rec.samples[i + 1].get, l);
	}
	auto fixRange = [&](double from, double to, double speed) {
		size_t a = cellOf(from), b = cellOf(to);
		for (size_t i = a; i <= b; i++)
			fixed[i] = std::min(fixed[i], speed);
	};
	for (size_t i = 0; i + 1 < rec.samples.size(); i++) {
		const RecSample& s = rec.samples[i];
		double next = rec.samples[i + 1].get;
		if (s.phase == PH_DESCENT && s.altitude < 200)
			fixRange(s.get, next, 1.0);
		if (s.phase == PH_ENTRY && s.altitude < 7000)
			fixRange(s.get, next, 2.0);
		const RecVehicle& csm = s.veh[REC_CSM];
		for (int k : {REC_LM, REC_BOOSTER}) {
			const RecVehicle& o = s.veh[k];
			if (csm.Present() && o.Present() && (o.r - csm.r).Length() < 30)
				fixRange(s.get, next, 1.5);
		}
	}
	// The countdown's last seconds and liftoff in real time.
	fixRange(-20, 20, 1.0);
	// Time to read what's said and see what happens.
	for (const RecEvent& e : rec.events) {
		switch (e.kind) {
			case LOG_CAPCOM:
			case LOG_CREW:
			case LOG_EVENT:
				limitRange(e.get - 8, e.get + 24, 4.0);
				break;
			case LOG_CAUTION:
			case LOG_WARNING:
				limitRange(e.get - 6, e.get + 20, 2.0);
				break;
			default:
				break;
		}
	}
	// Everything else at the coast speed that fits the running time; if
	// even an instant coast is too long, the watched moments speed up too
	// (all but the fixed ones).
	double want = std::max(60.0, minutes * 60);
	auto totalAt = [&](double k, double coast) {
		double t = 0;
		for (size_t i = 0; i < n; i++)
			t += kCell / std::min({limit[i] * k, coast, fixed[i]});
		return t;
	};
	double k = 1;
	if (totalAt(1, 1e7) > want) {
		double lo = 1, hi = 1000;
		for (int it = 0; it < 60; it++) {
			double mid = std::sqrt(lo * hi);
			if (totalAt(mid, 1e7) > want)
				lo = mid;
			else
				hi = mid;
		}
		k = hi;
	}
	double lo = 1, hi = 1e7;
	for (int it = 0; it < 80; it++) {
		double mid = std::sqrt(lo * hi);
		if (totalAt(k, mid) > want)
			lo = mid;
		else
			hi = mid;
	}
	double coast = hi;
	std::vector<double> speed(n);
	for (size_t i = 0; i < n; i++)
		speed[i] = std::min({limit[i] * k, coast, fixed[i]});
	// Smooth: the speed changes by at most kRamp per real second, slowing
	// down ahead of an event and speeding up after it.
	for (int pass = 0; pass < 2; pass++) {
		for (size_t k = 1; k < n; k++) {
			size_t i = pass == 0 ? k : n - 1 - k;
			size_t j = pass == 0 ? k - 1 : n - k;
			double realPrev = kCell / speed[j];
			speed[i] = std::min(speed[i], speed[j] * std::pow(kRamp, realPrev));
		}
	}
	fGET.resize(n + 1);
	fReal.resize(n + 1);
	fSpeed = speed;
	fGET[0] = start;
	fReal[0] = 0;
	for (size_t i = 0; i < n; i++) {
		fGET[i + 1] = std::min(end, start + kCell * double(i + 1));
		fReal[i + 1] = fReal[i] + (fGET[i + 1] - fGET[i]) / speed[i];
	}
}


double
ReplayPacer::GETAt(double seconds) const
{
	if (fReal.empty())
		return 0;
	if (seconds <= 0)
		return fGET.front();
	if (seconds >= fReal.back())
		return fGET.back();
	size_t i = size_t(std::upper_bound(fReal.begin(), fReal.end(), seconds)
		- fReal.begin()) - 1;
	return fGET[i] + (seconds - fReal[i]) * fSpeed[i];
}


double
ReplayPacer::SpeedAt(double seconds) const
{
	if (fSpeed.empty())
		return 1;
	if (seconds <= 0)
		return fSpeed.front();
	if (seconds >= fReal.back())
		return fSpeed.back();
	size_t i = size_t(std::upper_bound(fReal.begin(), fReal.end(), seconds)
		- fReal.begin()) - 1;
	return fSpeed[std::min(i, fSpeed.size() - 1)];
}


double
ReplayPacer::SecondsAt(double get) const
{
	if (fGET.empty())
		return 0;
	if (get <= fGET.front())
		return 0;
	if (get >= fGET.back())
		return fReal.back();
	size_t i = size_t(std::upper_bound(fGET.begin(), fGET.end(), get)
		- fGET.begin()) - 1;
	return fReal[i] + (get - fGET[i]) / fSpeed[i];
}

}  // namespace ui
