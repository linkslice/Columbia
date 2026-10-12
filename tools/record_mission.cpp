// Columbia — fly a mission with the automatic crew and Mission Control and
// write a flight recording (for the screensaver and replays).
//
// Usage: record_mission <rope-dir> <mission> <out.colrec>
//            [--start N] [--difficulty nominal|asflown|challenging|hard|nightmare]
//            [--seed N] [--quiet]

#include "../core/flight/AutoCrew.h"
#include "../core/flight/Contingency.h"
#include "../core/flight/FlightSim.h"
#include "../core/flight/MissionDirector.h"
#include "../core/flight/MissionSpec.h"
#include "../core/flight/Recorder.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>

using namespace flight;
using namespace sim;

namespace {

std::string
Stamp(double get)
{
	char b[32];
	long t = long(std::fabs(get));
	std::snprintf(b, sizeof(b), "%s%03ld:%02ld:%02ld", get < 0 ? "-" : "", t / 3600,
		t / 60 % 60, t % 60);
	return b;
}

bool
ParseDifficulty(const char* s, Difficulty* d)
{
	struct { const char* n; Difficulty d; } table[] = {
		{"nominal", Difficulty::Nominal}, {"asflown", Difficulty::AsFlown},
		{"challenging", Difficulty::Challenging}, {"hard", Difficulty::Hard},
		{"nightmare", Difficulty::Nightmare}};
	for (auto& t : table)
		if (std::strcmp(s, t.n) == 0) {
			*d = t.d;
			return true;
		}
	return false;
}

}  // namespace


int
main(int argc, char** argv)
{
	if (argc < 4) {
		std::fprintf(stderr, "usage: %s <rope-dir> <mission 7-17> <out.colrec>"
			" [--start N] [--difficulty D] [--seed N] [--quiet]\n", argv[0]);
		return 2;
	}
	std::string ropes = argv[1];
	int number = std::atoi(argv[2]);
	std::string out = argv[3];
	int start = 0;
	Difficulty difficulty = Difficulty::Nominal;
	uint32_t seed = uint32_t(std::time(nullptr));
	bool quiet = false;
	for (int i = 4; i < argc; i++) {
		if (std::strcmp(argv[i], "--start") == 0 && i + 1 < argc)
			start = std::atoi(argv[++i]);
		else if (std::strcmp(argv[i], "--difficulty") == 0 && i + 1 < argc) {
			if (!ParseDifficulty(argv[++i], &difficulty)) {
				std::fprintf(stderr, "unknown difficulty %s\n", argv[i]);
				return 2;
			}
		} else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc)
			seed = uint32_t(std::strtoul(argv[++i], nullptr, 10));
		else if (std::strcmp(argv[i], "--quiet") == 0)
			quiet = true;
	}
	const MissionSpec* m = FindMission(number);
	if (!m) {
		std::fprintf(stderr, "no Apollo %d\n", number);
		return 2;
	}
	// Experiments: another LGC program, or the Apollo 11-14 LM loading.
	MissionSpec spec = *m;
	if (const char* lgc = std::getenv("COLUMBIA_LGC"))
		spec.lgcProgram = lgc;
	if (std::getenv("COLUMBIA_LIGHT_LM")) {
		MissionSpec base;
		spec.lmDescentPropellant = base.lmDescentPropellant;
		spec.lmDescentDry = base.lmDescentDry;
	}
	if (const char* d = std::getenv("COLUMBIA_DATE"))   // YYYY-MM-DD
		std::sscanf(d, "%d-%d-%d", &spec.year, &spec.month, &spec.day);
	FlightSim f(spec, ropes);
	std::string err;
	if (!f.Start(StartPoint(start), &err)) {
		std::fprintf(stderr, "start: %s\n", err.c_str());
		return 1;
	}
	if (!quiet)
		f.onLog = [&](const LogEntry& e) {
			if (e.kind != LogEntry::Info)
				std::printf("%s %s%s\n", Stamp(e.get).c_str(),
					e.kind == LogEntry::Crew ? "  crew: " : "", e.text.c_str());
			std::fflush(stdout);
		};
	FlightRecorder recorder(f);
	recorder.Data().header.difficulty = DifficultyName(difficulty);
	recorder.Attach();
	AutoCrew crew(f);
	crew.Install();
	ContingencyManager contingencies(f, difficulty, seed);
	contingencies.Plan();
	f.secondHooks.push_back([&](FlightSim&, double) { contingencies.Tick(); });
	MissionDirector director(f, crew);
	director.Install();

	double end = m->plan.splashdown > 0 ? m->plan.splashdown + 6 * 3600
		: f.GET() + 12 * 86400;
	f.SetWarp(100000);
	while (!f.Lost() && f.CurrentPhase() != Phase::Recovery && f.GET() < end)
		f.Advance(0.05);
	// A minute on the water (or of silence) to end the replay on.
	if (!f.Lost())
		f.Run(60);

	for (const auto& s : contingencies.Schedule())
		if (s.fired)
			recorder.Data().header.contingencies.push_back(s.spec->title);
	std::string outcome;
	bool ok = false;
	if (f.Lost())
		outcome = f.LossReason();
	else if (f.CurrentPhase() == Phase::Recovery) {
		double lat, lon, r;
		Vec3 fixed = world::Ephemeris::EarthFixedToInertial(f.JulianDate()).Transposed()
			* f.CSMVehicle().position;
		world::VectorToLatLon(fixed, &lat, &lon, &r);
		Vec3 tgt = world::LatLonToVector(director.SplashLatitude(),
			director.SplashLongitude(), 1.0);
		double miss = std::acos(Clamp(fixed.Unit().Dot(tgt), -1, 1)) * world::kREarth;
		char b[160];
		std::snprintf(b, sizeof(b), "Splashdown at %.1f%c %.1f%c, %.1f nmi from the"
			" recovery ship", std::fabs(lat), lat >= 0 ? 'N' : 'S', std::fabs(lon),
			lon >= 0 ? 'E' : 'W', miss / 1852);
		outcome = b;
		ok = true;
		recorder.Data().header.splashLatitude = director.SplashLatitude();
		recorder.Data().header.splashLongitude = director.SplashLongitude();
	} else
		outcome = std::string("Ended in ") + PhaseName(f.CurrentPhase());
	recorder.Finish(outcome, ok);
	if (!recorder.Data().Save(out)) {
		std::fprintf(stderr, "can't write %s\n", out.c_str());
		return 1;
	}
	const Recording& rec = recorder.Data();
	std::printf("%s: %s\nwrote %s: %zu samples, %zu CMC / %zu LGC displays, %zu"
		" events, GET %s to %s\n", m->name.c_str(), outcome.c_str(), out.c_str(),
		rec.samples.size(), rec.cmc.size(), rec.lgc.size(), rec.events.size(),
		Stamp(rec.StartGET()).c_str(), Stamp(rec.EndGET()).c_str());
	return ok ? 0 : 3;
}
