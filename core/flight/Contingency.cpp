// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).

#include "Contingency.h"

#include "FlightSim.h"
#include "Saturn.h"
#include "../gnc/PadLoads.h"

#include <algorithm>
#include <cmath>

namespace flight {

using namespace sim;

namespace {

const double kH = 3600.0;

bool SMOn(FlightSim& f) { return f.SMAttached(); }

void
Capcom(FlightSim& f, const std::string& text)
{
	f.Log(LogEntry::Capcom, text);
}

std::mt19937&
Noise()
{
	static std::mt19937 rng(1969);
	return rng;
}

double
Uniform()
{
	return std::uniform_real_distribution<double>(0, 1)(Noise());
}

std::vector<ContingencySpec>
Build()
{
	std::vector<ContingencySpec> v;

	// ---------------------------------------------------------------- Apollo 7
	{
		ContingencySpec c;
		c.id = "a7-colds";
		c.title = "Head colds";
		c.mission = 7;
		c.severity = 1;
		c.event = Event::Insertion;
		c.earliest = 14 * kH;
		c.latest = 20 * kH;
		c.history = "Schirra caught a cold on the first day and passed it to "
			"Eisele and Cunningham. Congested and irritable, the crew argued "
			"with the ground all flight and refused to wear helmets for "
			"entry so they could clear their ears.";
		c.cue = "The commander reports a head cold; the others follow.";
		c.remedy = "Decongestants, rest periods kept, and helmets off for "
			"entry to equalise ear pressure.";
		c.inject = [](FlightSim& f) {
			for (CrewMember& m : f.crew.members) {
				m.sick = true;
				m.condition = "head cold";
				m.morale -= 15;
				m.fatigue += 10;
			}
			f.Log(LogEntry::Crew, "CDR: I've got a cold. A bad one.");
		};
		c.resolved = [](FlightSim& f) {
			return f.CurrentPhase() == Phase::Recovery;
		};
		v.push_back(c);
	}
	// ---------------------------------------------------------------- Apollo 8
	{
		ContingencySpec c;
		c.id = "a8-p01";
		c.title = "P01 selected in flight";
		c.mission = 8;
		c.severity = 2;
		c.event = Event::TEI;
		c.earliest = 18 * kH;
		c.latest = 22 * kH;
		c.history = "On the way home, Lovell keyed V37E01E by mistake while "
			"doing star sightings. The prelaunch program wiped the "
			"platform alignment and part of the CMC's state; he realigned "
			"by hand on Rigel and Sirius.";
		c.cue = "The DSKY shows P01 and the attitude ball swings: the "
			"platform no longer knows where it is.";
		c.remedy = "Uplink a new state vector and REFSMMAT, then realign "
			"with P51 (star sightings) and P52.";
		c.inject = [](FlightSim& f) {
			for (char k : std::string("V37E01E")) {
				int code = k == 'V' ? agc::KEY_VERB : k == 'E' ? agc::KEY_ENTR
					: k == '0' ? agc::KEY_0 : k - '0';
				f.PressCMC(code);
				f.Run(0.3);
			}
			f.Log(LogEntry::Crew, "CMP: Houston, I've got a P01 on the DSKY. "
				"I think I did it.");
		};
		c.resolved = [](FlightSim& f) {
			const agc::DSKYState& d = f.CMCDSKY().State();
			return !(d.prog[0] == '0' && d.prog[1] == '1')
				&& !(d.prog[0] == '0' && d.prog[1] == '2');
		};
		v.push_back(c);
	}
	// ---------------------------------------------------------------- Apollo 9
	{
		ContingencySpec c;
		c.id = "a9-sickness";
		c.title = "Space adaptation sickness";
		c.mission = 9;
		c.severity = 2;
		c.event = Event::Insertion;
		c.earliest = 40 * kH;
		c.latest = 46 * kH;
		c.history = "Rusty Schweickart vomited twice on the third day. The "
			"planned EVA from the LM to the CM was cut to a stand-up on the "
			"LM porch.";
		c.cue = "The LMP is nauseated and can't keep food down.";
		c.remedy = "Rest, slow movements, and a shortened EVA.";
		c.inject = [](FlightSim& f) {
			CrewMember& m = f.crew.members[2];
			m.sick = true;
			m.condition = "nausea";
			m.health -= 20;
			m.hydration -= 15;
			f.Log(LogEntry::Crew, "LMP: I'm not feeling too good.");
		};
		c.resolved = [](FlightSim& f) {
			return f.crew.members[2].health > 85;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 10
	{
		ContingencySpec c;
		c.id = "a10-gyration";
		c.title = "Staging gyration";
		c.mission = 10;
		c.severity = 3;
		c.needsLM = true;
		c.event = Event::Undock;
		c.earliest = 5 * kH;
		c.latest = 5 * kH;
		c.history = "With the AGS mode switch in AUTO instead of ATT HOLD, "
			"Snoopy's ascent stage tried to point its radar at the CSM the "
			"moment it staged and gyrated wildly, nearly gimbal locking.";
		c.cue = "At staging the ascent stage starts tumbling.";
		c.remedy = "Take manual control and null the rates before the "
			"gimbals lock; switch the AGS to ATTITUDE HOLD.";
		c.inject = [](FlightSim& f) { f.faults.stagingGyration = true; };
		c.resolved = [](FlightSim& f) {
			return !f.faults.stagingGyration && f.LMVehicle()
				&& f.LMVehicle()->rates.Length() < 2 * kDeg;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 11
	{
		ContingencySpec c;
		c.id = "a11-1202";
		c.title = "Program alarms 1201 and 1202";
		c.mission = 11;
		c.severity = 3;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::PDI;
		c.earliest = -300;
		c.latest = -300;
		c.history = "The rendezvous radar switch was in SLEW, so the RR's "
			"CDUs hunted against an unsynchronised 800 Hz reference and "
			"stole about 15% of the LGC's time. During the descent the "
			"executive ran out of core sets and VAC areas: 1202, 1201, "
			"restart, again and again. The software shed its low-priority "
			"jobs each time and kept flying. Steve Bales: \"We're GO on that "
			"alarm.\"";
		c.cue = "PROGRAM ALARM lights and the DSKY blanks for a moment; "
			"V05N09 shows 1202 or 1201.";
		c.remedy = "GO, as long as they don't come continuously. Keep the "
			"DSKY quiet (no extra monitor verbs). After the landing: RR mode "
			"switch to LGC.";
		c.inject = [](FlightSim& f) {
			gnc::LMPGNCS* p = f.LMPGNCS();
			if (!p)
				return;
			p->rrCDUHunting = true;
			// The DSKY monitor and display jobs the crew ran (N68) added load
			// we don't model directly; calibrated so the first alarm comes
			// about five minutes into the burn, as it did.
			p->huntScale = 1.5;
		};
		c.resolved = [](FlightSim& f) {
			gnc::LMPGNCS* p = f.LMPGNCS();
			return !p || !p->rrCDUHunting || f.LMLanded();
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "a11-long-landing";
		c.title = "Boulder field at the target";
		c.mission = 11;
		c.severity = 3;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::PDI;
		c.earliest = 420;
		c.latest = 420;
		c.history = "The automatic targeting was taking Eagle into West "
			"crater, a football-field-sized crater full of boulders. "
			"Armstrong took over in P66, flew long over it and set down "
			"with about 25 seconds of fuel to the bingo call.";
		c.cue = "Out the window, the landing point designator sits on a "
			"crater full of car-sized boulders.";
		c.remedy = "Redesignate with the hand controller in P64, or take "
			"P66 and fly over the field to a clear spot. Watch the fuel.";
		c.inject = [](FlightSim& f) {
			// Put the crater where the LGC is taking the LM: the true
			// position plus the guidance's remaining range to LAND.
			Vehicle* lm = f.LMVehicle();
			gnc::PadLoader* pad = f.LGCPad();
			gnc::LMPGNCS* p = f.LMPGNCS();
			if (!lm || !pad || !p)
				return;
			Vec3 land, r;
			bool ok = true;
			for (int i = 0; i < 3; i++) {
				double a = 0, b = 0;
				ok = pad->ReadDouble("LAND", 24, &a, 2 * i) && ok;
				ok = pad->ReadDouble("R", 24, &b, 2 * i) && ok;
				(&land.x)[i] = a;
				(&r.x)[i] = b;
			}
			Vec3 target;
			Mat3 smToRef = p->imu.REFSMMAT().Transposed();
			if (ok && (land - r).Length() < 50000)
				target = lm->position + smToRef * (land - r);
			else
				target = lm->position + lm->velocity.Unit() * 8000;
			double lat, lon, rr;
			f.Moon().LatLon(target, &lat, &lon, &rr);
			world::Terrain::Feature west{lat, lon, 190, 0.18, true,
				"West crater"};
			f.EditTerrain().AddFeature(west);
		};
		c.resolved = [](FlightSim& f) { return f.LMLanded(); };
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "a11-breaker";
		c.title = "Broken engine-arm breaker";
		c.mission = 11;
		c.severity = 3;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::Landing;
		c.earliest = 7 * kH;
		c.latest = 7 * kH;
		c.history = "Back in the cabin after the EVA, Aldrin found the "
			"ascent engine arm circuit breaker on the floor, its plunger "
			"snapped off by a backpack. He pushed the breaker in with a "
			"felt-tip pen.";
		c.cue = "A circuit breaker plunger lies on the floor; the ascent "
			"engine won't arm.";
		c.remedy = "Push the breaker in with a felt-tip pen.";
		c.inject = [](FlightSim& f) {
			if (f.LMPGNCS())
				f.LMPGNCS()->ascentArmBreakerOpen = true;
		};
		c.resolved = [](FlightSim& f) {
			return !f.LMPGNCS() || !f.LMPGNCS()->ascentArmBreakerOpen;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 12
	{
		ContingencySpec c;
		c.id = "a12-lightning";
		c.title = "Lightning strikes at liftoff";
		c.mission = 12;
		c.severity = 4;
		c.event = Event::Liftoff;
		c.earliest = 36.5;
		c.latest = 36.5;
		c.history = "Yankee Clipper was struck by lightning at 36.5 s and "
			"again at 52 s. The fuel cells dropped off line, almost every "
			"warning light came on, telemetry was garbage and the platform "
			"tumbled. John Aaron called for \"SCE to AUX\"; Bean found the "
			"switch, the data came back, and the Saturn's own guidance had "
			"flown on untouched.";
		c.cue = "Master alarm, every caution light, the main bus "
			"undervolts, the FDAI ball spins.";
		c.remedy = "SCE to AUX. Reset the fuel cells to the buses. Realign "
			"the platform in orbit (P51, P52).";
		c.inject = [](FlightSim& f) {
			for (FuelCell& fc : f.csmSystems.fuelCell)
				fc.online = false;
			f.faults.sceFailed = true;
			f.csmSystems.sceAux = false;
			gnc::IMU& imu = f.CMPGNCS().imu;
			imu.tumbled = true;
			imu.Torque(0, 37 * kDeg);
			imu.Torque(1, -71 * kDeg);
			imu.Torque(2, 112 * kDeg);
			f.Log(LogEntry::Warning, "MASTER ALARM: main bus A and B "
				"undervolt, AC bus overload, fuel cells disconnected");
			f.Log(LogEntry::Crew, "CDR: I don't know what happened here. We "
				"had everything in the world drop out.");
		};
		c.resolved = [](FlightSim& f) {
			return !f.faults.sceFailed && f.csmSystems.FuelCellsOnline() >= 3;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 13
	{
		ContingencySpec c;
		c.id = "a13-pogo";
		c.title = "S-II center engine shutdown";
		c.mission = 13;
		c.severity = 2;
		c.event = Event::Liftoff;
		c.earliest = 330.6;
		c.latest = 330.6;
		c.history = "Severe pogo oscillation (68 g at the engine mount) "
			"tripped the S-II center engine's low-thrust cutoff two minutes "
			"early. The IU burned the four outboard J-2s and the S-IVB "
			"longer and still reached orbit.";
		c.cue = "The center engine light comes on early in the S-II burn.";
		c.remedy = "None needed: the IU's iterative guidance absorbs it. "
			"Watch the S-IVB's remaining propellant for TLI.";
		c.applicable = [](FlightSim& f) {
			return f.IU() && f.IU()->CurrentMode() == SaturnIU::Mode::SecondStage;
		};
		c.inject = [](FlightSim& f) {
			f.IU()->FailEngine("S-II J-2 5");
			f.Log(LogEntry::Crew, "CDR: Inboard out.");
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "a13-o2-tank";
		c.title = "Oxygen tank 2 explodes";
		c.mission = 13;
		c.severity = 5;
		c.event = Event::Liftoff;
		c.earliest = 55 * kH + 54 * 60 + 53;
		c.latest = 55 * kH + 54 * 60 + 53;
		c.history = "Stirring the cryo tanks shorted damaged wiring inside "
			"O2 tank 2. The tank failed, blew out a panel of the service "
			"module and took tank 1 with it. Two fuel cells died within "
			"minutes. The crew lived in Aquarius for four days, built an "
			"adapter for the CM's square lithium hydroxide canisters, and "
			"burned the LM's descent engine to get home.";
		c.cue = "A bang, a shudder, MAIN BUS B UNDERVOLT. O2 tank 2 reads "
			"zero; tank 1 is falling. \"Houston, we've had a problem.\"";
		c.remedy = "Power the LM up, transfer the platform alignment, power "
			"the CM down to save its batteries for entry. Free-return "
			"with a DPS burn (PC+2). Build the square-in-round LiOH adapter "
			"when the LM's CO2 climbs. Don't use the SPS.";
		c.applicable = [](FlightSim& f) { return f.SMAttached(); };
		c.inject = [](FlightSim& f) {
			CSMSystems& s = f.csmSystems;
			s.o2[1].failed = true;
			s.o2[1].quantity = 0;
			s.o2[1].pressure = 0;
			s.o2[0].leakRate = s.o2[0].quantity / 2.2;   // gone in ~2 h
			s.fuelCell[0].failed = true;
			s.fuelCell[0].online = false;
			s.fuelCell[2].failed = true;
			s.fuelCell[2].online = false;
			f.faults.smDamaged = true;
			f.CSMVehicle().rates += Vec3(0.01, -0.02, 0.015);
			f.Log(LogEntry::Warning, "MASTER ALARM: MAIN BUS B UNDERVOLT");
			f.Log(LogEntry::Crew, "CMP: Houston, we've had a problem here.");
			Capcom(f, "This is Houston. Say again, please.");
		};
		c.resolved = [](FlightSim& f) {
			return f.CurrentPhase() == Phase::Recovery;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 14
	{
		ContingencySpec c;
		c.id = "a14-docking-probe";
		c.title = "Docking probe won't capture";
		c.mission = 14;
		c.severity = 3;
		c.needsLM = true;
		c.event = Event::TLI;
		c.earliest = 0;
		c.latest = 0;
		c.history = "Five times Roosa brought Kitty Hawk's probe into "
			"Antares' drogue and five times the capture latches didn't "
			"engage. On the sixth, with the probe retracted and the "
			"spacecraft thrust together, the docking latches fired.";
		c.cue = "Contact, but the capture latches don't hold.";
		c.remedy = "Keep trying with a firmer closing rate; if it still won't "
			"capture, hard dock with the probe retracted. No LM, no landing.";
		c.inject = [](FlightSim& f) { f.faults.probeMisses = 5; };
		c.resolved = [](FlightSim& f) { return f.LMDocked(); };
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "a14-abort-switch";
		c.title = "Faulty ABORT switch";
		c.mission = 14;
		c.severity = 4;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::PDI;
		c.earliest = -2.5 * kH;
		c.latest = -2.5 * kH;
		c.history = "A loose solder ball in Antares' ABORT switch set the "
			"abort discrete at random. Once powered descent began, the LGC "
			"would have aborted the landing. Don Eyles at MIT wrote a "
			"workaround in two hours: keystrokes to make the LGC ignore the "
			"switch, keyed in by Mitchell just after ignition.";
		c.cue = "The ABORT light flickers in the LGC's channel 30 display; "
			"tapping the panel clears it.";
		c.remedy = "Key the uplinked procedure right after DPS ignition so "
			"the LGC ignores the abort discrete (LETABORT off), then throttle "
			"up manually.";
		c.inject = [](FlightSim& f) {
			f.Log(LogEntry::Caution, "LGC sees the ABORT discrete with the "
				"switch out");
		};
		c.resolved = [](FlightSim& f) {
			gnc::LMPGNCS* p = f.LMPGNCS();
			if (!p || f.LMLanded())
				return true;
			// The solder ball floats: the discrete comes and goes.
			p->panel.abort = Uniform() < 0.08;
			return false;
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "a14-lr";
		c.title = "Landing radar won't lock on";
		c.mission = 14;
		c.severity = 4;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::PDI;
		c.earliest = 0;
		c.latest = 0;
		c.history = "A scale-switching fault kept Antares' landing radar "
			"in its high range scale. Without radar data the mission rules "
			"called for an abort at 10,000 feet. Cycling the LR circuit "
			"breaker cleared it at about 22,000 feet.";
		c.cue = "No ALT or VEL lights going out: the radar never reports "
			"data good.";
		c.remedy = "Cycle the LANDING RADAR circuit breaker.";
		c.inject = [](FlightSim& f) {
			if (f.LMPGNCS())
				f.LMPGNCS()->lrFailed = true;
		};
		c.resolved = [](FlightSim& f) {
			return !f.LMPGNCS() || !f.LMPGNCS()->lrFailed;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 15
	{
		ContingencySpec c;
		c.id = "a15-sps-switch";
		c.title = "SPS thrust switch short";
		c.mission = 15;
		c.severity = 3;
		c.event = Event::TLI;
		c.earliest = 2 * kH;
		c.latest = 2 * kH;
		c.history = "After TLI the SPS THRUST light came on with no "
			"command: a short in the delta-V thrust A switch. Arming bank A "
			"would fire the engine. The crew flew LOI and the later burns "
			"with bank B, and bank A switched in by hand only once lit.";
		c.cue = "SPS THRUST light on, engine not firing.";
		c.remedy = "Arm bank B only; bring bank A on manually after "
			"ignition, and off before the planned cutoff.";
		c.inject = [](FlightSim& f) {
			f.CMPGNCS().spsThrustShortA = true;
			f.Log(LogEntry::Caution, "SPS THRUST light on");
		};
		c.resolved = [](FlightSim& f) {
			return !f.SMAttached() || !f.CMPGNCS().panel.spsBankA;
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "a15-chute";
		c.title = "Main parachute fails";
		c.mission = 15;
		c.severity = 2;
		c.event = Event::Entry;
		c.earliest = 0;
		c.latest = 0;
		c.history = "Dumping the CM's RCS propellant through the main "
			"parachutes burned one of their risers; Endeavour came down on "
			"two, harder than planned but safely.";
		c.cue = "One main canopy collapses.";
		c.remedy = "Nothing to do but brace: two mains are survivable.";
		c.inject = [](FlightSim& f) { f.faults.mainChuteFouled = true; };
		c.resolved = [](FlightSim& f) {
			return f.CurrentPhase() == Phase::Recovery;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 16
	{
		ContingencySpec c;
		c.id = "a16-sps-yaw";
		c.title = "SPS secondary yaw servo oscillation";
		c.mission = 16;
		c.severity = 4;
		c.lunarOnly = true;
		c.event = Event::Undock;
		c.earliest = 1 * kH;
		c.latest = 1 * kH;
		c.history = "Preparing for the circularization burn, Mattingly found "
			"the SPS secondary yaw gimbal servo oscillating. Mission rules "
			"said wave off the landing. Six hours of analysis showed the "
			"primary system was fine and the backup would still work; "
			"Orion landed six hours late.";
		c.cue = "The yaw gimbal position indicator oscillates in the "
			"gimbal test.";
		c.remedy = "Wave off the burn and wait for Houston's analysis; "
			"fly the burn on the primary servo loop.";
		c.inject = [](FlightSim& f) {
			f.CMPGNCS().gimbalFailedYaw = true;
			f.Log(LogEntry::Caution, "SPS yaw gimbal oscillating in the "
				"secondary loop");
		};
		c.resolved = [](FlightSim& f) {
			return !f.CMPGNCS().gimbalFailedYaw;
		};
		v.push_back(c);
	}
	// --------------------------------------------------------------- Apollo 17
	{
		ContingencySpec c;
		c.id = "a17-fender";
		c.title = "Rover fender torn off";
		c.mission = 17;
		c.severity = 1;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::Landing;
		c.earliest = 6 * kH;
		c.latest = 6 * kH;
		c.history = "Cernan caught the rover's right rear fender extension "
			"with a hammer and tore it off. Rooster tails of dust coated "
			"everything. Overnight the crew built a new fender from four "
			"cronopaque maps, duct tape and clamps from the LM's lights.";
		c.cue = "Dust showers the crew and the rover; batteries and suits "
			"run hot.";
		c.remedy = "Tape four maps together into a fender.";
		c.inject = [](FlightSim& f) {
			f.crew.members[0].fatigue += 10;
			f.crew.members[2].fatigue += 10;
			f.lmCabin.temperature += 3;
		};
		c.resolved = [](FlightSim& f) {
			return f.CurrentPhase() != Phase::LunarSurface;
		};
		v.push_back(c);
	}

	// ----------------------------------------------------------- Generic ones
	{
		ContingencySpec c;
		c.id = "g-sic-engine-out";
		c.title = "F-1 engine out";
		c.severity = 3;
		c.event = Event::Liftoff;
		c.earliest = 75;
		c.latest = 125;
		c.history = "Mission rule: with one F-1 out after about T+70 s the "
			"Saturn V still makes orbit; the IU flies the others longer.";
		c.cue = "An ENGINE light; the acceleration sags.";
		c.remedy = "Continue: the IU's guidance absorbs it. Abort if a "
			"second engine goes.";
		c.applicable = [](FlightSim& f) {
			return f.IU() && f.IU()->CurrentMode() == SaturnIU::Mode::FirstStage
				&& f.Mission().saturnV;
		};
		c.inject = [](FlightSim& f) {
			f.IU()->FailEngine("F-1 " + std::to_string(1 + int(Uniform() * 4)));
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-sii-engine-out";
		c.title = "J-2 engine out on the S-II";
		c.severity = 3;
		c.event = Event::Liftoff;
		c.earliest = 250;
		c.latest = 420;
		c.history = "Apollo 6 lost two S-II engines (one shut down by "
			"crossed wiring) and the S-IVB still reached a lopsided orbit. "
			"One engine out is within the IU's reach.";
		c.cue = "An ENGINE light on the S-II; burn time stretches.";
		c.remedy = "Continue; watch for S-IVB propellant enough for TLI.";
		c.applicable = [](FlightSim& f) {
			return f.IU() && f.IU()->CurrentMode() == SaturnIU::Mode::SecondStage;
		};
		c.inject = [](FlightSim& f) {
			f.IU()->FailEngine("S-II J-2 " + std::to_string(1 + int(Uniform() * 4)));
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-tli-no-go";
		c.title = "S-IVB won't restart";
		c.severity = 4;
		c.lunarOnly = true;
		c.event = Event::TLI;
		c.earliest = -900;
		c.latest = -700;
		c.history = "Apollo 6's S-IVB failed to restart in orbit. Mission "
			"rules had alternate missions ready: a high-apogee Earth orbit "
			"flown with the SPS, or a CSM-only lunar flyby.";
		c.cue = "No TLI: the S-IVB's restart sequence fails.";
		c.remedy = "Separate from the S-IVB and fly an alternate mission "
			"with the SPS, or come home.";
		c.applicable = [](FlightSim& f) {
			return f.IU() && f.BoosterAttached();
		};
		c.inject = [](FlightSim& f) {
			f.IU()->FailEngine("J-2");
			f.Log(LogEntry::Caution, "S-IVB restart: J-2 failed to start");
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-stuck-jet";
		c.title = "RCS jet stuck on";
		c.severity = 3;
		c.event = Event::TLI;
		c.earliest = 6 * kH;
		c.latest = 60 * kH;
		c.history = "Gemini 8's stuck thruster spun Armstrong and Scott to "
			"one revolution a second. In Apollo the fix is to find the "
			"quad and close its isolation valves.";
		c.cue = "The spacecraft starts rolling and the rate keeps climbing.";
		c.remedy = "Find the quad from the roll direction and close its "
			"propellant isolation valves; switch to the other ring.";
		c.applicable = SMOn;
		c.inject = [](FlightSim& f) {
			for (Module& m : f.CSMVehicle().Modules())
				if (m.name == "Service Module")
					for (Thruster& t : m.thrusters)
						if (t.kind == THRUSTER_RCS) {
							t.failedOn = true;
							f.Log(LogEntry::Warning, "RCS jet " + t.name
								+ " firing continuously");
							return;
						}
		};
		c.resolved = [](FlightSim& f) {
			for (const Module& m : f.CSMVehicle().Modules())
				for (const Thruster& t : m.thrusters)
					if (t.failedOn && !t.failedOff)
						return false;
			return true;
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-fuel-cell";
		c.title = "Fuel cell failure";
		c.severity = 2;
		c.event = Event::TLI;
		c.earliest = 10 * kH;
		c.latest = 120 * kH;
		c.history = "Apollo 7, 10 and 13 all lost fuel-cell performance. "
			"Two cells carry a full mission; one carries a powered-down "
			"spacecraft home.";
		c.cue = "FC 2 condenser exit temperature runs high, then the cell "
			"drops off the bus.";
		c.remedy = "Take it off line, shed the non-essential loads.";
		c.applicable = SMOn;
		c.inject = [](FlightSim& f) {
			f.csmSystems.fuelCell[1].failed = true;
			f.csmSystems.fuelCell[1].online = false;
			f.Log(LogEntry::Caution, "FC 2 off line");
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-imu-fail";
		c.title = "IMU failure";
		c.severity = 4;
		c.event = Event::TLI;
		c.earliest = 4 * kH;
		c.latest = 100 * kH;
		c.history = "Mission rules covered a failed platform: the SCS and "
			"its body-mounted gyros fly the CSM, the LM's AGS can align "
			"and navigate, and the ground supplies the burns.";
		c.cue = "ISS warning, the attitude ball freezes, NO ATT on the DSKY.";
		c.remedy = "Fly on the SCS; perform burns with ground-computed "
			"attitudes and the EMS delta-V counter.";
		c.inject = [](FlightSim& f) {
			f.CMPGNCS().imu.failed = true;
			f.Log(LogEntry::Warning, "ISS warning: IMU failed");
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-cabin-leak";
		c.title = "Slow cabin leak";
		c.severity = 3;
		c.event = Event::TLI;
		c.earliest = 6 * kH;
		c.latest = 120 * kH;
		c.history = "A micrometeoroid or a failed seal: the rules called for "
			"suits on and the oxygen budget rechecked.";
		c.cue = "Cabin pressure drifting down; the O2 flow indicator high.";
		c.remedy = "Suits on and pressurised, find the leak, budget the "
			"surge tank and repress bottles.";
		c.inject = [](FlightSim& f) {
			f.cmCabin.leakRate = 0.15;
			f.Log(LogEntry::Caution, "CABIN PRESSURE falling");
		};
		c.resolved = [](FlightSim& f) { return f.cmCabin.leakRate <= 0; };
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-sps-fail";
		c.title = "SPS won't ignite";
		c.severity = 5;
		c.lunarOnly = true;
		c.event = Event::LOI;
		c.earliest = -600;
		c.latest = -300;
		c.history = "The SPS had no backup. Before LOI the LM's descent "
			"engine could bring the docked stack home on the free return; "
			"after LOI, a failed SPS would strand the crew.";
		c.cue = "The engine doesn't light: no thrust, no chamber pressure.";
		c.remedy = "Before LOI: stay on the free return and use the LM's DPS "
			"for corrections. Don't try LOI.";
		c.applicable = SMOn;
		c.inject = [](FlightSim& f) { f.CMPGNCS().spsFailed = true; };
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-lr-dropout";
		c.title = "Landing radar dropouts";
		c.severity = 2;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::PDI;
		c.earliest = 200;
		c.latest = 500;
		c.history = "Apollo 15 and 16 both saw the LR lose lock briefly over "
			"rough ground.";
		c.cue = "VEL and ALT lights flicker on.";
		c.remedy = "Keep going; the LGC coasts on inertial data.";
		c.inject = [](FlightSim& f) {
			if (f.LMPGNCS())
				f.LMPGNCS()->lrFailed = true;
		};
		c.resolved = [](FlightSim& f) {
			gnc::LMPGNCS* p = f.LMPGNCS();
			if (!p)
				return true;
			p->lrFailed = Uniform() < 0.3;
			return f.LMLanded();
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-co2";
		c.title = "Lithium hydroxide canister saturates early";
		c.severity = 2;
		c.event = Event::TLI;
		c.earliest = 20 * kH;
		c.latest = 120 * kH;
		c.history = "A bad canister means CO2 climbing until it is changed.";
		c.cue = "CO2 partial pressure rising past 7.6 mm.";
		c.remedy = "Change the canister early; recount the spares.";
		c.inject = [](FlightSim& f) { f.lioh.activeLife = 0.5; };
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-illness";
		c.title = "Crew illness";
		c.severity = 2;
		c.event = Event::Insertion;
		c.earliest = 20 * kH;
		c.latest = 150 * kH;
		c.history = "Apollo 13's Haise ran a fever from a urinary infection; "
			"Apollo 15's Irwin had irregular heartbeats on the surface.";
		c.cue = "A crewman reports feeling ill.";
		c.remedy = "Rest, fluids, the medical kit, and a lighter workload.";
		c.inject = [](FlightSim& f) {
			int i = int(Uniform() * 3);
			CrewMember& m = f.crew.members[i];
			m.sick = true;
			m.condition = "fever";
			m.health -= 25;
			f.Log(LogEntry::Crew, m.role + ": I'm running a fever.");
		};
		v.push_back(c);
	}
	{
		ContingencySpec c;
		c.id = "g-aps-arm";
		c.title = "Ascent engine arm fault";
		c.severity = 4;
		c.needsLM = true;
		c.lunarOnly = true;
		c.event = Event::Liftoff2;
		c.earliest = -3 * kH;
		c.latest = -1 * kH;
		c.history = "The ascent engine had no backup and no second chance. "
			"Every arming path had a manual bypass in the checklist.";
		c.cue = "The ascent engine arm doesn't take.";
		c.remedy = "Find and reset the breaker; use the manual ascent "
			"engine arm and the ABORT STAGE path.";
		c.inject = [](FlightSim& f) {
			if (f.LMPGNCS())
				f.LMPGNCS()->ascentArmBreakerOpen = true;
		};
		c.resolved = [](FlightSim& f) {
			return !f.LMPGNCS() || !f.LMPGNCS()->ascentArmBreakerOpen;
		};
		v.push_back(c);
	}
	return v;
}

}  // namespace


const std::vector<ContingencySpec>&
ContingencyLibrary()
{
	static const std::vector<ContingencySpec> library = Build();
	return library;
}


const ContingencySpec*
FindContingency(const std::string& id)
{
	for (const ContingencySpec& c : ContingencyLibrary())
		if (c.id == id)
			return &c;
	return nullptr;
}


const char*
DifficultyName(Difficulty d)
{
	switch (d) {
		case Difficulty::Nominal: return "Nominal";
		case Difficulty::AsFlown: return "As flown";
		case Difficulty::Challenging: return "Challenging";
		case Difficulty::Hard: return "Hard";
		case Difficulty::Nightmare: return "Nightmare";
	}
	return "";
}


const char*
DifficultyBlurb(Difficulty d)
{
	switch (d) {
		case Difficulty::Nominal:
			return "Everything works. Learn the flight.";
		case Difficulty::AsFlown:
			return "What really went wrong on this mission, when it did.";
		case Difficulty::Challenging:
			return "As flown, plus a couple of the problems the flight "
				"controllers trained for.";
		case Difficulty::Hard:
			return "As flown, plus several more, some of them serious.";
		case Difficulty::Nightmare:
			return "Something that ends missions. Bring them home.";
	}
	return "";
}


ContingencyManager::ContingencyManager(FlightSim& sim, Difficulty difficulty,
	uint32_t seed)
	:
	f(sim),
	fDifficulty(difficulty),
	fRandom(seed ? seed : std::random_device()())
{
	for (double& h : fHappened)
		h = -1e30;
}


double
ContingencyManager::_EventGET(Event e) const
{
	const FlightPlan& p = f.Mission().plan;
	switch (e) {
		case Event::Liftoff: return 0;
		case Event::Insertion: return f.Mission().saturnV ? 709 : 626;
		case Event::TLI: return p.tli;
		case Event::LOI: return p.loi;
		case Event::Undock: return p.undock;
		case Event::PDI: return p.pdi;
		case Event::Landing: return p.landing;
		case Event::Liftoff2: return p.liftoff;
		case Event::TEI: return p.tei;
		case Event::Entry: return p.entry;
	}
	return 0;
}


void
ContingencyManager::Plan()
{
	fSchedule.clear();
	if (fDifficulty == Difficulty::Nominal)
		return;
	const MissionSpec& m = f.Mission();
	bool lunar = m.plan.tli > 0;
	bool landing = m.plan.pdi > 0;
	auto usable = [&](const ContingencySpec& c) {
		if (c.lunarOnly && !lunar)
			return false;
		if (c.needsLM && m.lgcProgram.empty())
			return false;
		if ((c.event == Event::PDI || c.event == Event::Landing
				|| c.event == Event::Liftoff2) && !landing)
			return false;
		if (c.event == Event::Undock && m.plan.undock == 0)
			return false;
		if (c.event == Event::TLI && m.plan.tli == 0)
			return false;
		if (c.event == Event::TEI && m.plan.tei == 0)
			return false;
		if (c.event == Event::LOI && m.plan.loi == 0)
			return false;
		if (c.event == Event::Entry && m.plan.entry == 0)
			return false;
		return true;
	};
	auto schedule = [&](const ContingencySpec& c) {
		double t0 = _EventGET(c.event);
		double t = t0 + c.earliest + (c.latest - c.earliest)
			* std::uniform_real_distribution<double>(0, 1)(fRandom);
		if (t < f.GET())
			return;      // before this flight's start point
		Scheduled s;
		s.spec = &c;
		s.get = t;
		s.offset = t - t0;
		fSchedule.push_back(s);
	};
	// What happened on this flight.
	for (const std::string& id : m.historical) {
		const ContingencySpec* c = FindContingency(id);
		if (c && usable(*c))
			schedule(*c);
	}
	if (fDifficulty == Difficulty::AsFlown)
		return;
	// Draws from the generic library.
	std::vector<const ContingencySpec*> pool;
	for (const ContingencySpec& c : ContingencyLibrary())
		if (c.mission == 0 && usable(c))
			pool.push_back(&c);
	std::shuffle(pool.begin(), pool.end(), fRandom);
	int count = 0, maxSeverity = 2;
	switch (fDifficulty) {
		case Difficulty::Challenging: count = 2; maxSeverity = 2; break;
		case Difficulty::Hard: count = 4; maxSeverity = 4; break;
		case Difficulty::Nightmare: count = 4; maxSeverity = 4; break;
		default: break;
	}
	for (const ContingencySpec* c : pool) {
		if (count == 0)
			break;
		if (c->severity > maxSeverity)
			continue;
		schedule(*c);
		count--;
	}
	if (fDifficulty == Difficulty::Nightmare) {
		std::vector<const ContingencySpec*> big;
		for (const ContingencySpec& c : ContingencyLibrary())
			if (c.severity >= 5 && usable(c))
				big.push_back(&c);
		if (!big.empty())
			schedule(*big[fRandom() % big.size()]);
	}
	std::sort(fSchedule.begin(), fSchedule.end(),
		[](const Scheduled& a, const Scheduled& b) { return a.get < b.get; });
}


void
ContingencyManager::_Fire(Scheduled& s)
{
	s.fired = true;
	s.firedGET = f.GET();
	if (s.spec->inject)
		s.spec->inject(f);
	f.Log(LogEntry::Warning, s.spec->title);
	if (!s.spec->cue.empty())
		f.Log(LogEntry::Info, s.spec->cue);
}


bool
ContingencyManager::Trigger(const std::string& id)
{
	const ContingencySpec* c = FindContingency(id);
	if (!c)
		return false;
	Scheduled s;
	s.spec = c;
	s.get = f.GET();
	fSchedule.push_back(s);
	_Fire(fSchedule.back());
	return true;
}


double
ContingencyManager::_Happened(Event e)
{
	double& when = fHappened[int(e)];
	if (when > -1e29)
		return when;
	Phase ph = f.CurrentPhase();
	bool now = false;
	switch (e) {
		case Event::Liftoff: now = ph != Phase::Prelaunch; break;
		case Event::Insertion:
			now = ph != Phase::Prelaunch && ph != Phase::Boost;
			break;
		case Event::TLI:
			now = ph == Phase::TranslunarInjection || ph == Phase::TranslunarCoast
				|| ph == Phase::LunarOrbit;
			break;
		case Event::LOI: now = ph == Phase::LunarOrbit; break;
		case Event::Undock:
			now = f.LMVehicle() && !f.LMDocked()
				&& (ph == Phase::LunarOrbit || ph == Phase::PoweredDescent
					|| ph == Phase::EarthParkingOrbit);
			break;
		case Event::PDI: now = ph == Phase::PoweredDescent; break;
		case Event::Landing: now = ph == Phase::LunarSurface; break;
		case Event::Liftoff2:
			now = ph == Phase::LunarAscent || ph == Phase::Rendezvous;
			break;
		case Event::TEI: now = ph == Phase::TransearthCoast; break;
		case Event::Entry: now = ph == Phase::Entry; break;
	}
	if (now) {
		// Already past when the flight began (a later start point): as
		// planned.
		when = fStarted ? f.GET() : std::min(f.GET(), _EventGET(e));
	}
	return now ? when : -1;
}


void
ContingencyManager::Tick()
{
	double get = f.GET();
	if (!fStarted) {
		for (int e = 0; e <= int(Event::Entry); e++)
			_Happened(Event(e));
		fStarted = true;
	}
	for (Scheduled& s : fSchedule) {
		if (!s.fired) {
			// Timed from the event as it happened (the flight plan only
			// guesses: a landing comes when the descent does).
			double at = _Happened(s.spec->event);
			if (at < 0 || get < at + s.offset)
				continue;
			if (s.spec->applicable && !s.spec->applicable(f)) {
				// Missed its moment (e.g. that stage is already gone).
				if (get > s.get + 600)
					s.fired = s.resolved = true;
				continue;
			}
			_Fire(s);
			continue;
		}
		if (!s.resolved) {
			bool done = s.spec->resolved ? s.spec->resolved(f) : true;
			if (done) {
				s.resolved = true;
				s.resolvedGET = get;
				if (s.spec->resolved)
					f.Log(LogEntry::Event, "Resolved: " + s.spec->title);
			}
		}
	}
}


int
ContingencyManager::Fired() const
{
	int n = 0;
	for (const Scheduled& s : fSchedule)
		n += s.fired;
	return n;
}


int
ContingencyManager::Resolved() const
{
	int n = 0;
	for (const Scheduled& s : fSchedule)
		n += s.resolved;
	return n;
}

}  // namespace flight
