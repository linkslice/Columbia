// Columbia — Apollo flight simulator for Haiku
// Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
//
// Mission data are historical facts from the Apollo mission reports and
// press kits (NASA, public domain): dates, crews, landing sites (the
// LRO-surveyed coordinates), and nominal ground elapsed times.

#include "MissionSpec.h"

namespace flight {

namespace {

double
H(int h, int m, double s)
{
	return h * 3600.0 + m * 60.0 + s;
}

std::vector<MissionSpec>
Build()
{
	std::vector<MissionSpec> v;

	{
		MissionSpec m;
		m.number = 7;
		m.name = "Apollo 7";
		m.motto = "First crewed Apollo: eleven days proving the Block II CSM";
		m.type = MissionType::EarthOrbitCSM;
		m.commander = "Walter M. Schirra Jr.";
		m.cmPilot = "Donn F. Eisele";
		m.lmPilot = "R. Walter Cunningham";
		m.csmCallsign = "Apollo 7";
		m.saturnV = false;
		m.year = 1968; m.month = 10; m.day = 11; m.hour = 15; m.minute = 2;
		m.second = 45;
		m.pad = "LC-34";
		m.launchAzimuth = 72;
		m.cmcProgram = "Colossus237";   // stand-in for Sundisk 282
		m.lgcProgram = "";
		m.padLoads = 7;
		m.plan.splashdown = H(260, 9, 3);
		m.splashLatitude = 27.54;
		m.splashLongitude = -64.07;
		m.csmSpsPropellant = 4500;     // partial load for Earth orbit
		m.historical = {"a7-colds"};
		m.summary = "Eight SPS burns, a rendezvous with the spent S-IVB and "
			"the first live television from orbit, flown by a crew fighting "
			"head colds the whole way.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 8;
		m.name = "Apollo 8";
		m.motto = "First to the Moon: ten orbits on Christmas Eve";
		m.type = MissionType::LunarOrbit;
		m.commander = "Frank Borman";
		m.cmPilot = "James A. Lovell Jr.";
		m.lmPilot = "William A. Anders";
		m.csmCallsign = "Apollo 8";
		m.year = 1968; m.month = 12; m.day = 21; m.hour = 12; m.minute = 51;
		m.launchAzimuth = 72.12;
		m.cmcProgram = "Colossus237";
		m.lgcProgram = "";
		m.padLoads = 8;
		m.plan.tli = H(2, 50, 37);
		m.plan.loi = H(69, 8, 20);
		m.plan.loi2 = H(73, 35, 7);
		m.plan.tei = H(89, 19, 16);
		m.plan.entry = H(146, 46, 13);
		m.plan.splashdown = H(147, 0, 42);
		m.splashLatitude = 8.13;
		m.splashLongitude = -165.02;
		m.csmSpsPropellant = 18200;
		m.historical = {"a8-p01"};
		m.summary = "With no LM, Apollo 8 carried a ballast test article and "
			"flew the CSM alone to lunar orbit. Lovell's accidental P01 on "
			"the way home erased the platform alignment.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 9;
		m.name = "Apollo 9";
		m.motto = "The LM's first flight, in Earth orbit";
		m.type = MissionType::EarthOrbitLM;
		m.commander = "James A. McDivitt";
		m.cmPilot = "David R. Scott";
		m.lmPilot = "Russell L. Schweickart";
		m.csmCallsign = "Gumdrop";
		m.lmCallsign = "Spider";
		m.year = 1969; m.month = 3; m.day = 3; m.hour = 16; m.minute = 0;
		m.launchAzimuth = 72;
		m.cmcProgram = "Colossus249";
		m.lgcProgram = "Sundance306ish";
		m.padLoads = 9;
		m.plan.undock = H(92, 39, 36);
		m.plan.docking = H(99, 2, 26);
		m.plan.splashdown = H(241, 0, 54);
		m.splashLatitude = 23.21;
		m.splashLongitude = -67.98;
		m.historical = {"a9-sickness"};
		m.summary = "Docked and undocked burns of every engine, an EVA from "
			"the LM porch, and a 113-mile separation and rendezvous of the "
			"two spacecraft.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 10;
		m.name = "Apollo 10";
		m.motto = "The dress rehearsal: down to 47,000 feet";
		m.type = MissionType::LunarOrbitLM;
		m.commander = "Thomas P. Stafford";
		m.cmPilot = "John W. Young";
		m.lmPilot = "Eugene A. Cernan";
		m.csmCallsign = "Charlie Brown";
		m.lmCallsign = "Snoopy";
		m.year = 1969; m.month = 5; m.day = 18; m.hour = 16; m.minute = 49;
		m.launchAzimuth = 72.03;
		m.cmcProgram = "Comanche045";
		m.lgcProgram = "LUM69R2";
		m.padLoads = 10;
		m.site = {"Apollo Landing Site 2 (overflight)", 0.71, 23.71, 1735300, 10};
		m.plan.tli = H(2, 39, 20);
		m.plan.loi = H(75, 55, 54);
		m.plan.loi2 = H(80, 25, 8);
		m.plan.undock = H(98, 11, 57);
		m.plan.doi = H(99, 46, 1);
		m.plan.docking = H(106, 22, 2);
		m.plan.tei = H(137, 36, 28);
		m.plan.entry = H(191, 48, 54);
		m.plan.splashdown = H(192, 3, 23);
		m.splashLatitude = -15.07;
		m.splashLongitude = -164.65;
		m.lmAscentPropellant = 1200;
		m.historical = {"a10-gyration"};
		m.summary = "Snoopy flew the descent orbit to within nine miles of "
			"the surface. At staging a mis-set AGS mode switch sent the "
			"ascent stage into a wild gyration.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 11;
		m.name = "Apollo 11";
		m.motto = "The first landing: Tranquility Base";
		m.type = MissionType::Landing;
		m.commander = "Neil A. Armstrong";
		m.cmPilot = "Michael Collins";
		m.lmPilot = "Edwin E. Aldrin Jr.";
		m.csmCallsign = "Columbia";
		m.lmCallsign = "Eagle";
		m.year = 1969; m.month = 7; m.day = 16; m.hour = 13; m.minute = 32;
		m.launchAzimuth = 72.058;
		m.cmcProgram = "Comanche055";
		m.lgcProgram = "Luminary099";
		m.padLoads = 11;
		m.site = {"Tranquility Base", 0.67408, 23.47297, 1735470, 10.8};
		m.plan.tli = H(2, 44, 16);
		m.plan.loi = H(75, 49, 50);
		m.plan.loi2 = H(80, 11, 36);
		m.plan.undock = H(100, 12, 0);
		m.plan.doi = H(101, 36, 14);
		m.plan.pdi = H(102, 33, 5);
		m.plan.landing = H(102, 45, 40);
		m.plan.liftoff = H(124, 22, 1);
		m.plan.docking = H(128, 3, 0);
		m.plan.tei = H(135, 23, 42);
		m.plan.entry = H(195, 3, 6);
		m.plan.splashdown = H(195, 18, 35);
		m.plan.evas = 1;
		m.plan.evaHours = 2.5;
		m.splashLatitude = 13.30;
		m.splashLongitude = -169.15;
		m.historical = {"a11-1202", "a11-long-landing", "a11-breaker"};
		m.summary = "Program alarms 1201 and 1202 rang through the descent "
			"while Armstrong flew past a boulder field on manual control, "
			"landing with about 25 seconds of fuel to the bingo call.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 12;
		m.name = "Apollo 12";
		m.motto = "Pinpoint landing beside Surveyor 3";
		m.type = MissionType::Landing;
		m.commander = "Charles Conrad Jr.";
		m.cmPilot = "Richard F. Gordon Jr.";
		m.lmPilot = "Alan L. Bean";
		m.csmCallsign = "Yankee Clipper";
		m.lmCallsign = "Intrepid";
		m.year = 1969; m.month = 11; m.day = 14; m.hour = 16; m.minute = 22;
		m.launchAzimuth = 72.029;
		m.cmcProgram = "Comanche067";
		m.lgcProgram = "Luminary116";
		m.padLoads = 12;
		m.site = {"Ocean of Storms (Surveyor 3)", -3.01239, -23.42157, 1736000, 5.1};
		m.plan.tli = H(2, 47, 23);
		m.plan.loi = H(83, 25, 23);
		m.plan.loi2 = H(87, 48, 48);
		m.plan.undock = H(107, 54, 2);
		m.plan.doi = H(109, 23, 39);
		m.plan.pdi = H(110, 20, 38);
		m.plan.landing = H(110, 32, 36);
		m.plan.liftoff = H(142, 3, 47);
		m.plan.docking = H(145, 36, 20);
		m.plan.tei = H(172, 27, 17);
		m.plan.entry = H(244, 22, 19);
		m.plan.splashdown = H(244, 36, 25);
		m.plan.evas = 2;
		m.plan.evaHours = 7.75;
		m.splashLatitude = -15.78;
		m.splashLongitude = -165.15;
		m.historical = {"a12-lightning"};
		m.summary = "Lightning struck the Saturn V twice in the first minute. "
			"Fuel cells dropped off line and the panel lit up until John "
			"Aaron's call came up: 'SCE to Aux.'";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 13;
		m.name = "Apollo 13";
		m.motto = "'Houston, we've had a problem.'";
		m.type = MissionType::Landing;
		m.commander = "James A. Lovell Jr.";
		m.cmPilot = "John L. Swigert Jr.";
		m.lmPilot = "Fred W. Haise Jr.";
		m.csmCallsign = "Odyssey";
		m.lmCallsign = "Aquarius";
		m.year = 1970; m.month = 4; m.day = 11; m.hour = 19; m.minute = 13;
		m.launchAzimuth = 72.043;
		m.cmcProgram = "Comanche072";
		m.lgcProgram = "LM131R1";
		m.padLoads = 13;
		m.site = {"Fra Mauro (planned)", -3.64530, -17.47136, 1736300, 10};
		m.plan.tli = H(2, 35, 46);
		m.plan.loi = H(77, 25, 0);
		m.plan.pdi = H(103, 42, 0);
		m.plan.landing = H(103, 53, 0);
		m.plan.tei = H(167, 26, 0);
		m.plan.entry = H(142, 40, 46);
		m.plan.splashdown = H(142, 54, 41);
		m.plan.evas = 2;
		m.plan.evaHours = 8;
		m.splashLatitude = -21.63;
		m.splashLongitude = -165.37;
		m.historical = {"a13-pogo", "a13-o2-tank"};
		m.summary = "An oxygen tank in the service module exploded at 55 "
			"hours 55 minutes. The crew rode the LM around the Moon as a "
			"lifeboat and came home on its descent engine.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 14;
		m.name = "Apollo 14";
		m.motto = "Fra Mauro, after all";
		m.type = MissionType::Landing;
		m.commander = "Alan B. Shepard Jr.";
		m.cmPilot = "Stuart A. Roosa";
		m.lmPilot = "Edgar D. Mitchell";
		m.csmCallsign = "Kitty Hawk";
		m.lmCallsign = "Antares";
		m.year = 1971; m.month = 1; m.day = 31; m.hour = 21; m.minute = 3;
		m.second = 2;
		m.launchAzimuth = 75.558;
		m.cmcProgram = "Comanche072";   // stand-in for Comanche 108
		m.lgcProgram = "Luminary178";
		m.padLoads = 14;
		m.site = {"Fra Mauro", -3.64530, -17.47136, 1736300, 10.3};
		m.plan.tli = H(2, 28, 32);
		m.plan.loi = H(81, 56, 40);
		m.plan.doi = H(86, 10, 53);
		m.plan.undock = H(103, 47, 42);
		m.plan.pdi = H(108, 2, 27);
		m.plan.landing = H(108, 15, 9);
		m.plan.liftoff = H(141, 45, 40);
		m.plan.docking = H(143, 32, 51);
		m.plan.tei = H(148, 36, 2);
		m.plan.entry = H(215, 47, 45);
		m.plan.splashdown = H(216, 1, 58);
		m.plan.evas = 2;
		m.plan.evaHours = 9.4;
		m.splashLatitude = -27.02;
		m.splashLongitude = -172.67;
		m.historical = {"a14-docking-probe", "a14-abort-switch", "a14-lr"};
		m.summary = "Six tries to dock with the S-IVB. A solder ball in the "
			"LM's ABORT switch threatened to end the landing until Don "
			"Eyles wrote a work-around keyed in by hand an hour before PDI.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 15;
		m.name = "Apollo 15";
		m.motto = "Hadley Rille and the first Rover";
		m.type = MissionType::LandingExtended;
		m.commander = "David R. Scott";
		m.cmPilot = "Alfred M. Worden";
		m.lmPilot = "James B. Irwin";
		m.csmCallsign = "Endeavour";
		m.lmCallsign = "Falcon";
		m.year = 1971; m.month = 7; m.day = 26; m.hour = 13; m.minute = 34;
		m.launchAzimuth = 80.088;
		m.cmcProgram = "Artemis072";
		m.lgcProgram = "Luminary210";
		m.padLoads = 15;
		m.site = {"Hadley-Apennine", 26.13222, 3.63386, 1735620, 12.2};
		m.plan.tli = H(2, 50, 2);
		m.plan.loi = H(78, 31, 46);
		m.plan.doi = H(82, 39, 49);
		m.plan.undock = H(100, 39, 16);
		m.plan.pdi = H(104, 30, 9);
		m.plan.landing = H(104, 42, 29);
		m.plan.liftoff = H(171, 37, 23);
		m.plan.docking = H(173, 36, 25);
		m.plan.tei = H(223, 48, 46);
		m.plan.entry = H(294, 58, 54);
		m.plan.splashdown = H(295, 11, 53);
		m.plan.evas = 3;
		m.plan.evaHours = 18.6;
		m.splashLatitude = 26.13;
		m.splashLongitude = -158.13;
		m.csmSpsPropellant = 18500;
		m.lmDescentPropellant = 8873;
		m.lmDescentDry = 2792;
		m.historical = {"a15-sps-switch", "a15-chute"};
		m.summary = "A short in a delta-V thrust switch, a landing beside "
			"Hadley Rille, three Rover traverses, and a splashdown under "
			"two of three main parachutes.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 16;
		m.name = "Apollo 16";
		m.motto = "Into the highlands at Descartes";
		m.type = MissionType::LandingExtended;
		m.commander = "John W. Young";
		m.cmPilot = "Thomas K. Mattingly II";
		m.lmPilot = "Charles M. Duke Jr.";
		m.csmCallsign = "Casper";
		m.lmCallsign = "Orion";
		m.year = 1972; m.month = 4; m.day = 16; m.hour = 17; m.minute = 54;
		m.launchAzimuth = 72.034;
		m.cmcProgram = "Artemis072";
		m.lgcProgram = "Luminary210";
		m.padLoads = 16;
		m.site = {"Descartes Highlands", -8.97301, 15.50019, 1737400, 11.9};
		m.plan.tli = H(2, 33, 33);
		m.plan.loi = H(74, 28, 28);
		m.plan.doi = H(78, 33, 45);
		m.plan.undock = H(96, 13, 31);
		m.plan.pdi = H(104, 17, 25);
		m.plan.landing = H(104, 29, 35);
		m.plan.liftoff = H(175, 31, 47);
		m.plan.docking = H(177, 41, 18);
		m.plan.tei = H(200, 21, 33);
		m.plan.entry = H(265, 37, 31);
		m.plan.splashdown = H(265, 51, 5);
		m.plan.evas = 3;
		m.plan.evaHours = 20.2;
		m.splashLatitude = -0.70;
		m.splashLongitude = -156.22;
		m.csmSpsPropellant = 18500;
		m.lmDescentPropellant = 8873;
		m.lmDescentDry = 2792;
		m.historical = {"a16-sps-yaw"};
		m.summary = "An oscillation in the SPS secondary yaw gimbal servo "
			"held off the landing for six hours while engineers proved the "
			"engine could still bring the crew home.";
		v.push_back(m);
	}
	{
		MissionSpec m;
		m.number = 17;
		m.name = "Apollo 17";
		m.motto = "The last of Apollo: Taurus-Littrow";
		m.type = MissionType::LandingExtended;
		m.commander = "Eugene A. Cernan";
		m.cmPilot = "Ronald E. Evans";
		m.lmPilot = "Harrison H. Schmitt";
		m.csmCallsign = "America";
		m.lmCallsign = "Challenger";
		m.year = 1972; m.month = 12; m.day = 7; m.hour = 5; m.minute = 33;
		m.launchAzimuth = 91.503;
		m.cmcProgram = "Artemis072";
		m.lgcProgram = "Luminary210";
		m.padLoads = 17;
		m.site = {"Taurus-Littrow", 20.19080, 30.77168, 1734900, 13.3};
		m.plan.tli = H(3, 18, 37);
		m.plan.loi = H(88, 21, 8);
		m.plan.doi = H(92, 47, 23);
		m.plan.undock = H(107, 47, 56);
		m.plan.pdi = H(112, 49, 49);
		m.plan.landing = H(113, 1, 58);
		m.plan.liftoff = H(188, 1, 36);
		m.plan.docking = H(190, 17, 40);
		m.plan.tei = H(236, 42, 9);
		m.plan.entry = H(304, 18, 42);
		m.plan.splashdown = H(304, 31, 59);
		m.plan.evas = 3;
		m.plan.evaHours = 22.1;
		m.splashLatitude = -17.88;
		m.splashLongitude = -166.11;
		m.csmSpsPropellant = 18500;
		m.lmDescentPropellant = 8873;
		m.lmDescentDry = 2792;
		m.historical = {"a17-fender"};
		m.summary = "A night launch, the longest stay on the Moon, a "
			"duct-taped Rover fender, and orange soil at Shorty crater.";
		v.push_back(m);
	}
	return v;
}

}  // namespace


const std::vector<MissionSpec>&
ApolloMissions()
{
	static const std::vector<MissionSpec> missions = Build();
	return missions;
}


const MissionSpec*
FindMission(int number)
{
	for (const MissionSpec& m : ApolloMissions())
		if (m.number == number)
			return &m;
	return nullptr;
}


const std::vector<LandingSite>&
CandidateSites()
{
	static const std::vector<LandingSite> sites = {
		{"Tranquility Base (A11)", 0.67408, 23.47297, 1735470, 10},
		{"Ocean of Storms (A12)", -3.01239, -23.42157, 1736000, 7},
		{"Fra Mauro (A14)", -3.64530, -17.47136, 1736300, 10},
		{"Hadley-Apennine (A15)", 26.13222, 3.63386, 1735620, 12},
		{"Descartes (A16)", -8.97301, 15.50019, 1737400, 12},
		{"Taurus-Littrow (A17)", 20.19080, 30.77168, 1734900, 13},
		{"Censorinus (studied)", -0.38, 32.68, 1737000, 10},
		{"Littrow (studied)", 21.8, 29.6, 1735500, 10},
		{"Marius Hills (studied)", 12.6, -56.6, 1735800, 10},
		{"Copernicus central peaks (studied)", 9.6, -20.1, 1735000, 10},
		{"Tycho (studied)", -41.0, -11.0, 1738500, 10},
		{"Schroter's Valley (studied)", 26.0, -50.5, 1735800, 10},
		{"Hyginus Rille (studied)", 7.8, 6.3, 1736400, 10},
		{"Davy Crater Chain (studied)", -11.0, -7.0, 1736500, 10},
	};
	return sites;
}


MissionSpec
CustomMission(const std::string& name, const LandingSite& site, int year,
	int month, int day)
{
	MissionSpec m = *FindMission(17);
	m.number = 0;
	m.name = name;
	m.motto = "A mission of your own";
	m.site = site;
	m.year = year;
	m.month = month;
	m.day = day;
	m.hour = 14;
	m.minute = 0;
	m.launchAzimuth = 72;
	m.commander = "Commander";
	m.cmPilot = "Command Module Pilot";
	m.lmPilot = "Lunar Module Pilot";
	m.csmCallsign = "Endurance";
	m.lmCallsign = "Resolute";
	m.historical.clear();
	m.summary = "Your landing site, your launch date. Mission Control "
		"will plan the trajectory; the contingencies depend on the "
		"difficulty you choose.";
	return m;
}

}  // namespace flight
