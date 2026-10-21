# Columbia

An Apollo flight simulator for [Haiku](https://www.haiku-os.org), running the
software that actually flew.

The Apollo Guidance Computers are emulated instruction by instruction and
run the original flight programs, assembled from the
[Virtual AGC](https://www.ibiblio.org/apollo/) transcriptions: Colossus,
Comanche and Artemis in the Command Module, Sundance and Luminary in the
Lunar Module. Around them is a simulation of the spacecraft, the Saturn
launch vehicle, the Earth and the Moon. You fly with the DSKY, the
switches and the hand controllers, and Mission Control reads up the PADs
the way Houston did.

You can get it wrong. Burn too long and the orbit dips into the hills; come
home too steep and the heat shield doesn't save you.

Runs on x86_64 and ARM64 (including the Raspberry Pi). MIT licensed.

![Apollo 11's map view during translunar coast: the trajectory from TLI out
past the second midcourse correction toward the Moon, with the CMC's DSKY
in P40](docs/map-translunar.png)

*Apollo 11, 38 hours in: the map view in the Earth-Moon rotating frame, from
TLI past MCC-2, with the Command Module computer's DSKY alongside.
Rendered with `make objects/preview/map`.*

## What's in it

- **Missions 7 through 17**, each with its own flight software, pad loads,
  launch date, landing site and crew timeline, plus **a mission of your
  own** to any of the Apollo sites or the ones NASA studied (Marius Hills,
  Tycho, Copernicus, ...).
- **Start anywhere**: launch, Earth orbit, translunar coast, lunar orbit,
  powered descent, on the surface, lunar ascent, transearth coast or entry.
- **An automatic crew** that can fly any of it, all of it, or just the parts
  you don't want to. Watch a whole mission at a hundred times speed or take
  over at PDI.
- **Difficulty**: *Nominal* (nothing goes wrong), *As flown* (what really
  went wrong, when it did: the 1201 alarms, Apollo 12's lightning, Apollo
  13's oxygen tank), *Challenging*, *Hard* and *Nightmare* (something that
  ends missions; bring them home).
- **Training drills**: transposition and docking, lunar orbit insertion,
  the last 500 feet, rendezvous from insertion, terminal phase initiation,
  and docking. The crew flies you to the moment and hands you the controls,
  with the computers in the state they'd really be in.
- **Flight's calls**: at the real decision points, Mission Control stops the
  clock and you're the Flight Director. Take Apollo 13 home on a direct
  abort instead of around the Moon, or skip the PC+2 burn, and live with what
  follows.
- **Three views**: the controls (panels and DSKY), out the window, and the
  map (ground track, orbits and trajectory).
- **Time warp**, with the clock stopping on its own for decisions.
- **Flight recordings** and a **screensaver** that replays them.

## Installing

### With HaikuPorter

The recipe is in `haikuports/games-simulation/columbia/`. Copy that
directory into your HaikuPorts tree (or point HaikuPorter at this one) and:

```sh
haikuporter --get-dependencies columbia   # build the package
pkgman install packages/columbia-*.hpkg
```

Or put the `.hpkg` in `/boot/system/packages`. The package installs:

| What | Where |
|---|---|
| The app | `/boot/system/apps/Columbia` (and in the Deskbar's Applications) |
| The flight software | `/boot/system/data/Columbia/rope` |
| The screensaver | `/boot/system/add-ons/Screen Savers/Columbia` |
| A recording of Apollo 11 | `/boot/system/data/Columbia/recordings` |

Building the package flies Apollo 11 headless to make the screensaver's
first recording, which takes several minutes (longer on ARM). Set
`RECORD_MISSIONS=""` in the recipe to skip it, or list more missions
(`"11 13 17"`).

### From source

You need Haiku's development tools (`pkgman install haiku_devel` if they
aren't there; `gcc` and `make` come with them).

```sh
git clone https://github.com/linkslice/Columbia.git
cd Columbia
make -j4                # the app: ./Columbia
make -j4 saver          # the screensaver add-on
```

Run `./Columbia` from the source directory and it finds the flight software
in `rope/` next to it. To install it for your user instead:

```sh
make install            # into ~/config/non-packaged
```

That puts the app in `~/config/non-packaged/apps`, the ropes in
`~/config/non-packaged/data/Columbia/rope`, the screensaver in
`~/config/non-packaged/add-ons/Screen Savers`, and any recordings you've
made with `make recordings` next to the ropes. Every directory can be
overridden: `make install DESTDIR=... APPS_DIR=... DATA_DIR=...
ADDONS_DIR=...`.

The parts can be installed separately: `install-app`, `install-ropes`,
`install-saver`, `install-recordings`.

## Flying

**The mission picker** opens first. Choose a mission, then:

- **Start**: where in the mission to begin, or one of the training drills.
- **Difficulty**: from Nominal to Nightmare (the line under it says what
  each means).
- **Let the automatic crew fly it**: watch, and take over whenever you like
  (Flight ▸ Automatic crew, or ⌘A).
- **Real time only**: no time warp.
- **Make Flight's calls yourself**: you decide at the decision points. With
  it off, Flight makes the call that was made on the real mission.
- **Landing site**: for a mission of your own.
- **Replay**: watch a recorded flight.

**In flight**, the View menu (or Tab) switches between the controls, the
window and the map, and chooses whose spacecraft you follow: the crew's,
the CM's or the LM's.

### Keys

| Key | Does |
|---|---|
| ↑ ↓ | pitch |
| ← → | roll |
| `[` `]` | yaw |
| Shift + ↑ ↓ | translate up / down |
| Shift + ← → | translate left / right |
| Page Up / Page Down | translate forward / aft |
| Shift + Page Up / Page Down | LM descent engine throttle (manual) |
| `.` / `,` | faster / slower |
| ⌘1 | real time |
| Space | pause |
| Tab | next view |
| ⌘S | save a flight recording |

**The DSKY**, on the controls view, takes the keyboard as well as the
mouse: `V` VERB, `N` NOUN, `+` `-`, `0`–`9`, `E` or Enter for ENTR, `R`
RSET, `C` or Backspace CLR, `K` KEY REL, `P` PRO.

If you're new to the AGC: `V37E` then a program number and `E` changes
program (`V37E30E` is P30, external delta-V), `V16N65E` shows the clock,
and a flashing VERB/NOUN is the computer asking you something: PRO (`P`) to
go on, or load what it's asking for and ENTR.

**A joystick** works as the hand controllers: the stick for roll and pitch,
twist for yaw, the hat to translate, trigger and thumb button for +X and
−X, button 3 for PRO, and the throttle lever for the LM's descent engine.
Set it up in the Joysticks preferences first.

Help ▸ Keys and controllers has all of this in the app.

### Flight's calls

When a decision comes up, the clock stops and Mission Control lays out the
situation and the options, with what each one costs; the one taken on the
real flight is marked "as flown". Today the decisions are Apollo 13's:
abort the mission (direct abort, or around the Moon on a free return), and
the PC+2 burn (the fast return to the Pacific, the slow one to the Indian
Ocean, or none at all). Every branch flies to its end, good or bad.

### Recordings and the screensaver

Closing a flight saves a recording of it to
`~/config/non-packaged/data/Columbia/recordings` (Flight ▸ Save flight
recording saves one any time). Watch them from the mission picker's Replay
menu, or choose **Columbia** in the ScreenSaver preferences, which replays
them, one flight or all of them shuffled, at a pace that keeps the
interesting parts.

To make recordings without flying them yourself:

```sh
make recordings MISSIONS_TO_RECORD="11 13 17"     # each takes minutes
make install-recordings
```

## Building and testing on other systems

The simulation is portable C++17 with no dependencies, and everything except
the Haiku interface builds and runs on Linux, macOS or any POSIX host:

```sh
make test               # AGC and mission tests (seconds)
make flighttest         # the flight regression: landings, burns, entries,
                        # launches, contingencies, whole missions (slow)
make objects/tests/drill_test objects/tests/decision_test \
     objects/tests/session_test
```

`objects/record_mission rope <mission> out.colrec [--start N]
[--difficulty nominal|asflown|challenging|hard|nightmare] [--seed N]
[--quiet]` flies a mission headless and records it.

With cairo installed, `make objects/preview/replay`,
`make objects/preview/panel` and `make objects/preview/map` render the
screensaver, the instrument panels and the map view to PNGs off Haiku, through a small shim of the Interface Kit.

`tools/haiku_syntax_check.sh` checks the Haiku-only sources against Haiku's
headers without a Haiku machine. `tools/verify_rope.sh` rebuilds every rope
from Virtual AGC and checks it (see `rope/PROVENANCE.md`).

Diagnostics for the flight software and the simulation are switched on with
environment variables (`PDIDIAG=1`, `DESCDIAG=1`, `ENTRYTRACE=1`,
`CREW_VERBOSE=1`, ...); grep for `getenv` to find them.

## Layout

| Directory | What's there |
|---|---|
| `core/agc` | the AGC emulator and DSKY |
| `core/gnc` | ropes, pad loads, guidance and navigation support |
| `core/sim` | the physics: Earth, Moon, vehicles, engines, aerodynamics |
| `core/flight` | missions, the Saturn, the automatic crew, Mission Control, contingencies, drills, recordings |
| `app` | the Haiku application |
| `ui` | drawing shared by the app and the screensaver |
| `screensaver` | the Screen Saver add-on |
| `rope` | the flight software images and their symbol tables |
| `tests`, `tools` | tests, the recorder, and maintenance scripts |
| `haikuports` | the HaikuPorter recipe |

## Status

Young. Apollo 11 and 13 through 17 fly end to end, headless, with the
automatic crew; the decision branches all reach their ends. Known problems:

- Apollo 12's flight after landing fails (the CSM's orbital plane by then
  doesn't suit the rendezvous).
- Apollo 16 splashes down about 64 nautical miles from the recovery ship.
- Apollo 17's lunar orbit plane change takes more delta-V than it should.
- The Haiku interface is new and has had far less flying than the
  simulation underneath it.

Reports and fixes are welcome.

## Credits

Columbia stands on a great deal of other people's work.

- **The Apollo flight software** was written by the
  MIT Instrumentation Laboratory (now
  [Draper](https://www.draper.com)) for NASA; its software
  engineering division was led by Margaret Hamilton. It is in the public
  domain.
- **[Virtual AGC](https://www.ibiblio.org/apollo/)**, led by Ron Burkey with
  Onno Hommes, Jim Lawton and Mike Stewart, rescued that software: gathering
  scans of the surviving listings, transcribing them by hand, reconstructing versions
  that didn't survive, and writing the assembler that turns them back into
  ropes. Every rope Columbia runs comes from their work
  ([GitHub](https://github.com/virtualagc/virtualagc); their
  [acknowledgements](https://www.ibiblio.org/apollo/index.html#Acknowledgements)
  credit the many volunteers, archivists and Apollo engineers who
  contributed listings and transcriptions). See `rope/PROVENANCE.md`.
- **[Project Apollo – NASSP](https://github.com/orbiternassp/NASSP)**, the
  Apollo add-on for Martin Schweiger's
  [Orbiter](https://github.com/orbitersim/orbiter) simulator, whose team
  reconstructed each mission's computer pad loads from NASA documents.
  Columbia's pad-load tables (`core/gnc/MissionPads.cpp`) are read from
  their mission scenarios.
- **NASA**: the mission reports and press kits behind the mission data
  (dates, crews, timelines), via the
  [NASA Technical Reports Server](https://ntrs.nasa.gov) and the
  [NASA History Office](https://history.nasa.gov).
- **[LROC](https://www.lroc.asu.edu)** (Lunar Reconnaissance Orbiter Camera,
  Arizona State University / NASA) for the surveyed coordinates of the
  landing sites.
- **[Natural Earth](https://www.naturalearthdata.com)** for the coastlines
  on the map, public domain.
- **[Haiku](https://www.haiku-os.org)** and
  **[HaikuPorts](https://github.com/haikuports/haikuports)**.

## License

MIT; see `LICENSE`. The flight software in `rope/` is public domain;
`NOTICE.md` lists everything in the tree that came from elsewhere and where
it came from.
