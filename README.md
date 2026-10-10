# Columbia

An Apollo flight simulator for [Haiku](https://www.haiku-os.org/). Fly the
Command Module from Earth orbit to the Moon and back, with the real Apollo 11
guidance software running on the DSKY.

- **Left pane: controls.** A working Command Module DSKY driven by
  **Comanche 055**, the actual Apollo 11 CM flight program, running unmodified
  on an AGC emulator. Below it are the flight status, time warp and your
  flight plan.
- **Right pane: the path.** Your trajectory around the Earth and the Moon:
  the path flown so far, the predicted path through every planned burn, and
  where it ends up (pericynthion, lunar impact, or the entry angle at the
  atmosphere).

## Flying a mission

1. **Plan TLI.** Click **TLI** for a free-return translunar injection with
   the S-IVB: a figure-8 that swings 110 km past the Moon and comes home even
   if nothing else happens. The predicted path appears at once.
2. **Warp.** Use **Faster** to go up to 50,000×. The simulation drops to 10×
   a minute before each burn so you can watch it (*Simulation → Slow down
   before burns*).
3. **Correct.** **MCC** plans a mid-course correction: it trims pericynthion
   on the way out and the entry angle on the way home.
4. **Enter orbit.** **LOI** plans a retrograde SPS burn at pericynthion into
   a roughly circular lunar orbit.
5. **Come home.** **TEI** finds the cheapest burn back to the entry
   corridor, preferring a coast of under ~66 hours.
6. **Splash down.** Enter between −5.3° and −7.4° at the 400,000 ft
   interface. Shallower and you skip out; steeper and you burn up.

You can also plan by hand. **Add…** or double-click a burn to set its
ignition time (GET) and delta-v components:

- **prograde**: along the velocity
- **normal**: along the orbit's angular momentum
- **radial**: outward, perpendicular to the velocity

The components are measured relative to the Earth or to the Moon, whichever
sphere of influence you are in. The predicted path redraws as soon as you apply a change, so
experimenting is the fun part: try a free return with no LOI, or a burn that
slingshots you out of the Earth–Moon system.

Flight plans save and load as small text files (**Flight → Save flight
plan…**).

### The trajectory view

| View | What it's good for |
|---|---|
| Earth–Moon rotating | The classic Apollo figure-8. The Moon stays put on the right. |
| Earth inertial | The real shape of the orbits; the Moon moves along its orbit. |
| Moon-centred | Lunar arrival, orbit and departure. |

Mouse wheel zooms, dragging pans, and double-click fits. **Follow spacecraft**
keeps the CSM centred.

### The DSKY

The computer is fresh-started into P00 when a flight begins. Click the keys,
or type with the DSKY focused:

| Keyboard | DSKY key |
|---|---|
| `v` / `n` | VERB / NOUN |
| `0`–`9`, `+`, `-` | digits and signs |
| `Enter` or `e` | ENTR |
| `c` or Backspace | CLR |
| `p` (hold) | PRO |
| `k` | KEY REL |
| `r` | RSET |

Some things to try:

| Keys | Result |
|---|---|
| `V16N65E` | Mission clock; it tracks GET even at high warp |
| `V35E` | Lamp test |
| `V05N09E` | Program alarm codes |
| `V37E00E` | Back to P00 |
| `V16N36E` | AGC clock time |

The **Computer** menu has these too.

## What is (and isn't) simulated

- **Gravity:** Earth and Moon in the restricted three-body problem. The Moon
  is on a circular orbit in the Earth–Moon plane, and the indirect term is
  included. Integration uses an adaptive Dormand–Prince 5(4) method.
- **Vehicle:** CSM alone (no LM), 11.9 t dry with 18.4 t of SPS propellant
  (91 kN, Isp 314 s), plus the S-IVB for TLI (1 MN, Isp 421 s). The S-IVB
  separates at the first SPS burn.
- **Burns:** finite, with mass flow. Attitude is held in the local orbital
  frame, as the guidance steered at orbital rate.
- **The AGC:** a Block II emulator: ones-complement arithmetic, banked
  memory, editing registers, timers and interrupts, the DSKY channels,
  telemetry DOWNRUPTs, and the night-watchman / TC-trap / rupt-lock
  restarts. It passes the full AGC instruction self-test (the Validation
  suite from Virtual AGC) with zero errors, and runs Comanche 055 with no
  alarms or restarts over a 140-hour mission.
- **Not yet:** the AGC doesn't fly the spacecraft. It runs alongside the
  flight with its clock synchronized, but there is no IMU, accelerometer or
  engine coupling, so P30/P40 can't execute a burn. That's the next step:
  feeding PIPA counts and CDU angles in and reading the SPS commands out.
  Earth rotation, atmosphere flight after entry interface, the LM and RCS
  translation aren't modelled either.

## Building

On Haiku (R1/beta5 or newer, `gcc` from the `haiku_devel` package):

```sh
make
./Columbia
```

The headless tests run on any POSIX system with a C++17 compiler:

```sh
make test
```

They cover a full planned mission, the mission controller driven like the
UI, and a Comanche 055 boot. To also run the AGC instruction self-test:

```sh
tools/verify_rope.sh                  # rebuilds and verifies the ropes too
make test VALIDATION=objects/virtualagc/Validation/Validation.agc.bin
```

## Layout

```
core/agc     AGC emulator, DSKY decoding, embedded Comanche 055 rope
core/sim     flight dynamics, propagator, maneuver planner
core/        Mission: ties the flight, the plan and the computer together
app/         Haiku UI (BApplication, DSKY view, trajectory view, editor)
rope/        verified rope images and their provenance
tests/       headless tests
tools/       rope verification
```

## Credits and licence

Columbia is MIT licensed. The AGC emulator was written for Columbia from the
public instruction-set documentation; it contains no Virtual AGC (GPL) code.

The flight software was written by the MIT Instrumentation Laboratory for
NASA under Margaret Hamilton (Colossus) and is in the public domain. The
transcription and the verification tools come from the
[Virtual AGC project](https://www.ibiblio.org/apollo/) (Ron Burkey and
contributors) and the MIT Museum scans. See `rope/PROVENANCE.md`.
