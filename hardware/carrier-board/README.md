# Carrier Board — v2 power and USB consolidation

_One board replacing the XT60 and XT30 splitters, both DC-DC modules, the servo
PDB, the lidar USB adapter and the powered hub. The design target is that the
only cables left are the ones reaching hardware mounted somewhere else._

**Status: pre-schematic.** Nothing here has been drawn, fabricated or measured.
Every number is either carried over from a v1 measurement (marked as such) or an
estimate (marked as such). Treat the [Still open](#still-open) table as a
to-do list, not a specification.

A rendered view of this document, with the topology diagram, lives at the
artifact link in the project notes. **This file is canonical** — if the two
disagree, this one is right.

## Why this exists

The v1 car carries roughly 22 discrete interconnects. Most of them exist because
nobody made a board, not because anything needs to be 30 cm apart. The data
cables mostly do need to span distance; the power cables almost entirely do not.

This board eats the power domain and consolidates the data domain behind one
hub, taking the count to about 12 — which is close to the floor set by the
number of physically separate objects on the car.

| Deleted | Kept, because it reaches something |
|---|---|
| XT60 Y-splitter | Battery → board |
| XT30 Y-splitter | Board → VESC |
| Both DC-DC modules and their pigtails | Board → Pi (×2, see below) |
| Servo PDB | Board → servo |
| Lidar USB adapter + micro-USB cable | Board → lidar |
| Powered hub + its power and uplink cables | Board → camera |
| Voltage-tester leads | VESC → motor, sensor, USB, servo signal |

**The Pi takes two cables.** The Pi 5's USB-C port is power only and carries no
data, so the board reaches it with USB-C for the 5 V rail and USB-A for the hub
uplink. A design that assumes one USB-C does both will not work.

## The pack

Zero Point Energy Labs **SkyVolt 4S1P**, Molicel P50B cells.

| | SkyVolt P50B | Orryx P45B |
|---|---|---|
| Energy | **5.0 Ah / 72 Wh** | 4.5 Ah / 64.8 Wh |
| Voltage | 14.4 V nom, 16.8 V full | 14.4 V nom, 16.8 V full |
| Continuous | 60 A | 67.5 A |
| Burst | 120 A / 10 s | 90 A / 10 s |
| Weight | 290 g | 206 g — see below |
| Footprint | **43 × 43 mm, 77 mm long** | ~84 × 72 mm, 21 mm tall |
| Balance | JST-XH 5-pin | JST-XH 5-pin |
| BMS | none | none |

Chosen for three reasons, in order of weight.

**Its numbers reconcile.** Four 21700 cells weigh ~280 g bare, so the Orryx's
206 g is not physically possible, and 58 mm of stated width fits under three
21 mm cells rather than four. The SkyVolt's 43 × 43 mm is exactly a 2 × 2 stack
of 21700s and 290 g is four cells plus packaging. Where two products are
otherwise close, buy the one whose data you can trust. _If you want the flat
form factor, get corrected figures from the vendor first._

**More energy**, which is what funds the camera — see the power budget.

**Smaller plan-view footprint**, counterintuitively. The SkyVolt is twice as
tall but occupies 43 × 43 mm against roughly 84 × 72 mm. Floor area on the
chassis plate is the contested resource once the Pi, this board and a camera
mount are on it. Centre of gravity does not matter at 0.5 m/s.

Discharge ratings did not enter into the choice. Both packs are enormously
beyond a car running at `--max-duty 0.05`.

**Confirm the 43 mm height clears the chassis before committing to either.**
The chassis is being sourced at the same time, so this is one decision, not two.

### No BMS — and what that costs

Neither pack lists protection, and both ship a JST-XH balance lead, so they are
balance-charged exactly like LiPos. The balance charger still has to be bought;
the "Li-ion needs only a cheap CC/CV brick" argument does not apply to these.

**The UVLO is not optional.** Without a BMS, the only over-discharge protection
is the VESC's low-voltage cutoff — and the VESC only protects what it powers.
The accessory branch draws ~2.4 A straight off the pack, entirely independent of
it. Leave the car switched on after a session and the bucks will walk the pack
to zero while the VESC sits there having correctly cut its own output. On a LiPo
with a beeping alarm you would hear that happen; here nothing stops it.

_Rule:_ adjustable UVLO on the accessory branch, trip at **12.0 V** (3.0 V/cell),
with hysteresis. Set the VESC's own cutoff for Li-ion too, not the LiPo default.

## Power budget

Estimates except where marked. Replace them with measurements as they arrive.

| Rail | Load | Current | Source |
|---|---|---|---|
| +5V | Raspberry Pi 5 | 2.5 A typ, 3 A peak | Capped at 3 A by the Type-C advertisement |
| +5V | 360 camera | 0.7–2.0 A | Depends on charge state — see below |
| +5V | LD06 lidar | 0.18 A | **Measured**, `docs/hardware-baseline.md` |
| +5V | VESC USB | 0.10 A | 12 Mbps device |
| +5V | F710 dongle | 0.05 A | — |
| +5V | Hub IC + switches | 0.05 A | — |
| +5V | **total** | **~4.4 A** | 5 A module minimum, 6 A preferred |
| +6V | Steering servo | 0.2 A idle, 2 A stall | Own rail — stall current must not touch the Pi |
| VACC | Accessory branch | **~2.4 A** | 34 W at 14.4 V. Was 3.1 A on 3S |
| VBATT | VESC | 60 A cont, 120 A burst | Bounded by the pack, not guessed |

At 34 W worst case the 72 Wh pack gives about two hours of electronics before
the motor takes anything. Realistic average draw is nearer 20 W.

### The camera's internal battery

The widest variable on the rail. Figures are for an Insta360 ONE X2 — about
1630 mAh at 3.85 V, so ~6.3 Wh of cell and ~4–5 W operating. **Verify against
whatever actually gets mounted.**

| State | Draw at 5 V | What is happening |
|---|---|---|
| Charged, idle | 0.2–0.3 A | Charger terminated |
| Charged, streaming | 0.7–1.0 A | Operating power plus trickle top-up |
| Depleted, idle | 0.9–1.2 A | Constant-current charge phase |
| Depleted, streaming | **1.5–2.0 A** | Operating power *plus* full charge current |

A fully charged camera is not a zero-draw camera. Most action cameras run from
the cell even while plugged in, so streaming drains it slightly and the charger
re-engages.

Refilling the cell costs ~8.2 Wh from the pack once buck and charger losses are
counted — about 11% of 72 Wh. It would have been a quarter of the 3S LiPo this
replaced, which is a fair illustration of what the pack upgrade bought.

**The port decides the ceiling.** A USB device limits itself by the port type it
detects: a plain USB 2.0 data port caps it at 500 mA, and it takes BC1.2
signalling to unlock 1.5 A. The downstream ports here are plain data ports
unless charging-port signalling is deliberately implemented, so the 2 A row is
largely opt-in.

- Plain port, 500 mA — rail never at risk, but the camera net-drains while
  streaming and caps session length.
- CDP, 1.5 A — charging and data at once, camera runs as long as the car does,
  at ~1 A more load. _Check whether the chosen hub IC supports CDP downstream._

Size the rail for the worst case regardless. The failure mode of under-sizing is
not slow charging, it is rail sag that browns out the Pi mid-run.

**For the purpose-fit camera later:** the internal battery is the problem, not
the 360-ness. A bus-powered module with no cell — which is what the OAK-D was —
removes the charge current, the pack cost, the charge-state variability and the
"did anyone charge it" failure mode at once. Weight **no internal battery**
heavily as a selection criterion, arguably above resolution.

## Topology

Two ideas carry the board.

**Drive current never travels far.** Battery XT60 in and VESC XT60 out sit 15 mm
apart, joined by a short wide pour. The difficulty of carrying current in copper
scales with path length, so placing the connectors adjacent makes the hard
problem stop existing.

**Everything else hangs off one fused, UVLO-gated tap** off that pour, which is
also the only place current sensing is needed. Pack current comes from the VESC
over USB, so no high-current shunt exists on the board.

```
  battery ──[anti-spark]──> XT60_IN ══15mm pour══ XT60_OUT ──> VESC
                                          │
                                    [FUSE 5A + UVLO 12.0V]
                                          │
                            ┌─────────────┴─────────────┐
                       [BUCK 5V/5A]                [BUCK 6V/3A]
                            │                            │
                   ┌────────┴────────┐                   └──> servo
                   │                 │
              USB-C → Pi        [USB2517 hub] ──> camera, lidar UART,
              (power only)            │           VESC, F710, spare
                                 USB-A → Pi
                                 (all data)
```

## Net list

The schematic in table form. Transcribe top to bottom.

| Net | Class | Connects | Note |
|---|---|---|---|
| `VBATT` | power | J1.+ · J2.+ · F1.1 · D6 | **12.0–16.8 V. Design to 30 V** — the VESC pushes above pack voltage on regen. The pour *is* this net |
| `GND` | power | all | One pour on the bottom layer, unbroken under USB pairs |
| `VFUSED` | power | F1.2 · U8.VIN | Between fuse and UVLO switch |
| `VACC` | power | U8.VOUT · M1.VIN · M2.VIN · U3 shunt | Accessory branch, downstream of both |
| `UVLO_SET` | analog | U8.EN ← divider from VBATT | Trip at 12.0 V. Hysteresis so it cannot chatter |
| `+5V` | power | M1.VOUT · J3.VBUS · U4–U7.VIN · J9.1 · U1.VBUS | Main logic and USB rail |
| `+6V_SRV` | power | M2.VOUT · J11.2 | Servo rail, isolated from +5V by design |
| `+3V3` | power | U1.VDD33 · U2.VDD · U3.VS | Hub's internal LDO if rated; else add one |
| `USB_UP_DP/DM` | USB 2.0 | J4.D± ↔ U1.USBDP/DM | 90 Ω differential, length-matched. The one critical pair |
| `USB_DN1_DP/DM` | USB 2.0 | U1.P1 ↔ J5 | 360 camera |
| `USB_DN2_DP/DM` | USB 2.0 | U1.P2 ↔ U2.D± | Lidar UART bridge — internal, no connector |
| `USB_DN3_DP/DM` | USB 2.0 | U1.P3 ↔ J6 | VESC |
| `USB_DN4_DP/DM` | USB 2.0 | U1.P4 ↔ J7 | F710 dongle |
| `USB_DN5_DP/DM` | USB 2.0 | U1.P5 ↔ J8 | Spare — GPS, second lidar, whatever comes |
| `LIDAR_TX` | UART | J9.3 → U2.RXD | 230400 baud. LD06 transmits only |
| `LIDAR_PWM` | signal | J9.4 → TP1 | Test point, unconnected by default — **verify against the v1 adapter's behaviour** |
| `SERVO_SIG` | signal | J10.sig → J11.3 | Straight through from the VESC. 3.3 V logic; most servos accept it |
| `SDA`/`SCL` | I²C | U3 ↔ J13 | Telemetry pigtail to the Pi header. 4k7 pull-ups to +3V3 |
| `PRTPWR1–5` | control | U1 → U4–U7.EN | Per-port power switching — reboot a wedged sensor over SSH |
| `CC1`/`CC2` | Type-C | J3.CC1/CC2 → R1/R2 → +5V | **10 kΩ each** = 5 V/3 A advertisement. Verify against the spec table |
| `VBAT_SENSE` | analog | J12 → divider → U3 / TP2 | High impedance only. **Rescale for 16.8 V.** J12 is 5-pin |
| `XTAL1`/`XTAL2` | clock | U1 ↔ Y1 | 24 MHz. Follow the datasheet placement exactly |

Two nets that can hurt you:

**`VBAT_SENSE` is a sense input, never a supply.** High-impedance divider into an
ADC and nothing else. Pulling real current through one cell tap unbalances the
pack, and an unbalanced Li-ion pack is a hazard, not just a shorter run.

**Do not fuse `VBATT`.** A fuse that opens mid-run leaves an unpowered car at
speed, which is worse than the fault it prevents, and the VESC has its own
overcurrent protection. F1 protects the accessory branch — the least-proven
circuitry on the car — and nothing else.

## Connector schedule

| Ref | Connector | Dir | Goes to |
|---|---|---|---|
| J1 | XT60PW | in | Battery via inline anti-spark. **Order the pack with XT60**, and confirm the switch is rated for 4S |
| J2 | XT60PW | out | VESC — short 12–14 AWG silicone pigtail |
| J3 | USB-C, 16-pin | out | Pi USB-C — 5 V power only |
| J4 | USB-A | out | Pi USB-A — hub uplink, all data |
| J5–J8 | USB-A ×4 | out | Camera, VESC, F710 dongle, spare |
| J9 | JST-ZH 4-pin | out | LD06 — 5 V, GND, TX, PWM |
| J10 | JST-PH 3-pin | in | VESC servo signal |
| J11 | 3-pin 2.54 mm | out | Steering servo |
| J12 | JST-XH **5-pin** | in | 4S balance lead — sense only. Four taps plus ground |
| J13 | 4-pin JST | out | Pi I²C header — telemetry |

**All downstream USB is type A, deliberately.** USB-A has no CC negotiation, so a
self-powered port simply supplies VBUS from the rail — which is exactly what a
battery-powered camera needs, with nothing to configure. USB-C downstream would
put CC resistors on every port and create the genuine hazard of VBUS driven from
two directions. Type-C stays on the uplink only, where one pair of resistors
handles it.

## Bill of materials

Machine-readable, in JLCPCB import format: [`bom/bom.csv`](bom/bom.csv).

| Ref | Part | Package | Why this one |
|---|---|---|---|
| U1 | USB2517I | QFN-64 | **Multi-TT** — one translator per port. Single-TT lets VESC polling starve the lidar stream |
| U2 | CH340N | SOP-8 | Lidar UART bridge. CP2102N instead keeps the existing by-id path working verbatim |
| U3 | INA226 | VSSOP-10 | Accessory branch only — 10 mΩ at 2 A is comfortable. 36 V common-mode, so 4S is fine |
| U4–U7 | AP2553 or similar | SOT-23-5 | Per-port load switches, driven by the hub's port-power pins |
| U8 | Load switch / eFuse with UVLO | varies | **New for 4S.** Must survive 30 V and take an adjustable enable threshold |
| D1–D5 | USBLC6-2SC6 | SOT-23-6 | ESD on every downstream port. Non-optional on a vehicle |
| D6 | TVS, ~20 V standoff | SMB | **New for 4S.** Regen braking pushes VBATT above 16.8 V |
| M1 | 5 V / 5 A buck module | header | **36 V-class input, not 24 V** |
| M2 | 6 V / 3 A buck module | header | Servo rail. Same input requirement |
| F1 | Blade fuse holder, 5 A | TH | Accessory branch draws 2.4 A at 14.4 V |
| Y1 | 24 MHz crystal | 3225 | Hub reference clock |
| R1, R2 | 10 kΩ 1% | 0603 | Type-C Rp, 3 A advertisement |

**Why modules for the regulators.** Buck layout is the one place a correct
schematic still yields a dead or noisy board — the input capacitor loop has to be
physically tiny, and getting it wrong produces symptoms that are miserable to
chase without a scope. A module drops onto a header footprint and gives a
pre-tested, thermally characterised regulator.

This costs nothing against the goal: a module soldered to the board is a
component, not a cable. Both DC-DC pigtails, the XT30 splitter and the servo PDB
still disappear. Draw a discrete switcher in v3, once everything around it is
proven.

## High-current pour

| Quantity | Value | Working |
|---|---|---|
| Sheet resistance, 2 oz | 0.24 mΩ/sq | 70 µm copper |
| Pour geometry | 25 mm wide × 15 mm long | 0.6 squares |
| Path resistance | 0.14 mΩ | 0.6 × 0.24 mΩ |
| At 30 A (realistic peak) | 4 mV · 0.13 W | Negligible |
| At 60 A (pack continuous) | 9 mV · 0.52 W | Comfortable |
| At 120 A (pack burst, 10 s) | 17 mV · 2.1 W | The hard ceiling — the pack cannot exceed it |

The 4S pack improves this twice: higher voltage means less current for the same
power, and the pack's own burst rating is a *known ceiling* rather than the
open-ended guess a LiPo leaves.

Two reinforcements, both standard on RC power boards and nearly free:

- **2 oz outer copper** at order time — a few dollars on a 2-layer board.
- **Open the soldermask over the pour and flood it with solder**, or lay bare
  copper wire along it and solder it down. This stacks on top of the plating.

## Layout rules

- **Mechanical first, always.** Board outline, mount holes, and every connector
  placed facing the direction its cable leaves, before a single trace. The most
  common first-board respin is a connector pointing into the chassis wall.
- **Copy the hub datasheet's layout section verbatim.** Crystal placement and
  decoupling are prescribed. Not a place for judgment.
- **USB pairs: 90 Ω differential, length-matched, over unbroken ground.** Short,
  and away from the pour, the bucks and the servo rail.
- **Keep switching nodes away from the USB pairs.** Tens of amps are switching a
  few centimetres away; that is what the ESD parts and ground pour defend against.
- **All fine-pitch parts on one side.** Cheaper assembly. The XT60s, JSTs and
  headers are hand-soldered regardless.
- **Silkscreen device names, not net names** — `LD06`, `STEER SERVO`, `VESC SIG`,
  with pinout beside each. PH, ZH and XH look identical at arm's length in a
  parking lot.
- **Test points on every rail:** VBATT, VFUSED, VACC, +5V, +6V_SRV, +3V3, GND.
  You will want these during bring-up and cannot add them later.

## JLCPCB order settings

| Option | Set to | Why |
|---|---|---|
| Layers | 2 | Sufficient with a USB 2.0-only hub |
| Outer copper | **2 oz** | The one upgrade that matters here |
| Thickness | 1.6 mm | Rigidity for XT60 insertion force |
| Surface finish | HASL or ENIG | ENIG if hand-soldering the QFN |
| Assembly | Yes, one side | The QFN-64 hub is not realistically hand-solderable |
| Quantity | 5 | Minimum, and spares for bring-up mistakes |

Released gerbers go in [`fab/`](fab/) — see the convention note there.

## Pre-order checklist

Respins are almost never caused by the electrical design. Each of these has
caused one.

- [ ] **1:1 print test.** Print at 100% and lay every through-hole part on it.
- [ ] **Mount holes against the real chassis plate.** Measure it, don't trust a drawing.
- [ ] **Every connector faces the way its cable leaves**, with the board in its mounted orientation.
- [ ] **Every part on VBATT rated to 30 V** — bucks, caps, TVS, UVLO switch. The most likely 4S mistake is a 24 V part that passes bench testing and dies on the first regen brake.
- [ ] **J12 is a 5-pin XH**, not the 4-pin muscle memory draws on a 3S design.
- [ ] **UVLO divider maths checked twice**, with hysteresis. Trip 12.0 V, recover higher.
- [ ] **XT60 gender matches the pack** — order it with XT60 rather than XT90, and the always-live side gets the shrouded contacts.
- [ ] **Battery and VESC XT60s cannot be confused.** Key by gender or separate physically.
- [ ] **Hub self-powered strap correct.** The classic failure: 5 A of rail behind a hub the kernel still thinks is bus-powered, so devices get refused for no visible reason.
- [ ] **Every LCSC part checked for live stock**, not just existence.
- [ ] **Buck module pinout matches the footprint** — against the module actually bought.
- [ ] **DRC clean, then read the warnings you suppressed.**

## Bring-up

Staged the way `docs/junction-bringup.md` stages the car: one question per stage,
and a gate that passes before the next. **A pack touches this board at stage 5
and not before.**

| # | Stage | Gate |
|---|---|---|
| 1 | Bare board, no power. Continuity-check VBATT/+5V/+6V to GND; confirm the pour joins both XT60 pads | No short on any rail |
| 2 | Bench supply, current-limited, no modules fitted. 16.8 V at a 200 mA limit | Current near zero, nothing warms |
| 3 | **UVLO sweep**, still on the bench supply. Walk the input down from 16.8 V watching VACC | Drops out at 12.0 V ±0.2 V, stays off below, recovers without chattering |
| 4 | Fit the bucks. Bench supply, dummy loads at rated current, five minutes | +5V within 4.9–5.1 V under 4 A, +6V steady, nothing above 60 °C |
| 5 | Pack, no Pi. Battery in through the anti-spark | Rails match the bench readings; anti-spark handles 16.8 V inrush without arcing |
| 6 | Pi only. USB-C power and USB-A data, nothing else | Pi boots; `lsusb -t` shows the hub; `lsusb -v \| grep -i "self.powered"` confirms self-powered |
| 7 | Sensors one at a time. Lidar first — it is the only one with a numeric baseline | `lidar_view.py --selftest` returns ~19 KB/s, ~400 frames/s, 9.9–10.1 Hz. Re-run after each addition |
| 8 | Servo and VESC, wheels off the ground. `--steer-only` on a stand. Set the VESC cutoff for Li-ion first | Servo sweeps full travel without +5V moving; VBATT stays under 20 V on regen |
| 9 | **Recalibrate**, then a full run | `vcgencmd get_throttled` returns `0x0`, EXT5V above 4.8 V, measured speed matches what the route tuning expects |

Stage 3 exists because it protects an unprotected pack, and stage 9 exists
because the pack voltage changed underneath every tuned constant — see below.

## Still open

Nothing in this table is verified. Each entry can invalidate a decision above.

| Question | Blocks | How to close it |
|---|---|---|
| **Camera streams on Linux?** | the whole architecture | Open it in OpenCV on the Pi. If it needs vendor software, consider a Ricoh Theta V/Z1 with `libuvc-theta` instead |
| **VESC max input voltage** | whether 4S works at all | Check when buying one. Nearly all are 8–60 V, so a formality — but do it before ordering the pack |
| **Speed recalibration** | every tuned constant | 14.4 V against 11.1 V spins the motor ~1.3× faster at the same duty. `DUTY_TO_MPS` and `--lookahead 0.8` both move. Dropping max duty to ~0.038 lands back at v1's measured 0.42 m/s |
| **Pi rail under real load** | the 3 A Type-C decision | Step 4 of the re-verification procedure in `docs/hardware-baseline.md`, still never run |
| Hub CDP support | camera session length | Datasheet — decides whether the camera sustains itself or drains while streaming |
| VESC stall current | margin, future BMS | pyvesc `GetValues.avg_input_current`, wheels locked |
| Anti-spark 4S rating | J1 branch | Most modules cover 3–12S; confirm before reusing the v1 part |
| Orryx dimensions/weight | pack choice | Listed figures are not physically possible. Ask the vendor if the flat form factor is wanted |
| LD06 PWM pin | J9 wiring | Check whether the v1 adapter drives it or leaves it floating |
| Chassis geometry | the whole software stack | A different wheelbase invalidates the measured 0.3302 m, the 0.127 m lidar plane and the 142° occlusion arc. Sourcing a chassis close to the class platform keeps those constants |

## Not electrical, but coupled

**The v3 weights will not transfer.** They were trained on OAK-D frames at a
specific framing and white balance, and `model/capture/oakd.py` is emphatic that
inference must match what training saw. A de-warped 360 view is a different image
distribution entirely. Budget a full dataset rebuild alongside the board — it is
the largest cost of the v2 camera change and none of it is on this page.

**A mirrored servo is mechanical.** `--invert-steering` is in the mandatory flag
list for v1. A reversed 3-pin servo lead does not invert direction, it stops the
servo working, so the mirroring is almost certainly servo orientation or linkage
side. Copper cannot correct it; check the linkage before assuming the flag
carries over.
