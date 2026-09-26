# MotoBat — Battery Module Specification (VDA 355, PHEV2 cells)

**One question this file answers: what exactly are we building, part by part, and how sure is
each number?**

This is the geometry reference for the Fusion 360 parts and for `defectgen`'s module generator.
Every dimension is tagged:

| Tag | Meaning |
|---|---|
| **S** | Sourced — from a datasheet, standard or paper (link given) |
| **D** | Derived — arithmetic from sourced numbers (working shown) |
| **A** | Assumed — a reasonable engineering value; replace when a better source appears |
| **M** | Measure — take it from a real-cell photo in Fusion (method in §6) |

Units: millimetres. Last updated 15 Sep 2026.

---

## 1. What we are building

A **VDA 355 module**: the standard German/Chinese automotive module envelope, holding twelve VDA
PHEV2-format prismatic cells. The target look is the reference image in the chat of 15 Sep: cells
in a row, finned end plates, side plates, a bottom liquid-cooling plate, a busbar carrier
("support structure"), busbars, a cell module controller, and a cover.

> The **cell module controller** (CMC, also called CSC) is the part of the battery management
> system that lives on the module: it measures each cell group's voltage through the busbars and
> reads temperature sensors, then reports to the pack BMS.

---

## 2. Module envelope — VDA 355

| Parameter | Value | Tag | Source / working |
|---|---|---|---|
| Outer dimensions (L × W × H) | 355 × 151 × 108 | S | [LYTH — VDA355 & MEB590](https://lythbattery.com/what-is-vda355-and-meb590-module%EF%BC%9F/), [DIY Solar Forum](https://diysolarforum.com/threads/has-anyone-considered-building-a-battery-with-vda-355-modules.116912/) |
| Cells | 12 | S | same |
| Configuration | **3P4S** (3 parallel × 4 series) | S | Real CALB VDA355 module: 3P4S, 174 Ah, 14.68 V ([LYTH](https://lythbattery.com/calb-3p4s-58ah-vda355-14-68v-174ah-ev-ncm-module/)). 4S3P and 3S4P are the common real configs. |
| Mass | ≈ 11 kg | S | DIY Solar Forum thread |
| End-plate preload | ≈ 3 kN initial | S | [batterydesign.net — prismatic cells](https://www.batterydesign.net/battery-cell/formats/prismatic-cells/) |
| Length budget for end plates + pads | 355 − 12 × 26.5 = **37** | D | 2 end plates + 11 interface pads |
| Width budget for side plates | 151 − 148 = **3** → ≈ 1.5 per side | D | thin steel/Al side plates |
| Height budget above/below cell can | 108 − 91 = **17** | D | base/cooling interface + terminals + carrier + cover |

**Why 3P4S and not 12S1P:** 3P4S is what real VDA 355 modules use. It also makes the busbar
pattern non-trivial (groups of three cells flip orientation), so a reversed cell is a meaningful
defect.

---

## 3. Cell — VDA PHEV2

| Parameter | Value | Tag | Source / working |
|---|---|---|---|
| Can width × thickness × height | **148.0 × 26.5 × 91.0** | S | [batterydesign.net — VDA prismatic](https://www.batterydesign.net/vda-prismatic-cells/) |
| Real example | CATL NCM27E892-50Ah: 148 × 27 × 92 can; listed 97–98 with terminals; 0.85 kg; 50 Ah; 3.65 V | S | [evlithium](https://www.evlithium.com/nmc-battery/catl-50ah-nmc-battery-cell.html), [dlcpo](https://dlcpo.com/catl-50ah-ncm-battery-cell-prismatic-ternary/) |
| Dimensional tolerance | ±0.5 | S | vendor listings above |
| Can material / wall | Aluminium, ≈ 1.1 | S | batterydesign.net |
| Insulation film | blue PET wrap, ≈ 0.1 | A | visual (the blue in the reference image); negligible geometrically |
| Mass | ≈ 0.85 | S | CATL example |

> **Caveat from batterydesign.net:** "even if it states they are VDA prismatic cells, check the
> dimensions on the specification sheet and on the actual cell" — manufacturers vary terminal
> height and tab thickness inside the same format. That is why the terminal is fully parametric.

---

## 4. Terminals (the most important part for inspection)

The pre-weld inspection measures terminal-top height to **±0.1 mm between adjacent terminals**
(MDPI *Batteries* 6(3):47 via search — confirm from the paper). So the terminal is modelled
exactly, but its dimensions are parameters.

| Parameter | Value | Tag | Source / working |
|---|---|---|---|
| Type | laser-weld pad (flat top, no stud) | S | module lines weld busbars to pads ([IPG](https://www.ipgphotonics.com/solutions/e-mobility-manufacturing-solutions/battery-module-busbar-welding)) |
| Material | + Al; − Al (Cu–Al composite inside the cell) | S | [NB Cell — terminal comparison](https://nbcellenergy.com/lifepo4-prismatic-battery-cell-terminal-comparison/) |
| Terminal pad thickness | ≈ 4 | S | weld study: 1.5 mm 1060 Al busbar on a 4 mm 1060 Al terminal ([ResearchGate](https://www.researchgate.net/publication/379641129_A_modelling_approach_for_laser_welding_of_busbars_to_lithium-ion_prismatic_cell_terminals_to_enhance_failure_prediction)) |
| Height above cap | **≈ 5–7** (default 6) | D | CATL listed 92 (can) vs 97–98 (with terminals) |
| Pad footprint | rectangular 21.5 × 17.5 (default) or 21.5 × 13.5; round Ø12–18 | S/A | NB Cell guide (LFP cells — representative, not PHEV2-specific) |
| Centre-to-centre spacing | **unknown — measure** | M | Web figures conflict (66 vs 76 mm) and come from an unreliable page. Measure from a photo (§6). |
| Polarity distinction | + and − differ in marking/colour; assume a small geometric difference (e.g. a chamfer on −) | A | so polarity is detectable in 3D before the 2D camera exists |
| Vent | burst disc, centre of cap | S (location) / M (size) | measure from a photo |

---

## 5. The rest of the module

| Part | Key dimensions | Tag | Notes |
|---|---|---|---|
| **End plates** (×2, finned — "passive cooling structure") | 151 × 108 footprint, ≈ 12–15 thick incl. fins | D/A | must fit the 37 mm length budget with 11 pads |
| **Interface pads** (×11, between cells) | 148 × 91, ≈ 0.6–1.0 thick | D/A | (37 − 2 × 14) / 11 ≈ 0.8 |
| **Side plates** (×2) | ≈ 355 × ~90 × 1.5 | D | welded to end plates after compression |
| **Bottom cooling plate** ("liquid cooling structure") | 355 × 151, ≈ 3–5 thick, internal channels | A | channels matter only visually; the thermal gap-filler bead on top of it is a defect class |
| **Thermal gap filler bead** | bead width ≈ 5–8, height ≈ 2 before compression | A | adhesive-bead defect target |
| **Busbar carrier** ("support structure", CCS frame) | ≈ 340 × 140 plastic frame, ≈ 3–5 thick, windows over each terminal | A | industry usually supplies carrier + busbars + sense wiring as **one subassembly** (cell contacting system) |
| **Busbars** | 1060 Al, **1.5 thick** | S | weld study above |
| — busbar pattern (3P4S) | end busbars span 3 terminals; series busbars span 6 terminals (3 + 3) | D | computed from config by the generator |
| — busbar width | ≈ 20–25 (≥ pad width) | A | |
| **Laser weld seams** | one seam per busbar–terminal joint; pattern: closed loop or 2–3 linear stitches ≈ 10–15 long, ≈ 1–1.5 wide | A | post-weld inspection target |
| **Cell module controller** (PCB) | ≈ 250 × 60 × 1.6 + components | A | sits on the carrier; sense tabs to each busbar |
| **Cover** | 355 × 151, ≈ 1.5–2 | A | plastic or Al |

**Height check (D):** base/cooling ≈ 4 + can 91 + terminal 6 + carrier/busbar ≈ 4 + CMC + cover
≈ 3 ≈ **108**. Consistent with the VDA 355 envelope.

---

## 6. How to measure the **M** values in Fusion 360

Without a manufacturer drawing, calibrate a real photo against a known dimension:

1. Find a straight-on photo of the top (cap) of a PHEV2-class cell (e.g. the CATL 50 Ah product
   photos linked above). Top-down, as little perspective as possible.
2. Fusion: **Insert → Canvas**, place it on the XY plane.
3. Right-click the canvas → **Calibrate**, click the two ends of the cell's long edge, enter
   **148 mm**.
4. Sketch over it: terminal centres, pad size, vent outline, distances to the edges.
5. Cross-check with a second photo, or the side view against the 91 mm height. If two photos
   disagree by more than ~1 mm, average them and tag the value **M (±1 mm)**.

That accuracy is plenty. The detector measures *relative* heights (terminal vs neighbouring
terminal), so exact absolute pad size barely matters to the results.

---

## 7. Fusion → `defectgen` handoff

**One source of truth for numbers:** every value above becomes a **Fusion User Parameter** with
the same name as the generator's config key (e.g. `cell_w = 148`, `terminal_h = 6`). They live in
`defectgen/configs/vda355_3p4s.yaml` when code starts. Change a number in one, change it in both.

**Export conventions** (per part, so the generator can assemble them):

| Rule | Value |
|---|---|
| Format | STEP (archive) + STL (generator / Gazebo) |
| Units | millimetres in the file; the generator converts to metres for Gazebo |
| Axes | Z up; X along the module length (cell stacking direction); Y across |
| Cell origin | centre of the can's bottom face |
| Other parts | origin at a named datum written next to the file (e.g. `busbar_series: origin at centre of underside`) |
| One body per file | cell can, terminal (+), terminal (−), pad, end plate, side plate, cooling plate, carrier, busbar (end), busbar (series), CMC, cover |

The generator places these meshes with one transform per instance, and **injects defects mostly
as transform perturbations** (cell pose, terminal height, busbar lift-off). Weld seams and spatter
are generated geometry. Until the Fusion parts exist, the generator uses its own box primitives
with the same parameters, so code does not wait on CAD.

---

## 8. Assembly sequence — start to end (what the robots do)

Modelled on a real prismatic module line: stacking → strapping → terminal positioning → busbar
placement → laser welding → end-of-line test
([line process](https://www.linkedin.com/pulse/prismatic-ev-lithium-battery-module-pack-assembly-line-sam-lau)).

| # | Step | Robot / station | Inspection after it |
|---|---|---|---|
| 1 | Cooling plate into fixture; dispense gap-filler bead | fixture + dispenser (simulated) | **Fixed Ranger3**: bead continuity and width |
| 2 | End plate A into fixture | GP7 | — |
| 3 | Cells 1–12 with pads, orientation per 3P4S pattern | **GP7**, cells located by overhead 2D camera | per-cell placement check → **correction loop, k = cell index** |
| 4 | End plate B, compress to ≈ 3 kN | GP7 + fixture | — |
| 5 | Side plates placed and welded | GP7 places, GP4 welds (simulated) | — |
| 6 | **Pre-carrier inspection** | **HC10DT multi-view** vs **fixed Ranger3** | cell pose, terminal height and tilt, polarity, missing cell |
| 7 | Busbar carrier (CCS subassembly) placed | GP4 | — |
| 8 | **Pre-weld inspection** | HC10DT | busbar pose, busbar-to-terminal gap |
| 9 | Laser weld every busbar–terminal joint | **GP4** (simulated weld head) | — |
| 10 | **Post-weld inspection** | HC10DT | seam continuity, width, spatter |
| 11 | Cell module controller placed, cover placed | GP4 | final visual (2D) |
| 12 | End-of-line electrical test | simulated pass/fail | — |

Reject/rework decisions happen at 6 and 8, **before the irreversible weld at 9**.

---

## 9. What to model in CAD, and at what fidelity

**The rule: model a thing if it can be hit, can block a sensor's view, or gets
measured. Everything else is a box, or is left out.**

Occlusion is the project's central question (§3.7.6 of the plan), so anything
that casts a shadow into a sensor's field of view has to exist in the scene —
but only as a shape, not as detail.

| Thing | Visual mesh | Collision | Notes |
|---|---|---|---|
| Module parts (§3–5) | your CAD, decimated | convex boxes | the only parts that get *measured* |
| Fixture / nest / pallet | simple CAD | boxes | defines where the part is; sets the 26V188-style contact damage |
| Conveyor | boxes | boxes | a frame, two side rails, a belt surface |
| Sensor bodies (Zivid, profilometer, Ranger3, cameras) | **box at datasheet outline** | same box | must exist: they occlude and they collide |
| Sensor bridge over the conveyor | boxes / extrusion profile | boxes | a fixed frame, **not** a motorised gantry |
| Robot grippers and weld head | simple CAD | convex hull | these occlude the arm's own view — the classic reason a view plan fails |
| End stops, clamps, cabling | omit or box | boxes | only where they intrude on a view |
| Robots | vendor URDF meshes | vendor collision | already in `src/_vendor` |

**Do not** put CAD-quality meshes in collision geometry. MoveIt and Gazebo both
want convex primitives; a raw STEP tessellation there will make planning crawl.
Keep triangle counts modest in visual meshes too (decimate in Blender or
`trimesh.simplify`), since the ray-cast sensor models in `defectgen` intersect
against them.

**Sensors: model the body, not the optics.** A floating frame would render
faster, but it would silently remove the housing that blocks a view, and
"where can the sensor actually see from" is the result we are after. A box with
the datasheet's outline and the optical origin placed correctly is enough.

---

## 10. Fixture, datums and pick features

This is the part that makes the line read as real automation rather than a
render. Worth modelling properly, and it is cheap.

**Locating scheme — 3-2-1 on the pallet.** Three pads define the primary plane
(Z, tip, tilt), two dowel pins define X/Y and rotation (one round pin, one
diamond pin so the part is not over-constrained), and one stop defines the last
translation. Every measurement in `defectgen` is expressed in the **pallet
datum frame**, not the world, which mirrors how a real cell reports results.

**Pick features to add to your parts:**

| Part | Pick method | Feature to model |
|---|---|---|
| Prismatic cell | vacuum on the 148 × 91 face, backed up by side clamping | flat, unobstructed pad area; keep the label/vent clear |
| Compression pad | vacuum, small pads | flat top face |
| End plate | vacuum or magnetic (if steel) | flat outer face |
| Side plate | vacuum, two pads | flat outer face |
| Busbar carrier (CCS) | vacuum on the frame's flat ribs + 2 locating pin holes | **2 pin holes**, Ø 6 (A), so the gripper repeats |
| Cover | vacuum, four pads | flat top |
| Cooling plate | vacuum | flat top, clear of the bead path |

Also define, per part, a **TCP offset** (where the gripper's tool point sits
relative to the part origin) and a **grip approach direction**. Both go in the
same YAML as the geometry, so the generator, MoveIt and your CAD agree.

**Why this matters beyond realism:** the nest clearance and the pin engagement
set the threshold at which a mis-placed part *touches tooling* — the mechanism
behind NHTSA 26V188 (plan §3.7.10, experiment §4B.9). Without a modelled nest,
that coupling has no geometry behind it.

---

## 11. Line layout: stations, conveyor, and why the part must move

**The module rides a pallet on a conveyor and indexes between stations.**
This is not decoration:

1. **The fixed line-scan baseline needs relative motion.** A Ranger3 builds a
   3D surface from successive profiles; with a stationary part it measures one
   line. Without a conveyor there is no fixed-sensor baseline, and the central
   comparison of §3.7.6 cannot be run.
2. **Cycle time needs a takt.** "Seconds per module" only means something if
   transport is in the budget alongside capture and motion.
3. It is how real prismatic module lines work.

**Simulate it cheaply.** Do not attempt friction-driven belt physics in Gazebo —
it is a well-known time sink. Attach the pallet to a **prismatic joint** along
the line and drive it with a position controller, or script the pose directly.
The belt surface is visual only.

**Station layout (matches the assembly sequence in §8):**

| Station | Contents | Sensors |
|---|---|---|
| S1 Cell placing | GP7, cell magazine/tray, pallet nest | overhead 2D camera (cell locating) |
| S2 Line-scan gate | a fixed bridge the pallet passes under | **Ranger3 + laser** (the baseline) |
| S3 Inspection cell | HC10DT, pallet stop, safety scanner | wrist: snapshot 3D + profilometer + 2D |
| S4 Busbar + weld | GP4, carrier magazine, weld head | (weld monitoring out of scope, §4B.2) |
| S5 Final | cover placing, EOL test stub | 2D camera |

S2 and S3 inspect the **same** module at the same stage, which is what makes
the fixed-vs-arm comparison a controlled experiment rather than two anecdotes.

---

## 12. Still open

- Terminal spacing, pad size, vent size: **measure (§6)**.
- The 0.1 mm adjacent-terminal tolerance: confirm from MDPI *Batteries* 6(3):47 directly.
- Busbar width, weld seam pattern, carrier and CMC dimensions: assumed, refine from photos of real
  VDA 355 modules if useful.
- A module-line cycle time for the fixed-vs-arm comparison: not found yet.

**Not found anywhere free:** a CAD model of a PHEV2 cell or VDA 355 module. batterydesign.net's
paid downloads are spreadsheets (cell/module/pack databases, calculators), **not CAD**.
