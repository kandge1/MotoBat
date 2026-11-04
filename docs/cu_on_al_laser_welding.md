# Cu-on-Al laser welding — reference digest

*Filed 4 Nov 2026 from an external research guide (compiled the same day). The links were not
rechecked when this was filed. This file keeps what bears on MotoBat and says why. It is a
reference, not a work plan; the tasks it feeds are in `status_update.md` (QUEUED items 1–3).*

Context: the busbars are **copper, lap-welded through the copper onto aluminium terminal pads**
(`module_spec.md` §5, "Busbar material"). Copper on top is the harder stack order. Do not carry
Al-on-Cu parameters across without checking them.

---

## 1. Caveat before using any number below

**Every published Cu-on-Al study here welds foils: 0.2 mm Cu on 0.3 mm Al, or 0.3 / 0.3.** That is
the cell-tab-to-busbar regime. MotoBat's joint is a **1.5 mm Cu busbar on a 4 mm Al pad**, which
is 5–7× thicker on top. None of these sources gives a procedure for that joint. Use them for
**mechanisms and qualitative trends**, not for thresholds. The weld limits stay *to source*.

## 2. What we can use

| # | Finding | Source | Where it lands in MotoBat |
|---|---|---|---|
| 1 | **Stitched linear seams** gave the most stable results of the strategies tested (stitched, circular oscillation, figure-eight). Stitches were 3 mm lines at 1 mm spacing, on foil. | Kaufmann 2023 | Supports the spec's choice of linear stitches (`module_spec.md` §5 weld note). The foil stitch geometry does not carry over to us. |
| 2 | With Cu on top, **heating up to the interface tends to overheat the Al below it**: the Al-on-Cu wobble strategy burned through the lower Al. The fix was a small melt pool and fast processing: 9 concentric rings in a spiral, two passes, ≈ 0.5 s. | Kaiser 2020 (TRUMPF) | Supports QUEUED item 3's **one cause, two symptoms** idea: excess heat shows as Cu tint on top and as over-melting of the Al below. Our 4 mm pad will not burn through the way a 0.3 mm foil does. Excess heat shows as deep Al melting and mixing (IMC growth) instead, which is internal and so falls under out-of-scope defects 16/17. |
| 3 | **Robot motion, scanner motion and laser timing are three separate functions.** The robot places the head. Galvo mirrors draw the fast pattern (up to 250 Hz). The laser is modulated in sync with them. The robot wrist does not trace a published wobble. | both papers; Yaskawa application page | **For the GP4 welder:** its trajectory moves to the weld pose, holds there (or moves at a steady speed), and the scanner draws the seam. In defectgen, the seam pattern is the scanner's output, not a robot path. This split also makes a clear talking point for controls/automation readers. |
| 4 | **Neither study gives a gap tolerance.** Kaufmann clamped to a nominal zero gap with a top beam. | Kaufmann 2023 | QUEUED item 1 (gap → weld outcome) is **still unsourced**. These papers do not close it. |
| 5 | Process windows reported for one setup: **±20 W power, ±0.25 mm Z (focus)**. This was a single-mode laser with a ≈ 40 µm spot at 520 W, on foil. | Kaiser 2020 | Gives only an order of magnitude: a few tenths of a mm of height error matters to the weld. It is consistent with pre-weld height inspection being worthwhile. It is **not** a limit for us (different spot, power and thickness). |
| 6 | **IMC thickness limits are not interchangeable across studies.** One cited figure is embrittlement above ≈ 5 µm; another reports a zone below 20 µm that was acceptable in its own structure. | Kaufmann 2023; Kaiser 2020; e-JWJ review | Keep the spec's "sources disagree" wording. IMC is internal, so it never becomes a surface label. |
| 7 | **The top-view appearance does not show interface quality or electrical quality.** Both studies needed sections, tensile tests and four-wire resistance to judge the joint. | both papers | Backs the limitation already stated for defects 16/17 (`project_plan.md` §4B.2): surface scans catch the causes (gap, height) and the surface symptoms, not the joint quality itself. |
| 8 | **Green (515 nm) light couples into Cu much better than near-IR.** Spot size and trajectory still matter as much as power, so comparing papers by watts alone is misleading. | Kaufmann 2023; review | Matches the spec's "Costs accepted" note. The simulation has no laser-source model. |

## 3. GP4 payload — checked 4 Nov 2026: the GP4 cannot carry a welding head

> **Resolved 4 Nov 2026: option (a), GP25 + MOTOPAC-RL2D300-GP25JP** (`project_plan.md` §4A.7).
> Yaskawa's own package: an 18.3 kg galvo scanner head (15.8 + 2.5 kg air knife) on a 25 kg GP25,
> 8 kW fibre laser at 1050–1090 nm, 320 µm spot, 25 mm field, wobble up to 500 Hz with power
> modulated within the circle. Its stated purpose, "a wide gap tolerance for workpieces with
> misalignments or gaps", is the gap problem in `weld.py`. The GA50 was considered and rejected.
> It is a 50 kg path-accuracy arm for large parts where the robot traces the seam, and here the
> scanner does.

Yaskawa's laser-welding package is built on a **GP25** (*MOTOPAC-RL2D300-GP25JP*, YRC1000,
doc no. 194166-1CD). Scanner head masses, from vendor datasheets:

| Head | Type | Mass |
|---|---|---|
| TRUMPF PFO 33 (the head in Kaufmann 2023) | 3D programmable focusing optics | < 35 kg (< 38 kg with OCT) |
| TRUMPF PFO SF | 2D small-field scanner | ≈ 13 kg |
| Coherent 2DWeld | 2D scanner | < 20 kg |
| SCANLAB intelliWELD II PR | 3D scanner | 21–37 kg |
| FANUC FiberSCAN CRW | compact 2D remote head, the lightest found | 5.5 kg |

**Every one exceeds the GP4's 4 kg payload,** and that is before the fibre, the cable dress and the
gas line. Option (b) below assumed a compact head the GP4 could carry. It does not hold for any
scanner found. The write-up needs one of these:

- (a) **a larger welding arm.** A GP12 or GP25 matches Yaskawa's own package. Say so in the line
  layout.
- (b) **fixed optics on the arm, scanner off it.** The arm only presents the part, or a gantry
  carries the head. This changes the cell layout.
- (c) **keep the GP4 and state that it stands in for a larger arm.** This is the weakest option
  for a controls and automation audience.

defectgen is unaffected: the bead comes from a beam trace, whichever device draws it. Finding 3
holds either way. In the write-up, the trace's drift terms read as *robot pose error during the
hold*, and its fast pattern as *scanner output*.

Sources: [TRUMPF PFO 33 flyer](https://Www.trumpf.com/filestorage/TRUMPF_Master/Products/Lasers/02_Brochures/TRUMPF-PFO-33-flyer-DE.pdf),
[TRUMPF PFO SF flyer](https://www.trumpf.com/filestorage/TRUMPF_Master/Products/Lasers/02_Brochures/TRUMPF-PFO-SF-flyer-EN.pdf),
[Coherent 2DWeld datasheet](https://www.coherent.com/resources/datasheet/components-and-accessories/2dweld_scanner_datasheet.pdf),
[SCANLAB intelliWELD II PR](https://scanlab.de/en/products/scan-systems/intelliweld/intelliweld-ii-pr),
[FANUC Remote Laser System catalogue](https://fanuc.co.jp/en/product/catalog/pdf/robot/RemoteLaserSystem(E)-01a.pdf).

## 4. Seen and set aside (scope creep for now)

- **Yaskawa manuals**: the remote-laser function manual and the MOTOPAC manual. Only needed for a
  real cell; MotoBat simulates.
- **Zenodo [14841254](https://zenodo.org/records/14841254)**: bead width, interface width and
  penetration against power and speed, for 0.3 / 0.3 mm Cu–Al foils. The stack order and schema
  are unverified. It is a *possible* calibration for the bead-width variation in QUEUED item 2,
  but it is foil, so park it.
- **Zenodo [12726375](https://zenodo.org/records/12726375)**: a wobble process window for foils.
  Its contents were not retrieved.
- **The proposed cell-recording schema** in the guide. defectgen's ground truth already covers the
  parts that a surface-only study can observe.
- Steel–Cu dataset, surface-roughness paper, *Handbook of Laser Welding Technologies*.
- **No public dataset combines** Cu-on-Al welds, Yaskawa trajectories, process signals and defect
  labels. This is a small but citable gap that MotoBat's synthetic labels partly address. Do not
  overclaim it.

## 5. Sources

- Kaiser et al., *The challenge of welding mixed material: strategies for joining copper and
  aluminum with a single mode laser*, LANE 2020 (TRUMPF) —
  [PDF](https://s9fdb5f62abe7a148.jimcontent.com/download/version/1599142082/module/12811882649/name/LANE2020_1122_Kaiser.pdf).
  2 kW single-mode fibre laser, PFO 20-2, ≈ 40 µm spot. Cu-on-Al: 9-ring spiral, 2 passes,
  520 W. Tensile > 200 N. Conductance was better than the bolted joints it was compared with.
- Kaufmann et al., *Seam properties of overlap welding strategies from copper to aluminum using
  green laser radiation for battery tab connections in electric vehicles*, Materials 16(3):1069,
  2023, [doi:10.3390/ma16031069](https://pmc.ncbi.nlm.nih.gov/articles/PMC9921288/). 0.2 mm
  Ni-plated (2.5 µm) Cu-ETP on 0.3 mm Al. TruDisk 3022 at 515 nm, PFO 33-2, 342.5 µm spot,
  zero-gap clamp. Example settings in its Table 4: stitched 1400 W at 400 mm/s and 1600 W at
  500 mm/s. These are research examples, not a procedure.
- *Review on laser welding technology on aluminum and copper dissimilar metals*,
  [e-JWJ](https://www.e-jwj.org/journal/view.php?number=2032318).
- Yaskawa — [laser welding application page](https://www.yaskawa.eu.com/application/type/application/laser-welding_a11050)
  (GP series, GA50, MC2000; robot-carried heads vs remote scanners);
  [product documentation portal](https://www.motoman.com/en-us/service-training/product-documentation)
  (search 194166-1CD).
