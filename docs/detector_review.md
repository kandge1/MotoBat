# Detector review: how ours compares, and what to take to the next level

16 Nov 2026. This covers why the detector is hand-written geometry rather than ML, what Path Robotics
and the literature do, and where MotoBat is behind, ranked. Each gap has an upgrade that can run beside
dataset 2. Related: [welding_followup.md](welding_followup.md) §4 (Path Robotics, open datasets),
[fixlist.md](fixlist.md) F10, plan §4B.

**How this was researched.** About 40 searches and page reads: papers, standards, patents, vendor and
company pages. That is not thousands. It is the set that turned up directly relevant work, each source
followed to its own page. Every source below was opened. Where the publisher blocked reading
(MDPI, ScienceDirect, PMC), the title, authors, journal and year were confirmed through Crossref
and the claims come from the abstract; the **Read** column says which.

---

## 1. What kind of detector this is

`detect.py` is a **CAD-referenced measuring gauge with an anomaly pass**:

- Each scan is compared with a noise-free "golden" scan of the CAD model, cast from the same sensor
  pose.
- Positions, heights, gaps and bead dimensions are measured as differences and judged against
  tolerances.
- Rays that disagree with golden are grouped into regions, and hand-written shape rules classify
  them (FOD, dent, spatter, anomaly).

It is not trained: thresholds are set in `detect.yaml`. The dataset does not teach it. The dataset
*measures* it (POD, false alarms, %GRR), the way a test bench measures a device.

## 2. Why not ML, and where ML does belong

**Measurement stays geometric.**
- A tolerance decision needs a number in mm with a known uncertainty: a gauge study (%GRR) and a
  decision rule (§4, G5) both run on measurements.
- A learned classifier gives a score. There is no %GRR of a score.
- The deep-learning battery-weld papers below report accuracy, miss rate and false-alarm rate, not
  measurement uncertainty. Wang 2023 reports weld width in pixels, uncalibrated, with an 11.5 %
  false-alarm rate on 1,268 production images [wang23].

**On 3D data, classical features hold their own.**
- On the MVTec 3D-AD benchmark, a 3D-only method with no deep learning and no pretraining beat the
  learned 3D methods of the time. Rotation invariance mattered most. Color-only methods beat the 3D
  ones [btf].
- Real3D-AD (2023) built its high-precision point-cloud benchmark because there were "no baseline
  methods" for that setting [real3d].
- In 3D weld inspection specifically, classical methods appear:
  - an aluminium-weld paper on laser line scanning uses a classical method and notes that deep
    learning needs large labelled sets [huang25];
  - a WAAM bead paper uses curvature [liu23].

**Path Robotics puts ML around the measurement, not in it.**
- In US 12,358,138 B2, the robot measures the weld it made, as a scan, mesh or point cloud. It
  compares that with a reference weld profile to get a *difference profile*. ML logic then learns
  how to update the weld parameters from that difference. The difference is geometry; the learned
  part is the process inverse [path138].
- That is the shape of our correction loop (1.12), with ML where the physics is unknown.
- In sensing, Path says "neural-network filtering rejects false reflections" in its laser stack
  [pathllms]. That is the specular inter-reflection problem on shiny metal [clark95], learned from
  real data.

**Where ML belongs in MotoBat**, in order:
1. **Region classification.** F10a and F10c were both failures of hand-written shape rules. A
   learned classifier on region features (gradient-boosted trees, as planned in 1.13) is the right
   first step. It is trained on dataset 1 and judged on the held-out set, beside the rules.
   [btf] suggests handcrafted features plus a simple model before a point network.
2. **Appearance, from 2D.** Flush weld vs no weld (F10d) and heat tint are invisible to height.
   This is ML's strongest ground in the literature ([wang23], [catti25]), and the 2D camera is
   deferred.
3. **Raw-signal cleanup, from real data.** Learned reflection filtering, as Path does. It needs real
   scans, so not before dataset 2 has a real sensor model.
4. **Process inverse, in the correction loop.** This is Path's patent; the weld follow-up territory.

**"Why a Python script"** is a separate question. Python is only the language; ML would also be
Python. The sensitivity script orchestrates simulated experiments, and reading 648 outcomes is a
statistics problem. The upgrade there is better experimental design (G2), not a model.

## 3. Path Robotics: what is public (adds to welding_followup.md §4.1)

| What | Source | Read |
|---|---|---|
| $300 M Series E led by Georgian, $2.45 B post-money, 10 Nov 2026. Obsidian runs the torch robot; data captured before, during and after each weld; 4 lasers + 2 cameras; a "weld world model" generates simulated jobs that train Obsidian | [siliconangle] | page |
| Sensor Stack v5: sub-mm point clouds, NN filtering of false reflections, "metrology-grade" optics. 97 %+ first-pass yield ("welds passing inspection on the first attempt"). World model trained on tens of millions of welded inches over 8 years | [pathllms] | page |
| US 12,358,138 B2 (filed 17 Nov 2022, granted 15 Jul 2025), *ML logic-based adjustment techniques for robots*. Claim 1: ML simulates a difference profile (weld vs reference), takes its derivatives with respect to the weld parameters, updates them, welds again | [path138] | page + claims |
| *Autonomous welding robots* family (scan → seams → graph-search path planning), simulated weld paths US 12,277,369 | welding_followup §4.1 | earlier |

No public Path patent or document describes **how they inspect a finished weld** beyond "passing
inspection". Their public method is: measure geometry, compare with a reference, learn the
correction.

**Talking point.**
- Path learns its simulator from fleet data.
- MotoBat's simulator is physics and CAD with exact labels, so its POD and %GRR are traceable.
- Their own patent measures a difference profile against a reference, which is the same structure
  as our golden comparison.

## 4. Gaps, ranked, with the upgrade

| # | What we do now | What the field does | Upgrade (beside dataset 2) | Cost |
|---|---|---|---|---|
| **G1** | F10's fixes were tuned on dataset 1's failures and checked on the same cases. Every first-pass number comes from the dev set | Freeze the method, then report on data it has never seen | **1.13 held-out set before any number goes on a resume.** Already planned; move it ahead of the learned classifier | ~2 days compute |
| **G2** | Sensitivity is one factor at a time | OAT is "perfunctory, unless the model is proven to be linear"; in 10 factors it leaves over 99.75 % of the input space unexplored [saltelli19]. Interactions are not estimable from OAT [czitrom99]; see also [saltelli10] | Run the dataset 2 sweep as a **designed experiment** (fractional factorial or Latin hypercube) over noise, pitch, points, standoff and each realism factor. Response: POD-curve parameters per family. Pitch × noise is an obvious interaction (noise dithers the 0.1 mm quantisation that limits cell X) | same compute as OAT for 4–6 factors; more information |
| **G3** | Hit/miss POD for every class | MIL-HDBK-1823A and ASTM E3023 use **â vs a** (signal response) when the system outputs a measurement; hit/miss (ASTM E2862) is for binary systems [e3023] | For tolerance classes we already record the measured value. Fit â vs a: bias and spread per family, a90/95 from fewer cases. Keep hit/miss for FOD, spatter, dents | analysis only |
| **G4** | POD from simulation alone | Model-assisted POD (MAPOD; AFRL, FAA, NASA): split the factors into those a physics model covers and those that must be measured, and validate the model against experiment [mapod09]; [li14], [smith07] | Write dataset 2's **factor table** in MAPOD form: each source of variation, modelled or empirical, and how it is checked. Plan one small real anchor (any real profiler scan of a mock module) | doc now; real scans when there is access |
| **G5** | A finding when the measurement is past tolerance, ignoring the gauge's own uncertainty | ISO 14253-1:2017 decision rules and JCGM 106:2012: conformity statements account for measurement uncertainty, with guard bands [iso14253], [jcgm] | Use each characteristic's measured σ (1.9) to set an explicit **guard band** and report conforming, nonconforming or uncertain. Cell X on the fixed line (50 % %GRR on ±0.2 mm) is where it bites. Frame the simulated uncertainty as JCGM 101 Monte Carlo propagation, the ISO/TS 15530-4 "virtual CMM" idea [jcgm], [vcmm] | small code + doc |
| **G6** | Hand-written shape rules for regions; F10a and F10c were rule failures | Learned classifiers where variability is high ([wang23], [sun24], [catti25]), classical where geometry is known ([btf], [huang25]) | 1.13's optional learned classifier, rules vs gradient-boosted trees on held-out data; keep the rules as the baseline | ~1 day |
| **G7** | Ideal sensor: no inter-reflection on polished Al/Cu | Inter-reflections on metal produce wrong range data [clark95]; Path filters them with a NN [pathllms] | A dataset 2 realism factor: multipath on caps and busbars. A learned filter only once real scans exist | sensor-model work |
| **G8** | Post-weld height is the only weld check | Penetration on Al and Cu battery welds is measured *in process*: OCT gives keyhole depth from the surface-to-keyhole-bottom distance [schmoeller19]; commercial systems include IPG's LDD [ipg] | State it in the plan: post-weld 3D checks bead geometry, not fusion. Weld integrity needs in-process monitoring, parked in welding_followup | doc only |
| **G9** | The fixed line's cell X fails AIAG (50 % %GRR to tolerance on dataset 1; arm 27 %) | AIAG guide: < 10 % acceptable, > 30 % not | 1.11 shows the cause: 0.05 mm pitch fixes cell pose at 1.1 × tol. A sensor and conveyor spec, not a detector change | none |

**Ordering beside dataset 2:**
1. G1, since every other number rests on it.
2. G3, G5, G4 (doc). These are analysis and writing on existing data.
3. G2, built into dataset 2's design.
4. G6 after G1.
5. G7 inside dataset 2.
6. G8, a paragraph.

## 5. Related work on 3D weld and battery inspection

| Work | What | Method | Read |
|---|---|---|---|
| Sun et al. 2024, *J. Manuf. Processes* 129 [sun24] | weld surfaces from a laser displacement sensor, online | VGG16-UNet segmentation on the scan; 88.77 % precision, 92.03 % recall; mapped back to 3D | abstract (via search) + Crossref |
| Huang et al. 2025, *Sensors* 25:664 [huang25] | aluminium reservoir welds, laser line scan | classical 3D method, > 97.1 % classification accuracy reported | abstract + Crossref |
| Liu et al. 2023, *Virtual Phys. Prototyping* 19 [liu23] | WAAM beads, line laser | 2D and 3D curvature, no learning | abstract + Crossref |
| Catti, Fabbro, Nikolakis 2025, *Sensors* 25:7613 [catti25] | EV battery modules | 2D CNN defect detection mapped onto the 3D CAD; synthetic defects added for rare classes; mm-scale localisation | abstract + Crossref |
| Wang et al. 2023, *Sensors* 23:8894 [wang23] | battery pole-to-adapter laser welds, production line | 2D, two-branch U-Net; Acc 0.979, miss 1.1 %, false alarm 11.5 % | full text |
| Liu et al. 2025, *J. Manuf. Processes* [liu25] | Al busbar welds | in-process spectrum + 2D/3D surface profiles, ML quality prediction | Crossref only (publisher blocked) |
| Horwitz & Hoshen, CVPRW 2023 [btf] | MVTec 3D-AD | handcrafted 3D (no DL) beats learned 3D; BTF combines it with color PatchCore | abstract |
| Bergmann et al. [mvtec3d], Roth et al. PatchCore [patchcore], Wang et al. M3DM [m3dm], Qi et al. PointNet [pointnet] | the standard 3D anomaly and point-network baselines, if G6 goes beyond trees | — | arXiv record |
| Tobin et al. 2017 [dr] | domain randomisation, sim to real | randomise the simulator so a model trained on it transfers | arXiv record |
| Scott, Roth, Rivest 2003, *ACM Comput. Surv.* [viewplan] | view planning for 3D reconstruction and inspection: the survey behind the arm's view plan | — | Crossref |

**What the literature has that we lack, and the reverse.**
- Almost every learned weld-inspection paper trains and tests on one real dataset. None reports POD
  with confidence bounds, a gauge study, or coverage per view.
- MotoBat has those, from simulation; it lacks any real data (G4).
- No paper found combines a CAD-referenced gauge, POD, %GRR and a fixed-vs-arm coverage study for a
  battery module. That is "none found", not "none exists".

## Sources

[wang23]: https://pmc.ncbi.nlm.nih.gov/articles/PMC10650081/ "Wang, Zheng, Dai, Tang, Peng. Deep Network-Assisted Quality Inspection of Laser Welding on Power Battery. Sensors 23(21):8894, 2023. doi:10.3390/s23218894"
[btf]: https://arxiv.org/abs/2203.05550 "Horwitz, Hoshen. Back to the Feature: Classical 3D Features are (Almost) All You Need for 3D Anomaly Detection. CVPR Workshops (VAND) 2023"
[real3d]: https://arxiv.org/abs/2309.13226 "Liu et al. Real3D-AD: A Dataset of Point Cloud Anomaly Detection. NeurIPS 2023 Datasets and Benchmarks"
[huang25]: https://doi.org/10.3390/s25030664 "Huang et al. Aluminum Reservoir Welding Surface Defect Detection Method Based on Three-Dimensional Vision. Sensors 25:664, 2025"
[liu23]: https://doi.org/10.1080/17452759.2023.2294336 "Liu et al. Detection and quantitative evaluation of surface defects in wire and arc additive manufacturing based on 3D point cloud. Virtual and Physical Prototyping 19, 2023"
[path138]: https://patents.google.com/patent/US12358138 "US 12,358,138 B2, Machine learning logic-based adjustment techniques for robots, Path Robotics Inc."
[pathllms]: https://www.path-robotics.com/llms-full.txt "Path Robotics, company product text"
[siliconangle]: https://siliconangle.com/2026/11/10/industrial-automation-startup-path-robotics-raises-300m-in-funding/ "SiliconANGLE, 10 Nov 2026"
[clark95]: https://bmva-archive.org.uk/bmvc/1995/bmvc-95-051.pdf "Clark, Trucco, Cheung. Using Light Polarization in Laser Scanning. BMVC 1995"
[catti25]: https://doi.org/10.3390/s25247613 "Catti, Fabbro, Nikolakis. On a Hybrid CNN-Driven Pipeline for 3D Defect Localisation in the Inspection of EV Battery Modules. Sensors 25:7613, 2025"
[saltelli19]: https://doi.org/10.1016/j.envsoft.2019.01.012 "Saltelli et al. Why so many published sensitivity analyses are false. Environ. Model. Softw. 114:29–39, 2019 (open copy: nora.nerc.ac.uk/id/eprint/522586)"
[saltelli10]: https://doi.org/10.1016/j.envsoft.2010.04.012 "Saltelli, Annoni. How to avoid a perfunctory sensitivity analysis. Environ. Model. Softw. 25:1508–1517, 2010"
[czitrom99]: https://doi.org/10.1080/00031305.1999.10474445 "Czitrom. One-Factor-at-a-Time versus Designed Experiments. The American Statistician 53(2):126–131, 1999"
[e3023]: https://store.astm.org/e3023-21.html "ASTM E3023-21, Standard Practice for Probability of Detection Analysis for â Versus a Data"
[mapod09]: https://ntrs.nasa.gov/citations/20090025450 "Thompson, Brasche, Forsyth, Lindgren, Swindell, Winfree. Recent Advances in Model-Assisted Probability of Detection. 4th European-American Workshop on Reliability of NDE, 2009"
[li14]: https://doi.org/10.1080/00401706.2013.818580 "Li, Meeker, Thompson. Physical Model-Assisted Probability of Detection of Flaws in Titanium Forgings Using Ultrasonic NDE. Technometrics, 2014"
[smith07]: https://doi.org/10.1063/1.2718184 "Smith, Thompson, Meeker, Gray. Model-Assisted Probability of Detection Validation for Immersion Ultrasonic Application. AIP Conf. Proc., 2007"
[iso14253]: https://standards.iteh.ai/catalog/standards/iso/db344fb2-cbb5-448c-b097-6a81d6e4414d/iso-14253-1-2017 "ISO 14253-1:2017, Decision rules for verifying conformity or nonconformity with specifications"
[jcgm]: https://www.bipm.org/en/committees/jc/jcgm/publications "JCGM 100:2008 (GUM), JCGM 101:2008 (Monte Carlo), JCGM 106:2012 (The role of measurement uncertainty in conformity assessment)"
[vcmm]: https://www.csagroup.org/store/product/iso_038696 "ISO/TS 15530-4:2008, CMM: technique for determining the uncertainty of measurement, use of application-specific simulations"
[schmoeller19]: https://doi.org/10.2351/1.5096104 "Schmoeller, Stadter, Liebl, Zaeh. Inline weld depth measurement for high brilliance laser beam sources using optical coherence tomography. J. Laser Appl. 31:022409, 2019"
[ipg]: https://www.ipgphotonics.com/newsroom/stories/developing-OTF-LDD-laser-battery-welding "IPG Photonics, LDD for battery welding (vendor)"
[sun24]: https://doi.org/10.1016/j.jmapro.2024.08.037 "Sun et al. Online detection and evaluation of weld surface defects based on lightweight network VGG16-UNet and laser scanning. J. Manuf. Processes 129:292–306, 2024"
[liu25]: https://doi.org/10.1016/j.jmapro.2025.08.054 "Liu et al. Spectrum and vision based battery busbar weld quality inspection. J. Manuf. Processes, 2025"
[mvtec3d]: https://arxiv.org/abs/2112.09045 "Bergmann, Jin, Sattlegger, Steger. The MVTec 3D-AD Dataset"
[patchcore]: https://arxiv.org/abs/2106.08265 "Roth et al. Towards Total Recall in Industrial Anomaly Detection (PatchCore)"
[m3dm]: https://arxiv.org/abs/2303.00601 "Wang et al. Multimodal Industrial Anomaly Detection via Hybrid Fusion (M3DM)"
[pointnet]: https://arxiv.org/abs/1612.00593 "Qi, Su, Mo, Guibas. PointNet"
[dr]: https://arxiv.org/abs/1703.06907 "Tobin et al. Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World"
[viewplan]: https://doi.org/10.1145/641865.641868 "Scott, Roth, Rivest. View planning for automated three-dimensional object reconstruction and inspection. ACM Computing Surveys 35(1), 2003"
