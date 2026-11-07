# MotoBat — Project Start
 
**Battery Module Assembly Line: Robotic Multi-View Inspection & Correction**
A simulated battery-module line. A GP7 places prismatic cells, a GP4 places the
busbar carrier, a GP25 with a remote laser scanner welds the busbars, and an
HC10DT inspects from several angles before the irreversible laser weld and
corrects the next placement. It is benchmarked against a fixed line-scan
station. *(Welder changed from GP4 to GP25 + MOTOPAC-RL2D on 4 Nov 2026, §4A.7.)*

> **Scope changed 15 Sep 2026: from inspecting the inside of a cell to
> inspecting a module assembly line.** The reasons are measured. See **§3.7**.
> Older sections still describe cell-stack inspection. Where a section
> conflicts with §3.7, §3.7 wins.
 
Rhishikesh Kandge — M.S. Electrical and Computer Engineering, The Ohio State University
 
---
 
## Who is building this, and why
 
**Author:** Rhishikesh Kandge — M.S. Electrical and Computer Engineering,
The Ohio State University. Graduating late August 2026.
 
MotoBat is a self-directed research and engineering project — not coursework,
not lab work. Its purpose is a defensible result: a simulated multi-sensor
inspection cell on a battery module assembly line that detects documented
defect classes (cell placement, terminal height, busbar fit, weld seams),
closes a correction loop before the irreversible laser weld, and measures
whether a robot-mounted multi-view sensor beats a fixed one. The results
carry measured detection performance and a characterised stability boundary.
(Originally scoped to cell-internal electrode stacks; §3.7 records why that
changed.)
 
**§3.6 states the claims this project must defend** and what separates a
research artifact from a demo. That section is the standard the work is held
to.
 
### Relevant background
 
- **ROS 2:** tutorials once, ~1 year ago. Currently rebuilding depth.
- **Built:** a vision-driven pick-and-place system on an Elephant Robotics
  myCobot 280 Pi — **no hardcoded coordinates**. The arm located AprilTag-marked
  zones, identified blocks by colour and shape within the pickup zone, and
  assembled composite structures (triangular prism on square = "house").
  That is a complete perception → planning → manipulation loop.
- **Built:** a control system for myCobot Pro 450 arms — highly compliant, with
  significant backlash — achieving **sub-1 mm repeatability**. Compensating
  backlash on a compliant low-cost arm to that tolerance is a genuine controls
  result, and harder than hitting the same number on a rigid industrial arm.
- **Studying:** John J. Craig, *Introduction to Robotics: Mechanics and Control*.
### Gaps this project is meant to close
 
Prior lab work centred on making rudimentary hardware behave, rather than on
the primary research it was meant to support. The planned multi-robot assembly
system was never built. Industrial-grade tooling (Jetson AGX Orin, ROS 2 at
scale, industrial arms, MoveIt2, Isaac Sim) was not part of that experience.
 
MotoBat exists to close exactly that gap: real industrial robot models, real
ROS 2 architecture, real sensor simulation, and a real defect-detection
contribution.
 
### Working principle for this repository
 
> **Every subsystem must be documented with the mathematics that governs it,
> not just the code that implements it.**
 
The goal is to understand *what* is being computed and *why* — from equations
and block diagrams — without reading generated code line by line. Where an
implementation is AI-assisted, the maths must be written down first, so the
code can be checked against it.
 
Practically, that means every module gets:
 
1. The governing equations, with symbols defined
2. A block or data-flow diagram showing where it sits
3. Units and frames stated explicitly
4. A citation to the textbook or paper it comes from
§9 collects the mathematics. §10 lists the study resources.
 
---
 
## What this document is
 
The handoff from environment setup to actual development. It records what is
built and verified, the decisions made and why, and the concrete next milestone:
**the HC10DT, planned via MoveIt2, inspecting a generated battery module
from several viewpoints in Gazebo**, with the fixed line-scan baseline alongside
(§3.7.6).
 
Read this first if you are picking the project up cold — including Claude Code.
 
**§9 is the mathematics** behind every subsystem, and **§10 the study
resources.** Per the working principle above, implementations are checked
against the equations in §9 — write the maths down before writing the code.
 
---
 
## 0. Current status
 
| Area | State |
|---|---|
| Ubuntu 24.04 LTS + ROS 2 Jazzy | Installed, verified (talker/listener) |
| MoveIt2, ros2_control, rqt, RViz2 | Installed |
| Open3D, PCL, OpenCV, Python 3.11 venv | Installed |
| PyTorch CUDA | **Fixed** 31 Aug 2026 — see §1.1 |
| **Motherboard / RAM** | **REPAIRED** 31 Aug 2026 — 16 GB dual-channel, verified (§1.0) |
| Yaskawa GP4/GP7/HC10DT descriptions | Cloned, verified |
| `motobat_description` (4-arm line) | Written, xacro compile-tested |
| Four arms visualized in RViz | Unblocked — §1.2 fixed |
| Repository structure | **Restructured** 31 Aug 2026 — repo root is the colcon workspace (`docs/project_structure.md`) |
| **Domain** | **Battery module assembly line** as of 15 Sep 2026 (§3.7) |
| Defect generation + detection | **Current work**: §4B, module generator, mesh-native |
| MoveIt configs | HC10DT first (§4A.2), then GP7 cell placement, then GP4 busbars |
| Sensor flange | Snapshot 3D + line profilometer; 2D camera only if task 1.11 requires it (§3.7.3, revised 28 Oct 2026) |
| **Battery module CAD** | **Complete 1 Nov 2026** — all 12 bodies of `module_spec.md` §7, plus two assembled references. STEP in `cad/module/`, STL in `src/motobat_description/meshes/module/` |
| **Inspector mount CAD** | **Started early** — `inspector_flange`, carrying the Zivid 2+ M60 and the Keyence LJ-X8200 (§4A.4). Task 2.3 is ahead of sequence. |
| Isaac Sim | Deferred, not cut: Gazebo first (§4) |
| Memory upgrade | Optional, deferred past 1 Oct — §1.0, §4.0b |
 
> **Current plan:** the motherboard was replaced and the machine is back to
> **16 GB dual-channel** (§1.0). That removes the hardware blocker, but does
> **not** change the sequencing: the active work is still **§4A** (RViz2 +
> Gazebo: arms, IK, sensor flange) and **§4B** (defect generation and detection
> in numpy/Open3D). Isaac Sim stays scheduled for **week 5** because §4B is the
> actual contribution and neither §4A nor §4B needs a simulator — not because
> the machine cannot run it.
>
> **Target: 1 Oct 2026** (§3.5). The bar the work is held to is **§3.6**.
 
---
 
## 1. Known issues to clear first
 
### 1.0 HARDWARE: dead DIMM slot — 8 GB ceiling
 
**Machine:** Dell G15 5530 · i9-13900HX (24C/32T) · RTX 4060 Laptop (8 GB VRAM)
· BIOS 1.34.0 · Service Tag `BZC6524` · purchased in India, in use in the US.
 
**Finding: memory slot DIMM A is dead. Both RAM modules are healthy.**
 
Diagnosed by physically swapping the modules between slots and re-reading
`dmidecode` both times:
 
| | Before swap | After swap |
|---|---|---|
| DIMM A | `No Module Installed` | `No Module Installed` |
| DIMM B | 8 GB, serial `442BC272` | 8 GB, serial **`442BC255`** |
 
The serial in slot B changed, proving a *different* physical module was moved
there — and it works. Slot A reads empty with either module in it. Confirmed
independently by SMBIOS type 19: `Memory Array Mapped Address → Range Size: 8 GB`.
 
Both modules are Micron `MTC4C10163S1SC48BA1`, 8 GB DDR5-4800, Rank 1.
 
**Conclusion:** motherboard fault, not a memory fault. **Buying RAM would not
fix this** — a new module in slot A would also read as absent. The board
supports 64 GB across 2 slots, but only slot B is functional, so the machine
is capped at whatever a single SODIMM holds.
 
A bluescreen some months earlier is the likely moment slot A failed. The system
has been silently running on one module since; nothing announces the loss.
 
**This almost certainly caused the earlier multi-day Isaac Sim debugging.**
Isaac Sim was being run on 7.4 GB visible RAM against a documented 32 GB
minimum, while believing the machine had 16 GB. Every OOM and freeze looked
like an Isaac Sim problem. It was not.
 
**Diagnostic worth reusing:**
 
```bash
free -h                                    # what Linux sees
sudo dmidecode -t memory | grep -E "Locator|Size|Serial"
```
 
If firmware reports `No Module Installed` for a populated slot, the fault is
below the OS. Swapping modules between slots distinguishes a bad module from a
bad slot — the serial number is what proves which module is where.
 
**Do not bother with memtest86+ here.** Both modules are proven good by the
swap test, and memtest cannot test a socket that reports nothing.
 
#### Warranty path — bought in India, used in the US
 
Roughly one year of warranty remains, but it was purchased in India and Dell
service contracts are **region-bound**: "the service contract purchased for
that computer at the point of sale is valid only in the country or region in
which the computer was purchased."
 
Dell's international support covers travel of **under six months**. Beyond
that, it is a permanent relocation and needs a transfer.
 
**Steps, in order:**
 
1. **Check current coverage.** dell.com/support → enter Service Tag `BZC6524`.
   Note the exact service plan name — the plan type decides everything below.
2. **File an Ownership/Warranty Transfer** at Dell's Ownership Transfer page,
   selecting the country/region relocation reason ("permanently relocated to
   another country for 6 months or more"). Requires the Service Tag and both
   old and new owner details; keep the India proof of purchase ready.
   **~15 business days** for international transfers. Dell representatives
   cannot file it for you — privacy policy requires the owner to submit.
3. **Then request service** through US Dell support with the transferred tag.
**Manage expectations on the service type.** Onsite next-business-day service
— an engineer coming to your home, as is common with Dell India consumer
plans — **is unlikely to carry over.** Dell states that "Return to Depot,
Complete Care, Same Day, and Parts Only contracts are not always available for
transfer outside of the region the coverage was purchased in." Realistically,
expect **mail-in depot service** in the US: ship the laptop to a Dell facility,
lose it for one to two weeks.
 
**If the transfer is refused,** paid options are a Dell out-of-warranty depot
repair or a local shop board-level repair. Motherboard replacement on a G15
runs roughly $300–500 with labor.
 
#### The decision this forces
 
DDR5 SODIMM prices rose sharply through 2026 — 2×16 GB now runs around $400.
Combined with a dead slot that makes new RAM useless, **buying memory is off
the table.** The realistic options:
 
| Option | Gets you | Cost | Downside |
|---|---|---|---|
| **Warranty repair** | Back to 16 GB, dual-channel | Free if approved | ~15 business days transfer + 1–2 weeks depot; laptop gone |
| **Paid board repair** | Back to 16 GB | ~$300–500 | Cost; still under 32 GB |
| **Single 32 GB SODIMM in slot B** | 32 GB single-channel | High at current prices | Single-channel bandwidth penalty; verify G15 supports a single 32 GB module |
| **Remote/lab machine** | Full 32 GB+ capability | Free if OSU provides | Requires access; container workflow |
| **Stay at 8 GB** | Nothing | Free | Isaac Sim not realistically usable |
 
#### RESOLVED — Dell onsite repair approved, completed, and verified

Dell approved a **free onsite motherboard replacement in the US**, despite the
machine being purchased in India. Better than the region-bound policy predicted;
worth noting that the documented policy is not always the outcome — it was worth
asking.

**The board was replaced and the fix is verified (31 Aug 2026).**

```
$ free -h
               total        used        free      shared  buff/cache   available
Mem:            15Gi       6.8Gi       792Mi       977Mi       8.3Gi       8.5Gi
Swap:           35Gi       380Ki        35Gi
```

15 Gi visible is the expected reading for a 16 GB machine — firmware and the
iGPU reserve the difference. The 8 GB ceiling is gone.

**Slot A confirmed alive by SMBIOS (4 Sep 2026).** `free -h` proves the
capacity came back; only `dmidecode` proves the *socket* did:

```
$ sudo dmidecode -t memory | grep -E "Locator|Size|Serial"
    Size: 8 GB
    Locator: DIMM A
    Serial Number: 442BC272
    Size: 8 GB
    Locator: DIMM B
    Serial Number: 442BC255
```

Both slots populated, both reporting 8 GB, **distinct serials**. Compare
against the fault table above: DIMM A read `No Module Installed` with *either*
module seated in it. It now reads one.

Two things this confirms:

1. **The serials are the original modules.** `442BC272` and `442BC255` are the
   same two Micron `MTC4C10163S1SC48BA1` sticks from the swap test — no new RAM
   was fitted. The diagnosis in §1.0 was correct in full: healthy modules, dead
   socket, motherboard fault. Buying RAM would indeed have fixed nothing.
2. **`442BC272` is now in slot A**, and that is the module which read fine in
   slot B before the repair. A known-good stick is what is proving the
   previously-dead socket. That is the strongest form of this test — it cannot
   be explained by a marginal module.

Dell case closed. **Keep both 8 GB modules** if a 2 x 16 GB upgrade ever
happens; they are the known-good fallback for exactly this diagnostic.

**What this changes, and what it does not.**

It removes the hardware blocker. It does **not** move Isaac Sim earlier. §3.5
schedules Isaac Sim for week 5 because §4B is the actual research contribution
and completes without a simulator — that ordering was never about the RAM.
16 GB is still below Isaac Sim's documented 32 GB minimum, so §4.0b's cloud
recommendation for weeks 5-6 stands.

**§4A** and **§4B** remain the work plan, and both now have more headroom than
they need.
 
#### Memory upgrade — decision and slot math
 
Post-repair the machine has **two working slots, both holding Dell's 8 GB
DDR5 modules**. Any upgrade must account for that:
 
| Purchase | Install | Result | Notes |
|---|---|---|---|
| 1 x 16 GB now | pull one 8 GB | **24 GB** | Intel Flex Mode; nets only +8 GB |
| 1 x 16 GB later | pull the other 8 GB | **32 GB** | both 8 GB modules now spare |
| **2 x 16 GB kit at once** | pull both 8 GB | **32 GB** | matched pair, dual-channel |
 
**Recommendation: save for the 2 x 16 GB kit and buy it in one purchase.**
The incremental path reaches the same 32 GB but passes through a month of
mismatched memory and nets only +8 GB for the first payment. Timing favours
waiting: §4A and §4B are 6-8 weeks of work that run comfortably in 16 GB, so
the kit arrives about when Isaac Sim actually becomes the bottleneck.
 
**DDR5 ONLY — DDR4 is not an option.** A 32 GB DDR4 kit at half the price is
not a viable substitute: DDR4 and DDR5 SODIMMs differ in pin count, notch
position, and voltage regulation (DDR5 carries its PMIC on the module). A DDR4
module will not physically seat in this machine. Confirmed by `dmidecode`:
`Type: DDR5`, `Configured Voltage: 1.1 V`. Buying DDR4 would be money lost.
 
Buying notes:
 
- Buy a **kit**, not two separate modules — kits are validated as a matched pair
- Spec: DDR5 SODIMM, 4800 MT/s or faster, 1.1 V, non-ECC
- Faster kits (5600) clock down to platform speed; fine, but no benefit
- **Keep the 8 GB modules.** They are the known-good fallback for the same
  swap test that diagnosed the dead slot
- Even 32 GB is Isaac Sim's *minimum* (64 GB ideal). The single-arm milestone
  will be fine; the full four-station line will still be tight
**Worth doing before spending anything:** ask the OSU ECE department about GPU
workstation access. A lab machine with 64 GB beats any laptop configuration and
is exactly the scenario Isaac Sim's container mode targets (§4.0).
 
### 1.1 PyTorch was broken — FIXED 31 Aug 2026

The installer ran `pip install torch --index-url .../cu124` (got `2.6.0+cu124`),
then ran `pip install stable-baselines3[extra]` **without** the index URL. Pip
resolved torch from PyPI, found `2.13.0`, and upgraded over the CUDA build.

Result: `torchvision 0.21.0` and `torchaudio 2.6.0` both pin `torch==2.6.0` and
were broken, plus ~4 GB of redundant CUDA 13 wheels alongside the cu124 ones.

```bash
./workstation_setup/fix_torch.sh
```

**A second bug surfaced while running the repair, and it is the more
interesting one.** The first version of `fix_torch.sh` tried to uninstall the
CUDA 13 strays by name — `nvidia-cublas-cu13`, `nvidia-cudnn-cu13`, and so on.
**Those package names do not exist.** NVIDIA publishes the CUDA 13 wheels
*unsuffixed* (`nvidia-cublas 13.1.1.3`, `nvidia-cufft 12.0.0.61`), and keeps
the `-cu12` suffix only on the CUDA 12 line. So every uninstall silently
no-op'd, pip reported success, and the CUDA 13 runtime stayed exactly where it
was.

Removing them afterwards then exposed a third failure: uninstalling the
unsuffixed wheels deleted files that the `-cu12` packages still claimed in
their metadata. `pip list` showed `nvidia-cudnn-cu12 9.1.0.70` installed while
`libcudnn.so.9` was absent from disk, and `import torch` failed on a library
pip was certain it had. The same then happened with `libnccl.so.2`.

The script now:

- selects strays **by shape, not by guessed name** — every `nvidia-*` wheel
  that is not `-cu12` — so it cannot silently match nothing again
- uses `--force-reinstall` so the resolve restores every dependency's files
  rather than trusting metadata
- verifies with an **actual CUDA matmul**, because `torch.cuda.is_available()`
  returns `True` against a half-installed cuDNN

**Three lessons, all cheap to keep:**

1. Any `pip install` that can pull `torch` transitively must either carry the
   same `--index-url` or use `--no-deps`.
2. `pip uninstall` on a name that is not installed **succeeds**. An uninstall
   list built from guessed names is indistinguishable from one that worked.
3. Installed-per-metadata is not installed-on-disk. Verify a native stack by
   running something on it.

Does not block RViz, Gazebo, or Isaac Sim work — only the Section 5.4 RL
stretch goal.
 
### 1.2 `bootstrap_rviz.sh` fails with "AMENT_TRACE_SETUP_FILES: unbound variable" — FIXED

`set -u` collides with `/opt/ros/jazzy/setup.bash`, which reads undefined
variables intentionally. The script uses `set -Eeo pipefail` without `-u`.

**This applies to every shell script in the project that sources ROS.** Do not
use `set -u` in anything that sources a ROS setup file. `motobat_env.sh` at the
repo root carries the same warning in a comment, for the same reason.

### 1.3 Minor, already handled

- `ros-jazzy-industrial-msgs` does not exist in apt; vcs pulls it from source.
- numpy is held at `1.26.4`. **Do not upgrade to 2.x** — Open3D wheels and
  several Jazzy Python packages still link the 1.x ABI. Note that a
  `--force-reinstall` of torch *will* pull numpy 2.x as a dependency; the
  repaired `fix_torch.sh` restores the pin afterwards and now asserts it.
- **stable-baselines3 is pinned to 2.5.0, not 2.9.0.** SB3 2.9 requires
  `torch>=2.8`, which conflicts with the `2.6.0+cu124` build pinned for CUDA
  12.4. 2.5.0 is the last line that accepts torch 2.6. `gymnasium` is
  correspondingly capped below 1.1.
- `pip check` reports one **benign** conflict: `databricks-sdk` (a transitive
  mlflow dependency) wants `protobuf<7`, while `grpcio-tools` needs `7.x` for
  the inspector-to-correction stubs. The databricks backend is unused here —
  local `mlruns/` only. Leave it.
 
### 1.4 `view_line.launch.py` dies with "Unable to parse the value of parameter robot_description as yaml" — FIXED 1 Sep 2026

Found by actually launching it, not by reading it. `bootstrap_rviz.sh` built
cleanly and the xacro expanded to the expected 39 links / 38 joints, so §1.2
looked like the last blocker. It was not.

```python
# broken — launch tries to YAML-parse an XML document
'robot_description': Command(['xacro ', path])

# correct
'robot_description': ParameterValue(Command(['xacro ', path]), value_type=str)
```

`Command(...)` returns a substitution with no declared type. The launch system
defaults to interpreting it as YAML, meets a URDF, and fails. The fix is the
`ParameterValue(..., value_type=str)` wrapper from `launch_ros.descriptions`.

**This applies to every `robot_description` parameter in the project** — the
MoveIt configs in §4A.1 and §4A.2 and the Gazebo bringup in §4A.3 will each
need it. Both `view_line.launch.py` and `view_single.launch.py` are fixed.

**Verified headlessly** (no display needed, which makes it CI-able later):

```bash
ros2 launch motobat_description view_line.launch.py gui:=false rviz:=false &
ros2 run motobat_description demo_line_motion.py &
ros2 topic hz /joint_states                    # -> ~50 Hz
ros2 topic echo /tf --once | grep -c child_frame_id   # -> 24 movable joints
ros2 topic echo /tf_static --once | grep -c child_frame_id   # -> 14 fixed
```

24 dynamic frames is exactly the 4 arms x 6 joints the compile test predicted,
so the TF tree is correct end to end and not merely syntactically valid.

**Lesson worth keeping:** a clean `colcon build` says the package compiles. It
says nothing about whether the launch files run. Launch every launch file once,
headlessly, before calling a milestone done.
 
---
 
## 2. Environment
 
**Ubuntu 24.04 LTS + ROS 2 Jazzy.** Not 26.04/Lyrical: Isaac Sim 6.0 supports
22.04/24.04 only, and its ROS 2 bridge names Jazzy on 24.04 as the recommended
configuration. Jazzy is supported to May 2029 — past both project phases.
 
### Layout

**The repository root is the colcon workspace.** There is no nested `ros2_ws/`
any more — `colcon build` runs from the repo root. Full reasoning and the
package-naming rules are in `docs/project_structure.md`.

```
~/projects/MotoBat/           git repo AND colcon workspace
├── motobat_env.sh            sourced from ~/.bashrc; self-locating
├── docs/
│   ├── project_plan.md       this file
│   ├── project_structure.md  layout, package rules, branch/commit practice
│   └── status_update.md      ordered task list and where we are
├── workstation_setup/        setup_workstation.sh, bootstrap_rviz.sh, fix_torch.sh
├── src/
│   ├── motobat_description/  4-arm line, launch files, RViz config
│   └── _vendor/              cloned deps — gitignored
├── logs/                     installer logs        (gitignored)
└── build/ install/ log/      colcon output         (gitignored)

~/motobat/
└── venv/                     Python 3.11 — deliberately OUTSIDE the repo
```

The venv stays outside the workspace on purpose: an active venv changes which
interpreter colcon resolves, and the resulting failures are slow to diagnose.
`~/motobat/` now contains nothing else.

Packages are created **when they are written**. The installer used to `mkdir`
seven empty `motobat_*` directories; it no longer does, because an empty
directory has no `package.xml`, is not a package, and only promises structure
that does not exist.

Aliases: `mb-venv` `mb-ws` `mb-build`
 
Aliases: `mb-venv` `mb-ws` `mb-build`
 
### Python: two interpreters, on purpose
 
System Python is 3.12. The venv is **3.11** because Isaac Sim 5.x/6.x embed
CPython 3.11 — importing Open3D from 3.12 into Isaac Sim's interpreter produces
ABI errors that look like Open3D bugs but aren't.
 
Use `mb-venv` for Open3D/PyTorch work. Never `pip install` into system Python;
24.04 blocks it via PEP 668 and overriding will eventually break apt.
 
### Hardware

Dell G15 5530 · i9-13900HX (24C/32T) · RTX 4060 Laptop (8 GB VRAM) ·
driver 595.84 · 789 GB free. GPU, driver, and disk all pass.

**RAM is 16 GB dual-channel** as of the 31 Aug 2026 motherboard replacement
(§1.0). The dead-slot ceiling is gone. 35 GB of swap remains configured.

That is still **below Isaac Sim's documented 32 GB minimum**, so §4.0 and
§4.0b remain live: 16 GB is workable for a single arm with simplified collision
geometry, and the cloud path stays the recommendation for weeks 5-6. What
changed is that RAM is no longer *the binding constraint* — it is now a
week-5 sizing question rather than a blocker on all work.

The CPU is excellent (24 cores) and the GPU meets the 8 GB VRAM minimum — this
was a memory-capacity problem specifically, not a weak machine.
 
---
 
## 3. What is built and verified
 
### `motobat_description`
 
Four arms in one scene, each instantiated from the Yaskawa xacro macros with a
distinct prefix so they share one TF tree without name collisions.
 
| Station | Role | Arm | Prefix | Macro |
|---|---|---|---|---|
| 1 | Stacker | GP4 | `s1_` | `motoman_gp4` |
| 2 | Tab Welder | GP4 | `s2_` | `motoman_gp4` |
| 3 | Inspector | HC10DT | `s3_` | `motoman_hc10dt` |
| 4 | Can Sealer | GP7 | `s4_` | `motoman_gp7` |
 
**Verified by compile test:** 39 links, 38 joints, 24 movable (4 x 6), single
`world` root, no missing joint limits.

**Verified running, on screen, 4 Sep 2026.** All four arms render in RViz with
correct meshes. The `joint_state_publisher_gui` sliders drive all 24 joints,
and each arm responds **independently** — no cross-talk between stations, which
is the real test of the `s1_`/`s2_`/`s3_`/`s4_` prefixing. The GUI's *centre*
and *randomise* buttons both behave.

That is the four-station description signed off end to end: xacro expands, TF
tree is single-rooted, prefixes isolate the arms, and the meshes load.

Verified headlessly too, which makes this CI-able later without a display:

| Check | Result |
|---|---|
| `/joint_states` rate | ~50 Hz |
| Dynamic TF frames (`/tf`) | **24** = 4 arms x 6 joints |
| Static TF frames (`/tf_static`) | 14 |
| Launch errors | none |

```bash
./workstation_setup/bootstrap_rviz.sh    # vendor deps, colcon build, verify

# sliders for all 24 joints
ros2 launch motobat_description view_line.launch.py

# or scripted motion (terminal 1, then terminal 2)
ros2 launch motobat_description view_line.launch.py gui:=false
ros2 run motobat_description demo_line_motion.py
```
 
> **`demo_line_motion.py` is a smoke test, not architecture.** It publishes
> `/joint_states` directly — no physics, no collision checking, no controllers.
> It proves the descriptions, TF tree, and meshes are correct. Delete it once
> real controllers exist so it does not quietly become load-bearing.
 
### Robot descriptions
 
`ros-industrial/motoman` is **ROS 1 only**. ROS 2 descriptions come from the
[`gavanderhoorn/motoman`](https://github.com/gavanderhoorn/motoman/tree/ros2)
fork, `ros2` branch — ament_cmake packages with meshes.
 
**On the HC10:** the fork ships `hc10`, `hc10dt`, and `hc10dt_b10` — there is no
`hc10dtp`. DTP is the IP67-rated variant of the DT with identical kinematics and
reach, so `hc10dt.xacro` is correct. Worth one sentence in Section 2 of the
proposal so it does not read as a spec mismatch.
 
---
 
## 3.5 Timeline to a portfolio-ready result — target 1 Oct 2026

> **No longer binding (15 Sep 2026).** There is no fixed deadline. Work is
> sequenced by dependency, not by date: §4B (numpy) → HC10DT inspection in
> Gazebo → GP7/GP4 line and the closed loop → optional Isaac Sim. The original
> table is kept below for the record.
 
**Scoped goal:** simulated structural defects in a battery cell stack, plus one
HC10DT carrying a sensor suite that scans them and reports what it found.
 
**That goal is achievable by 1 October.** The full four-station line is not, and
should not be attempted on this schedule. Narrowing to one arm is the right
call regardless of time — it is the honest scope for a solo six-week build, and
a working single-station system demonstrates more than a half-built line.
 
| Week | Dates | Focus | Depends on |
|---|---|---|---|
| 1 | Aug 21–27 | §4B.1–4B.3 stack generator, defect injection, simulated scan | nothing — pure numpy, runs on 8 GB today |
| 2 | Aug 28–Sep 3 | §4B.4–4B.5 Open3D detection + labelled validation set | week 1 |
| 3 | Sep 4–10 | §4A.1–4A.2 GP4 and HC10DT MoveIt configs, marker IK in RViz | 16 GB (post-repair) |
| 4 | Sep 11–17 | §4A.4 flange CAD, sensor placement study, URDF frames | week 3 + §3.7.2 scan-window requirement |
| 5 | Sep 18–24 | Isaac Sim install, HC10DT import, OmniGraph, MoveIt bridge | **RAM — see risk below** |
| 6 | Sep 25–Oct 1 | Integration, results, README, demo video | all above |
 
### The one real risk: weeks 5–6
 
The memory upgrade (§1.0) is deferred roughly two months, which lands **after**
this deadline. Weeks 5–6 therefore run Isaac Sim on **16 GB against a documented
32 GB minimum**.
 
A single HC10DT with simplified collision meshes is far lighter than Isaac Sim's
worst case, and 35 GB of swap is configured, so it will probably run — slowly.
But this is the part of the plan that can slip.
 
**Primary mitigation: rent a cloud GPU for weeks 5–6 (§4.0b).** 40–80 hours on
an L4 instance costs roughly $20–60 — an order of magnitude less than the RAM
kit, and it removes the hardware purchase from the critical path entirely.
 
**Fallback, decided in advance:** if Isaac Sim will not cooperate by ~Sep 24,
ship the Gazebo version. Gazebo with a depth camera on the HC10DT produces the
same demonstration — arm scans defective stack, detector reports the defect — at
lower sensor fidelity. Swap the hardware interface, keep everything else (§4A.3).
 
This fallback exists because of how the work is ordered: **§4B is the actual
research contribution and completes in week 2, independent of any simulator.**
Even in the worst case, there is a real result with real precision/recall
numbers by early September.
 
### Working pace
 
Six weeks at a sustainable pace beats four weeks of 16-hour days followed by
two weeks of exhaustion. This is stated as an engineering observation, not
advice about life: most of what remains is debugging — TF frames, joint-name
mismatches, ICP convergence, Isaac Sim's ActionGraph. Debugging while tired is
where days disappear, and the failure modes in §4.4 are exactly the kind that
look like software bugs and are not.
 
The schedule above assumes roughly 6–8 focused hours a day and has slack in it.
It does not require 16-hour days, and the project will finish sooner without
them.
 
---
 
## 3.6 What makes this read as research rather than a demo
 
The goal is work that stands up next to a published paper. That is a specific
bar, and it is **not** about scope, polish, or how impressive the simulation
looks. A four-station line with no measurements impresses no one who reads
carefully. A single arm with a defensible quantitative claim does.
 
The currency is: **a claim someone could disagree with, defended with numbers.**
 
### The claims MotoBat can actually defend

> **Updated 15 Sep (§3.7).** The claims now read:
>
> 1. **Three findings about sensing, each from building rather than assuming**
>    (§3.7.2–3.7.4): height maps miss lateral error; most of the proposal's
>    sensors are the wrong class for measurement; cell-internal defects are an
>    optical-access problem, not a resolution problem.
> 2. **Arm vs fixed sensor, measured** (§3.7.6): coverage, recall per class
>    and cycle time for multi-view HC10DT inspection against a fixed Ranger3
>    line-scan. Either outcome is a result.
> 3. **Closed-loop stability**, with *k* = cell index: the K-sweep and
>    open-loop vs closed-loop placement defect rate.
>
> The original battery-cell text follows.

Three, all reachable within the §3.5 timeline:
 
**1. A negative result, already found.**
Top-down height mapping is insufficient for lateral electrode misalignment —
the primary defect class. (This sentence originally also credited NHTSA
26V188; that attribution was wrong — see §3.7.10.) Measured in §3.7.2
(formerly §4B.0):
a 0.8 mm layer shift produced a −2.3 µm max-Z change and **zero** footprint
change, while lateral envelope deviation gave a clean monotonic signal
(+200 µm at 0.2 mm shift, rising linearly to +2000 µm at 2.0 mm).
 
Negative results with a mechanism are exactly what papers are built from. This
one says something specific about sensing modality selection that a practitioner
could act on, and it was found by building rather than assuming.
 
**2. A sensor-placement requirement derived, not guessed.**
The scan window must exceed the nominal part envelope, because material that has
moved outside the expected footprint is invisible to a scan covering only that
footprint. This converts flange design (§4A.4) from a modelling exercise into a
requirement with a numerical justification — and if the four-sensor suite does
*not* fit within that constraint, **that is also a publishable finding.**
 
**3. A closed-loop stability characterisation.**
The correction loop (§9.9) has a derived stability bound, `0 < K < 2/g`. A sweep
of K showing convergence, critical damping, and divergence — plus open-loop vs
closed-loop defect rate — is a controls result, not a demo. Most simulation
projects show one tuned gain that works. Showing the boundary demonstrates the
system is understood.
 
### What turns these into a research artifact
 
| Deliverable | Why it matters | When |
|---|---|---|
| Labelled defect dataset + generator | Reproducible by others; the generator *is* a contribution | Week 2 |
| Precision/recall per defect class, with error bars | Benchmarkable against the 62.07 % Hi-Pot and 93.82 % YOLOv4 figures (Section 1.2) | Week 2 |
| Minimum detectable defect per class | Directly comparable to the 0.5 mm production threshold (Section 1.1) | Week 2 |
| K-sweep: convergence → divergence | Controls result with a derived bound | Week 6 |
| Open-loop vs closed-loop defect rate | The headline claim of the whole project | Week 6 |
| Public repo, documented maths (§9), reproducible runs | The difference between "I built a thing" and "here is a method" | Throughout |
 
### The write-up is not optional
 
The single highest-leverage hour in this project is spent on the README and
results write-up, not on code. Structure it as a paper:
 
1. **Problem** — in-process geometric inspection before irreversible steps,
   grounded in NHTSA 26V188 as read from the report itself (§3.7.10)
2. **Method** — architecture, the maths from §9, the detection pipeline
3. **Results** — the tables above, with plots
4. **Limitations** — simulation-only, geometric defects only, single station
5. **Future work** — carbon-fibre layup generalisation (proposal Section 6)
Stating limitations plainly is what separates research writing from marketing.
It also pre-empts the first question any reviewer or interviewer will ask.
 
### Publication paths, honestly assessed
 
| Path | Feasible by Oct? | Notes |
|---|---|---|
| **arXiv preprint (cs.RO)** | **Yes** | No peer review. Needs one endorsement for a first submission — any robotics author can give it, **not necessarily the advisor**. Produces a citable artifact with a permanent link. |
| **Workshop paper** | Yes, next cycle | 4–6 pages, far higher acceptance than the main track, real venue. ICRA/IROS workshop CfPs open periodically — track them. |
| **Full conference paper** (ICRA/IROS) | No | Main-track deadlines and quality bar do not fit six weeks of solo work. A rushed rejection helps nothing. Target a later cycle with this work as the foundation. |
 
**Recommended:** finish the project, write it properly, post the preprint. Then
extend toward a workshop paper for the next cycle. The preprint is a real,
linkable, citable thing — and unlike a LinkedIn post, it is permanent.
 
### On the comparison to other students' papers
 
A published paper and a rigorous engineering project are different currencies,
and they trade differently depending on the buyer.
 
- **Industry robotics roles** — a working system with quantitative results, a
  clean repo, and a demo video is often *worth more* than a paper. Hiring
  managers can evaluate a repo directly; most cannot evaluate a paper outside
  their subfield.
- **Academic RA / PhD applications** — papers weigh more, but a substantial
  independent project with a preprint is legible evidence of research capability,
  and it demonstrates something a co-authored paper does not: the ability to
  scope, execute, and finish a project alone.
The scoped MotoBat result — a simulated multi-sensor inspection cell that
detects real documented defect classes and closes a correction loop, with
measured precision/recall and a stability characterisation — is not a small
project. Done properly and written honestly, it competes.
 
---
 
## 3.7 Domain decision — the resolution gate (15 Sep 2026)

> **Decision: MotoBat inspects a battery *module* assembly line, not the inside
> of a single cell.** A GP7 places prismatic cells into a module. A GP4 places
> busbars and performs a simulated laser weld *(superseded 4 Nov 2026: a GP25
> with the MOTOPAC-RL2D remote laser package welds; the GP4 places the carrier,
> §4A.7)*. The HC10DT inspects from several
> angles before and after the weld. A fixed line-scan station is the baseline
> it has to beat.
>
> Two alternatives were set aside, each for a measured reason: electrode-stack
> inspection inside the cell (§3.7.4) and carbon-fibre layup (§3.7.5). Nothing
> is discarded. The cell-level findings motivate the whole project, cell
> internals continue later with a battery-technology collaborator (§3.7.7),
> and carbon fibre stays the follow-up that tests generality.

### 3.7.1 Why the domain was re-opened

The original plan assumed an arm-mounted sensor could see the defect, and no
one had checked. The check is datasheet arithmetic, and whatever it shows is a
result. So it ran before any Phase 1 code. The project's purpose was also made
explicit at this point: **work that a controls and automation practitioner
would recognise as a real line, solving a real problem, with the robot doing a
job only a robot does well.** That is the standard every choice below was
held to.

### 3.7.2 Finding A — height maps miss lateral misalignment

*(Originally §4B.0. Kept verbatim; it stays a valid result.)*

While probing feasibility, the first detector measured **stack height** and was
nearly blind to lateral misalignment: a 0.8 mm layer shift produced a −2.3 µm
max-Z change and **zero** footprint change. Obvious in hindsight — shifting
layers sideways does not change how tall the stack is.

Switching the metric to **lateral envelope deviation** produced a clean,
monotonic signal:

| Layer shift | Scanned X-extent | Δ vs reference | Protruding points |
|---|---|---|---|
| 0.0 mm | 66.450 mm | +0 µm | 0 |
| 0.2 mm | 66.650 mm | +200 µm | 3,840 |
| 0.5 mm | 66.950 mm | +500 µm | 9,600 |
| 0.8 mm | 67.250 mm | +800 µm | 15,360 |
| 1.2 mm | 67.650 mm | +1200 µm | 23,040 |
| 2.0 mm | 68.450 mm | +2000 µm | 38,400 |

*(24 layer-pairs, 66.5 × 48.0 mm footprint, 1.5 mm cathode inset, defect
injected from layer 12 onward, 50 µm scan pitch.)*

Two consequences carry into the module line:

1. **Choose the metric to match the defect's dominant direction.** A cell
   placed off-position in a module is a lateral error. A terminal standing
   proud is a height error. One metric does not serve both.
2. **The scan window must extend beyond the nominal envelope.** Material that
   has moved outside the expected footprint is invisible to a scan covering
   only that footprint.

**Caveat found at the gate:** the prototype shifted *whole layer pairs*,
anode included, far enough to protrude. That case is real, but it is not the
dangerous one. See Finding C.

### 3.7.3 Finding B — sensor roles: most of the proposal's sensors cannot measure a defect

**Rule of thumb used:** to detect a defect of size *D*, the sensor needs noise
σ ≲ *D*/3 and sample spacing ≲ *D*/2.

| Sensor | What it is | Key figures (datasheet) | Can it measure a 0.1–0.5 mm defect? | Role in MotoBat |
|---|---|---|---|---|
| Keyence LJ-X8020 / 8030 | laser line profilometer | 2.5–5 µm pitch, 0.3–0.4 µm Z, 8–16 mm width, 20–28 mm standoff | ✅ | Too narrow and too close for an arm working around a module. Relevant only to cell-level work. |
| **Keyence LJ-X8200 class** | laser line profilometer | 72 mm width at 245 mm reference distance, 50 µm pitch, 1 µm Z | ✅ | **Wrist, precision:** covers two adjacent terminals in one profile, at a safe standoff |
| **Zivid 2+ M60 class** | structured-light snapshot 3D camera | 0.24 mm spatial resolution at 60 cm, 58 × 47 cm field of view, 1 kg, ~100 ms capture | ✅ lateral / ⚠️ fine Z | **Wrist, coverage:** stop-and-shoot views of the whole module |
| **SICK Ranger3** (or calibrated Ruler3000) | 3D laser-triangulation camera | 2560 points/profile, up to 46 k profiles/s | ✅ | **Fixed over the conveyor: the baseline station** |
| **SICK midiCam2** | 2D GigE Vision camera, raw frames | 1–12 MP, global shutter | ✅ in-plane | **Fixed overhead at S1 (cell locating) and S5 (final visual). Wrist only if task 1.11 requires it** — revised 28 Oct 2026, see below. |
| SICK Inspector83x | 2D smart camera, processes onboard | no depth | ⚠️ in-plane | Superseded by midiCam2: the pipeline needs raw images |
| Stereolabs ZED 2i | passive stereo | < 1 % of range, ≳ 3 mm at best | ❌ | Not used for measurement |
| SICK multiScan136 | 3D lidar | statistical error ≤ 20 mm | ❌ | Not used |
| SICK multiScan165S | 3D lidar, safety-rated **PL b / Cat. B** for mobile machinery | systematic ±35 mm | ❌ | Not used: PL b is not enough to guard a robot cell |
| SICK picoScan150 LOC | 2D lidar with mobile-robot localisation | systematic ±20 mm; not a safety component | ❌ | Only if a mobile robot delivers parts |
| **SICK microScan3** | safety laser scanner (machine guarding) | — | n/a | **Cell safety:** speed and separation monitoring for the collaborative HC10DT (ISO/TS 15066) |

The consequence: the wrist carries a **snapshot 3D camera for coverage and a
line profilometer for precision**. Four sensors on one wrist was
never the right question. The right question is which sensor belongs *where*.

*(Superseded 28 Oct 2026 — this paragraph used to end "plus a 2D camera. That
is about 3 kg with the flange." Both the third sensor and the mass figure are
revised below.)*

#### Revision 28 Oct 2026 — the wrist 2D camera is removed by default

**The Zivid 2+ M60 already captures 5 MP colour 2D (2448 × 2048), registered
pixel-for-pixel to its own point cloud.** Every colour pixel carries an XYZ, so
there is no camera-to-camera calibration to do and no extra URDF frame to get
wrong. The midiCam2 was chosen on 15 Sep only as a raw-frame replacement for
the Inspector83x (which processes onboard). **It was not chosen for Isaac Sim
or ROS 2 compatibility**, and nobody checked whether the Zivid's own 2D made
it redundant.

Zivid 2D pixel footprint: ≈ **0.23 mm/px at 600 mm** (FOV 570 × 460 mm),
≈ 0.12 mm/px near its 300 mm minimum (D).

| Wrist 2D job | Feature size | Zivid 2D enough? |
|---|---|---|
| Polarity (fill plug, vent, terminal shapes) | several mm | ✅ |
| Weld discolouration / heat tint | seam 1–1.5 mm, 5–12 px | ✅ |
| Weld spatter | 0.1–0.5 mm particles, 1–4 px | ⚠️ marginal |

**Decision (A): wrist = Zivid 2+ M60 class + LJ-X8200 class profilometer.**
Add a wrist midiCam2 **only if task 1.11** (minimum detectable defect,
sensitivity to point spacing) shows that 0.12–0.23 mm/px misses spatter. That
makes the second camera a *measured requirement* rather than an assumption. A
dedicated 2D camera would win on pixel size (≈ 50 µm/px close up) and on
controlled ring-light illumination of shiny aluminium — if the measurement
says it is needed.

**midiCam2 keeps its fixed role:** overhead at **S1** (locating cells for the
GP7) and **S5** (final visual). Suggested variant **5 MP mono (I2D605M)**,
since locating needs edges, not colour; ≈ 0.16 mm/px over the module. If a
wrist 2D camera is ever added, **5 MP colour (I2D605C)**. *The part-number-to-
resolution mapping is inferred from SICK naming — verify on sick.com.*
midiCam2 resolutions are 1.58 / 3.15 / 5.01 / 12.29 MP, colour and mono,
global shutter, C-mount, body 42.7 × 53 × 41 mm. If one is ever modelled on
the wrist, **include the C-mount lens cylinder (≈ 30–50 mm)** — that is what
collides and occludes.

**Zivid 2+ M60 facts (S)** — [Stemmer Imaging product page]:

| Item | Value |
|---|---|
| Working distance | 300–1100 mm |
| FOV | 570 × 460 mm at 600 mm |
| 2D | 5.0 MP colour, 2448 × 2048 |
| Body | 169 × 124 × 56 mm, 1.0 kg, GigE / 10 GigE, M12 connectors |
| Status | Listed under **"Zivid 2+ (Legacy)"** on Zivid's downloads; the R-series (MR60 etc.) is current. The "M60 *class*" wording above covers this. |
| ROS 2 | Official driver `zivid_camera` (ROS index lists Jazzy) |
| Isaac Sim | Zivid ships an **experimental** Isaac Sim extension (2D capture, point cloud, FOV display, model dropdown). Its docs do not list models, noise or ROS 2. |

**midiCam2 in Isaac Sim is not confirmed as an asset — and it does not
matter.** In simulation every 2D camera is a pinhole model (resolution, FOV,
pose); brand is not a simulation decision. On real hardware it would use a
generic GigE Vision / GenICam driver.

**Payload must be recomputed.** The "≈ 3 kg" figure above included a midiCam2.
Recompute from datasheets: Zivid 1.0 kg + LJ-X8200-class head + mount +
cabling. The Keyence head mass is **TBD** — it is not in the repo and must not
be invented.

**Which sensor makes which measurement.** Zivid 2+ M60 datasheet figures:
point precision 80 µm (1σ), local planarity precision 100 µm, global planarity
trueness < 0.10 mm, spatial resolution 0.24 mm at 600 mm. A terminal pad is
22.0 × 18.5 mm *(corrected 26 Sep 2026 from ~21.5 × 17.5 — vendor drawing,
`module_spec.md` §4)*, so a plane fitted to it averages hundreds of points and the
*local* comparison of two adjacent pads (27 mm apart, same capture) is far
better than the single-point figure. But the 0.1 mm tolerance is the same size
as the camera's global trueness, so the snapshot camera **screens** and the
profilometer (1 µm) **adjudicates**:

| Measurement | Fixed line-scan | Wrist snapshot 3D | Wrist profilometer |
|---|---|---|---|
| Terminal height, adjacent Δh (0.1 mm) | **primary** — flat, top-down, part moves under it | screen only | **adjudicates** marginal cases |
| Cell pose (x, y, θ) | **primary** | yes | no (too narrow) |
| Busbar pose and lift-off gap | **primary** | yes | yes |
| Weld seam width, undercut, spatter | partial (top-down only) | screen | **primary** |
| Cell-to-cell gaps, side-plate welds, module flanks | **blind** | **primary** | with reorientation |
| Busbar side walls, shadowed terminal edges | **blind** | **primary** | with reorientation |
| Polarity, markings, discolouration | no | **primary** — the Zivid's own registered 5 MP 2D (28 Oct 2026) | no |

This table *is* the fixed-vs-arm hypothesis (§3.7.6) written as predictions.
§4B.6 measures whether it holds.

### 3.7.4 Finding C — cell-internal defects cannot be seen from outside

This is why the project does not inspect the inside of a cell. It is about
geometry, not sensor specs.

From the outside in, a stacked cell is separator (largest), then anode, then
cathode (inset 1.5 mm). A cathode that shifts 0.8 mm still sits **inside** the
anode footprint. The overhang margin drops from 1.5 mm to 0.7 mm. That is the
overhang violation, and **no external
surface scan can observe it**.

**Even the best profilometer does not help.** An LJ-X8020 viewing the stack's
side has the resolution: its 2.5 µm pitch resolves 20–90 µm layers. What it
lacks is line of sight. The separators protrude as fins about 185 µm apart,
with the electrodes recessed ~1 mm between them and the cathode a further
1.5 mm back. A triangulation sensor, with laser and camera at an angle, cannot
see to the bottom of slots that deep and narrow. In a Z-fold stack the
continuous separator also wraps two of the four faces. **This is an
optical-access limit, not a resolution limit.**

Industry agrees. It inspects **each sheet during stacking**, typically with
four synchronised cameras, one per corner, and uses X-ray/CT afterwards.

**It also exposed a contradiction in the old plan.** The README placed
inspection after the stack was complete. §9.9's loop, `u_{k+1} = −K·e_k`,
corrects the *next* item, and a finished stack has no next layer. **The loop
only makes sense with in-process inspection.** In the module line it is
natural: *k* is the cell index. Cell *k* is measured before cell *k*+1 is
placed.

### 3.7.5 The candidates compared

| | Cell internals (electrode stack) | Carbon-fibre AFP layup | **Battery module line** |
|---|---|---|---|
| Defects visible to an external optical sensor? | **No** (Finding C) | Yes | **Yes**: cell position, terminal height, busbars, welds, beads |
| Real tolerance with a mechanism | 0.2 mm stacker spec | gap/overlap limits (to source) | **> 0.1 mm height difference between adjacent terminals can cause a flawed laser weld** |
| Does the arm have a job? | Weak: 66 mm part, 1.5 s/unit, a fixed sensor is the obvious design | Yes: curved tool | **Yes**: large part, features on several faces, busbars block terminal edges |
| Is it an assembly line? | Yes, but inspection must sit inside the stacker | No: one process, one robot | **Yes**: four robots, each with a real job |
| Irreversible step to inspect before | can sealing | next ply | **laser welding** |
| Audience (controls and automation) | niche | aerospace niche | **mainstream integrator work** |

Carbon fibre was chosen first on detection grounds alone. The module line
replaced it once the project's purpose (§3.7.1) was part of the comparison.

### 3.7.6 Arm versus fixed sensor: the rule, and the question MotoBat answers

Industry's rule of thumb: **fixed sensors or gantries** for small, flat,
identical parts at high rate; **robot-mounted sensors** for large 3D parts with
occluded features and many variants. On car-body lines, systems from ZEISS
(AIMax), Hexagon (360 SIMS) and Perceptron measure 150–300 features per body
within a 60–90 s cycle.

A module sits between the two cases. That makes it the right place to ask:

> **Does a robot-mounted multi-view sensor find defects that a fixed line-scan
> sensor misses, and at what cost in cycle time?**

Measured as coverage (% of required surfaces observed), recall per defect
class, and seconds per module, for Ranger3-on-conveyor vs HC10DT-multi-view.
If the fixed sensor wins for some classes, that is still a result, and it is
the question an automation engineer asks first.

**Design principle:** *use the arm for coverage and the sensor for measurement.*
Critical comparisons (e.g. the height of adjacent terminals) happen **within
one capture**, so the arm's path accuracy never enters a 0.1 mm measurement.
The arm decides *where* to look. The sensor frame does the metrology.

### 3.7.7 What carries over

**Carries over unchanged:** the scan simulator approach, ICP registration, the
residual metric, the precision/recall harness, `motobat_interfaces`, the §9.9
loop and its stability bound, the four-arm description, and the Gazebo
control path.

**Cell internals → future work with a battery-technology collaborator.**
A specific hypothesis to take to her: **after each layer is placed, the
anode's 90 µm edge should leave a step in the 20 µm separator laid over it.
A profilometer might measure cathode-to-anode overhang *through* the
separator.** If it works, it is a 3D alternative to today's 2D corner cameras.
Testing it needs real separators and cells: whether the separator conforms,
and how the foil and coating reflect the laser.

**Carbon fibre → the follow-up generality project**, as the README originally
planned.

### 3.7.8 Corrections recorded so they are not re-made

- *"CF defects are visible to a decent depth camera."* Wrong for gaps and
  overlaps (a ~0.15 mm step against the ZED's ≳ 3 mm).
- *"Isaac Sim cannot render 205 µm layers."* Imprecise. Any ray-cast sensor
  returns geometry at profilometer precision. What a simulator cannot
  reproduce is the optics: specular reflection, speckle, multi-bounce. Our own
  ray-cast models in `defectgen/scan.py` *are* the sensor models.
- *"We need an LJ-X8040."* Keyence's current table has no 8040. The 16 mm /
  0.4 µm head is the **LJ-X8030**.
- *"Put a small X-ray sensor on the HC10DT to see inside the cell."* **Rejected
  for the wrist, kept as future work.** Transmission X-ray needs the source and
  detector on **opposite sides** of the part, so a single wrist cannot do it.
  Single-sided backscatter exists, but its resolution is around 0.3 mm to
  millimetres, too coarse for 20–90 µm layers. Ionising radiation also needs a
  shielded, interlocked enclosure, which defeats the point of a collaborative
  arm sharing space with people. X-ray is the right physics for cell internals.
  Industry uses 2D radiography of the four corners, where anode and cathode
  show up as distinct grey zones, and CT for full volume. But it belongs at
  the cell factory, in a shielded cabinet, before cells ever reach a module
  line. The robotic version is real: Fraunhofer's **RoboCT** uses two
  cooperating industrial robots, one carrying the source and one the
  detector, at ~50 µm resolution. It is also simulable with the open-source
  **gVirtualXray** (`pip install gvxr`; Beer–Lambert projection through
  triangle meshes). That makes it a candidate for the cell-level
  collaboration (§3.7.7), not for this line.

### 3.7.9 Open items before results are claimed

1. **Module geometry defaults.** Cell: VDA PHEV2 format, 148 × 91 × 26.5 mm.
   Now fully specified in [`module_spec.md`](module_spec.md): VDA 355
   envelope, 3P4S, and a 1.5 mm Al busbar on a 4 mm Al terminal, all sourced.
   The Huiyao Laser page is **unreliable**: it labels 148 × 91 × 26.5 as
   "EV1", contradicting batterydesign.net. Its 66 mm terminal spacing is
   discarded. Terminal spacing, pad size and vent are to be measured from
   photos (module_spec §6).
2. **Tolerances.** 0.1 mm adjacent-terminal height difference (MDPI
   *Batteries* 6(3):47 via search; confirm from the paper). Cell placement,
   busbar gap and weld seam limits: *to source*.
3. **Cycle-time benchmark** for a module line: *to source*. Search found the
   process sequence but no takt figure.
4. **Reporting convention.** Measured characteristics (terminal height, cell
   position) get a **Gage R&R / %GRR** in automotive MSA form, which is the
   language automotive quality engineers use. Attribute defects get
   precision/recall with confidence intervals.

### 3.7.10 NHTSA 26V188, read from the report — what it actually says

Earlier drafts described 26V188 as an electrode-overhang violation. **That was
wrong**, and it mattered, because the real cause is closer to this project than
the invented one. From the Part 573 report
([PDF](https://static.nhtsa.gov/odi/rcl/2026/RCLRPT-26V188-8548.pdf)), filed
31 Mar 2026 by Nissan North America, 51 MY2026 LEAF vehicles, 78 kWh packs,
cells by Automotive Energy Supply Corporation:

> "damage to the edges of the battery cell cathode material in the subject
> batteries may have occurred during the supplier's manufacturing process. If a
> cathode with a torn edge is assembled into a battery cell, the damaged portion
> of the cathode may fold onto itself, potentially resulting in an internal short
> circuit within the battery module."

And the mechanism, from the chronology:

> "after the cathode is cut to shape, a **misalignment during transfer** to a
> subsequent production step may allow the cathode to **contact manufacturing
> equipment**, which can result in a **torn cathode edge**."

Three things follow, and all three support the project's thesis:

1. **The root cause is a handling misalignment, not an electrochemical fault.**
   A part was placed inaccurately, touched equipment, and was damaged. That is
   precisely the failure mode a placement-correction loop (§9.9) addresses.
2. **The defect is optically observable at the moment it is created** — a torn
   edge on a flat, exposed cathode sheet, before stacking. Unlike the buried
   overhang of §3.7.4, this one *can* be inspected geometrically.
3. **It was caught electrically, at end of line, after two thermal events.** The
   remedy included revising the end-of-line Charge-Discharge Check. That is the
   "inspect after the irreversible step" pattern this project argues against.

**Consequence for scope.** The station that would have prevented 26V188 is
cathode-sheet transfer in the cell plant, not module assembly. The module line
stays the primary build (§3.7.5), and §4B.9 adds a small, honest experiment
that replicates the recall mechanism itself in the same numpy pipeline.

**Sources for this section:**
[Keyence LJ-X8000 specs](https://www.keyence.com/products/measure/laser-2d/lj-x8000/specs/) ·
[Zivid 2+ M60](https://www.zivid.com/zivid-2-plus-m60) ·
[SICK Ranger3 datasheet](https://www.sick.com/media/pdf/7/47/947/dataSheet_V3DR3-60NE31111_1091560_en.pdf) ·
[SICK midiCam2](https://www.sick.com/br/en/catalog/products/machine-vision-and-identification/machine-vision/midicam2/c/g541250) ·
[SICK multiScan165S datasheet](https://www.sick.com/media/pdf/8/98/398/dataSheet_MULS1AAS-114322-multiScan165S_1143873_en.pdf) ·
[SICK picoScan150 LOC datasheet](https://www.sick.com/media/pdf/1/71/471/dataSheet_PICS150-01000-LOC-Prime_1141396_en.pdf) ·
[SICK multiScan136 datasheet](https://www.sick.com/media/pdf/2/32/032/dataSheet_MULS1AA-112211-multiScan136_1140133_en.pdf) ·
[ZED 2i datasheet](https://www.mouser.com/datasheet/2/1520/StereoLabs_ZED_2i_Datasheet-3401402.pdf) ·
[Basler — stacking QA, per-corner cameras](https://www.baslerweb.com/en/use-cases/stacking-battery-assembly/) ·
[Optical measuring in laser-welded EV battery modules (MDPI Batteries 6(3):47)](https://www.mdpi.com/2313-0105/6/3/47) ·
[LMI — 3D smart sensors for inline EV battery inspection](https://chargedevs.com/whitepapers/3d-smart-sensors-for-inline-ev-battery-inspection/) ·
[ZEISS AIMax](https://www.zeiss.com/metrology/products/systems/process-control-and-inspection/car-body-solutions/aimax.html) ·
[Robot-mounted gap & flush](https://metrology.news/inline-dual-sensor-automotive-gap-flush-robotic-inspection/) ·
[VDA prismatic cell formats](https://www.batterydesign.net/vda-prismatic-cells/) ·
[Prismatic module line process sequence](https://www.linkedin.com/pulse/prismatic-ev-lithium-battery-module-pack-assembly-line-sam-lau) ·
[Electroimpact AFP inspection (CF comparison)](https://www.electroimpact.com/whitepapers/2017-01-2150.pdf)

**Added 28 Oct – 1 Nov 2026** (sensor revision, flange interface and CAD):
[Zivid CAD index](https://www.zivid.com/documentation#cad) (not linked from the product page) ·
[Zivid 2+ M60 with FOV — STEP](https://www.zivid.com/hubfs/Zivid%202%20Plus%20M60%20with%20FOV.step) ·
[Zivid 2+ M60 with FOV — STL](https://www.zivid.com/hubfs/Zivid%202%20Plus%20M60%20with%20FOV.stl) (rev 1.0, 08/23) ·
[Zivid ROS 2 driver `zivid_camera`](https://index.ros.org/p/zivid_camera) ·
[Zivid Isaac Sim extension (experimental)](https://support.zivid.com/api-reference/wrappers/isaac-sim.html) ·
[Zivid downloads — 2+ listed as Legacy](https://www.zivid.com/downloads) ·
SMC JMHZ2-X7400B gripper catalogue (HC10DTP flange: ISO 9409-1-50-4-M6) ·
Schmalz ISO 9409-1 flange set for HC10DT (63 mm pitch circle — unconfirmed)

---

## 4A. ACTIVE WORK PLAN — RViz2 + Gazebo (no Isaac Sim)
 
**This is what to work on now.** Everything here runs in 8 GB and none of it is
throwaway: the MoveIt configs, the flange, and the controller wiring all carry
directly into Isaac Sim afterwards.
 
The strategic point: once `MoveIt2 → FollowJointTrajectory → Gazebo` works,
swapping Gazebo for Isaac Sim is a **hardware-interface change, not a rewrite**
— the same seam that makes the MotoROS2 hardware story true (§6). Doing Gazebo
first de-risks Isaac Sim rather than delaying it.
 
### 4A.1 — GP4 with MoveIt2 in RViz (start here)

> **Re-sequenced (15 Sep).** The GP4's job is busbar placement plus the
> simulated weld pass (§4B). *(4 Nov 2026: the weld moves to a GP25 + MOTOPAC-RL2D,
> §4A.7. The GP4 keeps busbar-carrier, CMC and cover placement, about 0.3 kg at
> most.)* It comes **after** the HC10DT (§4A.2), which
> carries the inspection contribution. The GP7 places cells. No OPW file ships
> for the GP7 (verified), so derive its OPW parameters from the URDF joint
> origins and check them against forward kinematics before trusting them.
 
Goal: drag the interactive marker in RViz, hit Plan & Execute, watch the GP4
follow.
 
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
 
Load `motoman_gp4_support/urdf/gp4.xacro`. Then:
 
- Generate the self-collision matrix
- One planning group: `gp4_arm`, kinematic chain `base_link` → `tool0`
- Add named poses: `home`, `ready`
- Output to `src/motobat_stacker/config/`
**Use the analytic IK solver, not KDL.** The GP4 has an ortho-parallel basis
and spherical wrist, and the vendor package ships the parameters for it:
 
```
motoman_gp4_support/config/opw_parameters_gp4.yaml
    a1: 0.000   a2: -0.015   b: 0.000
    c1: 0.330   c2: 0.260    c3: 0.290   c4: 0.072
    joint_offsets:          [0, 0, deg(-90), 0, 0, deg(180)]
    joint_sign_corrections: [1, 1, -1, -1, -1, -1]
```
 
Install `moveit_opw_kinematics_plugin` and set it in `kinematics.yaml` instead
of the default `KDLKinematicsPlugin`. Closed-form IK is faster and returns all
8 solution branches rather than one numerically-converged answer — the same
choice a real integrator makes, and worth a sentence in the proposal.
 
### 4A.2 — HC10DT: expect this to be harder, and know why
 
**The HC10DT is NOT an ortho-parallel/spherical-wrist arm, so OPW does not
apply.** Verified from the URDF joint origins:
 
| Joint | GP4 origin (xyz) | HC10DT origin (xyz) |
|---|---|---|
| `joint_4_r` | `0.290 0 0.015` | `0 0 -0.500` |
| `joint_5_b` | `0 0 0` | **`0 0.162 0`** ← wrist offset |
| `joint_6_t` | `0 0 0` | `0 0 0` |
 
On the GP4, joints 4/5/6 intersect at a point — a spherical wrist, which is
what makes the closed-form OPW solution valid. On the HC10DT, `joint_5_b` is
offset 162 mm in Y, so the wrist axes do **not** intersect. This is normal for
collaborative arms and is precisely why the vendor package ships no
`opw_parameters_hc10*.yaml` (confirmed: `motoman_hc10_support` has no `config/`
directory at all).
 
Consequences to plan for:
 
- Use **KDL** (default) or build an IKFast solution for the HC10DT
- Numerical IK is slower and can fail to converge near singularities
- Do not spend hours hunting for a missing OPW file — it does not exist, and
  the geometry is the reason
This asymmetry is itself a finding worth reporting: it is a concrete,
measurable difference between the industrial GP-series and collaborative
HC-series arms, and it reinforces the Section 2.1 argument that the two classes
suit different tasks.

#### As built — 6 Nov 2026 (`motobat_hc10dt_moveit_config`)

**What exists.** A MoveIt config written by hand in the exact layout the Setup
Assistant generates (it still opens there). The robot is
`motobat_description/urdf/inspector_station.urdf.xacro`: the HC10DT with the
`s3_` prefix at its line pose in `world`. Station poses now live in one file,
`urdf/line_layout.xacro`, which the full line also reads. The refactor was
checked by expanding `motobat_line.urdf.xacro` before and after: all 464
elements are identical. One planning group, `inspector_arm`
(`s3_base_link` → `s3_tool0`). One named pose, `ready`: forearm forward,
`tool0` straight down at (0.500, 0.162, 0.805) m in `s3_base_link`, checked
by FK from the URDF. The 0.162 is the wrist offset. All-zeros is a poor
start, because `joint_3_u = 0` folds the forearm down beside the upper arm.

**Self-collision matrix.** Generated with `collisions_updater`, the command-line
tool behind the Setup Assistant's matrix page: 100 000 random states, 12 of 45
pairs disabled (6 adjacent, 6 never in contact). Nothing touches at
all-zeros, so the "default" option is not used: if a future tool collides at
zero, that collision should be visible. Rerun
`scripts/regen_collision_matrix.sh` whenever geometry changes (task 2.4).

**IK solver: KDL, by measurement.** KDL, TRAC-IK (Speed and Distance) and
pick_ik were compared on identical targets (`scripts/compare_ik.py`). There
were 1000 per set, 50 ms per call, seeded from `ready`. An answer was accepted
only within 0.1 mm / 0.06° by FK. Success is counted over targets any solver
reached, so every miss is a real solution the solver did not find.

| solver | down (tool vertical) | cone (0–45° tilt) | random pose | median solve |
|---|---|---|---|---|
| **KDL** | **100.0 %** | 99.6 % | **97.7 %** | 0.5–2.9 ms |
| TRAC-IK Distance | 99.9 % | 99.8 % | 94.7 % | 51 ms |
| pick_ik (global) | 99.7 % | 99.5 % | 90.1 % | 5–10 ms |
| TRAC-IK Speed | 98.1 % | 97.3 % | 91.8 % | 0.5–0.7 ms |

- On the inspection-like sets the top three are within a few targets in 1000.
  KDL matches them at a tenth to a hundredth of the time. A 300-target run
  showed pick_ik and TRAC-IK slightly ahead; at 1000 that was noise.
- TRAC-IK Distance returns slightly shorter moves (median joint-time bound
  0.72 s against KDL's 0.86 s on random poses), because it searches the whole
  timeout for the answer nearest the seed. On the inspection-like sets the
  difference is about 0.01 s. Worth revisiting in 2.5, where move time is
  the objective.
- **TRAC-IK Speed's MoveIt plugin gives up when its first answer fails the
  collision check** instead of searching again. 25 of its 27 random-set
  misses had a colliding answer when re-solved without collision checking.
  Planning always collision-checks, so this is how it would behave in use.
- The project_swarm experience (KDL failing on a downward grasp) does not
  carry over. That was a myCobot 280 near its wrist singularity, some targets
  were outside its reach, and the timeout was 5 ms. Here it is 50 ms on a
  1.2 m arm.

**Planners.** OMPL (default), Pilz, CHOMP and STOMP all load. Pilz's
PTP / LIN / CIRC are the same set of moves as Yaskawa INFORM's `MOVJ` / `MOVL` /
`MOVC`, the vocabulary an integrator already uses.

**Acceleration limits are assumed (tag A).** Yaskawa does not publish them.
Without them MoveIt silently uses 1 rad/s² for every joint.
`config/joint_limits.yaml` assumes each axis reaches full speed in 0.5 s
(a = 2·v_max). **Revisit before task 2.7 reports a cycle time.**

**Known noise.** `move_group` often segfaults in its executor destructor on
Ctrl-C. It happens with every IK plugin, KDL included, and in Gazebo too, but
only after the SIGINT, when all work is done. Harmless.

**How to run it** (verified 6 Nov 2026, headless and in RViz):

```bash
# Terminal 1: MoveIt + RViz, simulated hardware (no physics)
cd ~/projects/MotoBat-arm && source motobat_env.sh
ros2 launch motobat_hc10dt_moveit_config demo.launch.py
# Terminal 2: acceptance check, an OMPL joint move then a Pilz LIN; prints PASS
ros2 run motobat_hc10dt_moveit_config plan_exec_check.py
# On its own (Ctrl-C Terminal 1 first): the IK solver benchmark
ros2 run motobat_hc10dt_moveit_config compare_ik.py
```

**To fold in at merge** (docs the Phase 1 lane owns): `WORKFLOW.md` gets the
recipes here and in §4A.3; `project_structure.md` marks `motobat_hc10dt_moveit_config` and
`motobat_bringup` as existing and lists the new `motobat_description` files
(`line_layout.xacro`, `inspector_station*.xacro`, `hc10dt_macro.xacro`).
 
### 4A.3 — Gazebo with ros2_control
 
Gazebo Harmonic is installed (`ros-jazzy-ros-gz`). Add a `ros2_control` block
to the arm URDF using `gz_ros2_control/GazeboSimSystem`, configure a
`joint_trajectory_controller`, and launch MoveIt2 against it.
 
**This is the rehearsal for Isaac Sim.** The controller config, the action
interface, and the MoveIt setup are all reused; only the hardware plugin
changes (`GazeboSimSystem` → `JointStateTopicSystem`, §4.2).
 
One arm at a time. Gazebo with a single arm is comfortable in 8 GB.

**Loading the module meshes (added 1 Nov 2026).** The battery CAD now exists
(`src/motobat_description/meshes/module/`, see `module_spec.md` §7), so task
2.6 can load it rather than box primitives. Three things to get right:

- **Gazebo cannot load STEP.** It loads STL / OBJ / DAE / glTF. STEP is the
  archive format only.
- **STL carries no units and Gazebo assumes metres.** The exports are in
  millimetres, so every mesh tag needs `<scale>0.001 0.001 0.001</scale>` or
  the module appears **1000× too large**. Expect this to be the first failure.
- Each part's origin must sit at its `module_spec.md` §7 named datum (cell:
  centre of the can's bottom face) so parts can be placed by transform alone.

#### As built — 6 Nov 2026 (`motobat_bringup/launch/inspector_gazebo.launch.py`)

**What exists.** The Inspector in Gazebo Harmonic (gz-sim 8), driven by
MoveIt through the same `FollowJointTrajectory` → `inspector_arm_controller`
path as the mock setup. The only change is the hardware plugin: the xacro arg
`sim:=gazebo` selects `gz_ros2_control/GazeboSimSystem`, and that plugin hosts
the controller_manager. `ros2_controllers.yaml` is shared by both setups.

**Mass properties had to be added.** The vendor HC10DT URDF has no
`<inertial>` tags. Gazebo drops a massless link, so the arm would not
assemble. `motobat_description/scripts/hc10dt_inertials.py` computes each
link's volume, centre of mass and inertia tensor exactly from its vendor
collision mesh (signed tetrahedra; checked against a unit cube). It assumes
one uniform density, scaled so the arm totals **48 kg (S, Yaskawa EU product
page, standard HC10DT; the IP67 variant is 58 kg)**. The resulting density is
984 kg/m³, plausible for a hollow aluminium arm. The per-link split is
**A**: real motors and gearboxes sit nearer the joints. That is fine for
position-commanded simulation, but not for dynamics identification or
effort-based collision detection. The numbers are in
`config/hc10dt_inertials.yaml`. URDF cannot add tags to links another macro
already defined, so `urdf/hc10dt_macro.xacro` is a copy of the vendor macro
(commit 93a282d) whose only additions are those inertials. The line and the
MoveIt config both use it.

**Position gain: 1.0, not the plugin's 0.1.** `GazeboSimSystem` turns a
position command into a joint velocity, v = k·(q_cmd − q)·f_update, so the
joint lags a moving setpoint. That lag is a simulation artifact; a real
controller tracks its own path tightly. Left alone, it would bend every LIN
scan pass. Measured with `plan_exec_check.py` (20 % speed, 100 Hz):

| k | worst tracking error, OMPL joint move | worst tracking error, Pilz LIN |
|---|---|---|
| 0.1 (plugin default) | 2.35° | 1.71° |
| 0.5 | 0.29° | 0.23° |
| **1.0** | **0.026°** | **0.022°** |
| 1.5 | 0.13° | 0.09° |
| mock hardware, for reference | 0.004–0.024° | 0.003–0.015° |

k is the fraction of the error corrected per control cycle, so it is stable
for 0 < k < 2 and fastest without overshoot at 1. The sweep agrees: 1.5
overshoots. The plugin reads the gain only at start-up; `ros2 param set`
reports success but changes nothing.

**Verified.** The arm holds `ready` under gravity (0.0000° drift over 10 s),
and Gazebo runs at a real-time factor of 1.00 on this machine, server only.
`plan_exec_check.py` passes in Gazebo: joint move within 0.04°, LIN within
0.001 mm of target. The GUI starts with no mesh errors (headless check only;
not yet inspected by eye).

```bash
# Terminal 1: Gazebo + MoveIt + RViz (gui:=false rviz:=false for headless)
cd ~/projects/MotoBat-arm && source motobat_env.sh
ros2 launch motobat_bringup inspector_gazebo.launch.py
# Terminal 2: acceptance check; prints PASS
cd ~/projects/MotoBat-arm && source motobat_env.sh
ros2 run motobat_hc10dt_moveit_config plan_exec_check.py
```

Not done here: the module meshes and simulated sensors (task 2.6), and speeds
above 20 %, which task 2.7 will need with real acceleration limits.

### 4A.4 — Custom sensor flange for the HC10DT

> **Redefined by §3.7.3 (15 Sep), revised 28 Oct 2026.** The wrist carries a
> **snapshot 3D camera (Zivid 2+ M60 class) for coverage and a line
> profilometer (LJ-X8200 class) for precision**. The 2D camera is **removed by
> default** — the Zivid's own registered 5 MP 2D covers polarity and heat
> tint; a midiCam2 is added only if task 1.11 shows it misses spatter
> (§3.7.3). **Payload: recompute** — the old "about 3 kg" included a midiCam2,
> and the Keyence head mass is TBD. The SICK lidars are not wrist sensors:
> microScan3 guards the cell, and the others have no inspection role. The
> design questions become: occlusion between the fields of view, standoff
> compatibility (snapshot at ~60 cm, profilometer at ~245 mm), and cable
> routing through the wrist. The URDF-from-CAD warning below applies in full.
 
The highest-value design work available right now, because it converts a
Section 4 *assertion* into a verified result.
 
> **Proposal status.** Proposal §4's sensor array, and its "officially
> validated vendor digital twins" argument, are **superseded** by §3.7.3
> (15 Sep) and the 28 Oct revision above. The proposal `.docx` is not tracked
> in this repo and has not been edited; this note is the record.

Section 4 of the proposal states the Inspector carries four sensors. That was
an assumption, and §3.7.3 answered it: **two**, with a third gated on task
1.11. Building the flange tests what remains:
 
- ~~Do the SICK microScan3, multiScan136, Inspector83x, and ZED 2i physically
  fit on one HC10DT wrist?~~ *(superseded 15 Sep by §3.7.3 — none of those
  four is a wrist sensor. The question is now whether the Zivid 2+ M60 and
  the LJ-X8200-class head fit together.)*
- Do they occlude each other's fields of view? **Housing-to-housing occlusion
  is a real constraint, and so are the module's own 101.5 mm walls — they
  block the outer terminal edges below ≈ 39° elevation (`module_spec.md` §5).**
- Is the total mass within the HC10's 10 kg payload, including the flange?
- Are the standoff distances compatible with each sensor's working range?
  **They are not co-located: Zivid ≈ 600 mm nominal (300–1100), LJ-X8200
  ≈ 245 mm. That implies separate arm poses per sensor, not one pose that
  serves both.**
**Any answer is a real result.** If they do not fit, discovering that now — and
reporting it with the geometry to prove it — is far better than assuming it
works.
 
Build it **parametrically** (FreeCAD or Blender), with sensor poses as
variables rather than baked geometry, so layouts can be swept rather than
remodelled.
 
> **The URDF is ground truth, not the STEP file.** Every sensor TF frame flows
> from the `<origin xyz rpy>` values in the xacro. A 5 mm or 2° mismatch
> between CAD and URDF silently corrupts every point-cloud registration
> downstream, and it will look like an algorithm bug. Derive the URDF origins
> from the CAD numerically; do not eyeball them.
 
Deliverables *(restated 28 Oct 2026 — item 2 previously read "four sensor
frames", which the §3.7.3 revision supersedes)*:
 
1. Parametric flange model (STEP + source)
2. `motobat_description/urdf/inspector_flange.xacro` — flange + **two sensor
   frames (snapshot 3D, profilometer), plus a third only if task 1.11
   requires a wrist 2D camera**, attached to `s3_tool0`
3. Mass/payload check against 10 kg — **recompute; the old ≈ 3 kg included a
   midiCam2, and the Keyence head mass is TBD**
4. FOV/occlusion study — even a simple ray-cast or a documented sketch. Must
   cover housing-to-housing occlusion **and** the ≈ 39° wall limit
5. TF tree verified in RViz: `ros2 run tf2_tools view_frames`

#### Mount CAD as built — 1 Nov 2026

`cad/inspector/inspector_flange.step` and
`src/motobat_description/meshes/inspector/inspector_flange.stl`.

| STEP product | What it is |
|---|---|
| `HC10 Flange V0.0` | top-level assembly |
| `Mount v3` | the adapter plate — **AL-5052** |
| `Zivid 2 Plus M60 with FOV v1` | Zivid 2+ M60 |
| `AS_108261` | **Keyence LJ-X8200** (confirmed 1 Nov 2026) |

No midiCam2, which matches the 28 Oct decision. If one is ever added, keep it
and record its status as "pending task 1.11" rather than deleting it.

**There is no FOV geometry in the file**, despite the product name — that is
just Zivid's download filename. The view cone will be generated in code for
TF2 / RViz visualisation, which is the better approach anyway: it can follow
the datasheet's 300–1100 mm working range instead of being frozen at one
distance. **If an FOV solid is ever imported, exclude it from both collision
and visual meshes**, or MoveIt treats the cone as solid and every scan pose
"collides".

**Payload — resolved 1 Nov 2026:**

| Item | Mass | Tag |
|---|---|---|
| Zivid 2+ M60 | 1.0 kg | S (datasheet) |
| Keyence LJ-X8200 head | 1.2 kg | S |
| `Mount v3` (AL-5052) | from CAD | — |
| Cabling + flange hardware | ≈ 0.3–0.5 | A |
| **Total** | **≈ 2.5–3.0 kg** of the HC10DT's 10 kg | D |

The old "≈ 3 kg" estimate survives for different reasons: dropping the
midiCam2 freed roughly what the Keyence head costs over what was assumed.

> **Take the sensor masses from the datasheets, not from the CAD.** The camera
> bodies are placeholder shells; whatever material they carry in Fusion is
> meaningless. Read `Mount v3`'s mass in isolation and add the two datasheet
> figures. Note also that the **LJ-X8000A controller is not on the arm** and
> must not be added — but the head-to-controller cable routes through the
> wrist, which is one of the open design questions above.

> **Size note.** The STL is **99 MB** and the STEP 69 MB, both committed raw;
> the STL is just under GitHub's 100 MB hard limit. Accepted for now
> (1 Nov 2026) — the detail is wanted and §9 only requires *collision*
> geometry to be simple, not visuals. Revisit on **performance**, not on the
> limit: if RViz or Gazebo turns sluggish, or `defectgen` ray-casting is slow,
> this file is the first suspect, since the sensor models intersect against
> every triangle in it.

#### HC10DT tool-flange interface

The robot's tool-flange holes are **tapped M6 directly, with no inserts**. The
adapter plate gets clearance holes and M6 socket-head cap screws pass through
it into the robot. The 5th hole is a **Ø6 H7 locating (dowel) hole** that
fixes rotation; a centring diameter (spigot/recess) locates concentrically,
and the bolts only clamp.

**It is the DT pattern — resolved 1 Nov 2026 by measurement.** The proposal
names the **HC10DTP**, but the URDF is `hc10dt`, and the measurement agrees
with the URDF: the proposal was wrong.

| Variant | Pattern | Tag |
|---|---|---|
| HC10DTP | **ISO 9409-1-50-4-M6**: 4 × M6 on a **50 mm** pitch circle, Ø6 H7 pin hole 5 deep, ø31.5 centring diameter | S — SMC JMHZ2-X7400B gripper catalogue |
| **HC10 / HC10DT ← this one** | **63 mm** pitch circle, M6 threads, Ø6 H7 pin hole. Most likely **ISO 9409-1-63-4-M6**. | **S — measured 1 Nov 2026**; the 63 mm figure also matches Schmalz's ISO 9409-1 flange set for the HC10DT |

**How it was settled.** Opposite hole centres *are* the pitch circle diameter,
and they measure **63 mm** — 13 mm larger than the DTP's 50. The other two
measurements confirm the thread but do not discriminate: the 5 mm holes are
the M6 tapping drill, and the 6 mm keying hole is the Ø6 H7 dowel. Both
variants share those. **Only the PCD separates them.**

Adapter plate: 4 × Ø6.6 counterbored clearance holes on **PCD 63**; Ø6 H7 hole
with a Ø6 m6 dowel pin. Screw length = plate thickness + ≈ 6–9 mm engagement,
short of the tapped depth. Take depth and torque from the manual.

> **The ø31.5 centring diameter does not carry over.** That figure is from the
> SMC **DTP** spec and is unverified for the DT. Tag **A** until confirmed
> from Yaskawa's HC10DT manual drawing.

**In simulation** the plate is a fixed joint to `s3_tool0`. Bolts and pins are
not simulated. Model holes as plain cylinders and **never model threads** —
triangles the ray caster intersects for nothing. What matters is
**orientation**: `tool0` sits at the flange-face centre with Z out of the
face, and the dowel hole fixes the X direction. The mount CAD's origin and X
axis must coincide with `tool0`, so sensor frames derived numerically from CAD
match the URDF (the rule above).

#### As built so far — 7 Nov 2026 (task 2.4, in progress)

**Pipeline: CAD → numbers → xacro, with no hand typing.**
`scripts/step_geometry.py` reads the STEP's exact analytic surfaces (planes,
cylinders) per part with numpy alone. `scripts/step_frames.py` locates features
and writes `config/inspector_flange.yaml`. `urdf/inspector_flange.xacro` loads
it, and `inspector_station.xacro` bolts it to `s3_tool0`, so the line and the
MoveIt config both carry it. After any Fusion change, re-run `step_frames.py`
and `regen_collision_matrix.sh`.

**Checked from the CAD, not assumed:** the Mount origin is the centre of the
bolt circle (PCD **63.000 mm**, 4 holes) on the robot-side face (z = 0), and +Z
points away from the robot. That matches `tool0`'s origin and Z.

**Frames now in the URDF** (`s3_tool0 → s3_flange → s3_zivid_mount, s3_ljx_mount`):

| frame | origin in flange frame (mm) | how it is defined |
|---|---|---|
| `zivid_mount` | (0, −57.0, 78.6) | centroid of the 3 Zivid screw holes the bracket counterbores, on the contact face; Z = mean of the two front-window normals (toed in 1.4° each, symmetric) |
| `ljx_mount` | (22.0, 27.0, 15.0) | centroid of the 3 LJ-X8200 screw holes matching the bracket's, on the bracket face; Z = laser direction |

Both sensors look along +Z. The Keyence receiver bore is tilted 21.7° off Z,
and its axis meets the laser plane about 258 mm past the emitter window,
consistent with the datasheet's ≈ 245 mm reference distance.

**Not yet: the optical frames.** The Zivid camera origin and the LJ-X laser
emission / reference point are defined by the vendors, relative to these
mounting holes. They will be datasheet offsets (S) on top of the mount frames.
The CAD cannot settle them: the Zivid's two windows are geometrically identical
(camera vs projector), and the LJ-X has two candidate exit slots about 5 mm apart.

**PENDING — `flange_yaw`, the plate's rotation about `tool0`.** The plate's Ø6
dowel hole is on its **+Y** axis, at r = 34.0 mm. The vendor `link_6_t` visual
mesh, the only HC10DT flange geometry available, shows its two extra holes on
`tool0`'s **±X** axis. Either the plate is rotated 90° relative to `tool0`, or
it was modelled to a different convention, and the sign is open too. Set
from Yaskawa's flange drawing or the Fusion assembly. Until then
`flange_yaw = 0` is a placeholder.

**Visual and collision.** The visual mesh is decimated from the 99 MB export
to 60 000 triangles / 3 MB (`decimate_flange_mesh.py`; deviation median 0.000,
p99 0.009, max 0.23 mm). Collision is one box per part, with a 2 mm margin. No
FOV geometry exists in the STEP or STL (checked), so nothing invisible collides.
The regenerated collision matrix adds 3 disabled pairs: `link_6_t` adjacent,
and `link_3_u` and `link_5_b` never in contact. The flange's pairs with the
other links stay checked.

**Mass: 2.7 kg of 10** (Zivid 1.0 S, LJ-X 1.2 S, plate + cabling 0.5 A),
each part treated as a solid box for the inertia. `plan_exec_check.py`
passes with the flange on, on mock hardware and in Gazebo. In Gazebo the arm
holds its pose (0.0000° drift), and the real-time factor is 0.98.

**For the next Fusion revision (task 2.3):**
- **The bolt holes are Ø5.0, the M6 tapping drill.** An M6 screw needs a
  Ø6.6 clearance hole through the plate, so the plate cannot be bolted on
  as modelled.
- **The STEP's `Mount v3` body is orphaned.** It is written as two identical
  closed shells that no part references, so a strict STEP reader loads the
  file without the plate. `step_geometry.py` adopts it and says so. Re-export.
- **The LJ-X8200 top sits 0.3 mm past the bracket face.** That is either an
  interference or a locating step: check in Fusion.
- The Fusion User Parameters (sensor poses) are not in the STEP. A copy of
  the parameter table would cross-check the frames above.
### 4A.5 — Deferred: simulated network switch for DDS
 
Considered and **deliberately deferred.** Recording the reasoning so it is not
revisited by accident:
 
- It targets Cyclone DDS, which §5 concludes is likely the wrong RMW for this
  project (NVIDIA recommends FastDDS for single-machine Isaac Sim; MotoROS2
  *requires* FastDDS). Building infrastructure around Cyclone deepens a
  commitment that is about to be reversed.
- DDS discovery already works across a real network. Two machines and
  `ROS_DOMAIN_ID` give genuine multi-host behaviour; a software switch mostly
  models one's own model of a switch.
- It does not advance the stated contribution — closed-loop geometric defect
  detection. It is adjacent infrastructure that could absorb weeks.
**Redirect that interest into latency instrumentation instead.** Section 3.2
promises detection-and-correction latency benchmarked against the 1.5–2.5 s/unit
industrial cycle time. Measuring where time actually goes — sensor publish →
registration → correction command — using `rosbag2` timestamps and QoS
experiments produces a number the proposal *needs*, while teaching real DDS
behaviour. Same curiosity, directly load-bearing.
 
### 4A.6 — Suggested order
 
| # | Task | Rough effort | Blocks |
|---|---|---|---|
| 1 | GP4 MoveIt config + OPW IK, marker-driven in RViz | 1–2 days | everything |
| 2 | HC10DT MoveIt config (KDL) | 1–2 days | flange work |
| 3 | Gazebo + `ros2_control` + `joint_trajectory_controller`, one arm | 2–3 days | Isaac Sim swap |
| 4 | Flange CAD + sensor placement study | 3–5 days | sensor sim |
| 5 | Flange → URDF with exact frames, TF verified in RViz | 1–2 days | registration work |
 
Steps 1–3 rehearse the full Isaac Sim control path. Step 4 is where the
genuinely novel engineering is.
 
---
 
### 4A.7 — Welding station: GP25 + MOTOPAC-RL2D (decided 4 Nov 2026)

> **Decision: the busbar laser weld is done by a Yaskawa GP25 carrying the
> MOTOPAC-RL2D300-GP25JP remote laser package.** The GP4 stays on the line for
> light handling: it places the busbar carrier, the CMC and the cover. The line
> is now four arms: **GP7** places cells, **GP4** handles light parts, the
> **HC10DT** inspects, and the **GP25** welds.

**Why the GP4 could not weld.** Its payload is 4 kg. Every laser welding head
found weighs more than that, before counting the fibre, the cable dress and the
gas line (`cu_on_al_laser_welding.md` §3):

| Head | Mass |
|---|---|
| TRUMPF PFO 33 (used in the green-laser Cu-on-Al study) | < 35 kg |
| SCANLAB intelliWELD II PR | 21–37 kg |
| Coherent 2DWeld | < 20 kg |
| TRUMPF PFO SF | ≈ 13 kg |
| FANUC FiberSCAN CRW, the lightest found | 5.5 kg |

**The package** ([Yaskawa catalogue](https://dlc.e-mechatronics.com/catalog/chepc94111700/chepc94111700a_0_0.pdf)):

| Item | Value |
|---|---|
| Arm | MOTOMAN-GP25, 25 kg payload, YRC1000 |
| Mass on the wrist | scanner head 15.8 kg + air knife 2.5 kg = **18.3 kg** (driver 4 kg, off the wrist) |
| Laser | fibre, 1050–1090 nm, up to 8 kW CW or quasi-CW |
| Focal spot | 320 µm (theoretical, 100 µm fibre), 3× magnification |
| Scanner field | 25 mm diameter at focus, work distance 329.5 mm |
| Scanner | spot moves at up to 8 m/s, pattern up to 500 Hz, ±30 µm repeatability |
| Welding method | **wobble welding.** Power can be modulated within one circle (e.g. high for half of it), set from the pendant |
| Cabling | scanner control cable, cooling, shield gas and air purge built into the GP25's arm; the process fibre is routed separately |
| Stated purpose | "laser welding with a wide gap tolerance for workpieces with misalignments or gaps" |

**Why this one:**

- **It carries a real head.** 18.3 kg on a 25 kg arm, with dress built in. It is
  Yaskawa's own package, so the station is a product rather than an assumption.
- **It matches how defectgen builds welds.** The robot holds still and the
  scanner draws the seam (`cu_on_al_laser_welding.md` finding 3). `weld.py`
  builds every bead from a beam trace. In the write-up, the trace's slow terms
  (`weld_path_drift`) read as robot pose error during the hold, and its fast
  pattern (`beam_wobble`) as scanner output.
- **Its numbers replace assumptions.** The 320 µm spot, 500 Hz wobble and 25 mm
  field can go into `weld_process` tagged S.
- **It sets up an experiment.** Power varies along the wobble circle, so
  "power along the trace" becomes the next weld parameter. Yaskawa's gap-tolerance
  claim becomes testable in simulation: does wobble widen the gap window under
  the `weld.py` gap model?
- **ROS support exists.** `motoman_gp25_support` (URDF + meshes) is in the
  vendored ROS-Industrial tree.

**Rejected:**

| Option | Why not |
|---|---|
| GP4 (as before) | 4 kg payload; no welding head fits |
| GP7 | 7 kg payload; the package's 18.3 kg head does not fit |
| GP12 | 12 kg payload; same problem |
| HC10DT | 10 kg payload, and it is the inspector. Sharing it would serialise inspection and welding |
| **GA50** | 50 kg, 2,038 mm reach, ±0.015 mm, 855 kg. A path-accuracy arm for 3D laser welding and cutting of *large* parts, where the **robot** traces the seam. Here the scanner draws the seam while the arm holds, so that accuracy goes unused. It would dwarf a 355 mm module. No ROS support package (only GP50) |
| Fixed optics or gantry, arm presents the part | Works, but changes the cell layout and removes the robot from the weld story |
| Keep the GP4 as a stand-in for a larger arm | Weakest for a controls and automation reader |

**Accepted costs and open checks:**

- **The laser is infrared, not green.** Copper absorbs green much better
  (`cu_on_al_laser_welding.md` finding 8). Yaskawa's answer is power (8 kW) plus
  wobble. That is a real industrial choice, so state it. It also makes the
  welded copper harder to scan, which the sensor models in 1.3 have to handle.
- **Unverified:** whether this package is rated for copper. The brochure does
  not name materials.
- **Payload margin:** 18.3 of 25 kg before the fibre and dress. Check it against
  the head's real centre of mass in Phase 3.
- **IK:** no OPW file ships for the GP25 (`urdf/` and `meshes/` only). Derive
  the parameters from the URDF and check them against forward kinematics, as
  for the GP7.
- **Laser safety:** an 8 kW infrared laser means an enclosed, interlocked cell
  (ISO 11553). The collaborative HC10DT (S3) and the weld cell (S4) must be
  physically separate; the laser enable is interlocked with the enclosure and
  pallet stop (status_update.md 3.5b).

## 4B. ACTIVE WORK PLAN — defect generation and detection (no simulator)

**Domain: battery module assembly line, per §3.7.** Runs entirely in numpy /
trimesh / Open3D. No ROS, no simulator. The battery prototype showed the scale
is trivial (96 layers plus a sweep: ~10 MB, 0.23 s). A 12-cell module is
smaller still.

This is the core of the contribution. Built first, against synthetic scans,
the detector and the fixed-vs-arm comparison already work when the simulator
arrives. The simulator's job is then motion, reachability and plumbing.

**Design rules, decided 15 Sep:**

1. **Mesh-native from line one.** The generator builds real, watertight 3D
   solids (`trimesh`) and exports STL/PLY immediately. The same mesh is the
   reference (defect-free seed) and the scan target, and Gazebo imports it
   unchanged.
2. **A narrow `Part` seam.** The generator returns `(mesh, labels)`. Scan,
   detection and evaluation never learn which domain they are looking at.
   Write only the module domain now.
3. **Arm for coverage, sensor for metrology** (§3.7.6). Every critical
   measurement is taken within a single capture.

### 4B.0 — Prior findings

The feasibility findings (height maps vs envelope, sensor roles,
observability of cell internals) now live in **§3.7**.

### 4B.1 — Procedural module generator

Parametric module, defects off by default. Everything a variable. **The full
part-by-part geometry, with each number tagged sourced / derived / assumed /
measure, is in [`module_spec.md`](module_spec.md).** Summary:

```
cell             VDA PHEV2 prismatic, 148 x 91 x 26.5 mm  (W x H x D)
module           VDA 355 envelope, 355 x 151 x 108 mm, 12 cells, 3P4S
cell gap         variable (compression pads)              — assumed, source it
terminals        laser-weld pads, ~4 mm thick, ~6 mm above cap (derived);
                 spacing and pad size measured from photos (module_spec §6)
can              aluminium, ~1.1 mm wall
module preload   ~3 kN on the end plates for a 12-cell PHEV2 module
busbars          flat strips joining adjacent terminals, thickness variable
laser weld       seam geometry on each busbar–terminal joint (post-weld state)
housing          end plates + side plates (+ optional adhesive bead on floor)
```

Emitted states: **pre-busbar**, **pre-weld** (busbars placed) and
**post-weld**, so each inspection point has its own ground truth. Per-feature
ground truth covers every cell pose, terminal top plane, busbar pose and weld
seam path. The labels are computed from these, never inferred back from the
mesh.

### 4B.2 — Defect injection

One function per class, each with a severity parameter so datasets sweep
rather than sample:

| # | Defect | Parameters | Stage | Optically detectable? | Scope |
|---|---|---|---|---|---|
| 1 | Cell placement error | `dx, dy, dθ` per cell | pre-busbar | yes | **primary — loop target** |
| 2 | Terminal height / tilt | `dz`, tilt | pre-busbar | yes | **primary — 0.1 mm** |
| 3 | Missing cell | index | pre-busbar | yes | primary |
| 4 | Reversed polarity | index | pre-busbar | yes (geometry + 2D) | primary |
| 5 | Busbar misplacement | `dx, dy, dθ` | pre-weld | yes | **primary** |
| 6 | Busbar lift-off gap | gap height | pre-weld | yes | **primary** |
| 7 | Weld seam geometry | see weld table below | post-weld | partly | **primary** |
| 8 | Spatter, landed | count, size, location | post-weld | yes | primary |
| 9 | Handling damage (dent / scratch / torn edge) | depth, area, location | any | yes | **primary — the 26V188 mechanism, §4B.9** |
| 10 | Gap-filler bead | break length, width deviation | pre-cell | yes | secondary |
| 11 | Compression pad missing / misplaced | index, offset | pre-busbar | yes | secondary |
| 12 | Insulation film tear / wrinkle | size, location | any | yes (geometry + 2D) | secondary |
| 13 | FOD (metal particle, debris) | size, location | any | yes if on a visible surface | secondary |
| 14 | Carrier not fully seated | `dz`, tilt | pre-weld | yes | secondary |
| 15 | Cover misalignment | `dx, dy, dθ` | final | yes | secondary |
| 16 | Weld porosity / cracks / lack of fusion | — | post-weld | **no — internal** | **out of scope, stated as a limitation** |
| 17 | Weld penetration depth | — | during weld | **no — needs inline OCT/ICI** | **out of scope, stated as a limitation** |

**Weld seam sub-classes (defect 7).** Surface geometry only, which is what a
profilometer can see:

| Sub-class | Measured as |
|---|---|
| Missing or short weld | seam length vs nominal |
| Seam deviation from the joint | lateral offset of seam centreline from the busbar–terminal edge |
| Width deviation | w(s) along the seam vs spec band |
| Excess reinforcement / sagging | bead height vs parent surface |
| Undercut | groove depth at the seam toe |
| Burn-through / hole | through-thickness void |
| Humping | periodic bead height variation |
| Crater at the end | depression at seam termination |

Acceptance limits come from **ISO 13919-2** (laser-beam welds in aluminium and
copper), which defines quality levels B, C and D with dimensional limits per
imperfection. Using a published standard rather than invented thresholds is
what makes the pass/fail defensible.

**On defects 16 and 17.** Porosity, cracks and lack of fusion are internal;
penetration depth is measurable only *during* welding, by inline coherent
imaging / OCT (e.g. IPG's LDD-700) looking down the beam axis. No post-weld
surface scan reaches them. Saying so plainly — and citing what industry
actually uses instead — is stronger than pretending the profilometer covers
the whole taxonomy.

Every generated module carries its ground truth: class, magnitude, location
and stage. **That label set is what makes precision/recall measurable.**

### 4B.3 — Sensor models (these are the real sensor models)

Per §3.7.8, fidelity lives here. The simulator will not improve on it. Three
models, all ray-cast against the mesh (Open3D `RaycastingScene`):

| Model | Stands for | Mount | Parameters pinned to |
|---|---|---|---|
| Fixed line-scan | Ranger3 + laser line | over the conveyor; the module moves under it | Ranger3 datasheet + chosen optics |
| Snapshot 3D | Zivid 2+ M60 class | HC10DT wrist, stop-and-shoot | Zivid datasheet |
| Line profilometer | LJ-X8200 class | HC10DT wrist, short sweeps | Keyence datasheet |

For all three:

- **Model both occlusions.** A point is returned only if the projector or laser
  reaches it *and* the camera can see it. That is what produces real shadowing
  beside busbars and terminals.
- **Mandatory, not optional:** Gaussian noise, grazing-angle dropout,
  specular dropout on bare aluminium and copper, and quantisation.
- **Freeze the parameters before looking at any detection result.** Store their
  hash in the dataset metadata.
- Output: N×3 arrays / `.ply` in the sensor frame, plus the sensor pose. That
  is the shape `sensor_msgs/PointCloud2` carries later.

The 2D camera is deferred. Polarity is detected from terminal
geometry first; image rendering comes later, if it is needed at all. *(Note
28 Oct 2026: if 2D is rendered, it comes from the Zivid's own registered 5 MP
2D, not a separate midiCam2 — §3.7.3.)*

#### Do simulated sensors "actually scan", or are they gamed? — recorded 1 Nov 2026

Neither. **No simulator has a "profilometer" sensor type.** Gazebo
(`gpu_lidar`, `depth_camera`) and Isaac Sim (RTX Lidar with config profiles,
cameras) offer only generic primitives: ray casters, depth cameras and RGB
cameras. A sensor model *is* a primitive **configured from the datasheet**
(resolution, field of view, range window, rate) plus post-processing. That is
standard practice, not a workaround.

- **A line profilometer is a single-row ray fan plus motion.** Each shot gives
  one profile; the 3D surface exists only by stitching profiles using the pose
  at each timestamp — exactly as on a real cell. Worked example: a 254 mm scan
  at 50 mm/s and 1 kHz → 50 µm profile spacing, ≈ 5,000 profiles. In
  Gazebo/Isaac the physics/sensor step must run at ≥ the profile rate, so the
  scan runs slower than real time. That is acceptable.
- **Snapshot 3D is a depth camera** at the datasheet resolution, FOV and
  working distance.

**Post-processing a bare ray cast lacks** (all parameters from datasheets,
frozen and hashed before results, per task 1.3):

| Real effect | Cause | Model |
|---|---|---|
| Triangulation shadowing | Emitter and camera sit at different points, so a point returns only if both see it | Second visibility ray from the camera origin; drop points the camera cannot see |
| **Specular dropout on aluminium** | Bright Al reflects the laser away from the camera | Incidence-angle- and material-dependent dropout and noise |
| Datasheet noise | Z repeatability, X resolution | Gaussian noise |
| Range window, grazing angle | Out of range or too oblique | Clip by range; drop by incidence angle |

> **Specular aluminium is the biggest sim-to-real gap in this project.**
> Terminals, busbars and weld seams are all bright Al. Either model it or
> state it in the README's Limitations section. Do not omit it silently.

**What the sensor models include and leave out — frozen 4 Nov 2026, params hash `779e2255dc4c`.**

Included (`scan.py`, `configs/sensors.yaml`, `configs/materials.yaml`):
- Both occlusions: laser/projector reach plus camera visibility.
- The range window and a grazing-angle cut.
- Per-material reflectance (diffuse plus Beckmann specular lobe, per wavelength band), giving dark and glare dropout.
- Noise from the datasheet. It grows as the returned light weakens and, for triangulation, with distance squared.
- Exposure falls as the line-sensor profile rate rises, up to each sensor's maximum rate.
- Height quantisation.
- Profilometer triangulation angle 22.0°, measured from the receiver-window tilt of the Keyence model in `cad/inspector/inspector_flange.step`.

**Left out, as stated limitations rather than silent omissions:**
- **Multipath / interreflection.** Laser light bouncing inside a concave weld bead, or between a busbar and the carrier wall, returns false points. This matters most on bright copper beads, and it is the biggest known gap after specular dropout. It needs a second-bounce ray model.
- **Speckle.** Coherent-light graininess on rough surfaces. Its effect is partly covered by the datasheet noise.
- **Motion during capture.** No robot vibration, no motion blur within a profile, no encoder jitter. Robot pose error enters only through `weld_path_drift`-style perturbations, not through the sensor.
- **Sub-surface scattering and translucency** in plastics.
- **Temperature drift and warm-up** (Zivid trueness changes during warm-up).

The README's Limitations section should carry this list.

**Architecture:** sensor models live **once, in `defectgen`** — ray cast with
Open3D `RaycastingScene` against the exported meshes, then the post-processing
above. Gazebo and Isaac supply only poses and raw rays/depth. This mirrors the
"detector written once" principle.

### 4B.4 — Detection pipeline (Open3D)

1. Coarse align: module pose from the fixture, or from the overhead camera
2. ICP refinement against the reference module for that stage
3. Per-feature measurement, each **within a single capture**: fit cell pose
   (x, y, θ), fit terminal top planes → adjacent Δh and tilt, fit busbar
   pose and gap, trace the weld seam
4. Residual segmentation for missing parts and FOD
5. Classify, measure, emit a `DefectReport` (§7)

Runs in `mb-venv`, not system Python.

### 4B.5 — Coverage and view selection (no kinematics yet)

Define the **required inspection surfaces** per stage: terminal tops and
edges, busbar edges, weld seams, cell-to-cell gaps. A surface element counts
as covered when it gets ≥ *N* points at an incidence angle below θ_max, in a
single capture.

- **Fixed baseline:** coverage from the Ranger3 line-scan in one pass
- **Arm:** candidate viewpoints on a hemisphere around the module, then greedy
  set cover over the required surfaces, which yields a view sequence
- Reachability, collision and motion time are added in Phase 2 with MoveIt.
  Phase 1 establishes the coverage ceiling each approach can reach.

### 4B.6 — Validation: the numbers the write-up needs

Generate a labelled dataset with several seeds per (class × severity) cell,
so every number has an error bar. Then measure:

- **Precision / recall per defect class**, with confidence intervals (Wilson
  or bootstrap)
- **Gage R&R (%GRR)** for measured characteristics: terminal height, cell
  position, busbar gap
- **Minimum detectable defect per class** (the POD 50 % and 90 % points)
- **Fixed vs arm:** coverage %, recall per class, captures per module. Cycle
  time comes in Phase 2.
- **Sensitivity** to noise, point spacing and standoff. This tells the flange
  which sensor class is actually required.
- **Detection latency** per capture and per module

### 4B.7 — Correction loop, in numpy

§9.9 needs no arm, so it runs here. *k* is the cell index. *e_k* is the
measured placement error of cell *k*. The GP7 offsets the next cell by
`u_{k+1} = −K·e_k` through a plant of gain *g* with placement noise and a
**systematic drift** (gripper offset, fixture wear), which is why integral
action matters. Deliverables:

- **K-sweep:** convergence → critical damping → divergence, against the derived
  `0 < K < 2/g`
- **Open-loop vs closed-loop placement defect rate.** This is the headline
  control result.
- Terminal height cannot be corrected by placement. Its decision is
  **reject / rework before the weld**, and that logic is part of the report.

### 4B.8 — How this meets the simulator

**Gazebo first.** Import module meshes at each stage. Model the snapshot
camera as a depth camera and the profilometer as a narrow-fan `gpu_lidar`.
Run the same detector on Gazebo-produced clouds. Be honest about what this
proves: Gazebo returns the same geometry as our ray-caster. Its value is the
ROS plumbing, the TF chain, reachability and real motion time.

Isaac Sim is deferred, not cut (§4). It returns if Gazebo's rendering becomes
the limiting factor.

### 4B.9 — The 26V188 replication experiment

A small, self-contained experiment that targets the actual recall mechanism
(§3.7.10), reusing the same pipeline. It is not a second robot cell.

**Setup.** A flat cathode sheet, cut to shape, is transferred into a nest. A
placement error `e_k` is drawn per sheet. **If |e_k| exceeds the nest
clearance, the sheet contacts the tooling and acquires a torn edge** — damage
depth and length scaling with the overtravel. That coupling is the point: the
defect is *caused by* the placement error, exactly as the report describes.

**Measured:**

- Detection of the torn edge by a fixed line-scan model (edge straightness and
  burr height along the cut edge) — precision/recall vs tear size
- **Defect rate vs loop gain K**: open loop, then the §9.9 correction applied to
  the next sheet. The claim is that closing the placement loop reduces the
  *damage* rate, not merely the placement error.
- Comparison against the recall's real detection path: an end-of-line
  electrical check that caught it only after cells were built, and after two
  thermal events.

This gives the write-up a documented, recent, safety-critical case with an
official root cause, and shows the method addressing that cause rather than a
hypothetical one.

---

## 4. Isaac Sim milestone — HC10DT planned from RViz (after repair)

> **Deferred, not cut (15 Sep).** Gazebo comes first for the whole line. Per
> §3.7.8, sensor fidelity lives in `defectgen/scan.py`, not in the renderer,
> so Isaac Sim returns only if Gazebo's rendering becomes the limiting factor.
> This section stays valid for when it does.
 
The target: open RViz, drag the MoveIt interactive marker to a pose, hit Plan &
Execute, and watch the HC10DT move in Isaac Sim. This is the first real
closed-loop between planner and simulator, and every later station reuses it.
 
### 4.0 Where to run Isaac Sim — decide before installing
 
The 8 GB ceiling (§1.0) makes this a real decision rather than a default.
 
**Isaac Sim 4.5 does not help.** Its documented minimum is 32 GB RAM / 8 GB
VRAM — *identical* to 6.0. There is no lighter tier. Downgrading also loses
Ubuntu 24.04 support (4.5 lists 20.04/22.04) and the Jazzy-recommended bridge
configuration. It costs a day and buys nothing.
 
**The container does not reduce memory requirements either.** NVIDIA does not
differentiate RAM requirements between container and workstation installs —
same application, different wrapper. The container solves *deployment*
problems, not memory pressure.
 
But the container **is** the right tool for the option that actually works:
 
> **Run Isaac Sim on a lab/remote machine, stream to the laptop.**
> This is exactly what the container is built for — headless servers with
> WebRTC streaming to a thin client. The laptop runs ROS 2, MoveIt2, RViz2,
> and Open3D locally (all comfortable in 8 GB); the remote box runs only
> Isaac Sim.
 
Container limitations to plan around:
 
- GUI mode needs a real local display and is not recommended — expect to work
  through WebRTC streaming, one browser tab or client at a time.
- Python apps and standalone examples run headless only.
- Tutorials needing the Content Browser require a Nucleus connection.
Streaming adds friction to interactive work — building an OmniGraph by hand,
tuning an imported URDF. It is workable, not pleasant.
 
**Recommended sequencing given the hardware situation:**
 
1. **Now, on the laptop:** everything that does not need Isaac Sim — the RViz
   work in §3, `motobat_msgs` definitions, the Open3D registration pipeline
   against synthetic point clouds, cell/fixture geometry in Blender. This is
   genuinely a lot of the project and none of it is blocked.
2. **In parallel:** file the Dell warranty transfer (§1.0) and ask the OSU ECE
   department about GPU workstation access. Both have long lead times, so
   start them today.
3. **When a capable machine is available:** §4.1–4.3 as written, container
   install if remote, workstation install if the laptop is repaired.
**Do not attempt a serious Isaac Sim install on 8 GB.** It will technically
launch on swap and then thrash badly enough that every problem looks like a
software bug. That failure mode has already cost multiple days once.
 
### 4.0b Cloud GPU — the recommended path, with the caveat that matters
 
Cloud is very likely the right answer for weeks 5–6, but **not for the reason
usually given**, and the naive architecture does not work.
 
#### Correcting a common misdiagnosis
 
A frequent claim is *"Isaac Sim's hard constraint is GPU/VRAM, not system RAM."*
**For this machine that is backwards**, and it matters:
 
| Resource | This laptop | Isaac Sim 6.0 minimum | Verdict |
|---|---|---|---|
| VRAM | 8 GB (RTX 4060) | 8 GB | **meets minimum** |
| System RAM | 7.4 GB → 16 GB post-repair | **32 GB** | **below minimum** |
 
The GPU is not the bottleneck. System RAM is, and it was diagnosed precisely
(§1.0). Advice premised on the opposite would lead to buying GPU capacity to
solve a memory problem.
 
Also note: the 32 GB DDR5 kit is roughly **$400**, not the ~$1,000 figure that
sometimes circulates. The comparison below uses the real number.
 
#### The cost comparison, with current figures
 
Isaac Sim work is confined to **weeks 5–6** (§3.5) — realistically 40–80 hours
of actual simulator time, not continuous use.
 
| Option | System RAM | VRAM | Cost |
|---|---|---|---|
| AWS `g6.2xlarge` (L4) | 32 GiB | 22.4 GiB | **$0.98/hr** |
| Runpod L4 | 24 GB | 24 GB | **$0.49/hr** |
| Vast.ai L4 | 63 GB | 24 GB | **$0.33/hr** |
| 32 GB DDR5 kit (local) | 32 GB | 8 GB (unchanged) | **~$400 one-time** |
 
At 60 hours: **$20–60 on cloud**, versus $400 for RAM that still leaves the
laptop at 8 GB VRAM and only just at Isaac Sim's RAM minimum.
 
**For this project's timeline, cloud wins clearly.** It also removes weeks 5–6
from the critical path — the deadline no longer depends on a hardware purchase.
 
#### The architecture caveat — do not split ROS 2 across the internet
 
The obvious layout — Isaac Sim in the cloud, ROS 2 and MoveIt2 local — **does
not work well**, and this is the part usually omitted:
 
- DDS discovery relies on multicast, which does not traverse the public internet
- The Isaac Sim container **requires `--network=host`** (the streaming SDK binds
  its UDP media socket to a real interface), so container-level port mapping
  cannot paper over it
- Even with a VPN (Tailscale/ZeroTier) or a Zenoh bridge, WAN latency sits
  inside the `FollowJointTrajectory` control loop
**Correct layout: run the entire stack on the cloud instance** — Isaac Sim,
ROS 2, MoveIt2, RViz2 — and stream only the display back. All DDS traffic stays
on `localhost` inside one machine. The laptop becomes a thin client.
 
```
  ┌─ cloud instance (32-64 GB RAM, L4) ──────────────┐
  │  Isaac Sim  ⇄  ROS 2 / MoveIt2 / RViz2           │      WebRTC
  │  all DDS traffic on localhost                     │  ═══════════▶  laptop
  └───────────────────────────────────────────────────┘               (display)
```
 
#### Practical setup notes
 
- **Ports:** 49100/TCP (WebRTC signalling), 47998/UDP (media), 8210/TCP if using
  the Docker Compose web viewer. Open these in the provider's firewall.
- **`--network=host` is mandatory** — see above.
- **Mount the cache volumes.** Shader compilation takes 10–30 minutes on first
  launch; without persistent `cache/`, `computecache/`, `config/`, and `data/`
  volumes, that cost is paid **every session**, which is ruinous on hourly
  billing. Pre-create them and `chown -R 1234:1234`.
- **Driver caveat:** NVIDIA notes the newest drivers are not always fully
  supported for livestreaming. Prefer a provider image with a known-good driver
  over the absolute latest.
- **Cost discipline:** bill is wall-clock, not compute. **Stop the instance when
  not actively simulating.** Keep the repo in git so a destroyed instance costs
  nothing but setup time.
#### Recommended plan
 
1. **Weeks 1–4: change nothing.** §4B is numpy, §4A is RViz2/Gazebo — all
   comfortable on the repaired 16 GB laptop. No cloud, no purchase.
2. **Week 5: rent for a few hours first.** Runpod or Vast.ai, cheapest tier,
   prove the container launches and streams before committing further.
3. **Weeks 5–6: run the full stack in the cloud**, laptop as thin client.
4. **Defer the RAM purchase entirely** until after 1 Oct. If cloud works, that
   $400 may be better spent elsewhere, or not at all.
This makes the memory upgrade (§1.0) genuinely optional rather than blocking,
and removes the single largest schedule risk in §3.5.
 
### 4.1 Architecture
 
```
RViz (MoveIt Motion Planning plugin)
        │  interactive marker → plan request
        ▼
   move_group  ──────────────────────────────┐
        │  FollowJointTrajectory action      │  planning scene,
        ▼                                     │  collision checking
   joint_trajectory_controller               │
        │  (ros2_control)                    │
        ▼                                     │
   TopicBasedSystem hardware interface       │
        │                                     │
        │  publishes /isaac_joint_commands   │
        │  subscribes /isaac_joint_states    │
        ▼                                     │
   Isaac Sim OmniGraph ActionGraph           │
        │  ROS2 Subscribe JointState  ───────┘
        │  Articulation Controller
        │  ROS2 Publish JointState
        ▼
   HC10DT articulation in the USD stage
```
 
The seam is deliberate. `joint_trajectory_controller` exposes exactly the
`control_msgs/FollowJointTrajectory` action that **MotoROS2 exposes on real
Yaskawa hardware** (see §6). Swapping Isaac Sim for a YRC1000 later means
changing the hardware interface, not the planning stack.
 
### 4.2 The bridge package
 
Two options, both released for Jazzy:
 
| | Package | Plugin class |
|---|---|---|
| **Recommended** | `topic_based_hardware_interfaces` (ros-controls) | `joint_state_topic_hardware_interface/JointStateTopicSystem` |
| Legacy | `topic_based_ros2_control` (PickNik) | `topic_based_ros2_control/TopicBasedSystem` |
 
Prefer the **ros-controls** one. The PickNik package is marked UNMAINTAINED on
the ROS index; the functionality moved upstream into `ros2_control` under
Simulator Integrations. Most tutorials still show the old name — translate as
you read them.
 
### 4.3 Steps
 
**1 — Install Isaac Sim 6.0.** ~30 GB. Not scripted deliberately: the installer
is version-sensitive enough that a wrapper rots faster than it helps.
 
```bash
# https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html
cd ~/isaacsim && ./post_install.sh && ./isaac-sim.selector.sh
```
 
First launch compiles shaders for 10–30 minutes. **It is not hung.** Then
uncomment `ISAAC_SIM_PATH` in `~/motobat/motobat_env.sh`.
 
**2 — Import the HC10DT into Isaac Sim.** Use the URDF Importer extension on
`hc10dt.xacro` (expand to plain URDF with `xacro` first). Save as USD.
 
Expect to fix after import: joint drive stiffness/damping, joint limits, and
collision meshes. **The visual meshes are too detailed to use as collision
geometry** — replace with convex hulls or primitives, or physics will crawl.
 
**3 — Build the OmniGraph.** Three nodes, wired to the articulation:
 
- `ROS2 Subscribe JointState` → topic `/isaac_joint_commands`
- `Articulation Controller` → targeting the HC10DT prim
- `ROS2 Publish JointState` → topic `/isaac_joint_states`
Follow the [Joint Control tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_manipulation.html)
— it covers this graph directly.
 
**4 — Generate the MoveIt config.**
 
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
 
Load `hc10dt.xacro`. Define one planning group (`inspector_arm`, all six joints),
generate the self-collision matrix, add a home pose. Output goes to
`src/motobat_inspector/config/`.
 
**5 — Add the `ros2_control` block** to the URDF used by MoveIt:
 
```xml
<ros2_control name="HC10DTIsaac" type="system">
  <hardware>
    <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
    <param name="joint_commands_topic">/isaac_joint_commands</param>
    <param name="joint_states_topic">/isaac_joint_states</param>
  </hardware>
  <!-- one <joint> block per joint_1_s … joint_6_t -->
</ros2_control>
```
 
**6 — Launch and test.** `move_group` + `joint_trajectory_controller` + RViz.
Drag the marker, Plan & Execute, watch Isaac Sim.
 
### 4.4 Where this will actually go wrong
 
Budget debugging time for these — they are the known failure modes, not
hypotheticals:
 
- **Joint name mismatch.** MoveIt's names must match Isaac Sim's articulation
  DOF names exactly. Check `ros2 topic echo /isaac_joint_states` against
  `ros2 param get /move_group robot_description`.
- **Trajectory following is inconsistent** even when connected — a documented
  Isaac Sim + `topic_based_ros2_control` issue. Usually joint drive gains being
  too soft to track the commanded trajectory. Tune stiffness/damping on the
  articulation first.
- **Nothing moves, no error.** Almost always the ActionGraph is not ticking, or
  the Articulation Controller points at the wrong prim.
- **RMW mismatch.** See §5 — this one is silent and wastes hours.
---
 
## 5. Open decision: Cyclone DDS vs FastDDS
 
`motobat_env.sh` currently sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` per
Section 5.3 of the proposal. **There are now two independent arguments against
that choice:**
 
1. **NVIDIA recommends FastDDS** for single-machine Isaac Sim — it uses shared
   memory for large `PointCloud2` traffic, which is essentially everything the
   Inspector does.
2. **MotoROS2 requires FastDDS** and does not support Cyclone. Keeping Cyclone
   means the stated hardware migration path needs an RMW change to work.
**Recommendation:** switch to FastDDS (comment the line out — it is the default)
and note the reversal in the proposal as an evidence-driven change. That reads
better than a spec nobody revisited.
 
If point clouds arrive truncated or drop under load, this is the first thing to
check — before debugging registration code.
 
---
 
## 6. MotoROS2: what it is and is not
 
**MotoROS2 cannot run on this machine.** It is a MotoPlus application that runs
*on* a Yaskawa controller (DX200 / YRC1000 / YRC1000micro). No PC build, no
simulation mode.
 
The MotoPlus SDK is Yaskawa's paid, license-gated C SDK, needed only to *build*
MotoROS2 from source. Yaskawa publishes prebuilt `.out` binaries. **The SDK is
not the barrier — the controller is.**
 
### Why this barely matters
 
MotoROS2 exposes itself to ROS 2 as a standard
`control_msgs/FollowJointTrajectory` action server plus `sensor_msgs/JointState`
publishing — the same interface MoveIt2 drives in Isaac Sim.
 
```
Simulation:  MoveIt2 → FollowJointTrajectory → Isaac Sim controller
Hardware:    MoveIt2 → FollowJointTrajectory → micro-ROS Agent → MotoROS2 → arm
```
 
Planning, inspection, and the correction loop are unchanged across that swap.
 
### The discipline that keeps this true
 
- Command arms **only** via `FollowJointTrajectory` actions
- Read state **only** from `/joint_states`
- **Never** call Isaac Sim's Python articulation API from a control node
- Isaac-Sim-specific code lives in `motobat_bringup` only
Breaking this welds the project to the simulator and invalidates the
sim-to-hardware claim.
 
### Proposal wording
 
Section 5.1 currently reads as though MotoROS2 is part of the running stack.
Replace with:
 
> The system targets the standard ROS 2 manipulation interfaces
> (`control_msgs/FollowJointTrajectory`, `sensor_msgs/JointState`) that
> MotoROS2 implements on real Yaskawa controllers. Arms are driven through
> ros2_control and MoveIt2 against Isaac Sim's articulation controller, with the
> command interface built against the genuine MotoROS2 contract — making
> sim-to-hardware migration a controller swap rather than a rewrite.
 
Section 2.2's Yaskawa-over-FANUC argument survives intact and is now the basis
of the migration path.
 
Add to Section 6 (future work): real controllers enforce motion limits, speed
overrides, and safety interlocks that Isaac Sim ignores. Trajectories valid in
simulation may be rejected outright by a YRC1000.
 
---
 
## 7. Roadmap after the Isaac Sim milestone
 
*(Written before the 15 Sep scope change. Superseded by §3.7 and §4B. The
sensor roles in item 1 are revised in §3.7.3.)*
 
1. **Sensor array on the HC10DT** — SICK microScan3 / multiScan136 / Inspector83x,
   ZED 2i, mounted on the flange from §4A.4. RTX Lidar OmniGraph →
   `sensor_msgs/PointCloud2`. Verify in RViz2.
2. **TF2 tree for the sensors** — already defined by §4A.4's flange URDF;
   verify it survives the move into Isaac Sim. Point clouds from different
   passes cannot fuse without correct frames.
3. **`motobat_msgs`** — `DefectReport`, `ScanResult`, `CorrectionCmd`. Define
   these before writing nodes that pass them around.
4. **Open3D registration pipeline** — ICP scan-vs-reference. Runs in the venv,
   not system Python.
5. **Cell and fixture geometry** — Blender/FreeCAD, procedurally generated so
   defect variants can be scripted rather than modeled by hand.
6. **Defect injection** — geometric, per Section 3.1.
7. **Remaining three stations** — reuse the HC10DT pattern. Needs the
   hardware situation resolved (§1.0); not viable on 8 GB.
8. **Latency instrumentation** — measure sensor publish → registration →
   correction command against the 1.5–2.5 s/unit industrial cycle time
   (Section 3.2 of the proposal). See §4A.5 for why this replaces the
   simulated-switch idea.
9. **Correction loop** — the actual contribution. Everything above is scaffolding.
---
 
## 8. Repo conventions
 
**`.gitignore`:**
 
```gitignore
build/
install/
log/
src/_vendor/
venv/
*.pyc
__pycache__/
```
 
`_vendor/` is cloned by the bootstrap script — never commit it.
 
**Commits:** the history is evidence of the development process (Section 5.7 of
the proposal). Write messages that explain *why*, not *what*.
 
**Before writing a control node,** re-read §6. That boundary is the whole
sim-to-hardware argument.
 
---
 
## 9. The mathematics — what each subsystem actually computes
 
Per the working principle above: equations first, then code. Every symbol is
defined, every frame and unit stated. Craig's *Introduction to Robotics* chapter
numbers are given where they apply, since that is the current study text.
 
### 9.1 Rigid-body transforms and the TF tree
 
Everything in this project is bookkeeping about frames. A pose of frame *B*
expressed in frame *A* is a homogeneous transform:
 
```
        ⎡ R_AB   p_AB ⎤
T_AB =  ⎢             ⎥        R ∈ SO(3),  p ∈ ℝ³
        ⎣  0ᵀ      1  ⎦
```
 
Composition chains along the kinematic tree, and inversion is cheap:
 
```
T_AC = T_AB · T_BC          T_AB⁻¹ = ⎡ Rᵀ   −Rᵀp ⎤
                                     ⎣ 0ᵀ     1  ⎦
```
 
**Why it matters here:** a sensor point measured in the sensor frame becomes a
point in the world frame by `p_world = T_world,tool · T_tool,sensor · p_sensor`.
The middle transform is the flange geometry from §4A.4. **An error there
propagates into every point of every scan** — which is why the URDF origins must
be derived from CAD numerically, not eyeballed.
 
*Craig ch. 2. Modern Robotics ch. 3 (as SE(3) and screw theory).*
 
### 9.2 Forward kinematics
 
Joint angles → end-effector pose. With Denavit–Hartenberg parameters
(αᵢ₋₁, aᵢ₋₁, dᵢ, θᵢ):
 
```
                 ⎡ cθᵢ            −sθᵢ           0          aᵢ₋₁         ⎤
  ᵢ⁻¹Tᵢ    =     ⎢ sθᵢcαᵢ₋₁    cθᵢcαᵢ₋₁    −sαᵢ₋₁    −sαᵢ₋₁dᵢ      ⎥
                 ⎢ sθᵢsαᵢ₋₁    cθᵢsαᵢ₋₁     cαᵢ₋₁     cαᵢ₋₁dᵢ      ⎥
                 ⎣ 0              0              0          1            ⎦
 
  ⁰T₆ = ⁰T₁ ¹T₂ ²T₃ ³T₄ ⁴T₅ ⁵T₆
```
 
*Craig ch. 3.* Modern Robotics uses the product-of-exponentials form instead,
which avoids DH's frame-assignment ambiguity:
 
```
  T(θ) = e^{[S₁]θ₁} e^{[S₂]θ₂} ⋯ e^{[S₆]θ₆} · M
```
 
where Sᵢ is the screw axis of joint *i* in the base frame and *M* is the
zero-configuration pose. Worth learning both — MoveIt and most ROS tooling are
DH/URDF-shaped, but PoE is cleaner to reason about.
 
### 9.3 Inverse kinematics — and why the two arms differ
 
Pose → joint angles. This is where the GP4 and HC10DT genuinely diverge (§4A.2).
 
**GP4 — closed-form (OPW).** The arm has an ortho-parallel basis and a
**spherical wrist**: joints 4, 5, 6 axes intersect at a single point. That
permits kinematic decoupling — position from the first three joints, orientation
from the last three:
 
```
  p_wrist = p_tool − d₆ · R_tool · ẑ        (wrist centre)
  θ₁,θ₂,θ₃ ← geometric solution from p_wrist
  R₃₆ = R₀₃ᵀ · R_tool                        (orientation residual)
  θ₄,θ₅,θ₆ ← Euler extraction from R₃₆
```
 
Yields up to **8 discrete solutions** (shoulder left/right × elbow up/down ×
wrist flip), all in closed form, in microseconds. Parameters ship in
`opw_parameters_gp4.yaml`.
 
**HC10DT — numerical.** `joint_5_b` is offset 162 mm in Y, so the wrist axes do
**not** intersect and the decoupling above is invalid. Solve iteratively with
damped least squares (Levenberg–Marquardt):
 
```
  Δθ = Jᵀ(J Jᵀ + λ²I)⁻¹ · e        e = pose error twist ∈ ℝ⁶
```
 
The damping λ is what keeps the solution finite near singularities, where **J**
loses rank and the undamped pseudo-inverse `Jᵀ(JJᵀ)⁻¹` blows up. Slower, may
fail to converge, no guarantee of finding all solutions.
 
*Craig ch. 4. The DLS form is Craig ch. 5 + Siciliano ch. 3.*
 
### 9.4 Velocity kinematics and the Jacobian
 
The Jacobian maps joint rates to end-effector twist:
 
```
  ⎡ v ⎤             ⎡ ẑᵢ₋₁ × (p_n − pᵢ₋₁) ⎤
  ⎢   ⎥ = J(θ) θ̇     Jᵢ = ⎢                      ⎥   (revolute joint i)
  ⎣ ω ⎦             ⎣ ẑᵢ₋₁                 ⎦
```
 
Static force duality — the same matrix, transposed:  `τ = Jᵀ · F`
 
**Singularity measure.** Yoshikawa's manipulability index:
 
```
  w(θ) = √( det( J(θ) J(θ)ᵀ ) )
```
 
`w → 0` at a singularity. Worth logging during scan trajectories: if the
Inspector's sweep passes near `w ≈ 0`, IK will struggle and motion will look
erratic. This is a measurable, reportable quantity, not a vague warning.
 
*Craig ch. 5. Modern Robotics ch. 5.*
 
### 9.5 Trajectory generation
 
MoveIt hands `joint_trajectory_controller` a time-parameterised path. For a
point-to-point move, a quintic keeps acceleration continuous:
 
```
  θ(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```
 
with coefficients fixed by θ(0), θ(T), θ̇(0), θ̇(T), θ̈(0), θ̈(T). Trapezoidal
velocity profiles are the alternative — faster, but with acceleration
discontinuities that a real controller feels.
 
**Relevant to Section 3.2 of the proposal:** cycle time is bounded below by
these profiles plus the velocity/acceleration limits in the URDF. Latency
budget = plan time + trajectory duration + detection time.
 
*Craig ch. 7. Modern Robotics ch. 9.*
 
### 9.6 Control — the joint_trajectory_controller loop
 
The controller tracks the commanded trajectory. Independent-joint PID:
 
```
  τ = Kp·e + Ki∫e dt + Kd·ė           e(t) = θ_desired(t) − θ_actual(t)
```
 
The model-based form, which is what a real industrial controller approximates:
 
```
  τ = M(θ)·θ̈_d  +  C(θ,θ̇)·θ̇  +  G(θ)  +  Kp·e + Kd·ė
       ⎣inertia⎦  ⎣Coriolis⎦   ⎣gravity⎦  ⎣  feedback  ⎦
```
 
**This connects directly to prior myCobot work.** Achieving sub-1 mm on a
compliant, backlash-prone arm meant compensating for terms this model assumes
away — joint compliance and hysteresis. The industrial arms here are stiff
enough that PID on a good trajectory suffices, which is *why* industrial arms
hit tolerance more easily. Worth stating explicitly in the writeup: it shows
awareness of what the hardware is doing for you.
 
*Craig ch. 9–10. Modern Robotics ch. 8, 11.*
 
### 9.7 Point cloud registration — ICP
 
The core of the Inspector. Given scanned points **P** = {pᵢ} and reference
**Q** = {qⱼ}, find the rigid transform aligning them:
 
```
  minimise over (R, t):    Σᵢ ‖ (R·pᵢ + t) − q_{c(i)} ‖²
```
 
where `c(i)` is the closest point in **Q**. Iterate: (1) find correspondences,
(2) solve for (R,t), (3) apply, repeat until convergence.
 
**Closed-form inner solve** (Arun / Horn, via SVD):
 
```
  p̄ = (1/n)Σpᵢ ,  q̄ = (1/n)Σqᵢ            (centroids)
  H  = Σ (pᵢ − p̄)(qᵢ − q̄)ᵀ                 (3×3 cross-covariance)
  H  = U Σ Vᵀ                               (SVD)
  R  = V · diag(1, 1, det(VUᵀ)) · Uᵀ        (det term prevents reflection)
  t  = q̄ − R·p̄
```
 
**Point-to-plane** converges faster on surfaces like an electrode stack, since
it penalises only motion along the surface normal:
 
```
  minimise    Σᵢ ( (R·pᵢ + t − qᵢ) · n̂ᵢ )²
```
 
**What to know before trusting it:** ICP is a local optimiser. It needs a decent
initial guess or it converges to a wrong minimum — which is why §4B.4 does
coarse alignment (FPFH features or RANSAC) before refinement. A defect that
shifts material can also bias the fit itself, so registration should be
performed against features known to be unaffected.
 
*Szeliski, Computer Vision, ch. 13. Open3D ICP tutorial.*
 
### 9.8 Defect metric — envelope deviation
 
From §3.7.2 (formerly §4B.0), the empirical finding: height maps are blind to lateral
misalignment. The metric that works:
 
```
  d(p) = signed distance from scanned point p to the reference surface
 
  Defect flagged when   |d(p)| > τ   over a connected region of area > A_min
```
 
Two thresholds, both requiring justification in the writeup: **τ** from sensor
noise (roughly 3σ, so ~15 µm for a 5 µm-σ profilometer) and **A_min** from the
smallest defect that matters physically.
 
Per-class specialisations. **Battery module (current scope, §4B)**, with every
measurement taken within a single capture (§3.7.6):

```
  Cell placement     (Δx, Δy, Δθ) = fitted cell pose − nominal pose
  Terminal height    Δh_i = z_i − z_{i+1}  between adjacent terminal top planes
                     (fitted by least-squares planes); flag |Δh_i| > 0.1 mm
  Terminal tilt      angle between fitted terminal plane normal and cap normal
  Busbar gap         g = min distance, busbar underside → terminal top plane
  Weld seam          continuity along the nominal path; width w(s) against spec
  Missing / FOD      connected residual regions |r| > τ, area > A_min
```

**Coverage** (§4B.5): a required surface element *j* is covered when some single
capture *c* returns ≥ *N* points on it with incidence angle below θ_max.

```
  Coverage = |{ j : ∃ c, n_{c,j} ≥ N and θ_{c,j} < θ_max }| / |required|
```

Measured characteristics are also reported as **%GRR** (Gage R&R, automotive
MSA).

*Cell-internal metrics (original; §3.7.4: the overhang metric needs the cathode
edge, which an external scan cannot see):*

```
  Misalignment      Δextent = extent(scan) − extent(reference)      [§3.7.2]
  Overhang          w_over = min distance, cathode edge → anode edge
  Thickness         Δh = h_measured − N·(t_anode + t_cath + 2·t_sep)
```
 
**Detection statistics** — the numbers Section 3.2 promises:
 
```
  Precision = TP / (TP + FP)          Recall = TP / (TP + FN)
  F₁ = 2·P·R / (P + R)
```
 
Benchmark against the 62.07 % Hi-Pot recall and 93.82 % YOLOv4 accuracy from
Section 1.2. Also report **minimum detectable defect** per class against the
0.5 mm threshold from Section 1.1.
 
### 9.9 The correction loop
 
The project's actual contribution. Given a measured misalignment **e** at layer
*k*, the correction applied to layer *k+1*:
 
```
  u_{k+1} = −K · e_k                    (proportional correction)
```
 
With integral action across layers, if error accumulates systematically:
 
```
  u_{k+1} = −Kp·e_k − Ki · Σⱼ₌₀..k e_j
```
 
*In the module line (§4B.7), "layer" means **cell**: e_k is the measured
placement error of cell k, and u_{k+1} offsets the GP7's placement of the next
cell. Measuring before the next placement is what makes the loop physically
meaningful (§3.7.4).*

**Stability condition.** This is a discrete-time loop indexed by layer, not
time. With plant gain *g* (commanded offset → achieved offset), the error
recursion is `e_{k+1} = (1 − gK)·e_k`, so convergence requires:
 
```
  |1 − g·K| < 1      ⟺      0 < K < 2/g
```
 
Over-correction (`K > 2/g`) oscillates and diverges — layers alternating past
the target. **Worth demonstrating deliberately in the results:** a sweep of *K*
showing convergence, critical damping, and divergence is far more convincing
than one tuned value, and it proves the loop is understood rather than guessed.
 
Open-loop vs closed-loop defect rate over the sweep is the headline result.
 
---
 
## 10. Study resources
 
Selected for this project specifically. Free where possible.
 
### 10.1 Robotics fundamentals
 
| Resource | Use | Cost |
|---|---|---|
| **Craig, *Introduction to Robotics*** (current text) | DH parameters, FK/IK, Jacobians, control. Matches ROS/URDF conventions. Ch. 2–5, 7, 9 are the ones that matter here | book |
| **Lynch & Park, *Modern Robotics*** | Screw theory / product of exponentials — cleaner than DH once it clicks. **Free PDF, free video lectures, free Coursera audit, Python library** | **free** |
| **Siciliano et al., *Robotics: Modelling, Planning and Control*** | Deeper on dynamics and control; the reference to reach for when Craig is too brief | book |
 
**Suggested approach:** stay with Craig as the primary — it maps directly onto
what MoveIt and URDF expect. Use Modern Robotics' video lectures for the same
topics from a second angle; the free Coursera audit covers ch. 3–6, which is
exactly §9.1–9.4 above.
 
### 10.2 Perception and point clouds
 
- **Szeliski, *Computer Vision: Algorithms and Applications*** — free PDF from
  the author's site. Ch. 13 covers 3D reconstruction and registration.
- **Open3D ICP tutorial** — worked examples for point-to-point and point-to-plane
- **Li et al. (2021), "A Tutorial Review on Point Cloud Registrations"** —
  survey covering coarse-to-fine pipelines and where each method fails
- **PCL tutorials** — even when using Open3D, the PCL docs explain the
  algorithms well
### 10.3a Battery module assembly and inline 3D inspection — current scope

- **MDPI *Batteries* 6(3):47, optical digital measuring in laser-welded EV
  battery modules.** Pre- and post-weld measurement needs and tolerances,
  including the 0.1 mm terminal height difference.
- **LMI, *3D smart sensors for inline EV battery inspection*.** How
  terminal height, gap and flush are measured on real module lines.
- **ZEISS AIMax / Hexagon 360 SIMS.** The reference for robot-based inline 3D
  inspection; read for how they split coverage from measurement.
- **Zivid, Keyence LJ-X, SICK Ranger3 datasheets.** The parameters the sensor
  models in §4B.3 are pinned to.
- **ISO/TS 15066 and ISO 13849-1.** Collaborative-robot speed and separation
  monitoring, and the performance levels behind the microScan3's role.
- **AIAG MSA manual.** Gage R&R, the form automotive quality engineers expect
  measurement results in.

### 10.3 Battery manufacturing — start here
 
- **PEM RWTH Aachen & VDMA, *Production Process of a Lithium-Ion Battery Cell***
  — **free PDF**, the authoritative industry guide. Covers electrode
  manufacturing, cell assembly (including stacking and tab welding), and
  finishing, across pouch/cylindrical/prismatic. **Read this first** — it is the
  process the project models.
- **VDMA Battery Production** — updated editions and process guides
- **"Advanced Electrode Processing for Lithium-Ion Battery Manufacturing"** —
  US DOE / OSTI, free
- The papers already cited in the proposal's References — particularly the
  defect taxonomy sources behind Section 1.4
### 10.4 ROS 2 and MoveIt2
 
- **Official ROS 2 Jazzy tutorials** — worth redoing given the year gap; the
  executor and QoS material matters more here than in the tutorial pick-and-place
- **MoveIt2 tutorials** — especially Setup Assistant and the move_group interface
- **ros2_control docs** — hardware interfaces and controller lifecycle, directly
  relevant to §4A.3 and the Isaac Sim swap
- **Articulated Robotics (YouTube)** — the clearest practical ROS 2 + Gazebo
  material available
### 10.5 What to read, in order, for this project
 
1. PEM/VDMA battery guide — understand the process being modelled *(week 1)*
2. Craig ch. 2–3 — transforms and FK, alongside §9.1–9.2 *(week 1–2)*
3. Szeliski ch. 13 + Open3D ICP — before writing §4B.4 *(week 2)*
4. Craig ch. 4–5 — IK and Jacobians, before §4A MoveIt work *(week 3)*
5. MoveIt2 Setup Assistant docs — during §4A.1 *(week 3)*
6. Craig ch. 7, 9 — trajectories and control, during §4A.3 *(week 4)*
Reading paired with the subsystem being built beats reading ahead — the
equations in §9 mean more once the thing they describe is misbehaving.
 
---
 
## References
 
- [Isaac Sim requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)
- [Isaac Sim ROS 2 installation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_ros.html)
- [Isaac Sim joint control tutorial](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [MoveIt: command a simulated Isaac robot](https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html)
- [topic_based_hardware_interfaces](https://control.ros.org/master/doc/topic_based_hardware_interfaces/doc/index.html)
- [Yaskawa MotoROS2](https://github.com/Yaskawa-Global/motoros2)
- [gavanderhoorn/motoman (ros2)](https://github.com/gavanderhoorn/motoman/tree/ros2)
**Study resources**
 
- [Modern Robotics (Lynch & Park) — free PDF, videos, code](http://hades.mech.northwestern.edu/index.php/Modern_Robotics)
- [PEM RWTH Aachen / VDMA — Production Process of a Lithium-Ion Battery Cell (free PDF)](https://www.pem.rwth-aachen.de/global/show_document.asp?id=aaaaaaaaabdqbtk)
- [VDMA — Production Process of a Li-Ion Battery Cell, 2026 edition](https://www.vdma.eu/documents/34570/35405938/Production_Process_of_a_Lithium-Ion_Battery_Cell_2026.pdf)
- [Open3D ICP registration tutorial](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html)
- [Li et al. — A Tutorial Review on Point Cloud Registrations (2021)](https://onlinelibrary.wiley.com/doi/10.1155/2021/9953910)
- [Advanced Electrode Processing for Li-Ion Battery Manufacturing (DOE/OSTI)](https://www.osti.gov/servlets/purl/2575264)
**Hardware / warranty**
 
- [Dell Ownership Transfer and Product Registration](https://www.dell.com/support/kbdoc/en-us/000125103/ownership-transfer-and-dell-product-registration-guides)
- [Dell support when traveling internationally](https://www.dell.com/support/contents/en-us/article/contact-information/international-support-services/dell-international-support)
- [Isaac Sim 4.5 requirements](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/requirements.html) (same 32 GB minimum as 6.0)
- [Isaac Sim container installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_container.html)
 
