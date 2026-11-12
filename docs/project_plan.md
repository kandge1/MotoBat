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
| MoveIt configs | HC10DT first (§4A.2), then GP7 cell placement, then GP4 busbars. **From 6 Nov 2026** the HC10DT work runs in parallel with §4B, in a second worktree (`../MotoBat-arm`, `feature/hc10dt_gazebo`); see §8 and `status_update.md` "Two lanes" |
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

#### As built — 6 Nov 2026 (`defectgen/detect.py`, `fit.py`, `evaluate.py`)

```
 captures (one per sensor shot or sweep)            clean reference, same stage
 points, ray index, poses ─────────┐                (generator, no defects)
                                   │                         │
                                   ▼                         ▼
                        ┌─────────────────────┐   same rays, same frozen sensor
                        │ 1-2  align to datum │◄── model, noise off = GOLDEN capture
                        └─────────┬───────────┘
                                  ▼
                                  ▼
   3  per-feature fits on scan AND golden, Δf = f(scan) − f(golden)
      terminals · cells · polarity · busbars · pad-to-busbar gap · carrier · seams
      (positions only from captures fine enough to judge them, §3.7.3 rule)
                                  │
                                  ▼  every measured part moved to where it was measured,
                                  │  same rays recast = the module AS MEASURED
   4-5  per-ray residual r_i against the module as measured,
        connected regions on the sensor's own grid
                                  ▼
            6  classify → DefectReport (findings, measurements, uncovered, OK/NOK/INCOMPLETE)
                                  ▼
            evaluate.py: match to the generator's labels → TP / FN / FP per class
```

Millimetres and degrees, module (pallet datum) frame, Z up, throughout.

**Organised captures.** Every scan point carries the index of the ray that
produced it: (profile, column) for a line sensor, (row, column) for the
snapshot. Real sensors return this (an LJ-X profile, a Zivid organised
cloud), and it is what makes the comparison below exact.

**The golden capture.** For each capture, the same rays are cast at the clean
module for that stage, through the same frozen sensor model (§4B.3), with noise
off. It is what this capture would return from a perfect module, with the same
shadows, dropouts and sampling. Two uses:

- *Golden-sample differencing.* Every feature is measured by the same function
  on both, and only the difference is reported: Δf = f(scan) − f(golden). A
  method's own bias (chamfered pad edges, a shadowed corner, where samples
  happen to fall) is the same on both and cancels. The detector compares like
  with like rather than against the CAD's nominal numbers.
- *Per-ray residual* (step 5).

**Steps 1–2: alignment to the datum.** Whole-module ICP was implemented first
and failed in two measured ways:

- It is dragged by the dominant surface. With a carrier seated 0.3 mm high,
  ICP lowered the whole capture by 0.3 mm. The defect vanished and every cap
  read 0.3 mm low.
- It barely constrains in-plane motion. Seen from above, the module is nearly
  all horizontal faces, which fix Z, roll and pitch. Only edges fix X, Y and
  yaw. A 0.15 mm fixture shift came back about 0.10 mm wrong, under every
  kernel and schedule tried, and under generalised ICP.

So, as §9.7's last paragraph asks, registration uses only features no defect
moves: the **housing** (end plates and side plates, picked out by the golden
capture's material per ray).

- Z, roll, pitch: point-to-plane ICP on the housing points only. The first pass
  is plain least squares; the second uses a Tukey kernel. Random subsampling
  (voxel averaging smears edges), and target normals are the reference mesh's
  face normals (estimated normals round edges off).
- X, Y, yaw: the outlines of the two end-plate tops (below), from the rigid 2D
  fit of their golden centres onto their scanned centres.
- A capture that sees no housing (a wrist sweep along one terminal row) keeps
  its fixture or robot pose. Its *absolute* positions inherit that pose's
  error. Its *relative* ones do not: Δh between adjacent terminals, busbar to
  pad, seam to busbar. That is the single-capture rule of §3.7.6 doing its job.

The housing is the datum rather than the cells deliberately. A frame fitted
to the cells would absorb any shift they share, such as a systematic gripper
offset, which is exactly the error the integral term in §9.9 exists to remove.

Measured (line-scan, injected fixture error 0.15 / −0.10 / 0.03 mm, 0.05° yaw,
0.02° roll): Z exact; X and Y within 50 µm. **50 µm is half the line-scan
profile pitch.** An edge across the scan direction can only be placed to the
nearest profile, so any outline-based X is quantised to ±pitch/2. With no
fixture error, scan and golden share a sampling phase and cell positions read
0.0 µm. That number flatters the sensor, and %GRR (1.9) must vary the phase.

**Which capture may measure what.** §3.7.3's rule of thumb (sample spacing
≲ D/2 to measure a feature of size D) is applied per capture. A position
placed by an outline is judged against tolerance T only by a capture whose
sample spacing is ≤ T/2. That covers a cell's or busbar's position, a
seam's shape and the X/Y datum. Anything else is listed as uncovered, not
passed. Plane fits (heights, tilt, gap, carrier) average thousands of points
and are open to every capture. The simulated Zivid samples every second
pixel (`stride: 2`), so its spacing is ≈ 0.5 mm. Before this rule, its
end-plate outlines moved a perfectly placed module 0.115 mm in X and its
cells read 0 or 0.25 mm, depending on which pad edges crossed a pixel: 23–56
false findings per clean module. Under the rule it reports heights,
polarity, gaps, carrier, missing parts and large residuals, and it lists
cell and busbar positions and seams as uncovered. That is exactly the
plan's "coverage, not precision" role for it.

**Step 3: per-feature fits** (`fit.py`).

*Top-face plane, trimmed least squares* (seeded by RANSAC, Fischler & Bolles
1981, so a busbar top with 0.3 mm beads on it or a pad beside a busbar fits
the right surface):

```
z = a + b(x − x₀) + c(y − y₀),   refit on |eᵢ| ≤ max(3·rms, floor), 5 passes
n̂ = (−b, −c, 1)/‖·‖,             tilt = arccos(n̂_scan · n̂_golden)
```

*Face points and outline.* Every point within `band = max(0.03, 4σ̂)` of the
plane, inside the nominal face grown by 2.5 mm, is on the face. Its outline is
the minimum-area enclosing rectangle (rotating calipers over the convex hull,
Freeman & Shapira 1975): centre (cₓ, c_y), yaw ψ, extents.

*Cell pose.* A cell's cap is flush with the pads either side, so it has no
edge to fit. Its pose comes from its terminal pads, which are rigid on the can.
With both pads in one capture, a 2D Kabsch fit maps golden pad centres onto
scanned ones (§9.7 in 2D): R, t, then Δ(x, y) = R·c + t − c at the cell centre
c, and Δθ = atan2(R₂₁, R₁₁). With one pad, Δθ is that pad's outline yaw.

*Terminal height.* h_k = plane_k(c_k) − golden. The row's common part is
removed by a straight line along the row, trimmed on the median absolute
deviation (a sweep may see only six terminals, and one raised terminal
inflates an rms enough to keep itself), d_k = h_k − (a + b·x_k), and
the §4 rule is applied: |d_k| > 0.1 mm. Adjacent Δh is reported for every
pair seen in one capture. Tilt tolerance: atan(0.1 / 18.5) = 0.31°, the tilt
that lifts one pad edge 0.1 mm. A tilted terminal stands on its low edge, so
its centre rises by (d/2)·sin(tilt). That rise is not reported again as a
height defect.

*Polarity.* Two CAD hypotheses per cell: as designed, and turned 180°. The
pads land in the same places either way; the fill plug beside the negative
pad does not. Each cap point is scored along its own measuring ray against
each hypothesis mesh M_h:

```
S_h = Σᵢ min(|t_h,i − 20|, 1 mm)    t_h,i = hit distance along âᵢ from pᵢ − 20·âᵢ
```

The cell is reversed if S_turned < S_as_designed. Along the ray, not by
nearest-face distance. A cap point where a hypothesis puts the plug lies on
that plug's bottom face, so nearest-face distance charges nothing for a plug
that is not there. That bug called every flipped cell reversed on a one-row
sweep. Measured margin (S_ad − S_t)/(S_ad + S_t): ±0.99 profilometer, ±0.89
line-scan.

*Busbar gap.* The busbar is 12 wide on a 22 wide pad, so the pad shows either
side of it. Fitting the pad from those strips puts the gap inside one capture:

```
g(x, y) = [B(x, y) − t_busbar] − P(x, y)        scan minus golden
```

with B the busbar top plane and P the pad plane, evaluated at the terminal
centre for `busbar_liftoff` (tolerance 0.15 = `weld_gap_ok`) and at each stitch
end for the weld (a busbar tilted on a chip opens toward one end).

*Weld stitch.* Heights h = z − B(x, y), only on the busbar (inside its
measured outline), in the stitch frame carried by the busbar's measured
motion. Bins of 0.25 mm along s. Per bin, the surface counts as melted where
bead_min_h < |h| and h > −t/2, giving the bead centre c(s), its width (2–98
percentile across), and its height. Then, over the nominal span:

```
c_scan(s) − c_gold(s) = a + q·(1 − (2s/L)²)
   a  offset at the ends → seam_deviation
   q  sagitta            → weld_arc        (the bow weld.py draws)

w_c = 1.4826·MAD(c − [1, bow, s]-fit of c)     each centreline about its own fit
drift = √2·√(max(w_scan² − w_gold², 0))        → weld_path_drift
```

Drift is the extra wander of the scanned centreline over the golden one, not
the wander of their bin-by-bin difference. The difference needs scan and
golden bins to line up along the stitch. Alignment leaves up to half a
profile pitch along X, and bead texture then read as 0.12–0.45 mm of drift
on every clean stitch under a 0.15 mm fixture error. Measured after the
change: clean stitches read ≤ 0.05; an injected 0.15 mm drift reads
0.136–0.146; and under the fixture error one clean stitch still reads 0.144,
against a 0.10 tolerance. Drift on the line-scan sits close to its noise
floor.

Length is the longest melted run, coverage the melted fraction (missing <
0.1). Undercut is measured in a toe band 0.03–0.30 mm outside the design melt
width, where weld.py cuts it. A hole is points deeper than t/2 inside the
stitch. **Causal rule:** where the measured gap under a stitch exceeds 0.15,
the gap is the cause. The stitch reports what the gap did (underfill,
undercut, hole), not the shape changes it made.

**Steps 4–5: residual against the module as measured.** The residual is
taken *after* the features are measured. Every part detection measured is
moved to where it was measured:
- each cell's in-plane pose, carrying its terminals;
- each terminal's and busbar's plane change;
- each busbar's outline motion;
- the carrier's plane;
- each weld bead, on its busbar plus its centreline offset.

The same rays are then recast at that module. Against the plain golden
capture, a part that moved *within* tolerance left its edges and shadows
behind as residual regions, because only a feature *finding* covered them.
A carrier 0.1 mm high (tolerance 0.2) gave 5,321 false regions, a busbar on
a 0.1 mm gap 133, a terminal 0.08 mm high one. A real line runs mostly just
under tolerance, so this was the main false-alarm source, and the
single-defect tests (all above tolerance) could not show it. Against the
module as measured, all of them read clean.

A correction is applied only when it is resolved. Plane changes are precise
to microns and always applied. An in-plane outline motion is applied only
if it moves some point of the part more than half the capture's sample
spacing. A bead offset is applied only above 0.1 mm: clean stitches read
±0.08 on the line-scan. Moving parts by sampling noise left its own
residuals, 4 false spatter findings on a clean weld module.

For each ray seen by both:

```
rᵢ = (p_pred,i − p_scan,i) · âᵢ          âᵢ = measuring direction, into the scene
                                          p_pred = same ray cast at the module as measured
flag |rᵢ| > τ_m = max(5·σ̂_m, 5 µm),     σ̂_m = 1.4826·MAD(r | nominal material m)
```

Positive r is material nearer the sensor than designed (raised, added). A ray
the golden capture returned but the scan did not counts as missing. Per-ray
comparison handles occlusion exactly. Closest-point distance cannot: the
cooling plate under a missing cell is a CAD face, so its points would sit "on
the reference". Regions are 8-connected components on the sensor's own grid,
with area Σ ray footprints. They are dropped when:

- smaller than 6 rays or 0.02 mm²;
- long and under 2.5 rays thick (a ray that grazed an edge);
- peak |r| > 3·√area: far deeper than wide, so a ray that crossed a step edge
  (after a recast, at a cap feature), not a dent or a particle;
- explained: on a fitted face, inside a feature finding's box, or within an
  outline's band. The outlines are terminals, busbars and weld beads, and the
  band is max(0.25 mm, the capture's sample spacing) plus how far that part
  moved. Bead outlines were added because the residual ±pitch/2 of alignment
  leaves slivers at bead ends and toes: 23 false spatter findings on a clean
  weld module under a fixture error. The cost is that a drop landing within
  the band of a bead is not reported (profilometer spatter: 10/10 → 9/10).

What remains is classified: raised and ≤ 4 mm² → spatter (post-weld) or FOD;
missing on foam → missing pad; sunk and ≤ 400 mm² → handling damage; the rest
→ anomaly.

**Thresholds:** `configs/detect.yaml`, tagged S/D/A like the other configs and
fingerprinted into every report (`detect_hash`). Tolerances come from the spec
where it gives one and are assumed elsewhere. They were set from the spec and
from clean-module noise, not tuned on defect results.

**Combining captures.** Each feature comes from the capture that saw it best
by design (most golden support), among those capable of it. A terminal row's
common height line is fitted per capture: two captures carry different
alignment offsets and must not share a fit. A residual region already
reported by an earlier capture is not reported again.

**Station result.** The report carries `result`:
- NOK if anything was found;
- INCOMPLETE if nothing was found but some required feature was never
  measured (uncovered), since not looked at is not passed;
- OK otherwise.

**Scan provenance.** Every scan records what it scanned: stage, `--include`d
groups, and a fingerprint of the module's labels file. Detection skips, and
says why, any scan that has no ray index, belongs to another stage, or was
taken of the module before it was last regenerated. Such a scan would
otherwise be scored against the wrong labels without complaint. A scan's
`--include` is carried into the golden capture.

**Scoring** (`evaluate.py`): same class, overlapping boxes (1 mm slack), each
label used once. A class is scored at the stage that inspects it. A finding on
a label of another class, which covers true downstream effects such as the
carrier a reversed cell lifts, is "other", not a false positive.
`lack_of_fusion` (class 16) is never in scope. The verdict per finding and
per label goes to `<stage>.evaluation.json`, which `show_detections.py` draws
in RViz.

Two scoring caveats for 1.7–1.8:
- **"Other" mixes two things.** It holds real knock-on effects (busbars a
  lifted carrier raised) and cross-talk between measures (a bead 60% too wide
  also reads as a 0.16 mm bow, tolerance 0.1). P/R tables must report it,
  and a class confusion matrix should separate the two.
- **A label is a defect only above its tolerance.** The generator labels
  every injection, so a 0.08 mm terminal (tolerance 0.1) scores as a missed
  `terminal_height` when passing it is correct. Severity sweeps must score
  against the tolerance, or labels must carry it.

**Limitations:**

- Sensor poses are exact in simulation. Absolute positions from a wrist sweep
  would carry the HC10DT's pose error.
- Cell pose relies on the pads, so a cell whose pads are both hidden has no
  pose.
- Spatter landing *on* a bead with a seam finding is absorbed into that seam.
- Handling damage on a cell's or side plate's side face is invisible from
  above. That is a coverage miss for 1.5 and 1.10, not a detection one.
- A dent at an edge (end-plate top edge) turns the surface away from the
  sensor and shows mostly as missing returns. It is flagged, but as
  `anomaly`, not `handling_damage`.
- The line-scan places an edge across its scan direction only to its profile
  pitch, so cell X reads in 0.1 mm-quantised steps (injected 0.15 reads
  0.10, 0.25 reads 0.20). Against the 0.2 mm cell tolerance that is coarse:
  the POD curve (1.11) will show it, and the profilometer's 0.05 mm pitch
  halves it.
- Seam acceptance limits are assumed (`A` in `detect.yaml`). §4B.2 says they
  should come from ISO 13919-2 quality levels. The standard is not in the
  repo, and its limits are not sourced yet.
- Seam shape measures interact: a 60% wider bead reads as a 0.16 mm bow (see
  scoring caveats).
- Weld drift on the line-scan is marginal. Under a 0.15 mm fixture error, one
  clean stitch reads 0.144 (tolerance 0.10), and one of two stitches with a
  real 0.15 mm drift reads under it. The profilometer's finer pitch is the
  remedy, which 1.11 should quantify.
- Spatter within 0.25 mm of a bead is treated as a bead-edge effect and not
  reported.
- Latency, line-scan, one capture: ~15–35 s per module on this laptop, most
  of it two ray casts (golden, and the module as measured) and ICP. A real
  cell casts the golden once per station pose, or on a GPU. The per-stage
  split is in `timings_s`.
- The cover (final stage) is not scanned.

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

#### Design — 7 Nov 2026 (`defectgen/coverage.py`, `configs/coverage.yaml`)

Coverage asks whether a capture *can* measure a surface, before any defect is
on it. So it is computed on the clean module of each stage, and it is a
property of the sensor and its pose, not of the detector.

**Surface elements and patches.** The faces of the clean module that some
class needs are subdivided until no triangle edge exceeds `element_size`.
Each triangle has an exact area and normal, and scan points are assigned to
triangles exactly (below). Coverage is judged on **patches**, though, not
triangles. The CAD is finely meshed (median triangle 0.06 mm²), and "one
point on a 0.06 mm² triangle" is luck, not density. A patch *j* is the
observable triangles of one body group that face the same way (normal
rounded to ~20°) within one `patch_size` = 1.5 mm cell: area a_j, mean
normal n̂_j. That gives 85 k patches at post-weld, median 2.2 mm².

**Which elements are required, and how finely.** Each defect class names the
surfaces it lives on and D, the smallest size that matters for it.
§3.7.3's rule of thumb sets the sample spacing it needs, d = D/2:

| Class family | Surfaces | D (mm) | d (mm) | Why |
|---|---|---|---|---|
| cell placement | terminal tops | 0.2 | 0.1 | `cell.pos_tol` |
| busbar pose | busbar tops | 0.2 | 0.1 | `busbar.pos_tol` |
| weld seam shape | weld beads + busbar top within 2 mm | 0.25 | 0.125 | `seam.offset_tol` |
| spatter | busbar, terminal and cap tops | 0.3 | 0.15 | smallest drop to report |
| terminal height/tilt, busbar gap | terminal tops, exposed pad strips | 1.0 | 0.5 | plane fits average; only the face must be sampled |
| polarity, missing cell, FOD | cap tops | 1.0 | 0.5 | features ≥ 1 mm |
| missing pad, carrier seating | pad foam, carrier top | 2.0 | 1.0 | whole-part features |
| handling damage | housing tops and outer faces, cap tops, busbars | 2.0 | 1.0 | dents of a few mm across |

An element's spacing requirement d_j is the finest of the classes that use
it.

**Observable at all.** An element is required only if it can be seen: some
direction within θ_max of n̂_j, at or above horizontal, escapes the module.
The module sits on a pallet, so nothing looks up from below. This is
checked by casting K rays from c_j + ε·n̂_j. It drops faces in contact (cell
sides behind pads, busbar undersides, end-plate faces against cells), so the
arm's ceiling is a real 100%, not a geometric impossibility.

**Covered (§9.8).** Capture *c* covers element *j* when, *in that one
capture*, enough points land on it at an acceptable incidence:

```
n_c,j,k = s_c · #{ points p of c on patch j :  ∠(−ray_p, n̂_p) < θ_max  and  σ_p ≤ D_k/3 }
covered_c,j,k  ⟺  n_c,j,k ≥ max(1, a_j / d_k²)                 d_k = D_k / 2
coverage(k, C) = Σ_{j ∈ R_k} a_j · [∃ c ∈ C : covered_c,j,k] / Σ_{j ∈ R_k} a_j
```

Both halves of §3.7.3's rule are applied. Spacing ≤ D/2 enters through the
count a_j/d². Noise σ ≤ D/3 enters per point: σ_p is the 1σ the sensor model
itself put on that point (distance², weakened by low returned light),
carried on `ScanResult.sigma`.

- **Counts come from the frozen sensor models** (§4B.3): shadowing,
  grazing, range and specular dark/glare dropout all count. A face a bright
  copper busbar returns no light from is not covered, however well it is
  placed.
- **Points are assigned to triangles exactly.** Each point's noise-free hit
  is matched to its closest primitive of the element mesh.
- **s_c rescales a cheaper simulation to the real sensor's density.** The
  snapshot casts every `stride`-th pixel, so s = stride². Candidate sweeps
  are cast at a coarser pitch than flown, so s = cast pitch / flown pitch.
  Counts scale linearly with density, so this is exact in expectation.

**Fixed baseline.** The Ranger3 pass detection uses: one sweep the length of
the module.

**Arm candidates** (no kinematics; reachability comes in Phase 2):

- *Snapshot (Zivid):* eye points on a hemisphere about the module centre at
  the focus distance (elevation × azimuth grid), looking at the centre, and
  closer views at the near working distance aimed at each module quadrant.
- *Sweep (Keyence):* lanes parallel to the module's X at several Y offsets,
  and parallel to Y over each end plate, each flown level and rolled about
  the sweep axis by ±30° and ±60° (to see terminal and busbar walls and weld
  toes). Side-on sweeps along each side plate's outer face. Standoff is the
  reference distance from the surfaces the lane targets.

**Greedy set cover.** Each step takes the candidate that adds the most
*mean class coverage* per second. Every class weighs the same, however
large its area: otherwise the housing walls (≈ 106,000 mm²) would swamp
the weld seams (≈ 3,700 mm²). A capture's time is

```
t_c = t_move + t_capture          t_capture = 0.1 s (snapshot) or L / v (sweep)
```

It stops when no candidate adds more than `min_gain` mm². Greedy is within a
factor ln(max |set|) of the optimum (Chvátal 1979), and every pick is
explainable. Each step records cumulative coverage per class, so the
write-up can plot coverage against captures and against seconds.

**What 1.5 reports**, per stage:
- coverage per class for the fixed pass, the arm plan and the arm ceiling
  (all candidates together);
- the elements the fixed pass misses that the arm sees;
- the view plan (`data/coverage/<stage>.plan.json`): sensor poses and sweeps
  in the module frame, in the order to fly them. Task 2.5 turns it into
  reachable flange poses, and 1.7 scans its dataset with it.

#### Results — 7 Nov 2026

`python -m defectgen.coverage` (all three stages, about 5 minutes; candidate
counts are cached). Percent of each class's required area that can be
measured, in a single capture:

| Stage | Class | D (mm) | Fixed line-scan | Arm plan | Arm ceiling | Both |
|---|---|---|---|---|---|---|
| pre-busbar | cell placement | 0.2 | 100.0 | 99.8 | 99.8 | 100.0 |
| | terminal height / tilt | 1.0 | 100.0 | 100.0 | 100.0 | 100.0 |
| | cell presence, polarity | 1.0 | 91.5 | 100.0 | 100.0 | 100.0 |
| | FOD | 1.0 | 94.0 | 100.0 | 100.0 | 100.0 |
| | missing pad | 2.0 | 100.0 | 100.0 | 100.0 | 100.0 |
| | **handling damage** | 2.0 | **25.0** | **99.8** | 99.8 | 99.8 |
| pre-weld | busbar pose | 0.2 | 100.0 | 100.0 | 100.0 | 100.0 |
| | busbar gap | 1.0 | 99.6 | 100.0 | 100.0 | 100.0 |
| | carrier seating | 2.0 | 99.0 | 100.0 | 100.0 | 100.0 |
| | FOD | 1.0 | 99.2 | 100.0 | 100.0 | 100.0 |
| | **handling damage** | 2.0 | **9.5** | **98.2** | 98.9 | 98.2 |
| post-weld | weld seam | 0.25 | 95.8 | 97.8 | 99.5 | 97.8 |
| | **spatter** | 0.3 | **79.8** | **93.0** | 93.8 | 93.2 |
| | FOD | 1.0 | 99.2 | 100.0 | 100.0 | 100.0 |
| | **handling damage** | 2.0 | **9.0** | **98.9** | 98.9 | 98.9 |

The arm plans take 9, 8 and 12 captures, 29 s, 28 s and 55 s. The moves are
an assumed 1.5 s each; Phase 2 replaces them with planned motion. The fixed
pass takes 3.6 s of conveyor travel.

**What it says:**

- **The fixed line-scan sees nearly everything that faces up.** That covers
  every placement, height, gap and seating feature, and 96% of the weld seam
  area, at the required density. For those classes the arm's coverage adds
  nothing. Its value there can only come from recall (1.10), i.e. the
  profilometer's finer metrology, not from seeing more.
- **Its blind spot is the walls.** Handling damage on the housing's outer
  faces is 9–25% covered from above, 98–99.8% by the arm. That is the
  26V188 mechanism (§3.7.10, §4B.9): damage the overhead station
  structurally cannot see.
- **Spatter: 80% fixed, 93% arm.** The arm's near snapshots (350 mm) and
  sweeps reach density the 500 mm line-scan does not, beside terminals and
  busbar walls. Coverage is necessary, not sufficient: the line-scan covers
  80% of spatter area, yet 1.4 found only 2 of 10 realistic drops on it.
  The D/2 rule is a floor, and detection needs margin above it, which 1.11
  measures.
- **The plan's order is cheap snapshots first, sweeps for what they cannot
  resolve.** The Zivid at 600 mm covers coarse classes (FOD, damage) but no
  0.1–0.15 mm requirement. Its stride-2 cast gives ≈ 0.24 mm effective
  spacing after scaling, and σ ≈ 0.08 mm. The near views at 350 mm cover
  spatter. Keyence sweeps cover the weld seams and the terminal rows.
- **The comparison is one overhead line-scan against an arm**, as §4B.5
  specifies. A fixed station could add side line-scans for the walls. That
  is the fair counter-design for 1.10 to price, not a reason to change this
  baseline.

**Bugs found on the way, both in `scan.py`.** Neither changes the parameter
hash or any earlier result.

- *Snapshot projector field of view.* World to camera axes was written
  `@ rot.T`; it is `@ rot`. Every snapshot before 1.5 looked straight down,
  where that rotation is its own transpose, so nothing showed. The first
  tilted candidate returned 0 points. Regression test:
  `test_a_tilted_snapshot_sees_what_a_straight_one_does`.
- *Per-point noise was not carried through `line_scan`'s chunk assembly.*
  Found by the coverage noise test before any result used it.

**Limitations:**
- No reachability, collision or motion time (Phase 2, task 2.5 consumes
  `data/coverage/<stage>.plan.json`).
- Candidates are a fixed family, not optimised poses. The ceiling is the
  ceiling of *these* candidates.
- θ_max = 60°, the D values and t_move are assumed (`configs/coverage.yaml`,
  tagged).

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

#### 1.7 dataset — Design, 8 Nov 2026 (`defectgen/dataset.py`, `configs/dataset.yaml`)

**A case is one module:** the stage that inspects the class, one injected
defect of that class at one size, and a seed. The seed fixes everything
random:
- where the defect goes (which cell, terminal, busbar or stitch, which
  direction, which surface point);
- each scan's fixture placement;
- each scan's sensor noise.

The dataset is a list of recipes, regenerable bit for bit. Only the small
outputs are kept: recipe, labels, report and score per sensor setup. A
point cloud is ~150 MB and comes back from its seed.

**Two sensor setups per case**, as 1.10 compares them:
- `fixed`: the Ranger3 pass;
- `arm`: the 1.5 view plan for that stage (`data/coverage/<stage>.plan.json`).

**Fixture placement per scan.** Before each scan, the module is moved by a
random rigid error, as a pallet reload would place it:

```
(dx, dy) ~ N(0, σ_xy²),  dz ~ N(0, σ_z²),  yaw ~ N(0, σ_yaw²),  roll, pitch ~ N(0, σ_tilt²)
```

σ values are assumed (`dataset.yaml`, A). This is also what varies the
sampling phase that §4B.4 found makes the line-scan's X quantised. Without
it, every scan of a part would share one phase, and %GRR would flatter the
sensor.

**Out of spec or not: the scoring fix 1.4 left open.** Every injection is
labelled, but not every label is a defect. Each class is one of three
kinds:

| Kind | Classes | Defect when | Found below the threshold |
|---|---|---|---|
| tolerance | cell placement (xy, yaw), terminal height and tilt, busbar pose and gap, carrier seating, weld shape | magnitude > its `detect.yaml` tolerance | **false reject** (FP): the part was in spec |
| presence | spatter, FOD, handling damage | size ≥ D (the smallest that matters) | neither credit nor penalty; kept for POD only |
| binary | missing cell, reversed polarity, missing pad, missing weld | always | — |

The judgement uses the recipe's true parameters, not the label's single
magnitude. Cell placement, for example, has separate position and yaw
tolerances, and the label holds only the shift.

Per case and setup, the score is:
- `hit` / `miss` for an out-of-spec defect (a same-class finding on it, or
  none);
- `not_covered` for an out-of-spec defect missed where the setup does not
  look by design: no patch the class requires inside the label's box is
  covered by that setup in 1.5's coverage map
  (`data/coverage/<stage>.patches.npz`). POD is conditional on the location
  being inspected (MIL-HDBK-1823A). What a setup cannot reach is a property
  of the station, and it is reported beside POD, not folded into it. Added
  9 Nov, after the fixed run's side-plate dents (below);
- `false_reject` / `pass` for an in-spec one;
- the count of false alarms elsewhere (findings on no label);
- `other` (findings of another class on the label: knock-on or cross-talk,
  kept apart, §4B.4 scoring caveats).

**Tiers.**

| Tier | Content | Feeds |
|---|---|---|
| `pod` | 20 class families × 6–7 sizes × seeds. Tolerance classes at f × tol, f ∈ {0.5, 0.8, 1.1, 1.3, 1.6, 2, 3}, so two levels are in spec and the transition is dense; presence classes span their detection limit; binary at one level | P/R and specificity with CIs (1.8), POD (1.11), fixed vs arm recall (1.10) |
| `clean` | clean modules per stage | false alarms per module, the denominator precision needs |
| `grr` | three characteristics (terminal height, cell position, busbar gap); 10 parts with true values spread over ±1.5 tol × 3 loads (re-fixtured) × 2 trials (noise only) | %GRR (1.9), AIAG crossed design, with "appraiser" = pallet load |

FOD and handling damage are injected at pre-busbar only. Their detector
(residual regions) does not depend on the stage, and later stages would
triple the cost for little new information.

#### 1.7 dataset — Pilot, 9 Nov 2026

**Fixed setup: works.** One case per family at its largest size (20
cases): 19 hits and 1 partial (spatter), each case re-fixtured at random.
The detector's reading of the injected feature matches the injection:

| | injected | measured |
|---|---|---|
| terminal height | 0.30 | 0.300 |
| tilt | 0.93° | 0.959° |
| cell yaw | 0.30° | 0.293° |
| cell shift | 0.60 | 0.654 (X quantised to the 0.1 mm pitch) |
| busbar shift | 0.60 | 0.611 |
| gap | 0.45 | 0.452 |
| carrier | 0.60 | 0.600 |
| short weld | 0.30 | 0.298 |
| seam offset | 0.75 | 0.752 |
| crown | 0.30 | 0.287 |
| drift | 0.30 | 0.286 |
| bow | 0.30 | 0.312 |
| width change | −0.60 | −0.78 (a very narrow bead; past tolerance either way) |

About 25 s per case.

**Two detector changes the pilot forced (§4B.4), both re-verified against
1.4's regression:**

- *Drift window.* Under a random fixture offset, a bead's fine ripple
  sampled at the 0.1 mm pitch aliased into a bin-by-bin alternation of the
  estimated centreline (±0.12 mm, period 0.5 mm). That read as 0.2 mm of
  `weld_path_drift` on clean stitches. Path drift wanders over millimetres,
  so the centreline is now smoothed over `drift_window` = 1 mm before its
  wander is measured. A real 0.3 mm drift still reads 0.288; a 6 mm
  wavelength loses ~5%.
- *Discontinuity band.* Residual regions mostly within misregistration
  reach of a discontinuity in the golden capture's own range image are not
  defects. A discontinuity is a depth step between neighbouring rays, or a
  shadow / silhouette boundary. Reach is half a sample where the end-plate
  datum fixed X and Y, else `fixture_tol`. This generalises the outline band
  to every edge in every view, and was needed for any capture that keeps the
  fixture pose.

**Memory, on a 16 GB laptop.** A full-length Keyence sweep is 24 M rays and
10.7 M points. The arm's post-weld plan has six of them. The first pilot
was OOM-killed at 12 GB. What it took, in `detect.py` and `dataset.py`:
- the golden cache is bounded by points, and aligned poses (never seen
  again) are forgotten;
- each capture's sorted copies and golden are released once it has been
  measured;
- an aligned capture replaces its original in place;
- the as-measured module is built once per detection, not per capture;
- freed memory is handed back to the OS (`malloc_trim`);
- the golden stores directions and footprints as float32;
- sweeps are joined field by field;
- the dataset spills captures to disk and memory-maps them back, as a real
  cell streams captures to disk.

Peak for the heaviest arm case is now ~5–7 GB. Dataset runs are capped at
7 GB (`systemd-run --user --scope -p MemoryMax=7G`), so an overrun kills the
job, not the desktop.

**Arm setup: blocked on oblique-view detection** (as found by the pilot;
resolved 9 Nov, see "Arm hardening" below: the cause was registration, and
the per-ray noise threshold and slope allowance proposed here have not been
needed). Detection was built and verified on level views. The 1.5 plan adds views it has never seen: tilted
snapshots, sweeps rolled ±60°, side-on sweeps. On the first arm case these
raised 90 false alarms; 32 remain after the band. Experiments on a clean,
perfectly placed module separate the causes:

- *The comparison itself is exact for oblique sweeps.* The noise-free scan
  matches the golden to 0.0 µm on all 12.5 M rays, so there is no geometry
  bug.
- *Partial outlines.* A rolled sweep sees part of a busbar at the end of its
  line. The outline is then set by where the view ends, and reads 1.4 mm /
  1.0° of "pose". Outline-based positions need the whole face in view.
- *Heavy-tailed noise on weak returns.* Grazing copper returns little light,
  and the model raises its noise up to 10×. A threshold from the material's
  median noise flags those rays. The threshold should be per ray, from
  returned signal, which real sensors report.
- *Misregistration on sloped surfaces seen obliquely* (the moulded
  carrier). A first per-ray slope allowance fragmented the regions instead
  of removing them, and was withdrawn.

Until these are fixed and re-verified, the arm column of the dataset would
measure the detector's oblique-view gaps, not the arm's value. The `pod`
tier therefore runs with `--setups fixed` first. The arm setup waits on a
dedicated hardening step, which is a prerequisite of 1.10.

The arm case costs ~4 min (post-weld, 12 captures) against ~25 s fixed.

#### 1.7 dataset — Fixed run, 9 Nov 2026

`clean` (30) and `grr` (180) finished with no false alarms. `pod` stopped at
case 462 of 565. Two things it showed:

- *A crown lost entirely divided by zero.* `reinforcement` at 3 × tol with a
  negative sign is −0.30 mm on the 0.30 mm crown. That leaves a flush weld,
  which is a real outcome (underfill), not an impossible draw. `weld._shape`
  scaled its ripple and crater by the crown height. A crown ≤ 0 is now built
  as flush or sunk, with its ripples and no crater. A crown above zero is
  built exactly as before, so finished cases still reproduce bit for bit.
  Before resuming, every remaining recipe was built once without scanning,
  and all built.
- *Dents on the side plates' outer faces were scored as misses.* All 8
  `handling_damage` misses, at every size up to 8 × D, sit on a vertical outer
  face that a downward line scan never sees. Every dent on a top face was
  found. These are coverage, not detection, so the scoring rule above now
  calls them `not_covered`. `python -m defectgen.dataset --tier <t>
  --rescore` re-scores stored reports without detecting again. On the stored
  cases it changed exactly those 8 and nothing else.

**Results, fixed setup (775 cases).** No false alarm anywhere: 0 findings
on any of the 30 clean modules, and 0 findings off the injected defect in
any of the 565 `pod` or 180 `grr` cases. Out-of-spec defects: 343 hit,
17 partial, 27 miss and 8 not covered, out of 395. In-spec defects: 8 false
rejects out of 120. Tabulated by `--summary`. Intervals, POD and %GRR are
1.8, 1.11 and 1.9. Per family:

| Family | Found on every seed from | In-spec false rejects | Notes |
|---|---|---|---|
| terminal height, gap, carrier, width | 1.1 × tol | 0 / 0 / 0 / 2 of 10 | readings within µm of truth, except width, which reads high (0.6 → 0.78) |
| terminal tilt | 1.1 × tol | 0 | |
| busbar shift, cell shift | 1.3 × tol | 2 / 2 of 10 | X steps at the 0.1 mm profile pitch: 0.22 can read 0.177 |
| seam offset, short weld, path drift | 1.3 × tol | 0 / 0 / 1 | short weld reads ~0.025 short |
| cell yaw, weld bow | 1.6 × tol | 0 / 1 | yaw also steps with the pitch (~0.025°) |
| crown | 1.3–2 × tol | 0 | a sagging crown reads 10–20 % short, so 1.1 × misses on every seed. At 3 ×, the two negative draws leave a flush weld, rejected as `missing_weld` on both stitches: right decision, wrong class. Height alone cannot tell a flush weld from no weld (heat tint can: [welding_followup.md](welding_followup.md)) |
| binary (missing cell, pad, weld; reversed polarity) | always | — | |
| FOD | D (0.25) | — | below D found 2 of 15 |
| handling damage | D, on faces the setup sees | — | every dent on a top face found, every side-plate dent `not_covered` |
| spatter | never | — | 2 of 15 cases complete. Of 75 drops ≥ D: 0 of 13 on a bead found, 4 of 16 within 1.5 mm of one, 30 of 43 clear of the beads (10 of those 13 misses are at D itself; 3 sit by a wall shadow or busbar edge). The bead band hides spatter on and beside the weld: a detector limitation, kept, since judging a bead against its expected shape is the welding follow-up's job |

`grr`: the 10 terminal heights and the 10 busbar gaps read to ~1 µm across
loads and trials. Cell position X steps with the profile pitch: a true
0.167 reads 0.100 or 0.175 depending on the load (3 false rejects), and a
true 0.233 reads 0.200 once (1 miss). This is the line-scan pitch, not
noise, and it is what %GRR (1.9) will show.

#### 1.7 dataset — Arm hardening, 9 Nov 2026

**What the arm's false alarms were.** The same clean modules, through the
arm plan:

| | pre_busbar | pre_weld | post_weld |
|---|---|---|---|
| placed exactly | 0 | 0 | 0 |
| with the fixture error (0.15 / −0.10 / 0.03 mm, 0.05° yaw, 0.02° roll) | 6 | 6 | 50 |

Detection on oblique views was not the problem: with the module placed
exactly, every view compared clean. The problem was registration. Measured
against the known fixture error (`python -m defectgen.probe_arm`):

- *No arm view registered itself.* X, Y and yaw came from the end-plate
  outlines. A sweep along X sees ~40 mm of a 148 mm end plate, so the view
  sets the outline's far edge, on the golden as on the scan. The ±60°
  sweeps were left 0.1 mm off in Y, the end-on sweeps 0.17–0.34 mm off
  (yaw from one plate), the snapshots 0.25 mm off (too coarse to register
  at all). Yet the residual pass assumed 0.025 mm. Seen at 60°, 0.1–0.2 mm
  of misregistration reads as 0.2–0.4 mm "dents" on the sloped carrier.
- *The end-plate datum was biased, in the fixed pass too.* The side plates
  run the full length and are flush with the end plates. An end-plate
  outline taken out to its corners runs on along them to the edge of the
  search window, which is fixed in the module frame. An end plate read
  0.045 of a 0.15 mm shift. In 1.4 the residual band absorbed it.

**What changed (`detect.py`: `datum_strips`, `solve_datum`, `joint_datum`,
`housing_fit`, `align`):**

- *Four housing strips, each trusted across its own width only.* These are
  the two end-plate tops and the two side-plate tops, over their middle
  spans, clear of the corners. An end strip fixes X, a side strip Y, and a
  long edge yaw. A strip whose golden outline is short of its design width
  (`fit.outline_slack` + 3 samples) was cut by the view and counts for
  nothing. X, Y and yaw are solved together by least squares, and an axis
  no strip fixes keeps the fixture pose.
- *One registration per load.* All captures of one pallet load see one
  module in one placement, and the station knows its own poses. So X, Y
  and yaw are solved once, from each strip's best whole view across all
  captures, and every capture is registered with it. Z, roll and pitch stay
  per capture, from ICP on the housing. The fixed pass sees all four strips
  in one sweep and so registers itself, as before but without the bias.
- *The residual pass allows the fixture tolerance on any capture not fully
  registered*, not only on the ones that kept the fixture pose outright.

Measured on post-weld through the arm plan with the fixture error: every
capture is registered to ≤ 0.002 mm in X and Y (0.017 mm in Z for the
tilted snapshots), and the clean module gives **0 findings (was 50)**.

Re-verified with `python -m defectgen.regress`, which gained an `arm` group
(the three stages through the arm plan, placed exactly and with the fixture
error) and a `--setup` option:
- `arm`: 0 findings on all six (was 0 / 6 / 0 / 6 / 0 / 50);
- `single`, `fixture`, `seeds`, `subtol` (line-scan): unchanged decisions and
  no false alarm. The misses are the known ones: spatter on beads (8 of 10
  drops), one liftoff stitch, and the under-tolerance cases, which are
  correct not to be flagged.

The fixed column was re-run with this detector so that both columns of the
dataset come from one detector; the earlier fixed results are kept in
`data/dataset_prestrips/`. `python -m defectgen.probe_arm --stage <s>
[--fixture]` prints, per capture, the registration error against the known
placement, the strips it saw whole or cut, and what the residual pass
assumed.

**Assumption.** Capture poses are exact. A real arm adds its own pose
error (HC10DT repeatability plus hand-eye calibration), which this
registration would inherit per capture. Modelling it belongs with Phase 2's
robot model.

#### 1.7 dataset — A strip that moves with the defect, 9 Nov 2026

The fixed column re-run on the strip datum finished (775 cases, kept in
`data/dataset_strips_v1/`): 0 findings on the 30 clean modules, 1 false
alarm in 775 (a weld bow of 0.1009 on an untouched seam, tol 0.10: the far
tail of the bow measure's noise, p99.9 0.098 mm, unchanged from the first
pass). In-spec short welds rejected fell from 5 of 10 to 0.

The full test suite then failed one test: a cell shifted (0.3, 0.2) read
(0.25, 0.14) through the profilometer. A clean module read exactly zero, and
HEAD's detector read it right, so it was the strip datum. A cell set off its
pitch toward a side plate bows the 1.5 mm sheet out (settle.py, by design),
and the side-plate strip moved with the bow. One outline round the whole
strip follows its worst edge, so the datum took up part of the cell's own
shift and subtracted it from the cell: 33 % of it through the part view, and
in the fixed run a dy error against true dy with slope −0.11 (first pass:
0.00). Recall did not show it (cell shift 21 of 25 vs 22), and no test of
the dataset would have: only the reading's error against truth did.

**Fix (`detect._strip_offset`, `icp.datum_bin`).** Each strip's centreline
is taken per 2 mm bin along it (midway between the edges seen in that bin).
A line is fitted to golden − scan, bins more than 3 robust σ off it (at
least one sample spacing) are dropped, and the line is refitted. The line
gives the strip's offset at its centre and, from its slope, the yaw. A bow
over a minority of the strip drops out: on a synthetic 0.2 mm bow over 40 of
140 mm, the old outline was 100 µm off and the robust line is 2 µm off. The
cell now reads 0.2007 for 0.20. All 25 detector tests pass, and the new
`test_a_locally_bowed_strip_does_not_move_the_datum` covers it.

The whole dataset re-runs on this detector: the regression first (every
group, `arm` included), then the arm `clean` tier, the fixed column, and the
arm `pod` tier.

**Lesson for 1.8.** Recall and decisions can hide a measurement bias that
the reading-vs-truth error shows at once. 1.8 reports the reading error per
family as well (mean, sd and slope against truth).

**Also found: a generator contradiction, not a detector fault.** In-spec
`busbar_shift` L0_0.1_s0 (dy −0.10) is rejected as `busbar_liftoff` 0.50.
The generated module really has busbar 0 resting 0.506 mm high, tilted
4.8°, at dy −0.163. So a 0.1 mm Y shift, inside the ±0.2 pose tolerance,
puts the busbar onto something it then rests on. This is the same kind as
the carrier at 0.16 mm lifting the busbars past the 0.15 gap tolerance:
either the geometry has less clearance than the tolerance allows, or the
tolerance is wrong. To be checked against the CAD.

#### 1.8 and 1.9 — Statistics over the dataset, 9 Nov 2026 (`defectgen/validate.py`)

`python -m defectgen.validate pr|grr [--setup fixed|arm]` reads only the stored
cases (index, recipe, labels, report), so it is light and runs beside a heavy
job. It prints the tables below and writes `data/dataset/validation/`.

**1.8 method.** The unit is a case (one module, one injected defect, one
setup).
- *Recall* is of out-of-spec defects, conditional on the setup looking there
  by design: `not_covered` cases are counted but outside it (MIL-HDBK-1823A).
  A case with several targets where only some were found (`partial`) is not
  a hit.
- *Rejected* is the module decision (NOK) on the same cases. A partial earns
  it, and so does a right decision with the wrong class: a flush weld rejected
  as `missing_weld`.
- *False reject rate* is over in-spec cases.
- *Precision* is per finding class, from `pod`: correct findings over
  correct + false-reject + false-alarm findings.
- *Other* findings (a finding of another class on the injected defect) are
  counted apart, and broken down by the case's injected class. This
  separates knock-on effects (a lifted carrier raises its busbars:
  `busbar_liftoff` on `carrier_seating`) from cross-talk between measures (a
  short weld's bare busbar end read as `handling_damage`; bead width read as
  bow). These are the two §4B.4 caveats.
- *False alarms* (findings on no label) are counted over every scanned module
  of all three tiers, each an independent fixture and noise draw. The bound
  is the exact Poisson 95% upper bound.
- *Reading error* per family: |reading| − injected size over every `pod`
  case with a reading, as mean, sd, and slope over the size (± its standard
  error). A slope off zero is a proportional bias. This is the check that
  found the datum fault above: cell shift slope +0.018 ± 0.048 in the first
  pass and −0.081 ± 0.034 on the unfixed strip datum, while recall barely
  moved.

All proportions are given with Wilson 95% intervals. At 25 cases per family,
a perfect score bounds recall only to ≥ 87%. Tightening that needs more
seeds, not a better detector.

Results: after the fixed re-run (`pod`) completes.

**1.9 %GRR, fixed setup (line-scan).** AIAG MSA ANOVA method, crossed:
10 parts × 3 loads × 2 trials per characteristic, with the pallet load as
"appraiser". The part × load interaction is pooled into repeatability when
p > 0.05. %GRR to tolerance is 6 σ_GRR / (USL − LSL); the gap's spec is
one-sided, from 0 to 0.15. The parts were spread over ±1.5 tol by design,
not drawn from a process. So %GRR to study variation and ndc describe that
design, and %GRR to tolerance is the figure to judge the gauge by.

| Characteristic | EV σ (µm) | AV σ (µm) | GRR σ (µm) | %GRR to tolerance | %GRR to study | ndc | mean bias (µm) |
|---|---|---|---|---|---|---|---|
| terminal height (±0.10) | 0.5 | 0.1 | 0.5 | **1.5 %** | 0.5 % | 285 | −0.0 ± 0.1 |
| busbar gap (0–0.15) | 0.1 | 0.0 | 0.1 | **0.5 %** | 0.1 % | 1781 | +0.5 ± 0.2 |
| cell position X (±0.20) | 7.2 | 36.7 | 37.4 | **56 %** | 18 % | 7 | −21 ± 5 |

AIAG's guide is < 10% acceptable, 10–30% conditional, > 30% not acceptable.
- *Heights and gaps* are plane fits over thousands of points: the gauge is
  far inside the guide.
- *Cell position along X fails it.* It is almost all reproducibility, load
  to load, with a strong part × load interaction (p < 0.001). The readings
  step in 0.05 mm: the centre of two edges, each quantised to the 0.1 mm
  profile pitch. Each load shifts the sampling phase, so the same part reads
  0.10 or 0.15 depending on the load. With σ_xy = 0.03 mm, a third of the
  pitch, 30 loads cover the phase unevenly. The −21 µm mean bias is most
  likely that, not a calibration offset (not chased further).
- This is the line-scan's pitch, not its noise (EV is 7 µm), and it is what
  1.10 and 1.11 should test: the profilometer's 0.05 mm pitch, or the arm's
  views. Before the strip datum (prestrips run) the same study gave 50%,
  with the same structure.

#### Later: a held-out validation set (added 10 Nov 2026)

**Why.** The 1.7 dataset is not a clean test set any more. Three detector
changes were made *because of* what it showed: the short-bead stretch, the
strip datum, and the robust strip centreline. Each was right, but numbers
reported on the data that shaped the detector are optimistic. That is the
same train/test leak as in ML, and a reviewer from quality engineering will
ask about it.

**What.** Once the detector is frozen (after the arm column and any fix it
needs), generate a fresh dataset with the same recipe (`configs/dataset.yaml`)
but **new seeds**, never looked at while tuning. Run both setups on it once,
change nothing, and report 1.8–1.12 from it. The current dataset becomes the
development set. Size: the same tiers, or at least `clean` and `pod`. The
`grr` tier measures the gauge, not a decision rule, so it can stay.

**Rule from then on.** Any detector change after the held-out run means a
new held-out seed set, not a re-run of the old one.

**Open from the arm column (partial, 222 of 565 `pod`, 10 Nov).** All 30
clean modules are clean, but 8 findings on no label in 4 cases, all on Zivid
snapshot views:
- three large cell shifts (2–3 × tol), where the shifted cell's bowed side or
  end plate is flagged as `fod` / `handling_damage`. The module was already
  rejected for the cell, so this is a knock-on effect, scored as a false
  alarm;
- one below-D dent case (`handling_damage` L1_0.05_s0), where the snapshot
  flags raised regions on the foam pads and rejects a module it should not.
Investigate once the column completes, before freezing the detector.

#### Optional, after the held-out set: a learned region classifier (added 10 Nov 2026)

**Where it fits.** The detector has two halves. *Measurement* (plane and
outline fits: terminal height, gap, cell and busbar pose, seam profile) is a
gauge, with %GRR of 0.5–1.6 % on heights and gaps, and stays rule-based so it
stays traceable (automotive MSA). *Classification* of residual regions (after
the comparison with the golden capture: is this region FOD, a dent, spatter, a
knock-on of another defect, or nothing?) is hand-written thresholds
(`residual` in `detect.yaml`: area, peak, aspect, material, edge band). The
known errors are all in that second half: a bowed side plate read as `fod`
on arm snapshots, foam pads read as raised regions, spatter on and beside
the bead missed, a flush weld classed `missing_weld`. Only that half is
replaced.

**What.** A gradient-boosted tree classifier (e.g. LightGBM or scikit-learn
`HistGradientBoostingClassifier`) over per-region features:
- area, peak |r|, aspect, sign;
- material under it;
- capture, its sample spacing and view angle to the surface normal;
- distance to the nearest measured outline, and how far that part moved;
- distance to the nearest bead.

Labels are exact and free, from the generator's labels: the defect class,
`knock_on` (inside a part that a labelled defect moved or deformed), or
`none`.

**Steps.**
1. `detect.py` writes every residual region it segments, with its features,
   into the report, kept or not. That is a report-only change; the
   decisions are unchanged.
2. Regenerate the development set (the current 1.7 recipe and seeds) to get
   the regions. The dataset does not keep point clouds, so this is one full
   re-run, both setups.
3. Train on development regions only. Tune with grouped cross-validation by
   module, never by region, so regions from one module do not land on both
   sides.
4. Score once on the held-out set: rules vs classifier, the same 1.8 tables
   (recall, precision, false alarms per module, Wilson / Poisson intervals),
   per setup. Report whichever wins, with feature importances to show what
   it learned.

**Stated limitation.** Trained and tested on synthetic scans only. It can
learn the generator's and sensor model's quirks, so the result says "a
learned classifier fixes these errors in simulation", not on a real line.
Real scans would be the test that closes that, which is out of scope.

**Not chosen, recorded so they are not re-asked:**
- *Learning the measurements.* It would break traceability and gain
  nothing on a gauge already well inside AIAG's limits.
- *Unsupervised anomaly detection on height maps (PatchCore-style).* The
  comparison with the golden capture is already anomaly detection with a
  perfect reference. A side-by-side would be a fair, probably negative
  result, worth at most a paragraph.
- *POD curves (1.11).* Hit/miss logistic regression: statistics, and
  reported as such.

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

### 4B.10 — Parked: process-driven welds (follow-up project)

Weld defects in §4B.2 are idealised shapes injected directly. A later,
welding-specific project would make them *consequences* of the process:
- scanner speed, power and wobble;
- seam-position error;
- part vibration on the conveyor;
- the gap.

Spatter would become time-ordered events with interactions (remelted
under the beam, on the bead, in a burn-through), and heat tint would join
the 2D camera. Out of MotoBat scope by decision (9 Nov 2026): none of
MotoBat's claims (coverage, recall, %GRR, the correction loop) needs a
physical weld model. Design sketch, research digest and first steps:
[`welding_followup.md`](welding_followup.md).

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

**Parallel worktrees (from 6 Nov 2026).** The Phase 2 ROS work lives in a second
worktree, `~/projects/MotoBat-arm` on `feature/hc10dt_gazebo`, beside the Phase 1
folder. Which tasks and docs belong to each lane: `status_update.md`, "Two lanes".
Setting up a worktree in this repo needs four things that git doesn't do for you:

- **`src/_vendor/` is gitignored, so a new worktree has none.** Symlink the Phase 1
  folder's copy (`ln -s ~/projects/MotoBat/src/_vendor src/_vendor`). Git doesn't count a
  symlink as a directory, so the `src/_vendor/` ignore rule misses it. `src/_vendor` is
  therefore listed in `.git/info/exclude`, which every worktree shares.
- **Build the first time from a clean shell.** `~/.bashrc` sources the Phase 1 folder's
  `motobat_env.sh`, and colcon writes whatever install is on `AMENT_PREFIX_PATH` into the
  new `install/setup.bash` as a permanent underlay. The worktree would then quietly resolve
  the vendor packages from the other folder. Do the first build under
  `env -i HOME=$HOME PATH=/usr/bin:/bin bash -c 'source motobat_env.sh && colcon build --symlink-install'`,
  and check with `grep MotoBat/install install/setup.bash`, which should find nothing.
- **In every terminal, source the worktree's own `motobat_env.sh`.** It finds its own
  folder, so it overlays that folder's `install/` on top of what `~/.bashrc` loaded.
- **`data/` is gitignored too.** Copy in what a task needs.

Remove a worktree with `git worktree remove ../MotoBat-arm`. Its branch and commits stay.
 
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
 
