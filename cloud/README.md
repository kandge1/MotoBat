# Running a dataset on Vultr

A full 1.7 dataset (775 cases × 2 setups) takes ~30 h on the laptop. The laptop runs one case at a
time, because RAM (15 GB) is the limit, not cores. On a 64-vCPU server, 14 cases run at once:
~2.5–3 h. Cost is ~$7 per full dataset on the plan below.

**Plan: `vx1-g-64c-256g`** (VX1 General Purpose, 64 dedicated vCPU, 256 GB, block storage,
$1.920/hr as listed 13 Nov 2026).
- CPU is what limits us: one case uses ~4.5 vCPU.
- The 512 GB plans buy RAM we don't use: 14 arm cases need ~100–150 GB.
- Local NVMe isn't needed: scan scratch goes to a RAM disk.
- Cost scales with vCPU-hours, so a bigger machine finishes sooner for about the same total.

## The billing rule that matters

**A server bills every hour it exists, powered on or off. Only destroying it stops the bill.** At
$1.92/hr, a forgotten server costs ~$46 a day. Fund the account prepaid (e.g. a $20 PayPal top-up,
no saved card) so it can never take more than you put in.

## One-time setup (on the Vultr site)

1. Create an account. Add a small prepaid top-up. Check what any first deposit is before paying.
2. SSH key: on the laptop run `ssh-keygen -t ed25519`, pressing Enter through the prompts. Paste
   the contents of `~/.ssh/id_ed25519.pub` into Account → SSH Keys.
3. Optional, for `destroy.sh`: Account → API → enable, copy the key, then:
   `mkdir -p ~/.config/motobat && (umask 077; cat > ~/.config/motobat/vultr_api_key)`.
   Paste the key, press Enter, then Ctrl-D. Restrict the key's access to this laptop's IP if the panel offers it.

## Each run

0. **Before paying for anything:** `python defectgen/known_cases/run.py --check` must say the known
   cases passed for this code. The server refuses to start otherwise
   (`defectgen/known_cases/README.md`).
1. **Deploy:**
   - Cloud Compute → VX1 → `vx1-g-64c-256g`;
   - a US-East location;
   - Ubuntu 24.04 LTS x64;
   - select your SSH key;
   - auto-backups **off**.

   The bill starts now.
2. `mkdir -p ~/.config/motobat && echo <server IP> > ~/.config/motobat/cloud_host`
3. `bash cloud/setup.sh`: system libraries, Python 3.11, venv. ~5 min.
4. `bash cloud/push.sh`: copies the working tree, the run inputs and the pinned packages
   (`defectgen/requirements-lock.txt`). Data files are excluded except what a run reads.
5. `bash cloud/verify.sh`: the tests, then two known cases, which must match the laptop. **Do
   not start if they differ.**
6. `bash cloud/start.sh 14`: runs detached on the server. Arm first (clean, pod, GRR), then
   fixed.
7. `bash cloud/status.sh --wait`: checks every 5 min and rings when done, or when the tripwire
   pauses the run.
8. `bash cloud/pull.sh`: copies results into a new `data/dataset_cloud_<date>/`, never over
   `data/dataset`, and counts them (complete = 1550 score files).
9. `bash cloud/destroy.sh`: refuses unless the pull is complete, then asks you to type `yes`.
    Without an API key, destroy the server in the panel. **Then check the panel lists no servers.**

## What the scripts rely on

- The server mirrors the laptop's paths (`~/projects/MotoBat`, `~/motobat/venv`), so
  `data/tripwire.py` and `data/pause.sh` work unchanged there.
- Those two are gitignored under `data/`, and `push.sh` copies them.
- `python -m defectgen.dataset --shard i/N` gives worker *i* every N-th case, so no two workers
  ever share a case. Two jobs on one case shared scratch files and crashed (SIGBUS, 11 Nov 2026).
- Each worker is memory-capped as on the laptop (fixed 10G, arm `data/arm_memmax`).
- A worker that dies is retried, up to 3 times per tier; finished cases are kept.
- `data/tripwire.py` pauses everything if false alarms look systematic (fixlist F7).
- `data/PROVENANCE.txt` records the commit and uncommitted files a run was computed from.
