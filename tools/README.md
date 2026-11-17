# tools

## Pause and resume a run

    bash tools/pause.sh       # stop whatever is running; what to restart goes in data/PAUSED
    bash tools/resume.sh      # start exactly that again, detached

These work for any dataset run, on the laptop or on a Vultr server.

What they handle:
- a chain started as `bash <script>.sh --detached` (`data/check_f10.sh`, `data/chain_v3.sh`, ...),
  or `cloud/run_remote.sh`;
- a job run on its own: `defectgen.dataset`, `known_cases/run.py`, `analysis.sensitivity`.

Use `--dry-run` on either to see what it would do.

- **Pause** stops each chain as a whole process group: the chain, its memory-capped job and the
  cloud tripwire. The chain therefore can't restart the job. It kills by PID/PGID, never by name.
- **Finished cases are kept.** The case in progress is redone, which loses at most a few minutes
  per worker.
- **Resume** refuses if anything is already running, because two runs on one dataset crash
  (SIGBUS, 11 Nov 2026).
- **A chain restarts from its first step**, and steps already finished skip their cases. That
  takes a few minutes, then it carries on where it was.
- **Safe across a reboot:** nothing is held in memory, and `data/PAUSED` is a file.
- **On Vultr, pausing does not stop the bill.** Only destroying the server does
  (`cloud/README.md`). To pause and come back later, pull first.

`data/pause.sh` is a wrapper kept for `data/tripwire.py`.
