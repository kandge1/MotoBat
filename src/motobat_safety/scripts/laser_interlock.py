#!/usr/bin/python3
"""S4b laser cell: the laser-enable interlock, and an exhaustive check of it. (task 3.5b)

    laser_interlock.py              verify (all reachable states, every single stuck channel) + timing

The GP25's MOTOPAC-RL2D carries an 8 kW, 1050-1090 nm fibre laser (project_plan.md §4A.7): Class 4,
so the process zone is enclosed (ISO 11553-1:2020, laser processing machines; IEC 60825-4:2022, laser
guards: both cited by number and title only, their texts were not read). The cell (line_fixtures.xacro
s4b_*): four walls and a roof, two pallet openings each closed by a laser-rated roller shutter, one
maintenance door, guard locking. The HC10DT cells next door are outside it (task 3.5).

Laser enable (to the laser source's safety interlock and safety shutter) requires, together:
    door closed AND door locked, both shutters closed, pallet clamped, robot standing still in its
    weld zone (the GP25 safety unit's safe zone/standstill output), e-stop circuit healthy, active
    guard (wall breach sensor) healthy, no latched fault, and the cell controller's request.
Guard locking: the door unlocks only on request, and only after the laser has been off for T_OFF
(power-to-unlock); a shutter opens only on request with the laser off for T_OFF.

Inputs are read on two channels (ISO 13849-1 Category 3: one fault must not lose the safety function,
and should be detected): a signal counts as safe only if BOTH channels say so; channels that disagree
for longer than T_DISC latch a fault, cleared only by a reset when both agree again. The robot's safe
zone/standstill signal is one already safety-rated output (its own dual channels inside the FSU).

The check (verify): breadth-first search over every event sequence (door, lock request, shutters,
pallet, clamp, robot, e-stop, guard, request, reset), with a safety-PLC scan after every event, for
the fault-free system and for every single sensor channel stuck at 0 or at 1. Invariant, on the TRUE
physical state:  laser on  =>  door shut and locked, shutters shut, pallet clamped, robot at its weld
pose, e-stop healthy, guard intact.  Control: the same search on a SINGLE-channel design must find a
violation (else the check could not fail).
"""
from __future__ import annotations

import itertools
from collections import deque
from dataclasses import dataclass, replace

T_DISC = 3          # scans a discrepancy may last before it latches a fault (A: switch bounce, ~30 ms)
T_OFF = 2           # scans the laser must have been off before a door or shutter may open (A)
SCAN_MS = 10        # ms per safety-PLC scan (A)

SAFE_INPUTS = ('door', 'lock', 'sh_in', 'sh_out', 'clamp', 'estop', 'guard')
# truth True = the safe value: door closed, lock engaged, shutter closed, clamped, e-stop ok, guard ok


@dataclass(frozen=True)
class State:
    # physical truth
    door: bool = True
    lock: bool = True
    sh_in: bool = True
    sh_out: bool = True
    clamp: bool = False
    robot_hold: bool = False
    estop: bool = True
    guard: bool = True
    request: bool = False
    # controller memory
    laser: bool = False
    off_scans: int = T_OFF
    disc: tuple = (0,) * len(SAFE_INPUTS)
    fault: bool = False
    unlock_req: bool = False
    open_req: tuple = (False, False)        # shutter in, out


def read(state: State, name: str, ch: int, fault):
    """Channel ch of input `name`: the truth, unless this channel is the stuck one."""
    if fault is not None and fault[0] == name and fault[1] == ch:
        return fault[2]
    return getattr(state, name)


def scan(s: State, fault, dual: bool) -> State:
    """One safety-PLC scan: read inputs, monitor discrepancies, decide outputs, actuate locks/shutters."""
    ok, disc, fault_latch = {}, list(s.disc), s.fault
    for i, n in enumerate(SAFE_INPUTS):
        a = read(s, n, 0, fault)
        if dual:
            b = read(s, n, 1, fault)
            ok[n] = a and b
            disc[i] = min(disc[i] + 1, T_DISC + 1) if a != b else 0   # saturate: only > T_DISC matters
            if disc[i] > T_DISC:
                fault_latch = True
        else:
            ok[n] = a
    robot_ok = s.robot_hold                         # safety-rated output of the robot's FSU
    laser = (all(ok.values()) and robot_ok and s.request and not fault_latch)
    off = 0 if laser else min(s.off_scans + 1, T_OFF)               # saturate: only >= T_OFF matters
    s = replace(s, laser=laser, off_scans=off, disc=tuple(disc), fault=fault_latch)
    # guard locking (power-to-unlock): release only on request, laser off for T_OFF
    if s.unlock_req and not laser and off >= T_OFF:
        s = replace(s, lock=False)
    # shutters: open only on request with the laser off for T_OFF
    sh_in, sh_out = s.sh_in, s.sh_out
    if s.open_req[0] and not laser and off >= T_OFF:
        sh_in = False
    if s.open_req[1] and not laser and off >= T_OFF:
        sh_out = False
    return replace(s, sh_in=sh_in, sh_out=sh_out)


def events(s: State):
    """Everything that can happen between two scans (people, process, the cell controller)."""
    yield 'request_on', replace(s, request=True)
    yield 'request_off', replace(s, request=False)
    yield 'unlock_request', replace(s, unlock_req=True)
    if s.lock is False:
        yield 'door_open', replace(s, door=False)
    if s.door:
        yield 'door_close_and_lock', replace(s, door=True, lock=True, unlock_req=False)
    yield 'shutter_in_open_req', replace(s, open_req=(True, s.open_req[1]))
    yield 'shutter_out_open_req', replace(s, open_req=(s.open_req[0], True))
    yield 'shutters_close', replace(s, sh_in=True, sh_out=True, open_req=(False, False))
    if s.sh_in is False or s.sh_out is False:       # the pallet can only move through an open shutter
        yield 'unclamp', replace(s, clamp=False)
    yield 'clamp', replace(s, clamp=True)
    yield 'robot_moves', replace(s, robot_hold=False)
    yield 'robot_holds', replace(s, robot_hold=True)
    yield 'estop_press', replace(s, estop=False)
    yield 'estop_release', replace(s, estop=True)
    yield 'guard_breach', replace(s, guard=False)
    yield 'reset', replace(s, fault=False, disc=(0,) * len(SAFE_INPUTS))


def violation(s: State):
    """The invariant, on the physical truth."""
    if not s.laser:
        return None
    bad = [n for n, v in (('door open', not s.door), ('door unlocked', not s.lock), ('shutter in open', not s.sh_in),
                          ('shutter out open', not s.sh_out), ('pallet not clamped', not s.clamp),
                          ('robot not at its weld pose', not s.robot_hold), ('e-stop pressed', not s.estop),
                          ('guard breached', not s.guard)) if v]
    return bad or None


def detected(fault, dual=True):
    """Is the stuck channel ever caught (fault latched) in some reachable state? With two channels a
    stuck one disagrees with its partner as soon as the truth moves away from the stuck value."""
    start = scan(State(), fault, dual)
    seen, q = {start}, deque([start])
    while q:
        s = q.popleft()
        if s.fault:
            return True
        for _, s2 in events(s):
            s3 = scan(s2, fault, dual)
            if s3 not in seen:
                seen.add(s3)
                q.append(s3)
    return False


# Station S4b cycle, s (A unless marked): laser-rated roller shutter 1.0 s per stroke, pallet
# lift-and-clamp 0.5 s each way, weld 10.7 s (task 3.3b: 4.68 s of motion + 24 x 0.25 s of scanning),
# laser off before a shutter or the door may open: T_OFF scans.
TIMING = dict(shutter_in_close=1.0, clamp=0.5, weld=10.7, laser_off=T_OFF * SCAN_MS / 1e3, unclamp=0.5,
              shutter_out_open=1.0)


def explore(fault, dual=True, limit=2_000_000):
    """All states reachable from power-up; returns (states seen, first violation path or None)."""
    start = scan(State(), fault, dual)
    seen = {start: None}
    q = deque([start])
    while q:
        s = q.popleft()
        v = violation(s)
        if v:
            path, x = [], s
            while seen[x] is not None:
                x, ev = seen[x]
                path.append(ev)
            return len(seen), (list(reversed(path)), v)
        for name, s2 in events(s):
            s3 = scan(s2, fault, dual)
            if s3 not in seen:
                seen[s3] = (s, name)
                q.append(s3)
                if len(seen) > limit:
                    raise RuntimeError('state space larger than expected')
    return len(seen), None


def faults():
    for n, ch, v in itertools.product(SAFE_INPUTS, (0, 1), (False, True)):
        yield (n, ch, v)


def main():
    print('S4b laser-enable interlock, exhaustive check (every reachable state from power-up)')
    n, bad = explore(None)
    print(f'  fault-free, dual channel: {n} reachable states; {"VIOLATION " + str(bad) if bad else "invariant holds"}')
    worst, failures = 0, []
    for f in faults():
        n, bad = explore(f)
        worst = max(worst, n)
        if bad:
            failures.append((f, bad))
    print(f'  every single stuck channel ({len(list(faults()))} faults), dual channel: '
          f'{"invariant holds in all" if not failures else f"{len(failures)} VIOLATIONS"} (up to {worst} states each)')
    for f, bad in failures[:3]:
        print(f'     {f}: {bad}')
    # control: single-channel inputs must fail somewhere
    ctrl = [(f, bad) for f in faults() if f[1] == 0 for _, bad in [explore(f, dual=False)] if bad]
    print(f'  control, SINGLE channel: {len(ctrl)} of {len([f for f in faults() if f[1] == 0])} stuck faults let the '
          f'laser on unsafely{" (the check can fail)" if ctrl else ": THE CHECK CANNOT FAIL"}')
    if ctrl:
        f, (path, v) = ctrl[0]
        print(f'     e.g. {f[0]} stuck at {f[2]}: {" -> ".join(path)}  =>  laser on with {", ".join(v)}')
    det = [f for f in faults() if detected(f)]
    print(f'  detection (Category 3): {len(det)} of {len(list(faults()))} stuck channels latch the fault in some '
          f'reachable state (a stuck channel is caught once the truth moves off its stuck value)')
    print(f'  S4b station time, pallet in to pallet out (excl. transport): {sum(TIMING.values()):.1f} s '
          f'({", ".join(f"{k} {v:g}" for k, v in TIMING.items())}); the robot reaches its first hold while the '
          f'shutter closes. Against the 59 s takt (2.7, S).')
    ok = not bad and not failures and ctrl and len(det) == len(list(faults()))
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
