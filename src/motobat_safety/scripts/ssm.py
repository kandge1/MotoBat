"""Speed and separation monitoring for the HC10DT inspection cells: the formulas. (task 3.5)

Pure numpy; no ROS. Every number carries its source letter (S sourced, D derived, A assumed).

1. The protective separation distance. ISO/TS 15066 (as Marvel & Norcross state it: "Implementing
   speed and separation monitoring in collaborative robot workcells", Robotics and Computer-Integrated
   Manufacturing 44 (2016) 144-155, doi 10.1016/j.rcim.2016.08.001, Eq. 1 and its linear form Eq. 3):

       S = v_H (T_R + T_S) + v_R T_R + B + C + Z_R + Z_S

   v_H  the person's speed toward the robot      T_R  sensing + reaction time
   v_R  the robot's speed toward the person      T_S  the robot's stopping time (from its speed)
   B    the robot's travel while braking         C    intrusion: how far a body part reaches past
   Z_R, Z_S  robot and sensor position uncertainty     the detection before it is detected

   Static fields around the robot's RESTRICTED SPACE (the volume its safety unit, Yaskawa's FSU, will
   not let it leave) remove the robot-travel terms: wherever the robot is, it is inside that space, so
   the distance is measured from the space's boundary and v_R T_R and B vanish (they live inside it).
   What is left is ISO 13855's form for a horizontal protective field, S = K T + C, with the
   ISO/TS 15066 refinement that the stopping time depends on how fast the robot is going:

       D(v) = K (T_R + T_S(v)) + C + Z_S + Z_R

2. ISO 13855 terms for a horizontal field (S: KEYENCE "Laser scanner installation and safety
   distance" page, citing ISO 13855 / IEC 61496-3; read 27 Nov 2026):
       K = 1600 mm/s (whole-body approach; Marvel & Norcross note ISO 13855's 2000 mm/s worst case,
           with 1600 allowed when S > 500 mm, and argue 2000 is the more prudent choice: both reported)
       C = 1200 - 0.4 H  (and not below 850 mm), H = scan-plane height
       1000 >= H >= 15 (d - 50), d = detection capability. Above 300 mm, someone could crawl under.

3. The scanner (S: SICK microScan3 Core I/O, MICS3-AAAZ40AZ1P01 data sheet, generated 3 Nov 2026):
   protective field range 4 m, warning field 40 m, 275 deg, resolution 30/40/50/70/150/200 mm,
   response time 70 ms, protective field supplement 65 mm (= Z_S), Type 3, SIL 2, PL d, Cat. 3,
   ONE OSSD pair and 3 universal (non-safety) I/Os, no measured-data output.
   With one OSSD pair only one field is safety-rated: a slowdown triggered by a warning field is not,
   so the stop field must assume the robot never slowed (variant 'core'). A safety-network variant
   evaluates up to 4 fields safely and can make the slowdown part of the safety case ('network').

4. The robot (A, not published by Yaskawa): stopping time T_S = (speed) / (deceleration); with the
   joint deceleration assumed equal to the acceleration limit a = 2 v_max (joint_limits.yaml),
   0.5 s from full speed, proportionally less from a reduced speed. Reaction time of the robot's
   safety unit to the scanner's OSSD: 50 ms (A). Reduced speed: 250 mm/s at the tool (A: the
   ISO 10218-1 reduced-speed figure, not verified here).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

K_APPROACH = 1600.0          # mm/s  (S, ISO 13855 via KEYENCE; 2000 = the worst case, also reported)
SCANNER_RESPONSE = 0.070     # s     (S, microScan3 Core I/O)
SCANNER_SUPPLEMENT = 65.0    # mm    (S, protective field supplement = Z_S)
SCANNER_RANGE = 4000.0       # mm    (S)
SCANNER_ANGLE = 275.0        # deg   (S)
ROBOT_REACTION = 0.050       # s     (A, FSU input to stop start)
T_STOP_FULL = 0.5            # s     (A, a = 2 v_max: full speed to rest)
Z_ROBOT = 100.0              # mm    (A, the FSU range limit sits 0.10 m outside the swept envelope)
V_REDUCED_TCP = 250.0        # mm/s  (A)


def intrusion_C(H: float) -> float:
    """ISO 13855 intrusion supplement for a horizontal field at scan height H (mm)."""
    return max(1200.0 - 0.4 * H, 850.0)


def min_scan_height(d: float) -> float:
    """Lowest legal scan-plane height for detection capability d (mm): H >= 15 (d - 50), >= 0."""
    return max(15.0 * (d - 50.0), 0.0)


@dataclass
class Design:
    K: float = K_APPROACH
    H: float = 300.0             # mm, scan plane (70 mm resolution needs >= 300)
    d: float = 70.0              # mm, detection capability (leg detection)
    t_r: float = SCANNER_RESPONSE + ROBOT_REACTION
    t_s_full: float = T_STOP_FULL
    z_s: float = SCANNER_SUPPLEMENT
    z_r: float = Z_ROBOT

    @property
    def C(self) -> float:
        return intrusion_C(self.H)

    def stop_time(self, speed_fraction: float) -> float:
        """Stopping time from a speed that is this fraction of full speed (linear braking)."""
        return self.t_s_full * speed_fraction

    def distance(self, speed_fraction: float) -> float:
        """D(v): how far outside the restricted space a field must start (mm)."""
        return self.K * (self.t_r + self.stop_time(speed_fraction)) + self.C + self.z_s + self.z_r

    def slowdown_distance(self, reduced_fraction: float) -> float:
        """Outer field for a SAFETY-RATED slowdown: the person must not reach the inner (stop) field
        before the robot is at the reduced speed. Slowing from full to reduced takes
        t_s_full (1 - reduced_fraction); the person walks K (t_r + that) meanwhile."""
        return self.distance(reduced_fraction) + self.K * (self.t_r + self.t_s_full * (1 - reduced_fraction))

    def table(self, reduced_fraction: float) -> dict:
        return dict(
            core_stop=self.distance(1.0),                                   # one safe field, robot at any speed
            network_stop=self.distance(reduced_fraction),                    # safe slowdown makes this enough
            network_slowdown=self.slowdown_distance(reduced_fraction),
            terms=dict(K=self.K, T_R=self.t_r, T_S_full=self.t_s_full, C=self.C, Z_S=self.z_s, Z_R=self.z_r,
                       H=self.H, d=self.d))


def speed_command(zone: str, design: str) -> str:
    """What the robot is told for the zone the person is in.
    zone: 'clear' | 'slowdown' | 'stop'.  design: 'core' | 'network'."""
    if zone == 'stop':
        return 'stop'
    if zone == 'slowdown':
        return 'reduced'
    return 'full'
