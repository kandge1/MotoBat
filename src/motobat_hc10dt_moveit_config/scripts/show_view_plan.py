#!/usr/bin/python3
"""
Fly the HC10DT through one stage of the 1.5 view plan in RViz (task 2.5).

Plays what `view_reach.py --plan` computed for one base: the views in their
planned order, every move between them planned by OMPL and collision-checked
against the keep-out box (module + pad), the pallet, the support, the floor and
the pedestal, timed at the robot's joint limits. Nothing here is interpolated
or invented: if a move failed to plan, it says so on screen.

Publishes:
  /joint_states          the arm (robot_state_publisher turns it into TF)
  TF s3_base_link->module where the module sits
  /view_plan/markers     scene boxes (keep-out in red), the Zivid's field of view and
                         the LJ-X laser fan (bright while capturing), every view of the
                         stage as a numbered tag (red = not reachable from this base),
                         the strip the current sweep measures
  /view_plan/caption     the caption, as a fixed-size 2D overlay (top left of RViz)

Normally started by view_plan.py (one command for all of it, see WORKFLOW.md).
On its own:
    ros2 run motobat_hc10dt_moveit_config show_view_plan.py data/reach/base_A/reach.json \\
        --stage post_weld [--view 3] [--sensor snapshot] [--rate 0.3] [--step] [--once]
System Python 3.12 (rclpy), not the project venv.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rviz_2d_overlay_msgs.msg import OverlayText
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parent))
import view_reach as vr  # noqa: E402

HZ = 30.0
SNAP_SHOW = 0.8      # s on screen for a 0.1 s snapshot, so it can be seen (not counted)
ARRIVE_SHOW = 0.4    # s pause on screen at each view (not counted)
# Zivid 2+ M60 camera FOV and working range (defectgen sensors.yaml, S)
ZIVID_HFOV, ZIVID_VFOV, ZIVID_RANGE = 52.84, 45.14, (0.30, 1.10)
# LJ-X8200: emitter to reference 275 mm (CAD + drawing), +-34 mm range, 64 / 80 mm line (S)
LJX_NEAR, LJX_FAR, LJX_W_NEAR, LJX_W_FAR = 0.241, 0.309, 0.064, 0.080
STAGE_NAME = {'pre_busbar': 'PRE-BUSBAR', 'pre_weld': 'PRE-WELD', 'post_weld': 'POST-WELD'}
SCENE_COLOUR = {'keepout': (0.9, 0.15, 0.1, 0.18), 'pallet': (0.35, 0.45, 0.6, 0.9),
                'support': (0.35, 0.45, 0.6, 0.35), 'floor': (0.5, 0.5, 0.5, 0.4),
                'pedestal': (0.45, 0.45, 0.4, 0.9), 'mount_plate': (0.45, 0.45, 0.4, 0.9)}


def rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def pt(v):
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def describe(v):
    if v['sensor'] == 'snapshot':
        return 'Zivid snapshot, 0.1 s'
    return (f"LJ-X sweep, {v['length'] * 1000:.0f} mm at {v['speed_mm_s']:.0f} mm/s, "
            f"{v['capture_s']:.1f} s with run-up")


def strip_of(v):
    ref = np.array(v['T_plan']) @ vr.tf(p=(0, 0, vr.PROFILE_REF))
    return [(ref @ vr.tf(p=(0, s, 0)))[:3, 3] for s in (0.0, v['length'])]


def tag_point(v):
    """Where a view's number goes, in the module frame: at the camera for a snapshot,
    5 cm off the middle of the measured strip (toward the sensor) for a sweep."""
    T = np.array(v['T_plan'])
    if v['sensor'] == 'snapshot':
        return T[:3, 3]
    a, b = strip_of(v)
    return (a + b) / 2 - 0.05 * T[:3, 2]


class Player(Node):
    def __init__(self, result, floor, pad, rate, step):
        super().__init__('show_view_plan')
        self.res = result
        self.base = result['base']
        self.floor, self.pad, self.rate, self.step = floor, pad, rate, step
        self.T_mb = vr.base_in_module(self.base['bx'], self.base['d'], self.base['bz'],
                                      0.0, self.base.get('mount', 'floor'))
        self.js = self.create_publisher(JointState, '/joint_states', 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mk = self.create_publisher(MarkerArray, '/view_plan/markers', qos)
        self.cap = self.create_publisher(OverlayText, '/view_plan/caption', qos)
        self.tf = TransformBroadcaster(self)
        self.q = list(result.get('home', vr.READY))
        self.tags = []          # (number, view, reachable)
        self.current = None     # view id
        self.clock = 0.0        # robot seconds into the cycle
        self.cycle = None

    # -- publishing ------------------------------------------------------------------
    def publish(self, lines, active=None, strip=None):
        now = self.get_clock().now().to_msg()
        self.js.publish(JointState(header=Header(stamp=now),
                                   name=vr.JOINTS, position=[float(v) for v in self.q]))
        T_bm = np.linalg.inv(self.T_mb)
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = vr.FRAME
        t.child_frame_id = 'module'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            (float(v) for v in T_bm[:3, 3])
        q = Rotation.from_matrix(T_bm[:3, :3]).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, \
            t.transform.rotation.w = (float(v) for v in q)
        self.tf.sendTransform(t)
        self.mk.publish(self.markers(active, strip))
        clock = f't = {self.clock:.1f} s' + (f' of a {self.cycle:.1f} s cycle' if self.cycle else '')
        lines = [ln for ln in lines if ln] + [clock]
        self.cap.publish(OverlayText(
            action=OverlayText.ADD, width=760, height=19 * len(lines) + 14,
            horizontal_distance=12, vertical_distance=12,
            horizontal_alignment=OverlayText.LEFT, vertical_alignment=OverlayText.TOP,
            bg_color=rgba(0, 0, 0, 0.55), fg_color=rgba(1, 1, 1, 1), line_width=1,
            text_size=12.0, font='DejaVu Sans Mono', text='\n'.join(lines)))
        rclpy.spin_once(self, timeout_sec=0.0)

    def markers(self, active, strip):
        ms = []

        def box(ns, size, centre, colour):
            m = Marker(type=Marker.CUBE, action=Marker.ADD, ns=ns, id=0)
            m.header.frame_id = 'module'
            m.pose.position = pt(centre)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = (float(s) for s in size)
            m.color = rgba(*colour)
            ms.append(m)

        for name, size, centre in vr.scene_boxes(self.T_mb, self.floor, self.pad,
                                                 self.base.get('mount', 'floor')):
            box('scene/' + name, size, centre, SCENE_COLOUR[name])

        # Zivid field of view: pyramid from the optical centre to the far working distance
        on = active == 'snapshot'
        m = Marker(type=Marker.LINE_LIST, action=Marker.ADD, ns='sensor/zivid_fov', id=0)
        m.header.frame_id = 's3_zivid_optical'
        m.pose.orientation.w = 1.0
        m.scale.x = 0.003 if on else 0.0015
        m.color = rgba(0.1, 0.9, 0.3, 0.9 if on else 0.2)
        tx, ty = math.tan(math.radians(ZIVID_HFOV / 2)), math.tan(math.radians(ZIVID_VFOV / 2))
        for z in ZIVID_RANGE:
            c = [(sx * tx * z, sy * ty * z, z) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            for i in range(4):
                m.points += [pt(c[i]), pt(c[(i + 1) % 4])]
                if z == ZIVID_RANGE[1]:
                    m.points += [pt((0, 0, 0)), pt(c[i])]
        ms.append(m)

        # LJ-X laser fan (in its x-z plane) and the measuring band
        on = active == 'profilometer'
        m = Marker(type=Marker.TRIANGLE_LIST, action=Marker.ADD, ns='sensor/ljx_fan', id=0)
        m.header.frame_id = 's3_ljx_laser'
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = rgba(0.45, 0.2, 1.0, 0.55 if on else 0.12)
        m.points = [pt((0, 0, 0)), pt((-LJX_W_FAR / 2, 0, LJX_FAR)), pt((LJX_W_FAR / 2, 0, LJX_FAR))]
        ms.append(m)
        m = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='sensor/ljx_range', id=0)
        m.header.frame_id = 's3_ljx_laser'
        m.pose.orientation.w = 1.0
        m.scale.x = 0.002
        m.color = rgba(1.0, 0.3, 0.9, 0.9 if on else 0.25)
        m.points = [pt((-LJX_W_NEAR / 2, 0, LJX_NEAR)), pt((LJX_W_NEAR / 2, 0, LJX_NEAR)),
                    pt((LJX_W_FAR / 2, 0, LJX_FAR)), pt((-LJX_W_FAR / 2, 0, LJX_FAR)),
                    pt((-LJX_W_NEAR / 2, 0, LJX_NEAR))]
        ms.append(m)

        # every view of the stage: a numbered tag, and its strip or line of sight, faint;
        # the current one bright yellow, the unreachable ones red
        for k, (num, v, ok) in enumerate(self.tags):
            cur = v['id'] == self.current
            col = (1.0, 0.85, 0.1, 1.0) if cur else ((0.85, 0.85, 0.85, 0.8) if ok else
                                                      (1.0, 0.2, 0.2, 0.9))
            m = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='plan/numbers', id=k)
            m.header.frame_id = 'module'
            m.pose.position = pt(tag_point(v))
            m.pose.orientation.w = 1.0
            m.scale.z = 0.035 if cur else 0.025
            m.color = rgba(*col)
            m.text = str(num) if ok else f'{num}x'
            ms.append(m)
            m = Marker(type=Marker.LINE_LIST, action=Marker.ADD, ns='plan/where', id=k)
            m.header.frame_id = 'module'
            m.pose.orientation.w = 1.0
            m.scale.x = 0.004 if cur else 0.0015
            m.color = rgba(col[0], col[1], col[2], 0.9 if cur else 0.35)
            if v['sensor'] == 'snapshot':
                T = np.array(v['T_plan'])
                m.points = [pt(T[:3, 3]), pt(T[:3, 3] + 0.12 * T[:3, 2])]
            else:
                m.points = [pt(p) for p in strip_of(v)]
            ms.append(m)
        for k in range(len(self.tags), 40):        # clear tags left from a longer stage
            ms.append(Marker(action=Marker.DELETE, ns='plan/numbers', id=k))
            ms.append(Marker(action=Marker.DELETE, ns='plan/where', id=k))
        return MarkerArray(markers=ms)

    # -- motion ----------------------------------------------------------------------
    def wait(self, seconds, lines, count=0.0, **kw):
        """Show for `seconds` of screen time (scaled by --rate); the cycle clock advances
        by `count` robot seconds over it."""
        n = max(1, int(seconds / self.rate * HZ))
        c0 = self.clock
        for i in range(1, n + 1):
            self.clock = c0 + count * i / n
            self.publish(lines, **kw)
            time.sleep(1.0 / HZ)

    def play_leg(self, leg, lines):
        if leg is None:
            self.wait(1.0, lines[:2] + ['MOVE FAILED TO PLAN: jumping (not checked)'])
            return
        t, Q = np.array(leg['t']), np.array(leg['q'])
        dur = float(t[-1])
        n = max(1, int(dur / self.rate * HZ))
        c0 = self.clock
        for i in range(1, n + 1):
            s = dur * i / n
            self.q = [float(np.interp(s, t, Q[:, j])) for j in range(6)]
            self.clock = c0 + s
            self.publish(lines)
            time.sleep(1.0 / HZ)
        self.q = list(Q[-1])

    def capture(self, v, lines):
        if self.step:
            self.publish(lines + ['paused: press Enter in the terminal'], active=v['sensor'],
                         strip=strip_of(v) if v['sensor'] == 'profilometer' else None)
            input(f"  at {v['id']}: Enter for the next view ")
        if v['sensor'] == 'snapshot':
            self.wait(ARRIVE_SHOW, lines[:2] + ['arrived'])
            self.wait(SNAP_SHOW, lines[:2] + ['capturing (0.1 s, held on screen)'],
                      count=v['capture_s'], active='snapshot')
            return
        self.wait(ARRIVE_SHOW, lines[:2] + ['at run-in'], strip=strip_of(v))
        path = v['path']
        dur = v['capture_s']
        n = max(1, int(dur / self.rate * HZ))
        c0 = self.clock
        for i in range(1, n + 1):
            self.q = path[min(len(path) - 1, round(i / n * (len(path) - 1)))]
            self.clock = c0 + dur * i / n
            self.publish(lines[:2] + [f"sweeping at {v['speed_mm_s']:.0f} mm/s"],
                         active='profilometer', strip=strip_of(v))
            time.sleep(1.0 / HZ)

    # -- a stage ---------------------------------------------------------------------
    def fly(self, stage, only=None, sensor=None):
        """Fly a stage in planned order. `only` = one view id; `sensor` = one sensor's views.
        A subset goes home between views (those moves are planned too)."""
        seq = self.res['sequences'][stage]
        views = {v['id']: v for v in self.res['views']}
        order = seq['order']
        self.tags = [(i + 1, views[i_d], True) for i, i_d in enumerate(order)]
        self.tags += [(len(order) + i + 1, views[i_d], False) for i, i_d in enumerate(seq['missing'])]
        pick = [i_d for i_d in order if (only is None or i_d == only)
                and (sensor is None or views[i_d]['sensor'] == sensor)]
        subset = len(pick) < len(order)
        self.cycle = None if subset else seq['cycle_s']
        self.clock = 0.0
        head = f'{STAGE_NAME[stage]} | base {self.base.get("name", "")}'.rstrip()
        missing = (f"not reachable from this base: {', '.join(seq['missing'])}"
                   if seq['missing'] else 'every view of this stage is reachable')
        self.current = None
        self.wait(2.0, [head + f' | {len(order)} views, cycle {seq["cycle_s"]:.1f} s '
                        f'(moves {seq["move_s"]:.1f} + captures {seq["capture_s"]:.1f})',
                        missing, 'home'])
        print(f"\n{head}: {len(order)} views, cycle {seq['cycle_s']:.1f} s "
              f"(moves {seq['move_s']:.1f} s, captures {seq['capture_s']:.1f} s)", flush=True)
        if seq['missing']:
            print(f'  not reachable: {seq["missing"]}', flush=True)
        for i_d in pick:
            k = order.index(i_d)
            v = views[i_d]
            self.current = i_d
            top = f'{STAGE_NAME[stage]} | view {k + 1} of {len(order)} | {i_d}'
            lines = [top, describe(v)]
            print(f'  {k + 1:2d}. {i_d:28s} {describe(v)}', flush=True)
            leg = self.res['solo'][i_d]['in'] if subset else seq['legs'][k]
            self.play_leg(leg, lines + ['moving (OMPL-planned, collision-checked)'])
            self.capture(v, lines)
            if subset:
                self.play_leg(self.res['solo'][i_d]['out'], lines + ['back home'])
        if not subset:
            self.play_leg(seq['legs'][-1], [f'{STAGE_NAME[stage]}  done', '', 'moving home'])
        self.current = None
        self.wait(2.0, [f'{STAGE_NAME[stage]}  done' + (f' in {self.clock:.1f} s' if not subset
                                                          else ''), missing, 'home'])


STAGE_ALIAS = {'busbar': 'pre_busbar', 'pre_busbar': 'pre_busbar', 'preweld': 'pre_weld',
               'pre_weld': 'pre_weld', 'weld': 'post_weld', 'postweld': 'post_weld',
               'post_weld': 'post_weld'}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('reach', type=Path, help='reach.json from view_reach.py --plan')
    ap.add_argument('--stage', default='post_weld', choices=sorted(STAGE_ALIAS))
    ap.add_argument('--view', default=None, help='one view: its number in the stage order, or its id')
    ap.add_argument('--sensor', choices=['snapshot', 'profilometer'], default=None)
    ap.add_argument('--rate', type=float, default=1.0, help='0.3 = 0.3x real speed')
    ap.add_argument('--step', action='store_true', help='stop at every view until Enter')
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--name', default='', help='base name for the caption')
    a = ap.parse_args()

    data = json.load(open(a.reach))
    res = data['results'][0]
    if 'sequences' not in res:
        sys.exit(f'{a.reach} has no planned sequences: rerun view_reach.py with --plan')
    res['base']['name'] = a.name
    stage = STAGE_ALIAS[a.stage]
    only = a.view
    if only is not None and only.isdigit():
        order = res['sequences'][stage]['order']
        if not 1 <= int(only) <= len(order):
            sys.exit(f'--view {only}: {stage} has views 1..{len(order)}')
        only = order[int(only) - 1]
    if only is not None and only not in res['sequences'][stage]['order']:
        sys.exit(f'--view {only} is not a reachable view of {stage}: '
                 f"{res['sequences'][stage]['order']}")

    rclpy.init()
    p = Player(res, data['floor'], data.get('pad', vr.PAD), a.rate, a.step)
    try:
        while rclpy.ok():
            p.fly(stage, only, a.sensor)
            if a.once:
                break
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        try:
            p.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
