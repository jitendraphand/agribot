"""Render the trot and the pick as mp4s from the same IK the controller uses.

    python -m sim.render_sim
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from kinematics.gait import sample_gait  # noqa: E402
from kinematics.gripper import finger_angle  # noqa: E402
from kinematics.model import arm_fk, arm_ik, arm_points, leg_points  # noqa: E402
from spec.parameters import (  # noqa: E402
    ARM_X,
    BODY_H,
    BODY_L,
    BODY_STAND_Z,
    BODY_W,
    DECK_T,
    FOOT_R,
    GRIP_FINGER_LEN,
    GRIP_PIVOT_X,
    GRIP_PIVOT_Y,
)

OUT = ROOT / "sim"
STOW = (0.0, math.radians(115), math.radians(-125), math.radians(-80))
# Arm-base frame target: a stem just past the nose, gripper pointing down.
PICK = np.array([128.0, 30.0, -76.0])
CARRY = (math.radians(12), math.radians(100), math.radians(-125), math.radians(-40))


def _box(ax, c, size, color, alpha=1.0):
    cx, cy, cz = c
    dx, dy, dz = (np.array(size) / 2.0)
    pts = np.array([
        [cx - dx, cy - dy, cz - dz], [cx + dx, cy - dy, cz - dz],
        [cx + dx, cy + dy, cz - dz], [cx - dx, cy + dy, cz - dz],
        [cx - dx, cy - dy, cz + dz], [cx + dx, cy - dy, cz + dz],
        [cx + dx, cy + dy, cz + dz], [cx - dx, cy + dy, cz + dz],
    ])
    faces = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [2, 3, 7, 6], [1, 2, 6, 5], [0, 3, 7, 4]]
    poly = Poly3DCollection([[pts[i] for i in f] for f in faces], alpha=alpha)
    poly.set_facecolor(color)
    poly.set_edgecolor("#1c1914")
    poly.set_linewidth(0.3)
    ax.add_collection3d(poly)


def _seg(ax, a, b, color, lw=2.4):
    ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color=color, lw=lw, solid_capstyle="round")


def draw_robot(ax, body, leg_q, arm_q, grip_deg):
    bx, by, bz = body
    _box(ax, (bx, by, bz), (BODY_L, BODY_W, BODY_H), "#3d4a32")
    # deck
    _box(ax, (bx, by, bz + BODY_H / 2 + DECK_T / 2), (BODY_L, BODY_W, DECK_T), "#4d5c3e")
    from kinematics.gait import SIDE, SIGN_X
    from spec.parameters import HIP_X, HIP_Y, HIP_Z

    for name, q in leg_q.items():
        side = SIDE[name]
        hip = np.array([
            bx + SIGN_X[name] * HIP_X,
            by + side * HIP_Y,
            bz + HIP_Z,
        ])
        pts = leg_points(*q, side)
        hip_p = hip + pts["hip"]
        knee = hip + pts["knee"]
        foot = hip + pts["foot"]
        _seg(ax, hip, hip_p, "#c4b49a", 3.0)
        _seg(ax, hip_p, knee, "#c4b49a", 3.2)
        _seg(ax, knee, foot, "#c4b49a", 2.6)
        ax.scatter([foot[0]], [foot[1]], [foot[2]], color="#5c4033", s=18, depthshade=False)

    base = np.array([bx + ARM_X, by, bz + BODY_H / 2 + DECK_T])
    pts = arm_points(*arm_q)
    shoulder = base + pts["shoulder"]
    elbow = base + pts["elbow"]
    wrist = base + pts["wrist"]
    tool = base + pts["tool"]
    _seg(ax, base, shoulder, "#e0a15a", 2.6)
    _seg(ax, shoulder, elbow, "#e0a15a", 3.0)
    _seg(ax, elbow, wrist, "#e0a15a", 2.6)
    _seg(ax, wrist, tool, "#e0a15a", 2.2)
    # Fingertips from the palm frame, opening with the linkage angle.
    ang = finger_angle(grip_deg, 1)
    # Tool frame axes from the wrist-to-tool direction and the yaw.
    direction = tool - wrist
    dist = np.linalg.norm(direction)
    x_axis = direction / dist if dist > 1e-6 else np.array([1.0, 0.0, 0.0])
    yaw = arm_q[0]
    y_axis = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    pivot = wrist + x_axis * (GRIP_PIVOT_X / max(dist, 1.0) * dist * 0 + GRIP_PIVOT_X * (dist / max(dist, 1)))
    # GRIP_PIVOT_X is along the tool axis; tool length is the full finger reach,
    # so the pivot sits GRIP_PIVOT_X along the unit tool direction.
    pivot_mid = wrist + x_axis * GRIP_PIVOT_X
    for s in (+1.0, -1.0):
        root = pivot_mid + y_axis * (s * GRIP_PIVOT_Y)
        tip_dir = x_axis * math.cos(ang) + y_axis * (s * math.sin(ang))
        tip = root + tip_dir * GRIP_FINGER_LEN
        _seg(ax, root, tip, "#8a9a62", 2.0)


def _style(ax, title, focus):
    ax.set_facecolor("#1c1914")
    ax.set_title(title, color="#f4f0e6", loc="left", fontsize=13, pad=8)
    span = 260
    ax.set_xlim(focus[0] - span, focus[0] + span * 0.7)
    ax.set_ylim(focus[1] - span * 0.7, focus[1] + span * 0.7)
    ax.set_zlim(0, 280)
    ax.set_box_aspect((1.4, 1.1, 0.7))
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.view_init(elev=22, azim=-58)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor("#1c1914")
        axis.pane.set_edgecolor("#1c1914")
    # ground grid
    xs = np.arange(focus[0] - 400, focus[0] + 500, 40)
    ys = np.arange(-240, 280, 40)
    for x in xs:
        ax.plot([x, x], [ys[0], ys[-1]], [0, 0], color="#3a342b", lw=0.4)
    for y in ys:
        ax.plot([xs[0], xs[-1]], [y, y], [0, 0], color="#3a342b", lw=0.4)


def render(path: Path, frames: int, dt: float, title: str, pose_fn):
    fig = plt.figure(figsize=(12.80, 7.20), dpi=100)
    fig.patch.set_facecolor("#1c1914")
    ax = fig.add_subplot(111, projection="3d")
    writer = FFMpegWriter(fps=int(round(1 / dt)), bitrate=2400)
    with writer.saving(fig, str(path), dpi=100):
        for i in range(frames):
            ax.cla()
            body, leg_q, arm_q, grip, focus = pose_fn(i * dt)
            draw_robot(ax, body, leg_q, arm_q, grip)
            _style(ax, title, focus)
            writer.grab_frame()
            if i % 10 == 0:
                print(f"  {path.name} {i}/{frames}")
    plt.close(fig)


def trot_pose(t):
    # A short straight scout, arm stowed, gripper open.
    s = sample_gait(t)
    body = np.array([s.body_x, 0.0, s.body_z])
    return body, s.q, STOW, -20.0, body


def pick_pose(t):
    # Stand, fold the arm out in joint space (limits are a convex box, so the
    # lerp stays legal), close the gripper, then fold back to a carry pose.
    s = sample_gait(0.0)
    body = np.array([0.0, 0.0, s.body_z])
    sol = arm_ik(PICK, tool_pitch=-math.pi / 2)
    if not sol.reachable or not _limits(sol.q):
        raise RuntimeError(f"pick pose is not a legal target, err {sol.error_mm}")
    if t < 1.3:
        arm = _lerp_arm(STOW, sol.q, _smooth(t / 1.3))
        grip = -18.0
    elif t < 2.3:
        arm = sol.q
        grip = -18.0 + 18.0 * _smooth((t - 1.3) / 0.7)
    else:
        arm = _lerp_arm(sol.q, CARRY, _smooth((t - 2.3) / 1.6))
        grip = 0.0
    return body, s.q, arm, grip, body


def _limits(q) -> bool:
    from kinematics.model import arm_within_limits
    return arm_within_limits(q)


def _smooth(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


def _lerp_arm(a, b, u):
    return tuple((1 - u) * x + u * y for x, y in zip(a, b))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    print("render trot")
    render(OUT / "agribot_trot.mp4", frames=96, dt=1 / 12, title="Agribot  ·  trot, zero stance slip", pose_fn=trot_pose)
    print("render pick")
    render(OUT / "agribot_pick.mp4", frames=60, dt=1 / 12, title="Agribot  ·  canopy arm pick", pose_fn=pick_pose)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
