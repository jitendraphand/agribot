"""Frames, forward kinematics, and analytical inverse kinematics.

Conventions
-----------
Body / hip frame: +X forward, +Y left, +Z up.

Leg (3-DOF), angles in radians
    q_abad  rotation about +X. Positive rolls +Y toward +Z.
    q_hip   pitch about the post-abad +Y axis. 0 = femur straight down.
            Positive q_hip swings the foot backward (−X). See FK.
    q_knee  additional pitch about the same axis. 0 = tibia colinear with
            the femur (straight). Negative q_knee bends the knee so the
            tibia swings back under a forward hip — the crouch we use.

    Coxa runs along ±Y by `side * COXA` (`side` = +1 left, −1 right) and is
    not pitched. Femur and tibia lie in the post-abad XZ plane.

Arm (4-DOF), radians, base frame at the yaw axis
    q_yaw       about +Z. 0 aims the arm forward (+X).
    q_shoulder  absolute pitch of the humerus. 0 = forward horizontal,
                positive pitches up toward +Z.
    q_elbow     pitch of the forearm relative to the humerus.
    q_wrist     pitch of the tool relative to the forearm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from spec.parameters import (
    ABAD_LIM,
    COXA,
    ELBOW_LIM,
    FEMUR,
    FOREARM,
    HIP_LIM,
    HUMERUS,
    KNEE_LIM,
    SHOULDER_LIM,
    TIBIA,
    TOOL,
    WRIST_LIM,
    YAW_LIM,
    YAW_TO_SHOULDER,
)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def mat4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = np.asarray(t, dtype=float)
    return M


@dataclass
class LegSolution:
    q: tuple[float, float, float]
    reachable: bool
    error_mm: float


@dataclass
class ArmSolution:
    q: tuple[float, float, float, float]
    reachable: bool
    error_mm: float


def leg_points(q_abad: float, q_hip: float, q_knee: float, side: int,
               l1: float = COXA, l2: float = FEMUR, l3: float = TIBIA):
    """Joint positions in the hip frame: abad origin, hip pitch, knee, foot."""
    R = rx(q_abad)
    hip = R @ np.array([0.0, side * l1, 0.0])
    knee = hip + R @ (ry(q_hip) @ np.array([0.0, 0.0, -l2]))
    foot = knee + R @ (ry(q_hip + q_knee) @ np.array([0.0, 0.0, -l3]))
    return {
        "abad": np.zeros(3),
        "hip": hip,
        "knee": knee,
        "foot": foot,
        "R_abad": R,
        "R_hip": R @ ry(q_hip),
        "R_knee": R @ ry(q_hip + q_knee),
    }


def leg_fk(q_abad: float, q_hip: float, q_knee: float, side: int,
           l1: float = COXA, l2: float = FEMUR, l3: float = TIBIA) -> np.ndarray:
    return leg_points(q_abad, q_hip, q_knee, side, l1, l2, l3)["foot"]


def leg_ik(target, side: int, l1: float = COXA, l2: float = FEMUR, l3: float = TIBIA,
           knee_sign: float = -1.0) -> LegSolution:
    """Closed-form IK. `knee_sign` −1 is the crouch used everywhere on Agribot
    (tibia folded under the femur). Position residual is exact on reachable
    targets; unreachable targets return the nearest pose on the workspace
    boundary with reachable=False."""
    px, py, pz = (float(target[0]), float(target[1]), float(target[2]))
    y = side * l1
    h2 = py * py + pz * pz - y * y
    reachable = h2 >= -1e-8
    z_local = -math.sqrt(max(0.0, h2))
    q1 = math.atan2(pz, py) - math.atan2(z_local, y)
    q1 = math.atan2(math.sin(q1), math.cos(q1))

    d = math.hypot(px, z_local)
    max_r = l2 + l3 - 1e-6
    min_r = abs(l2 - l3) + 1e-6
    if d > max_r + 1e-4 or d < min_r - 1e-4:
        reachable = False
    d_use = _clamp(d, min_r, max_r)
    # Project the planar target onto the reachable annulus for the pose.
    scale = d_use / d if d > 1e-9 else 1.0
    px_u, z_u = px * scale, z_local * scale

    c3 = (d_use * d_use - l2 * l2 - l3 * l3) / (2.0 * l2 * l3)
    c3 = _clamp(c3, -1.0, 1.0)
    q3 = knee_sign * math.acos(c3)

    # Femur/tibia FK in the post-abad XZ plane:
    #   px = -l2 sin q2 - l3 sin(q2+q3)
    #   z  = -l2 cos q2 - l3 cos(q2+q3)
    # which is linear in (sin q2, cos q2).
    k = l2 + l3 * math.cos(q3)
    m = l3 * math.sin(q3)
    det = k * k + m * m
    # Inverse of [[-k, -m], [m, -k]] is (1/det) [[-k, m], [-m, -k]].
    s2 = (-k * px_u + m * z_u) / det
    c2 = (-m * px_u - k * z_u) / det
    q2 = math.atan2(s2, c2)

    foot = leg_fk(q1, q2, q3, side, l1, l2, l3)
    err = float(np.linalg.norm(foot - np.array([px, py, pz])))
    if err > 0.25:
        reachable = False
    return LegSolution((q1, q2, q3), bool(reachable), err)


def _within(deg_q: float, lim: tuple[float, float]) -> bool:
    return lim[0] - 1e-6 <= math.degrees(deg_q) <= lim[1] + 1e-6


def leg_within_limits(q) -> bool:
    return (
        _within(q[0], ABAD_LIM)
        and _within(q[1], HIP_LIM)
        and _within(q[2], KNEE_LIM)
    )


# ----- arm -------------------------------------------------------------------

def arm_points(q_yaw: float, q_sh: float, q_el: float, q_wr: float,
               l0: float = YAW_TO_SHOULDER, l1: float = HUMERUS,
               l2: float = FOREARM, l3: float = TOOL):
    """Joint positions in the arm-base frame (yaw axis at the origin)."""
    a1 = q_sh
    a2 = q_sh + q_el
    a3 = q_sh + q_el + q_wr

    def along(length: float, ang: float) -> np.ndarray:
        return np.array([length * math.cos(ang), 0.0, length * math.sin(ang)])

    shoulder = np.array([0.0, 0.0, l0])
    elbow = shoulder + along(l1, a1)
    wrist = elbow + along(l2, a2)
    tool = wrist + along(l3, a3)
    R = rz(q_yaw)
    pts = {
        "yaw": np.zeros(3),
        "shoulder": R @ shoulder,
        "elbow": R @ elbow,
        "wrist": R @ wrist,
        "tool": R @ tool,
    }
    # Link frames: X along the outgoing link, Y = yawed lateral, Z = X × Y? 
    # Store rotations that map local +X to the link direction and local +Z up
    # in the arm plane (local +Y stays the pitched axis, i.e. yawed world Y).
    y_axis = R @ np.array([0.0, 1.0, 0.0])

    def frame(origin: np.ndarray, ang: float) -> np.ndarray:
        direction = R @ np.array([math.cos(ang), 0.0, math.sin(ang)])
        # local X = direction, local Y = pitch axis, local Z = X × Y
        x = direction
        y = y_axis
        z = np.cross(x, y)
        n = np.linalg.norm(z)
        if n < 1e-9:
            z = R @ np.array([0.0, 0.0, 1.0])
        else:
            z = z / n
        # re-orthogonalize y
        y = np.cross(z, x)
        return mat4(np.column_stack([x, y, z]), origin)

    pts["F_yaw"] = mat4(R, np.zeros(3))
    pts["F_shoulder"] = frame(pts["shoulder"], a1)  # humerus frame at shoulder, X along humerus
    # Rebuild shoulder frame at the shoulder origin but rotation of humerus
    pts["F_humerus"] = frame(pts["shoulder"], a1)
    pts["F_forearm"] = frame(pts["elbow"], a2)
    pts["F_tool"] = frame(pts["wrist"], a3)
    # Shoulder bracket itself only yaws and rises; its local +Z is up.
    pts["F_bracket"] = mat4(R, np.zeros(3))
    return pts


def arm_fk(q_yaw, q_sh, q_el, q_wr, l0=YAW_TO_SHOULDER, l1=HUMERUS, l2=FOREARM, l3=TOOL):
    return arm_points(q_yaw, q_sh, q_el, q_wr, l0, l1, l2, l3)["tool"]


def arm_ik(target, tool_pitch: float = 0.0, l0=YAW_TO_SHOULDER, l1=HUMERUS,
           l2=FOREARM, l3=TOOL, elbow_up: bool = True) -> ArmSolution:
    """Position IK. `tool_pitch` is the absolute pitch of the gripper in the
    arm's vertical plane (0 = horizontal forward, −π/2 = pointing down)."""
    x, y, z = (float(target[0]), float(target[1]), float(target[2]))
    yaw = math.atan2(y, x)
    radial = math.hypot(x, y)
    # Wrist centre so the tool leaves it at tool_pitch.
    r_w = radial - l3 * math.cos(tool_pitch)
    z_w = z - l3 * math.sin(tool_pitch)
    dx = r_w
    dz = z_w - l0
    d = math.hypot(dx, dz)
    max_r = l1 + l2
    min_r = abs(l1 - l2)
    reachable = min_r - 1e-6 <= d <= max_r + 1e-6
    d_use = _clamp(d, min_r + 1e-9, max_r - 1e-9)
    cos_el = (d_use * d_use - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_el = _clamp(cos_el, -1.0, 1.0)
    # Elbow bend. elbow_up keeps the elbow above the shoulder–wrist line,
    # which clears the chassis on this canopy arm.
    q_el = -math.acos(cos_el) if elbow_up else math.acos(cos_el)
    beta = math.atan2(dz, dx)  # angle of the wrist ray from +radial toward +Z
    gamma = math.atan2(l2 * math.sin(-q_el), l1 + l2 * math.cos(q_el))
    # For elbow_up, q_el < 0, gamma > 0, shoulder is the ray plus gamma.
    q_sh = beta + gamma
    q_wr = tool_pitch - q_sh - q_el
    q_wr = math.atan2(math.sin(q_wr), math.cos(q_wr))
    q_sh = math.atan2(math.sin(q_sh), math.cos(q_sh))
    tool = arm_fk(yaw, q_sh, q_el, q_wr, l0, l1, l2, l3)
    err = float(np.linalg.norm(tool - np.array([x, y, z])))
    ok = reachable and err <= 0.75
    return ArmSolution((yaw, q_sh, q_el, q_wr), ok, err)


def arm_within_limits(q) -> bool:
    return (
        _within(q[0], YAW_LIM)
        and _within(q[1], SHOULDER_LIM)
        and _within(q[2], ELBOW_LIM)
        and _within(q[3], WRIST_LIM)
    )


def deg(q) -> list[float]:
    return [math.degrees(v) for v in q]
