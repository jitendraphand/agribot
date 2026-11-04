"""Trot with zero stance slip in the world frame.

The body advances at constant speed. During stance a foot is planted, so its
body-frame X runs from +step/2 to −step/2 and the world position stays put.
Swing lifts the foot on a half-sine and places it a step ahead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from kinematics.model import leg_ik, leg_within_limits
from spec.parameters import (
    BODY_STAND_Z,
    COXA,
    FOOT_R,
    GAIT_DUTY,
    GAIT_PERIOD,
    GAIT_STEP,
    GAIT_SWING_H,
    HIP_X,
    HIP_Y,
    HIP_Z,
)

LEGS = ("FL", "FR", "RL", "RR")
# Trot pairs: diagonal legs share a phase.
PHASE = {"FL": 0.00, "RR": 0.00, "FR": 0.50, "RL": 0.50}
SIDE = {"FL": 1, "FR": -1, "RL": 1, "RR": -1}
SIGN_X = {"FL": 1.0, "FR": 1.0, "RL": -1.0, "RR": -1.0}


@dataclass
class GaitSample:
    t: float
    body_x: float
    body_z: float
    feet_world: dict
    q: dict
    reachable: bool
    limits_ok: bool
    max_ik_mm: float


def stance_y() -> float:
    return HIP_Y + COXA


def body_speed(step: float = GAIT_STEP, period: float = GAIT_PERIOD, duty: float = GAIT_DUTY) -> float:
    return step / (duty * period)


def foot_body_offset(phase: float, step: float, swing_h: float, duty: float, nominal_z: float):
    """Foot position relative to the body centre, before the hip offset in Y.

    X is relative to the hip's X (the ±HIP_X term is added by the caller).
    """
    local = phase % 1.0
    if local < duty:
        u = local / duty
        x = step / 2.0 - u * step
        z = nominal_z
        swinging = False
    else:
        u = (local - duty) / (1.0 - duty)
        x = -step / 2.0 + u * step
        z = nominal_z + swing_h * math.sin(math.pi * u)
        swinging = True
    return x, z, swinging


def sample_gait(t: float, step: float = GAIT_STEP, swing_h: float = GAIT_SWING_H,
                 period: float = GAIT_PERIOD, duty: float = GAIT_DUTY,
                 body_z: float = BODY_STAND_Z, yaw_rate: float = 0.0) -> GaitSample:
    """Yaw rate is radians per second about +Z, applied as a slow heading.

    For the straight trot, yaw_rate is 0. A non-zero rate rotates the nominal
    foothold around the body so the robot walks an arc; stance feet are still
    recomputed in the body frame (small slip on a tight turn — quantified).
    """
    v = body_speed(step, period, duty)
    body_x = v * t
    heading = yaw_rate * t
    c, s = math.cos(heading), math.sin(heading)
    phase0 = (t / period) % 1.0
    nominal_z = FOOT_R  # foot-centre height on the ground
    feet = {}
    q = {}
    reachable = True
    limits_ok = True
    max_err = 0.0
    for name in LEGS:
        local = (phase0 + PHASE[name]) % 1.0
        dx, fz, _swing = foot_body_offset(local, step, swing_h, duty, nominal_z)
        # Nominal foot in the body frame, then rotate by heading and add body origin.
        bx = SIGN_X[name] * HIP_X + dx
        by = SIDE[name] * stance_y()
        rx_ = c * bx - s * by
        ry_ = s * bx + c * by
        foot_w = np.array([body_x + rx_, ry_, fz])
        hip_w = np.array([
            body_x + c * (SIGN_X[name] * HIP_X) - s * (SIDE[name] * HIP_Y),
            s * (SIGN_X[name] * HIP_X) + c * (SIDE[name] * HIP_Y),
            body_z + HIP_Z,
        ])
        # Express the foot in the hip frame, rotated into the body heading.
        d = foot_w - hip_w
        R_inv = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        # d is already in world; hip axes are rotated by heading (Rz).
        # World delta → hip frame: Rz(-heading) @ d
        local_foot = R_inv @ d
        sol = leg_ik(local_foot, SIDE[name])
        feet[name] = foot_w
        q[name] = sol.q
        max_err = max(max_err, sol.error_mm)
        reachable = reachable and sol.reachable
        limits_ok = limits_ok and leg_within_limits(sol.q)
    return GaitSample(t, body_x, body_z, feet, q, reachable, limits_ok, max_err)


def stance_slip(samples: list[GaitSample], duty: float = GAIT_DUTY, period: float = GAIT_PERIOD) -> float:
    """Max planar speed of a foot while it is in stance, mm/s. Straight gait."""
    if len(samples) < 2:
        return 0.0
    worst = 0.0
    for name in LEGS:
        for a, b in zip(samples, samples[1:]):
            dt = b.t - a.t
            if dt <= 0:
                continue
            phase_a = ((a.t / period) + PHASE[name]) % 1.0
            if phase_a >= duty:
                continue
            # Also require the next sample still in stance to avoid the lift edge.
            phase_b = ((b.t / period) + PHASE[name]) % 1.0
            if phase_b >= duty or phase_b < phase_a:
                continue
            d = np.linalg.norm((b.feet_world[name] - a.feet_world[name])[:2])
            worst = max(worst, float(d / dt))
    return worst
