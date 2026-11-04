"""One-servo linkage gripper. Circle-circle solution, no iteration."""

from __future__ import annotations

import math

import numpy as np

from spec.parameters import (
    GRIP_CRANK,
    GRIP_CRANK_OFFSET_DEG,
    GRIP_FINGER_LEN,
    GRIP_FINGER_T,
    GRIP_HORN_R,
    GRIP_LINK,
    GRIP_PIVOT_X,
    GRIP_PIVOT_Y,
)


def _hits(p1, r1, p2, r2):
    d = float(np.linalg.norm(p2 - p1))
    if d > r1 + r2 - 1e-8 or d < abs(r1 - r2) + 1e-8 or d < 1e-9:
        return []
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h2 = r1 * r1 - a * a
    if h2 < 0:
        return []
    h = math.sqrt(h2)
    p = p1 + a * (p2 - p1) / d
    perp = np.array([-(p2 - p1)[1], (p2 - p1)[0]]) / d
    return [p + h * perp, p - h * perp]


def finger_angle(servo_deg: float, side: int = 1) -> float:
    """Finger pitch in radians. 0 = parallel with the palm axis (closed).

    Positive opens the left finger outward. The right finger returns the
    mirrored angle (positive also opens).
    """
    servo = math.radians(servo_deg) * side
    pin = np.array([GRIP_HORN_R * math.cos(servo), GRIP_HORN_R * math.sin(servo)])
    pivot = np.array([GRIP_PIVOT_X, side * GRIP_PIVOT_Y])
    hits = _hits(pin, GRIP_LINK, pivot, GRIP_CRANK)
    if not hits:
        raise ValueError(f"gripper linkage does not assemble at servo {servo_deg}°")
    # Outboard branch: the tie-rod joint sits farther from the palm centreline.
    hit = max(hits, key=lambda p: side * p[1])
    v = hit - pivot
    off = math.radians(GRIP_CRANK_OFFSET_DEG)
    ang = math.atan2(v[1], v[0]) - side * off
    ang = math.atan2(math.sin(ang), math.cos(ang))
    return side * ang


def opening_mm(servo_deg: float) -> float:
    """Clear gap between the inner faces of the two fingers."""
    ang = abs(finger_angle(servo_deg, side=1))
    centerline = 2.0 * (GRIP_PIVOT_Y + GRIP_FINGER_LEN * math.sin(ang))
    return centerline - GRIP_FINGER_T
