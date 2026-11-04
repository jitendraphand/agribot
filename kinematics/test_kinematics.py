"""Round-trip and gait gates. Run: python -m kinematics.test_kinematics"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from kinematics.gait import sample_gait, stance_slip  # noqa: E402
from kinematics.model import (  # noqa: E402
    arm_fk,
    arm_ik,
    arm_within_limits,
    leg_fk,
    leg_ik,
    leg_within_limits,
)
from spec.parameters import BODY_STAND_Z, FOOT_R, HUMERUS, FOREARM, TOOL, YAW_TO_SHOULDER  # noqa: E402


def check(name: str, cond: bool, detail: str = "") -> bool:
    print(("PASS " if cond else "FAIL ") + name + (f" — {detail}" if detail else ""))
    return cond


def test_leg_roundtrip() -> bool:
    ok = True
    samples = []
    for side in (+1, -1):
        for q in (
            (0.0, 0.2, -1.2),
            (0.15, -0.3, -0.9),
            (-0.2, 0.4, -1.5),
            (0.0, 0.0, -1.1),
            (0.25, 0.6, -1.35),
        ):
            target = leg_fk(*q, side)
            sol = leg_ik(target, side)
            samples.append(sol.error_mm)
            ok &= check(
                f"leg side {side} q {[round(math.degrees(v), 1) for v in q]}",
                sol.reachable and sol.error_mm < 0.05,
                f"err {sol.error_mm:.4f} mm",
            )
    print(f"  leg max residual {max(samples):.4f} mm")
    return ok


def test_stand() -> bool:
    from kinematics.gait import SIDE, stance_y, SIGN_X, HIP_X, HIP_Y
    # Import HIP from gait module attributes — they re-export via sample only.
    from spec.parameters import HIP_X as HX, HIP_Y as HY, HIP_Z
    ok = True
    for name, side, sx in (("FL", 1, 1), ("FR", -1, 1), ("RL", 1, -1), ("RR", -1, -1)):
        foot = np.array([sx * HX, side * stance_y(), FOOT_R])
        hip = np.array([sx * HX, side * HY, BODY_STAND_Z + HIP_Z])
        sol = leg_ik(foot - hip, side)
        ok &= check(
            f"stand {name}",
            sol.reachable and sol.error_mm < 0.05 and leg_within_limits(sol.q),
            f"deg {[round(math.degrees(v), 1) for v in sol.q]} err {sol.error_mm:.3f}",
        )
    return ok


def test_gait() -> bool:
    samples = [sample_gait(i * 0.01) for i in range(0, 180)]
    ik_ok = all(s.reachable and s.max_ik_mm < 0.08 for s in samples)
    lim_ok = all(s.limits_ok for s in samples)
    ground = True
    for s in samples:
        for p in s.feet_world.values():
            if p[2] < FOOT_R - 0.05:
                ground = False
    slip = stance_slip(samples)
    # Numerical dt noise only — true slip of this formulation is 0.
    slip_ok = slip < 1.0
    ok = True
    ok &= check("gait IK", ik_ok, f"worst {max(s.max_ik_mm for s in samples):.4f} mm")
    ok &= check("gait limits", lim_ok)
    ok &= check("feet clear ground", ground)
    ok &= check("stance slip", slip_ok, f"{slip:.3f} mm/s")
    return ok


def test_arm() -> bool:
    ok = True
    poses = [
        (0.0, 0.4, -0.8, 0.3),
        (0.5, 0.9, -1.4, 0.4),
        (-0.4, 0.2, -0.5, -0.4),
        (0.2, 1.2, -1.8, 0.6),
    ]
    worst = 0.0
    for q in poses:
        target = arm_fk(*q)
        # Recover with the tool pitch the pose actually has.
        pitch = q[1] + q[2] + q[3]
        sol = arm_ik(target, tool_pitch=pitch)
        worst = max(worst, sol.error_mm)
        ok &= check(
            f"arm {[round(math.degrees(v), 1) for v in q]}",
            sol.reachable and sol.error_mm < 0.08,
            f"err {sol.error_mm:.4f} mm",
        )
    # Field sample just ahead of the nose, gripper pointing straight down.
    # Target is in the arm-base frame (yaw axis on the deck).
    seedling = np.array([128.0, 30.0, -76.0])
    sol = arm_ik(seedling, tool_pitch=-math.pi / 2)
    ok &= check(
        "seedling pick",
        sol.reachable and sol.error_mm < 0.2 and arm_within_limits(sol.q),
        f"deg {[round(math.degrees(v), 1) for v in sol.q]} err {sol.error_mm:.3f}",
    )
    print(f"  arm max residual {worst:.4f} mm")
    return ok


def test_gripper() -> bool:
    from kinematics.gripper import finger_angle, opening_mm
    from spec.parameters import GRIP_CLOSED_SERVO_DEG, GRIP_OPEN_SERVO_DEG
    closed = opening_mm(GRIP_CLOSED_SERVO_DEG)
    opened = opening_mm(GRIP_OPEN_SERVO_DEG)
    ok = check("gripper closed gap", 8.0 < closed < 14.0, f"{closed:.1f} mm")
    ok &= check("gripper opens", opened > closed + 25.0, f"{opened:.1f} mm")
    ok &= check(
        "gripper mirror",
        abs(finger_angle(GRIP_OPEN_SERVO_DEG, 1) - finger_angle(GRIP_OPEN_SERVO_DEG, -1)) < 1e-6,
    )
    return ok


def main() -> int:
    ok = test_leg_roundtrip() and test_stand() and test_gait() and test_arm() and test_gripper()
    print("ALL PASS" if ok else "KINEMATICS FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
