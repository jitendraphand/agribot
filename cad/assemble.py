"""Place every solid in the stand pose (legs) with the arm stowed.

Transforms are the same frames the IK uses, so a joint axis in CAD is the
joint axis in the solver.
"""

from __future__ import annotations

import math

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

from kinematics.model import arm_points, leg_points, mat4, ry, rz
from spec.parameters import (
    ARM_X,
    ARM_Y,
    BODY_H,
    BODY_L,
    BODY_STAND_Z,
    BODY_W,
    COXA,
    DECK_T,
    FOREARM,
    HIP_X,
    HIP_Y,
    HIP_Z,
    HUMERUS,
    YAW_TO_SHOULDER,
)

# Arm stow, radians. Folded over the deck, inside joint limits.
STOW = (0.0, math.radians(115), math.radians(-125), math.radians(-80))


def _unit(v):
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("zero axis")
    return np.asarray(v, dtype=float) / n


def frame(origin, x_axis, z_axis) -> np.ndarray:
    z = _unit(z_axis)
    x = np.asarray(x_axis, dtype=float)
    x = x - z * float(np.dot(x, z))
    if np.linalg.norm(x) < 1e-8:
        raise ValueError("x axis parallel to z")
    x = _unit(x)
    y = np.cross(z, x)
    x = np.cross(y, z)
    return mat4(np.column_stack([_unit(x), _unit(y), _unit(z)]), origin)


def T(x, y, z) -> np.ndarray:
    return mat4(np.eye(3), np.array([x, y, z], dtype=float))


def apply(shape, M: np.ndarray):
    tr = gp_Trsf()
    tr.SetValues(*[float(M[i, j]) for i in range(3) for j in range(4)])
    return shape.located(cq.Location(tr))


def stand_angles():
    """IK stand, one solution per leg name."""
    from kinematics.gait import SIDE, SIGN_X, stance_y
    from kinematics.model import leg_ik
    from spec.parameters import FOOT_R

    out = {}
    for name, side, sx in (
        ("FL", 1, 1.0), ("FR", -1, 1.0), ("RL", 1, -1.0), ("RR", -1, -1.0),
    ):
        foot = np.array([sx * HIP_X, side * stance_y(), FOOT_R])
        hip = np.array([sx * HIP_X, side * HIP_Y, BODY_STAND_Z + HIP_Z])
        sol = leg_ik(foot - hip, side)
        if not sol.reachable:
            raise RuntimeError(f"stand IK failed for {name}: {sol.error_mm}")
        out[name] = (side, sx, sol.q, hip)
    return out


def leg_transforms(name, side, q, hip_origin):
    """World frames for one leg's links. Part-local frames match cad.parts."""
    pts = leg_points(*q, side)
    R_abad = pts["R_abad"]
    # Hip-frame vectors, then into the world (hip frame is body-aligned).
    p_hip = hip_origin + R_abad @ np.array([0.0, side * COXA, 0.0])
    p_knee = pts["knee"]  # hip frame
    p_knee = hip_origin + p_knee
    p_foot = hip_origin + pts["foot"]
    R_hip = pts["R_hip"]
    R_knee = pts["R_knee"]
    # Link solids are authored so local +X/+Y/+Z match the hip frame at zero.
    coxa = mat4(R_abad, hip_origin)
    femur = mat4(R_hip, p_hip)
    tibia = mat4(R_knee, p_knee)
    foot = mat4(R_knee, p_foot)
    # Abduction servo: shaft +X, case hanging −X, 40 mm case axis vertical.
    abad_servo = frame(hip_origin, (0, 0, 1), (1, 0, 0))
    # Hip servo shaft is the pitch axis. Case hangs inboard.
    shaft = R_abad @ np.array([0.0, float(side), 0.0])
    hip_servo = frame(p_hip, R_abad @ np.array([0.0, 0.0, 1.0]), shaft)
    # Knee servo case hangs outboard (opposite inboard).
    knee_shaft = R_hip @ np.array([0.0, float(-side), 0.0])
    knee_servo = frame(p_knee, R_hip @ np.array([1.0, 0.0, 0.0]), knee_shaft)
    return {
        "coxa": coxa,
        "femur": femur,
        "tibia": tibia,
        "foot": foot,
        "abad_servo": abad_servo,
        "hip_servo": hip_servo,
        "knee_servo": knee_servo,
        "horn_abad": frame(hip_origin + np.array([6.0, 0, 0]), (0, 0, 1), (1, 0, 0)),
        "horn_hip": frame(p_hip + shaft * 6.0, R_abad @ np.array([0.0, 0.0, 1.0]), shaft),
        "horn_knee": frame(p_knee + knee_shaft * 6.0, R_hip @ np.array([1.0, 0.0, 0.0]), knee_shaft),
    }


def arm_transforms(base: np.ndarray):
    yaw, sh, el, wr = STOW
    pts = arm_points(yaw, sh, el, wr)
    Ryaw = rz(yaw)

    def arm_frame_to_world(local_M):
        # local_M is in the arm base. Shift by `base`.
        M = local_M.copy()
        M[:3, 3] = M[:3, 3] + base
        return M

    # Parts are modelled along local +X with the pitch axis as local +Y,
    # and positive pitch is Ry(-angle) on that geometry (see kinematics notes).
    def link_R(abs_pitch: float) -> np.ndarray:
        return Ryaw @ ry(-abs_pitch)

    shoulder = base + Ryaw @ np.array([0.0, 0.0, YAW_TO_SHOULDER])
    elbow = base + pts["elbow"]
    wrist = base + pts["wrist"]
    # Yaw servo hangs under the deck: shaft +Z, case −Z. Identity in arm frame.
    yaw_servo = frame(base, (1, 0, 0), (0, 0, 1))
    # Shoulder servo at the shoulder, shaft along yawed +Y, case toward +Y (left).
    y_axis = Ryaw @ np.array([0.0, 1.0, 0.0])
    shoulder_servo = frame(shoulder, Ryaw @ np.array([1.0, 0.0, 0.0]), -y_axis)
    elbow_servo = frame(elbow, link_R(sh) @ np.array([0.0, 0.0, 1.0]), -y_axis)
    # Wrist micro, shaft along the forearm pitch axis.
    wrist_servo = frame(wrist, link_R(sh + el) @ np.array([1.0, 0.0, 0.0]), y_axis)
    palm_R = link_R(sh + el + wr)
    palm = mat4(palm_R, wrist)
    return {
        "yaw_housing": T(*base),
        "shoulder_bracket": mat4(Ryaw, base),
        "humerus": mat4(link_R(sh), shoulder),
        "forearm": mat4(link_R(sh + el), elbow),
        "wrist_hub": mat4(link_R(sh + el), wrist),
        "palm": palm,
        "yaw_servo": yaw_servo,
        "shoulder_servo": shoulder_servo,
        "elbow_servo": elbow_servo,
        "wrist_servo": wrist_servo,
        "horn_yaw": frame(base + np.array([0, 0, 4.0]), (1, 0, 0), (0, 0, 1)),
        "horn_shoulder": frame(shoulder + y_axis * 6.0, Ryaw @ np.array([1, 0, 0]), y_axis),
        "horn_elbow": frame(elbow + y_axis * 6.0, link_R(sh) @ np.array([0, 0, 1]), y_axis),
        "finger_left": mat4(palm_R, wrist + palm_R @ np.array([22.0, 9.0, 0.0])),
        "finger_right": mat4(palm_R, wrist + palm_R @ np.array([22.0, -9.0, 0.0])),
        "grip_servo": frame(wrist + palm_R @ np.array([8.0, 0.0, 8.0]), palm_R @ np.array([1, 0, 0]), palm_R @ np.array([0, 0, 1])),
    }


def instance_list(shapes: dict):
    """(instance_name, shape, world_matrix) for the stand/stow assembly."""
    body = T(0, 0, BODY_STAND_Z)
    items = []

    def add(instance, shape_name, M):
        items.append((instance, shapes[shape_name], M))

    add("chassis_tub", "chassis_tub", body @ T(0, 0, 0))
    add("top_deck", "top_deck", body @ T(0, 0, BODY_H / 2 + DECK_T / 2))
    add("belly_hatch", "belly_hatch", body @ T(-8, 0, -BODY_H / 2 - 1.2))
    add("front_bezel", "front_bezel", body @ T(BODY_L / 2 - 1.4, 0, 0))
    add("rear_panel", "rear_panel", body @ T(-BODY_L / 2 + 1.4, 0, 0))
    add("camera_hood", "camera_hood", body @ T(BODY_L / 2 + 8, 0, 10))
    add("battery", "battery_3s2200", body @ T(-28, 0, -6))
    add("pi", "board_pi_zero2", body @ T(48, 0, -8))
    add("driver", "board_pca9685", body @ T(8, 0, -6))
    add("ubec", "ubec_5v", body @ T(-10, 32, -8))
    add("imu", "imu_mpu6050", body @ T(-20, 0, BODY_H / 2 + DECK_T + 2))
    add("camera", "camera_module", body @ T(BODY_L / 2 - 6, 0, 10))
    add("ultrasonic", "sensor_hcsr04", body @ T(BODY_L / 2 + 2, 0, 2))
    add("switch", "switch_rocker", body @ T(-BODY_L / 2 - 2, -18, 2))
    add("xt30", "connector_xt30", body @ T(-BODY_L / 2 - 4, 16, -2))

    for strap_y, name in ((-20, "battery_strap_a"), (20, "battery_strap_b")):
        add(name, "battery_strap", body @ T(-28, strap_y, 8))

    legs = stand_angles()
    for name, (side, _sx, q, hip) in legs.items():
        tag = "left" if side > 0 else "right"
        xf = leg_transforms(name, side, q, hip)
        add(f"cradle_{name}", f"hip_cradle_{tag}", mat4(np.eye(3), hip))
        add(f"coxa_{name}", f"coxa_{tag}", xf["coxa"])
        add(f"femur_{name}", f"femur_{tag}", xf["femur"])
        add(f"tibia_{name}", f"tibia_{tag}", xf["tibia"])
        add(f"foot_{name}", "foot", xf["foot"])
        add(f"servo_abad_{name}", "servo_ds3225", xf["abad_servo"])
        add(f"servo_hip_{name}", "servo_ds3225", xf["hip_servo"])
        add(f"servo_knee_{name}", "servo_ds3225", xf["knee_servo"])
        add(f"horn_abad_{name}", "horn_clamp", xf["horn_abad"])
        add(f"horn_hip_{name}", "horn_clamp", xf["horn_hip"])
        add(f"horn_knee_{name}", "horn_clamp", xf["horn_knee"])

    base = np.array([ARM_X, ARM_Y, BODY_STAND_Z + BODY_H / 2 + DECK_T])
    arm = arm_transforms(base)
    add("arm_yaw_housing", "arm_yaw_housing", arm["yaw_housing"])
    add("shoulder_bracket", "shoulder_bracket", arm["shoulder_bracket"])
    add("humerus", "humerus", arm["humerus"])
    add("forearm", "forearm", arm["forearm"])
    add("wrist_hub", "wrist_hub", arm["wrist_hub"])
    add("gripper_palm", "gripper_palm", arm["palm"])
    add("finger_left", "finger_left", arm["finger_left"])
    add("finger_right", "finger_right", arm["finger_right"])
    add("servo_yaw", "servo_ds3225", arm["yaw_servo"])
    add("servo_shoulder", "servo_ds3225", arm["shoulder_servo"])
    add("servo_elbow", "servo_ds3225", arm["elbow_servo"])
    add("servo_wrist", "servo_mg90s", arm["wrist_servo"])
    add("servo_grip", "servo_mg90s", arm["grip_servo"])
    add("horn_yaw", "horn_clamp", arm["horn_yaw"])
    add("horn_shoulder", "horn_clamp", arm["horn_shoulder"])
    add("horn_elbow", "horn_clamp", arm["horn_elbow"])
    for i, y in enumerate((-18, 18)):
        add(f"gripper_link_{i}", "gripper_link", arm["palm"] @ T(10, y, 6))
    # A few structural screws so the assembly shows the stack. The BOM has the full count.
    for i, (x, y) in enumerate(((89, 43), (89, -43), (-89, 43), (-89, -43))):
        add(f"deck_screw_{i}", "fastener_m3x12", body @ T(x, y, BODY_H / 2 + DECK_T + 3))
    return items


def color_for(name: str) -> cq.Color:
    if "servo" in name:
        return cq.Color(0.16, 0.16, 0.15)
    if "foot" in name:
        return cq.Color(0.35, 0.22, 0.14)
    if name.startswith("finger") or "palm" in name or "gripper" in name or "humerus" in name or "forearm" in name or "shoulder" in name or "wrist" in name or "yaw" in name:
        return cq.Color(0.72, 0.48, 0.24)
    if any(k in name for k in ("femur", "tibia", "coxa", "cradle", "horn")):
        return cq.Color(0.73, 0.66, 0.52)
    if any(k in name for k in ("battery", "pi", "driver", "ubec", "imu", "camera", "ultra", "switch", "xt30")):
        return cq.Color(0.25, 0.32, 0.28)
    return cq.Color(0.24, 0.29, 0.18)
