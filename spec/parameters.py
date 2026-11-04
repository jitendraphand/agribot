"""Agribot MkI — single source of truth.

Units are millimetres, grams, seconds, and degrees unless a function says
otherwise. Every CAD feature, BOM line, IK link, and the live studio reads
these values (via robot_spec.json) so the design cannot drift.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Body shell. Origin: geometric centre. +X forward, +Y left, +Z up.
# Sized to print on a 220 mm bed (largest single part is the tub).
# ---------------------------------------------------------------------------

BODY_L = 200.0
BODY_W = 108.0
BODY_H = 44.0
WALL = 3.0
FLOOR = 3.2
DECK_T = 3.0

# Abduction-axis location of each hip, body frame. Left side is +Y.
HIP_X = 58.0
HIP_Y = 66.0
HIP_Z = 0.0

# ---------------------------------------------------------------------------
# Leg links, axis-to-axis. Digitigrade 3-DOF: abad (X), hip (Y), knee (Y).
# ---------------------------------------------------------------------------

COXA = 52.0
FEMUR = 70.0
TIBIA = 86.0
FOOT_R = 11.0

# World height of the body centre in the nominal stand. Foot contact is z=0,
# foot-centre height is FOOT_R.
BODY_STAND_Z = 104.0

# ---------------------------------------------------------------------------
# Servos. Standard class matches DS3225 / MG996R envelope.
# Shaft exits the centre of the L×W face; the body extends BODY_H along the
# shaft, behind that face.
# ---------------------------------------------------------------------------

SERVO_L = 40.4
SERVO_W = 20.0
SERVO_H = 36.0
SERVO_BOSS_H = 4.0
SERVO_BOSS_R = 6.0
SERVO_SHAFT_R = 2.9
SERVO_SHAFT_H = 4.5
SERVO_MASS_G = 60.0
SERVO_COST = 14.50

MICRO_L = 22.8
MICRO_W = 12.2
MICRO_H = 23.0
MICRO_MASS_G = 14.0
MICRO_COST = 4.25

# ---------------------------------------------------------------------------
# Arm. Yaw axis sits on the deck. Lengths are axis-to-axis, then tool.
# ---------------------------------------------------------------------------

ARM_X = 42.0
ARM_Y = 0.0
ARM_Z = BODY_H / 2.0 + DECK_T  # yaw axis, on top of the deck
YAW_TO_SHOULDER = 32.0
HUMERUS = 90.0
FOREARM = 84.0
TOOL = 58.0  # wrist axis → fingertip pinch centre (pivot 22 + finger 36)

# Gripper four-bar, left finger. Proven window: servo −35°…0° maps the
# finger from parallel (closed) to ~32° open. Right finger is mirrored.
GRIP_PIVOT_X = 22.0
GRIP_PIVOT_Y = 9.0
GRIP_HORN_R = 10.0
GRIP_LINK = 21.0
GRIP_CRANK = 13.0
GRIP_CRANK_OFFSET_DEG = 120.0
GRIP_FINGER_LEN = 36.0
GRIP_FINGER_T = 7.0
GRIP_CLOSED_SERVO_DEG = 0.0
GRIP_OPEN_SERVO_DEG = -35.0

# ---------------------------------------------------------------------------
# Gait defaults
# ---------------------------------------------------------------------------

GAIT_PERIOD = 0.55
GAIT_STEP = 36.0
GAIT_SWING_H = 22.0
GAIT_DUTY = 0.50

# Servo travel we are willing to use (programmable 180° part, keep margin).
SERVO_TRAVEL_DEG = 160.0

# Joint soft limits in degrees, about the kinematic zero defined in kinematics/.
ABAD_LIM = (-28.0, 28.0)
HIP_LIM = (-40.0, 100.0)
KNEE_LIM = (-150.0, -25.0)
YAW_LIM = (-100.0, 100.0)
SHOULDER_LIM = (-40.0, 130.0)
ELBOW_LIM = (-150.0, 20.0)
WRIST_LIM = (-110.0, 110.0)

# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------

PLA_DENSITY = 1.24  # g/cm^3
TPU_DENSITY = 1.16
PLA_COST_PER_KG = 22.0
TPU_COST_PER_KG = 32.0

# Rough slicer yield used only for the BOM time estimate.
PRINT_G_PER_MIN = 1.6

# Money
FASTENER_COST = 0.06
NUT_COST = 0.04
WASHER_COST = 0.03
INSERT_COST = 0.08


def spec_dict() -> dict:
    keys = [
        "BODY_L", "BODY_W", "BODY_H", "WALL", "FLOOR", "DECK_T",
        "HIP_X", "HIP_Y", "HIP_Z", "COXA", "FEMUR", "TIBIA", "FOOT_R",
        "BODY_STAND_Z", "SERVO_L", "SERVO_W", "SERVO_H",
        "MICRO_L", "MICRO_W", "MICRO_H",
        "ARM_X", "ARM_Y", "ARM_Z", "YAW_TO_SHOULDER", "HUMERUS", "FOREARM", "TOOL",
        "GRIP_HORN_R", "GRIP_LINK", "GRIP_CRANK", "GRIP_CRANK_OFFSET_DEG",
        "GRIP_PIVOT_X", "GRIP_PIVOT_Y", "GRIP_FINGER_LEN", "GRIP_FINGER_T",
        "GRIP_CLOSED_SERVO_DEG", "GRIP_OPEN_SERVO_DEG",
        "GAIT_PERIOD", "GAIT_STEP", "GAIT_SWING_H", "GAIT_DUTY",
        "SERVO_TRAVEL_DEG",
        "ABAD_LIM", "HIP_LIM", "KNEE_LIM", "YAW_LIM", "SHOULDER_LIM",
        "ELBOW_LIM", "WRIST_LIM",
    ]
    out = {k: globals()[k] for k in keys}
    out["stance_y"] = HIP_Y + COXA
    out["track_width"] = 2.0 * (HIP_Y + COXA)
    out["hip_spacing_x"] = 2.0 * HIP_X
    return out
