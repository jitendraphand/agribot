"""Every unique solid on Agribot MkI.

Printed parts are designed for a 220 mm bed, 0.4 mm nozzle, PETG except the
feet (TPU). Purchased parts are envelope models so the assembly proves fit,
not manufacturer geometry.
"""

from __future__ import annotations

import math

import cadquery as cq

from spec.parameters import (
    ARM_X,
    ARM_Z,
    BODY_H,
    BODY_L,
    BODY_W,
    COXA,
    DECK_T,
    FEMUR,
    FLOOR,
    FOOT_R,
    FOREARM,
    GRIP_CRANK,
    GRIP_FINGER_LEN,
    GRIP_FINGER_T,
    GRIP_HORN_R,
    GRIP_LINK,
    GRIP_PIVOT_X,
    GRIP_PIVOT_Y,
    HIP_X,
    HIP_Y,
    HIP_Z,
    HUMERUS,
    MICRO_H,
    MICRO_L,
    MICRO_W,
    SERVO_BOSS_H,
    SERVO_BOSS_R,
    SERVO_H,
    SERVO_L,
    SERVO_SHAFT_H,
    SERVO_SHAFT_R,
    SERVO_W,
    TIBIA,
    TOOL,
    WALL,
    YAW_TO_SHOULDER,
)

# Bolt pattern shared by the tub bosses and the deck.
def deck_bolts():
    ix, iy = BODY_L / 2 - 11, BODY_W / 2 - 11
    return [
        (ix, iy), (ix, -iy), (-ix, iy), (-ix, -iy), (0.0, iy), (0.0, -iy),
    ]


def _cyl(radius: float, length: float, axis: str = "z") -> cq.Workplane:
    """Cylinder centred on the origin, axis along x, y, or z."""
    c = cq.Workplane("XY").circle(radius).extrude(length).translate((0, 0, -length / 2))
    if axis == "x":
        c = c.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "y":
        c = c.rotate((0, 0, 0), (1, 0, 0), -90)
    return c


def _hole_along(part, axis: str, radius: float, length: float, center):
    return part.cut(_cyl(radius, length, axis).translate(center))


def _bolt_circle(part, axis: str, origin, pcd: float, radius: float = 1.6):
    ox, oy, oz = origin
    for i in range(4):
        a = math.radians(45 + 90 * i)
        if axis == "x":
            c = (ox, oy + (pcd / 2) * math.cos(a), oz + (pcd / 2) * math.sin(a))
        elif axis == "y":
            c = (ox + (pcd / 2) * math.cos(a), oy, oz + (pcd / 2) * math.sin(a))
        else:
            c = (ox + (pcd / 2) * math.cos(a), oy + (pcd / 2) * math.sin(a), oz)
        part = _hole_along(part, axis, radius, 24, c)
    return part


def _as_shape(wp: cq.Workplane):
    return wp.val()


# ---------------------------------------------------------------------------
# Purchased envelopes
# ---------------------------------------------------------------------------

def servo_standard():
    """Output face on z=0, case hanging toward −Z, shaft toward +Z."""
    body = cq.Workplane("XY").box(SERVO_L, SERVO_W, SERVO_H).translate((0, 0, -SERVO_H / 2))
    boss = cq.Workplane("XY").circle(SERVO_BOSS_R).extrude(SERVO_BOSS_H)
    shaft = (
        cq.Workplane("XY")
        .circle(SERVO_SHAFT_R)
        .extrude(SERVO_SHAFT_H)
        .translate((0, 0, SERVO_BOSS_H))
    )
    tab = cq.Workplane("XY").box(54.0, SERVO_W, 2.2).translate((0, 0, -SERVO_H + 1.1))
    part = body.union(boss).union(shaft).union(tab)
    for x in (-24.0, 24.0):
        for y in (-5.0, 5.0):
            part = part.cut(
                cq.Workplane("XY").circle(1.15).extrude(5).translate((x, y, -SERVO_H - 1))
            )
    return part


def servo_micro():
    body = cq.Workplane("XY").box(MICRO_L, MICRO_W, MICRO_H).translate((0, 0, -MICRO_H / 2))
    boss = cq.Workplane("XY").circle(4.0).extrude(2.6)
    shaft = cq.Workplane("XY").circle(2.2).extrude(3.2).translate((0, 0, 2.6))
    return body.union(boss).union(shaft)


def battery():
    # 3S 2200 mAh hard pack, lying flat. Origin at centre.
    body = cq.Workplane("XY").box(105, 35, 24)
    label = cq.Workplane("XY").box(70, 22, 0.6).translate((0, 0, 12))
    return body.union(label)


def board_pi():
    pcb = cq.Workplane("XY").box(65, 30, 1.6).translate((0, 0, 0.8))
    soc = cq.Workplane("XY").box(12, 12, 2).translate((8, 0, 2.2))
    usb = cq.Workplane("XY").box(8, 16, 6).translate((-32, 0, 4))
    return pcb.union(soc).union(usb)


def board_driver():
    pcb = cq.Workplane("XY").box(62, 25, 1.6).translate((0, 0, 0.8))
    chip = cq.Workplane("XY").box(8, 8, 2).translate((0, 0, 2.4))
    headers = cq.Workplane("XY").box(52, 5, 8).translate((0, 8, 5))
    return pcb.union(chip).union(headers)


def ubec():
    return cq.Workplane("XY").box(43, 20, 12)


def imu_board():
    return cq.Workplane("XY").box(21, 16, 3)


def camera():
    board = cq.Workplane("XY").box(24, 24, 2).translate((0, 0, 1))
    lens = cq.Workplane("XY").circle(4).extrude(8).translate((0, 0, 2))
    return board.union(lens)


def ultrasonic():
    board = cq.Workplane("XY").box(45, 20, 1.6).translate((0, 0, 0.8))
    cans = None
    for y in (-13, 13):
        can = cq.Workplane("XY").circle(8).extrude(12).translate((0, y, 1.6))
        cans = can if cans is None else cans.union(can)
    return board.union(cans)


def rocker_switch():
    body = cq.Workplane("XY").box(15, 10, 16)
    bezel = cq.Workplane("XY").box(19, 13, 2).translate((0, 0, 8))
    return body.union(bezel)


def xt30():
    return cq.Workplane("XY").circle(6).extrude(14)


def fastener(length: float):
    """M3 socket head. Origin at the head/shank junction, shank toward −Z."""
    head = cq.Workplane("XY").circle(2.75).extrude(3.0)
    shank = cq.Workplane("XY").circle(1.5).extrude(length).translate((0, 0, -length))
    pocket = cq.Workplane("XY").polygon(6, 2.6).extrude(1.6).translate((0, 0, 1.2))
    return head.union(shank).cut(pocket)


def nut_m3():
    return cq.Workplane("XY").polygon(6, 6.35).extrude(2.4)


def washer_m3():
    outer = cq.Workplane("XY").circle(3.5).extrude(0.5)
    return outer.cut(cq.Workplane("XY").circle(1.7).extrude(0.5))


def heatset_m3():
    return cq.Workplane("XY").circle(2.3).extrude(4.0).cut(
        cq.Workplane("XY").circle(1.25).extrude(4.0)
    )


# ---------------------------------------------------------------------------
# Printed structure
# ---------------------------------------------------------------------------

def chassis_tub():
    L, W, H = BODY_L, BODY_W, BODY_H
    outer = cq.Workplane("XY").box(L, W, H)
    inner = (
        cq.Workplane("XY")
        .box(L - 2 * WALL, W - 2 * WALL, H - FLOOR + 0.2)
        .translate((0, 0, FLOOR / 2 + 0.1))
    )
    tub = outer.cut(inner)
    for x, y in deck_bolts():
        boss = (
            cq.Workplane("XY")
            .circle(5.5)
            .extrude(H - FLOOR)
            .translate((x, y, -H / 2 + FLOOR))
        )
        tub = tub.union(boss)
        tub = tub.cut(
            cq.Workplane("XY").circle(2.05).extrude(7).translate((x, y, H / 2 - 6.8))
        )
    # Belly service opening.
    tub = tub.cut(cq.Workplane("XY").box(74, 40, 10).translate((-8, 0, -H / 2)))
    # Hatch screw pilots in the floor, just outside the opening.
    for x, y in ((-8 - 31, 16), (-8 + 31, 16), (-8 - 31, -16), (-8 + 31, -16)):
        tub = _hole_along(tub, "z", 1.65, 12, (x, y, -H / 2 + 1))
    # Side bolt pattern for the four hip cradles.
    for sx in (+1.0, -1.0):
        for side in (+1.0, -1.0):
            for dx in (-16.0, 16.0):
                for dz in (-14.0, 14.0):
                    tub = _hole_along(
                        tub, "y", 1.7, 16,
                        (sx * HIP_X + dx, side * (W / 2), HIP_Z + dz),
                    )
    # Front camera / ultrasonic window (cables, and a lens sight-line).
    tub = tub.cut(cq.Workplane("XY").box(12, 36, 22).translate((L / 2, 0, 4)))
    # Rear service slot.
    tub = tub.cut(cq.Workplane("XY").box(12, 28, 16).translate((-L / 2, 0, 0)))
    # Battery rails.
    for y in (-16, 16):
        rail = cq.Workplane("XY").box(90, 3, 6).translate((-30, y, -H / 2 + FLOOR + 3))
        tub = tub.union(rail)
    return tub


def top_deck():
    L, W = BODY_L, BODY_W
    deck = cq.Workplane("XY").box(L, W, DECK_T)
    for x, y in deck_bolts():
        deck = _hole_along(deck, "z", 1.7, 10, (x, y, 0))
    # Yaw-servo shaft and boss clearance, plus four screws for the clamp.
    deck = _hole_along(deck, "z", 8.0, 12, (ARM_X, 0, 0))
    for a in (45, 135, 225, 315):
        r = 16
        deck = _hole_along(
            deck, "z", 1.65, 12,
            (ARM_X + r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), 0),
        )
    # Lightening slots, kept clear of the arm and the bolt bosses.
    for x in (-55, -25, 70):
        deck = deck.cut(cq.Workplane("XY").box(16, 28, 8).translate((x, 0, 0)))
    # Raised plinth so the arm clamp has meat around the shaft hole.
    plinth = cq.Workplane("XY").circle(22).extrude(3).translate((ARM_X, 0, DECK_T / 2))
    deck = deck.union(plinth)
    deck = _hole_along(deck, "z", 8.0, 16, (ARM_X, 0, 2))
    return deck


def belly_hatch():
    hatch = cq.Workplane("XY").box(90, 54, 2.6)
    # Recessed lip that plugs the opening.
    plug = cq.Workplane("XY").box(72, 38, 2.2).translate((0, 0, 2.0))
    hatch = hatch.union(plug)
    for x, y in ((-31, 16), (31, 16), (-31, -16), (31, -16)):
        hatch = _hole_along(hatch, "z", 1.7, 12, (x, y, 0))
    return hatch


def front_bezel():
    """Sits on the front wall. Local origin at the bezel centre, +Z up, +Y left.
    The lens looks toward +X (out of the robot)."""
    plate = cq.Workplane("YZ").box(BODY_W, BODY_H, 3.5) if False else None
    plate = cq.Workplane("XY").box(3.5, BODY_W, BODY_H)
    # Ultrasonic cans and a camera bore, looking out +X so the holes run along X.
    plate = _hole_along(plate, "x", 8.2, 12, (0, -13, 2))
    plate = _hole_along(plate, "x", 8.2, 12, (0, 13, 2))
    plate = _hole_along(plate, "x", 4.6, 12, (0, 0, 10))
    for y, z in ((-40, 14), (40, 14), (-40, -14), (40, -14)):
        plate = _hole_along(plate, "x", 1.65, 12, (0, y, z))
    return plate


def rear_panel():
    plate = cq.Workplane("XY").box(3.5, BODY_W - 8, BODY_H - 6)
    plate = plate.cut(cq.Workplane("XY").box(8, 13, 9).translate((0, -18, 2)))  # switch
    plate = _hole_along(plate, "x", 4.2, 12, (0, 16, -2))  # XT30
    plate = plate.cut(cq.Workplane("XY").box(8, 12, 6).translate((0, 0, 8)))  # USB
    for y, z in ((-36, 12), (36, 12), (-36, -12), (36, -12)):
        plate = _hole_along(plate, "x", 1.65, 12, (0, y, z))
    return plate


def camera_hood():
    hood = cq.Workplane("XY").box(18, 20, 14)
    hood = hood.cut(cq.Workplane("XY").box(14, 14, 12).translate((2, 0, -2)))
    hood = _hole_along(hood, "x", 5, 20, (-2, 0, 0))
    return hood


def hip_cradle_left():
    """Origin at the abduction axis. Body (inboard) is −Y. Shaft exits +X."""
    # Shell around the servo case.
    outer = cq.Workplane("XY").box(
        SERVO_H + 7, SERVO_W + 6, SERVO_L + 6
    ).translate((-SERVO_H / 2 + 1.5, 0, 0))
    cavity = cq.Workplane("XY").box(SERVO_H + 0.8, SERVO_W + 0.6, SERVO_L + 0.8).translate(
        (-SERVO_H / 2 + 0.2, 0, 0)
    )
    # Open the mouth toward +X so the case slides in and the horn lives outside.
    mouth = cq.Workplane("XY").box(10, SERVO_W + 0.6, SERVO_L + 0.8).translate((3, 0, 0))
    cradle = outer.cut(cavity).cut(mouth)
    # Horn clearance through whatever lip remains.
    cradle = _hole_along(cradle, "x", 8.5, 20, (2, 0, 0))
    # Inboard flange lands on the body wall. Wall is at local y = BODY_W/2 - HIP_Y.
    wall_y = BODY_W / 2 - HIP_Y
    flange = cq.Workplane("XY").box(50, 4, 46).translate((-8, wall_y - 2, 0))
    cradle = cradle.union(flange)
    for dx in (-16.0, 16.0):
        for dz in (-14.0, 14.0):
            cradle = _hole_along(cradle, "y", 1.7, 20, (dx - 8, wall_y, dz))
    # Retention pins above and below the shaft, across the mouth.
    for z in (-16.0, 16.0):
        cradle = _hole_along(cradle, "y", 1.6, 40, (-1.5, 0, z))
    return cradle


def coxa_left():
    """Abduction horn at the origin, hip-pitch axis at y = COXA. Beam arches
    over the hip servo, which hangs inboard under the beam."""
    prox = cq.Workplane("YZ").circle(13).extrude(5).translate((-2.5, 0, 0))
    dist = cq.Workplane("YZ").circle(12).extrude(5).translate((-2.5, COXA, 0))
    beam = cq.Workplane("XY").box(12, COXA - 10, 12).translate((0, COXA / 2, 32))
    riser_p = cq.Workplane("XY").box(12, 12, 28).translate((0, 8, 16))
    riser_d = cq.Workplane("XY").box(12, 12, 28).translate((0, COXA - 8, 16))
    part = prox.union(dist).union(beam).union(riser_p).union(riser_d)
    part = _bolt_circle(part, "x", (0, 0, 0), 16)
    part = _bolt_circle(part, "x", (0, COXA, 0), 16)
    return part


def femur_left():
    """Hip axis at the origin (shaft along Y). Knee axis at z = −FEMUR.
    The spar sits inboard (−Y) so the knee servo can hang outboard (+Y)."""
    prox = cq.Workplane("XZ").circle(12.5).extrude(5)  # occupies −Y
    dist = cq.Workplane("XZ").circle(11.5).extrude(5).translate((0, 0, -FEMUR))
    spar = cq.Workplane("XY").box(16, 12, FEMUR - 12).translate((0, -11, -(FEMUR / 2)))
    part = prox.union(dist).union(spar)
    part = _bolt_circle(part, "y", (0, 0, 0), 16)
    part = _bolt_circle(part, "y", (0, 0, -FEMUR), 15)
    return part


def tibia_left():
    prox = cq.Workplane("XZ").circle(11.5).extrude(5)
    spar = cq.Workplane("XY").box(14, 11, TIBIA - 18).translate((0, -10, -(TIBIA / 2) - 2))
    bridge = cq.Workplane("XY").box(14, 16, 12).translate((0, -6, -TIBIA + 8))
    tenon = cq.Workplane("XY").circle(5.0).extrude(12).translate((0, 0, -TIBIA - 12))
    part = prox.union(spar).union(bridge).union(tenon)
    part = _bolt_circle(part, "y", (0, 0, 0), 15)
    return part


def foot():
    ball = cq.Workplane("XY").sphere(FOOT_R)
    # Tenon socket along +Z (the tibia arrives from above in the part frame
    # only after the link rotation; the socket is on the ball, opposite a nub).
    sock = cq.Workplane("XY").circle(5.25).extrude(FOOT_R + 2).translate((0, 0, -2))
    part = ball.cut(sock)
    for i in range(8):
        a = math.radians(i * 45)
        # Nubs biased toward −Z so a ball foot still has tread on the ground side.
        nub = (
            cq.Workplane("XY")
            .circle(2.0)
            .extrude(3.2)
            .translate((7.2 * math.cos(a), 7.2 * math.sin(a), -FOOT_R + 0.5))
        )
        part = part.union(nub)
    return part


def horn_clamp():
    """Clamps a 25T spline. Local +Z is the servo shaft. Flange at +Z."""
    barrel = cq.Workplane("XY").circle(8.2).extrude(11)
    bore = cq.Workplane("XY").circle(3.05).extrude(14)
    flange = cq.Workplane("XY").circle(12.0).extrude(3.4).translate((0, 0, 11))
    part = barrel.union(flange).cut(bore)
    part = part.cut(cq.Workplane("XY").box(1.3, 18, 11).translate((6.4, 0, 5.5)))
    part = _hole_along(part, "y", 1.6, 20, (6.5, 0, 6))
    part = _bolt_circle(part, "z", (0, 0, 11), 16, 1.6)
    return part


def arm_yaw_housing():
    """Clamps the yaw servo to the underside of the deck. Origin is the yaw
    axis on the top of the deck; the part hangs below it (−Z)."""
    ring = cq.Workplane("XY").circle(20).extrude(6).translate((0, 0, -DECK_T - 6))
    ring = ring.cut(cq.Workplane("XY").circle(11).extrude(8).translate((0, 0, -DECK_T - 7)))
    # Ears for the four deck screws.
    plate = cq.Workplane("XY").circle(22).extrude(3).translate((0, 0, -DECK_T - 3))
    part = ring.union(plate)
    part = _hole_along(part, "z", 8, 20, (0, 0, -10))
    for a in (45, 135, 225, 315):
        r = 16
        part = _hole_along(
            part, "z", 1.65, 16,
            (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), -8),
        )
    return part


def shoulder_bracket():
    """Yaw horn at the origin, shoulder axis at z = YAW_TO_SHOULDER, shaft along Y."""
    base = cq.Workplane("XY").circle(13).extrude(4)
    upright = cq.Workplane("XY").box(16, 12, YAW_TO_SHOULDER - 6).translate(
        (0, -8, (YAW_TO_SHOULDER - 6) / 2 + 2)
    )
    cheek = cq.Workplane("XZ").circle(12).extrude(5).translate((0, 0, YAW_TO_SHOULDER))
    part = base.union(upright).union(cheek)
    part = _bolt_circle(part, "z", (0, 0, 0), 16)
    part = _bolt_circle(part, "y", (0, 0, YAW_TO_SHOULDER), 16)
    return part


def humerus():
    """Shoulder at the origin, elbow at x = HUMERUS. Spar offset to −Y."""
    prox = cq.Workplane("XZ").circle(12).extrude(5)
    dist = cq.Workplane("XZ").circle(11).extrude(5).translate((HUMERUS, 0, 0))
    spar = cq.Workplane("XY").box(HUMERUS - 14, 12, 16).translate((HUMERUS / 2, -11, 0))
    part = prox.union(dist).union(spar)
    part = _bolt_circle(part, "y", (0, 0, 0), 16)
    part = _bolt_circle(part, "y", (HUMERUS, 0, 0), 15)
    return part


def forearm():
    prox = cq.Workplane("XZ").circle(11).extrude(5)
    dist = cq.Workplane("XZ").circle(9).extrude(4).translate((FOREARM, 0, 0))
    spar = cq.Workplane("XY").box(FOREARM - 12, 11, 14).translate((FOREARM / 2, -10, 0))
    part = prox.union(dist).union(spar)
    part = _bolt_circle(part, "y", (0, 0, 0), 15)
    part = _bolt_circle(part, "y", (FOREARM, 0, 0), 12, 1.4)
    return part


def wrist_hub():
    """Micro-servo horn adapter between the forearm and the palm. Short."""
    base = cq.Workplane("XZ").circle(9).extrude(4)
    block = cq.Workplane("XY").box(16, 14, 14).translate((10, 0, 0))
    part = base.union(block)
    part = _bolt_circle(part, "y", (0, 0, 0), 12, 1.4)
    part = _hole_along(part, "z", 1.6, 16, (12, 0, 0))
    return part


def gripper_palm():
    """Wrist interface at the origin, fingers reach toward +X, open along ±Y.
    Micro servo shaft along +Z."""
    palm = cq.Workplane("XY").box(40, 32, 14).translate((22, 0, 0))
    # Servo pocket, open upward.
    pocket = cq.Workplane("XY").box(MICRO_L + 0.6, MICRO_W + 0.5, 10).translate((8, 0, 6))
    palm = palm.cut(pocket)
    # Finger pivot bores (along Z).
    for s in (+1, -1):
        palm = _hole_along(palm, "z", 1.6, 20, (GRIP_PIVOT_X, s * GRIP_PIVOT_Y, 0))
    # Wrist screw into the hub.
    palm = _hole_along(palm, "z", 1.6, 20, (4, 0, 0))
    # Side cheeks so the pivots have a pair of ears. Already in the block.
    return palm


def finger(side: int = 1):
    """Pivot at the origin, finger reaches +X, crank pin outboard."""
    s = float(side)
    blade = cq.Workplane("XY").box(GRIP_FINGER_LEN, GRIP_FINGER_T, 10).translate(
        (GRIP_FINGER_LEN / 2, 0, 0)
    )
    # Soft pad rib on the inner face.
    pad = cq.Workplane("XY").box(GRIP_FINGER_LEN - 8, 1.6, 6).translate(
        (GRIP_FINGER_LEN / 2 + 2, -s * (GRIP_FINGER_T / 2), 0)
    )
    # Crank boss.
    boss = cq.Workplane("XY").circle(4.5).extrude(4).translate((0, 0, 5))
    part = blade.union(pad).union(boss)
    part = _hole_along(part, "z", 1.6, 20, (0, 0, 0))
    # Crank hole at the linkage radius, along the offset direction used by IK
    # (120° from the finger centreline, outboard).
    off = math.radians(120.0)
    cx = GRIP_CRANK * math.cos(s * off)
    cy = GRIP_CRANK * math.sin(s * off)
    part = _hole_along(part, "z", 1.6, 16, (cx, cy, 4))
    # Tooth notches.
    for x in (14, 22, 30):
        part = part.cut(
            cq.Workplane("XY").box(2.2, 2.5, 5).translate((x, -s * (GRIP_FINGER_T / 2 + 0.4), 0))
        )
    return part


def gripper_link():
    """Tie rod, eye to eye = GRIP_LINK."""
    span = GRIP_LINK
    rod = cq.Workplane("XY").box(span - 8, 4.5, 3.2).translate((span / 2, 0, 0))
    eye_a = cq.Workplane("XY").circle(4.2).extrude(3.2)
    eye_b = cq.Workplane("XY").circle(4.2).extrude(3.2).translate((span, 0, 0))
    part = rod.union(eye_a).union(eye_b)
    part = _hole_along(part, "z", 1.6, 10, (0, 0, 1.6))
    part = _hole_along(part, "z", 1.6, 10, (span, 0, 1.6))
    return part


def cable_clip():
    clip = cq.Workplane("XY").box(12, 8, 7)
    clip = clip.cut(cq.Workplane("XY").box(8, 5, 5).translate((0, 1.2, 1.5)))
    clip = _hole_along(clip, "z", 1.6, 10, (0, -2.2, 0))
    return clip


def battery_strap():
    strap = cq.Workplane("XY").box(50, 8, 2.4)
    strap = _hole_along(strap, "z", 1.7, 8, (-20, 0, 0))
    strap = _hole_along(strap, "z", 1.7, 8, (20, 0, 0))
    return strap


def mirror_y(part: cq.Workplane) -> cq.Workplane:
    return part.mirror("XZ")


# ---------------------------------------------------------------------------
# Catalogue
# ---------------------------------------------------------------------------

# name, builder, kind, material, qty, unit_cost (None → computed for prints),
# vendor note, print note
CATALOGUE = [
    ("chassis_tub", chassis_tub, "printed", "PETG", 1, None, "Print open-side up. 4 perimeters, 30% gyroid.", "Main structural tub"),
    ("top_deck", top_deck, "printed", "PETG", 1, None, "Print plinth up. No supports.", "Top deck and arm plinth"),
    ("belly_hatch", belly_hatch, "printed", "PETG", 1, None, "Print plug down.", "Battery service hatch"),
    ("front_bezel", front_bezel, "printed", "PETG", 1, None, "Print flat.", "Camera and ultrasonic bezel"),
    ("rear_panel", rear_panel, "printed", "PETG", 1, None, "Print flat.", "Switch, XT30, USB panel"),
    ("camera_hood", camera_hood, "printed", "PETG", 1, None, "Print upright.", "Sun hood for the lens"),
    ("hip_cradle_left", hip_cradle_left, "printed", "PETG", 2, None, "Print flange down. FL and RL share this part.", "Abduction servo cradle, left"),
    ("hip_cradle_right", lambda: mirror_y(hip_cradle_left()), "printed", "PETG", 2, None, "Mirror of the left cradle. FR and RR.", "Abduction servo cradle, right"),
    ("coxa_left", coxa_left, "printed", "PETG", 2, None, "Print proximal flange down. Light supports under the arch.", "Coxa link, left"),
    ("coxa_right", lambda: mirror_y(coxa_left()), "printed", "PETG", 2, None, "Mirror of the left coxa.", "Coxa link, right"),
    ("femur_left", femur_left, "printed", "PETG", 2, None, "Print spar down.", "Femur, left"),
    ("femur_right", lambda: mirror_y(femur_left()), "printed", "PETG", 2, None, "Mirror of the left femur.", "Femur, right"),
    ("tibia_left", tibia_left, "printed", "PETG", 2, None, "Print spar down, tenon up.", "Tibia, left"),
    ("tibia_right", lambda: mirror_y(tibia_left()), "printed", "PETG", 2, None, "Mirror of the left tibia.", "Tibia, right"),
    ("foot", foot, "printed", "TPU", 4, None, "TPU 95A. 3 perimeters, 20% infill. Ball down if the slicer allows.", "Spherical tread foot"),
    ("horn_clamp", horn_clamp, "printed", "PETG", 15, None, "Print flange down. 100% infill. Ream the bore to 6.1 only if the spline is tight — bore is modelled 6.1 mm.", "25T clamping horn"),
    ("arm_yaw_housing", arm_yaw_housing, "printed", "PETG", 1, None, "Print ring down.", "Under-deck yaw clamp"),
    ("shoulder_bracket", shoulder_bracket, "printed", "PETG", 1, None, "Print base down.", "Shoulder upright"),
    ("humerus", humerus, "printed", "PETG", 1, None, "Print spar down.", "Upper arm"),
    ("forearm", forearm, "printed", "PETG", 1, None, "Print spar down.", "Forearm"),
    ("wrist_hub", wrist_hub, "printed", "PETG", 1, None, "Print flat.", "Wrist adapter"),
    ("gripper_palm", gripper_palm, "printed", "PETG", 1, None, "Print pocket up.", "Gripper palm"),
    ("finger_left", lambda: finger(1), "printed", "PETG", 1, None, "Print on the outer face.", "Gripper finger, left"),
    ("finger_right", lambda: finger(-1), "printed", "PETG", 1, None, "Print on the outer face.", "Gripper finger, right"),
    ("gripper_link", gripper_link, "printed", "PETG", 2, None, "Print flat. 100% infill.", "Gripper tie rod"),
    ("cable_clip", cable_clip, "printed", "PETG", 8, None, "Print flat.", "Cable clip"),
    ("battery_strap", battery_strap, "printed", "PETG", 2, None, "Print flat.", "Battery strap"),
    ("servo_ds3225", servo_standard, "purchased", "DS3225 class", 15, 14.50, "IP-rated 25 kg·cm standard servo, 180° or 270° programmable. 12 leg + yaw + shoulder + elbow.", "Standard digital servo"),
    ("servo_mg90s", servo_micro, "purchased", "MG90S class", 2, 4.25, "Wrist and gripper.", "Micro servo"),
    ("battery_3s2200", battery, "purchased", "LiPo 3S", 1, 24.00, "2200 mAh ≥30C, XT30. Charge outside the robot.", "Flight pack"),
    ("board_pi_zero2", board_pi, "purchased", "Pi Zero 2 W", 1, 18.00, "Or any 65×30 class SBC. USB gadget for bench bring-up.", "Computer"),
    ("board_pca9685", board_driver, "purchased", "PCA9685", 1, 6.50, "16-channel servo PWM. Logic 5 V from the UBEC, servos on the pack.", "Servo driver"),
    ("ubec_5v", ubec, "purchased", "5V 5A UBEC", 1, 8.00, "Logic rail only. Servo current does not pass through it.", "Regulator"),
    ("imu_mpu6050", imu_board, "purchased", "MPU-6050", 1, 3.50, "Mount on the deck centreline, connectors aft.", "IMU"),
    ("camera_module", camera, "purchased", "CSI camera", 1, 9.00, "8 mm class lens, behind the hood.", "Camera"),
    ("sensor_hcsr04", ultrasonic, "purchased", "HC-SR04", 1, 2.00, "Crop-height / obstacle ping. Optional on tall grass.", "Ultrasonic"),
    ("switch_rocker", rocker_switch, "purchased", "rocker", 1, 1.50, "Pack disconnect on the rear panel.", "Power switch"),
    ("connector_xt30", xt30, "purchased", "XT30", 1, 1.20, "Panel-side pack connector.", "XT30"),
    ("fastener_m3x8", lambda: fastener(8), "purchased", "steel", 24, 0.05, "Horn pinch screws and clips.", "M3×8 SHCS"),
    ("fastener_m3x12", lambda: fastener(12), "purchased", "steel", 40, 0.06, "Deck, bezels, cradles, flanges.", "M3×12 SHCS"),
    ("fastener_m3x16", lambda: fastener(16), "purchased", "steel", 16, 0.07, "Flange stacks that include a horn.", "M3×16 SHCS"),
    ("nut_m3", nut_m3, "purchased", "nyloc", 64, 0.04, "Nyloc, inside the shell where a boss is not used.", "M3 nyloc nut"),
    ("washer_m3", washer_m3, "purchased", "steel", 32, 0.02, "Under screw heads on plastic.", "M3 washer"),
    ("heatset_m3", heatset_m3, "purchased", "brass", 10, 0.08, "Six deck bosses plus four yaw-clamp ears.", "M3 heat-set insert"),
]


def build_all(progress=print):
    shapes = {}
    for name, builder, *_rest in CATALOGUE:
        progress(f"  build {name}")
        shapes[name] = _as_shape(builder())
    return shapes
