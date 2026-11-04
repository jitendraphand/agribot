# Kinematics

Frames are millimetres. +X is forward, +Y is left, +Z is up. Angles are radians in code and degrees in the horn table.

## Leg

Three joints, proximal to distal.

1. `q_abad` rotates about +X.
2. The coxa is a fixed lateral offset `side * COXA` (`side` is +1 on the left). It does not pitch.
3. `q_hip` pitches about the post-abad +Y. Zero hangs the femur straight down. Positive hip swing moves the foot backward, toward −X.
4. `q_knee` is an additional pitch about the same axis. Zero is colinear with the femur. Negative knee is the crouch: the tibia folds back under the hip so a bent leg can still place the foot under the hip axis.

Forward map, hip frame:

```
p = Rx(q_abad) · ( (0, side·COXA, 0)
                 + Ry(q_hip) · (0, 0, −FEMUR)
                 + Ry(q_hip + q_knee) · (0, 0, −TIBIA) )
```

Inverse, in closed form:

- The coxa fixes the post-abad Y coordinate, so `q_abad` and the vertical coordinate fall out of a 2-D rotation in YZ. The foot is taken below the hip (`z < 0`).
- Planar distance from the hip-pitch axis to the foot sets `cos q_knee` by the law of cosines. The negative branch is the crouch.
- Substituting into the pitch equations leaves a linear system in `(sin q_hip, cos q_hip)`, solved with `atan2`. Reachable targets round-trip to well under 0.05 mm. Unreachable targets clamp to the workspace boundary and report the true miss.

Nominal stand puts every foot at the hip's X and at `y = side · (HIP_Y + COXA)`, foot centre at `z = FOOT_R` in the world (the ball rests on the soil; the tread nubs engage about 3 mm, which is intentional on soft ground).

## Trot

Duty factor 0.5, diagonal pairs. In the body frame a stance foot moves from `+step/2` to `−step/2` while the body advances at `step / (duty · period)`. The world position of a planted foot therefore does not move. Swing is a half-sine of height `GAIT_SWING_H`. The QA gate requires stance slip under 1 mm/s; the formulation's true slip is zero and the residual is the sample interval.

A non-zero yaw rate rotates the nominal footholds with the heading. That arc is a preview, not a zero-slip manoeuvre — the straight trot is the one the gate certifies.

## Arm

Yaw about Z, then three pitches in the arm plane. Zero shoulder is horizontal forward; positive shoulder pitches up. Elbow and wrist are relative. With a chosen tool pitch (0 level, −π/2 down) the wrist point is backed out of the target along the tool and a 2-link solution places the shoulder and elbow. The elbow-up branch (`q_elbow < 0`) keeps the elbow above the chord so the arm clears the chassis.

The field target used everywhere is `(128, 30, −76)` mm in the arm-base frame, tool straight down. It is inside the joint limits. The pick film moves from the stow pose to that solution in joint space, closes the gripper, then joint-lerps to the carry pose. Joint limits are a box, so the lerp cannot leave them.

Stow: yaw 0°, shoulder 115°, elbow −125°, wrist −80°.
Carry: yaw 12°, shoulder 100°, elbow −125°, wrist −40°.

## Gripper

Left finger, palm frame. Servo at the origin, pivot at `(22, 9)`, crank radius 13 mm at 120° from the finger centreline, tie rod 21 mm, horn radius 10 mm. The joint is the outboard intersection of the two circles. The right finger is the mirror and uses the same opening angle with the opposite sign.

| Servo | Gap between inner faces |
|---:|---:|
| 0° (closed) | ~11 mm |
| −35° (open) | ~49 mm |

## Horn zeros

The gait does not use the middle of a servo's 180° travel, because kinematic zero (femur straight down, knee straight) is not the crouch. `qa/run_qa.py` records the min and max of each joint over two seconds of trot and parks the 1500 µs pulse at the midpoint. The span has more than 10° of margin inside the 160° budget. Install the horn at that angle with the servo centred, then let the kinematic angle be the command.
