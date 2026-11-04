# Agribot MkI — QA report

Gates: **174 passed**, **0 failed**, of 174.

## Servo horn zeros

Install each horn with the servo at 1500 µs parked on this kinematic angle, so the gait sits in the middle of the travel.

| Joint | Min ° | Max ° | Span ° | Horn zero ° |
|---|---:|---:|---:|---:|
| abad | -0.0 | 0.0 | 0.0 | -0.0 |
| hip | 49.81 | 77.81 | 28.0 | 63.81 |
| knee | -127.05 | -106.02 | 21.02 | -116.53 |

## Failures

None.

## What this pass actually checks

- Every catalogue solid exists as STEP (ISO-10303-21) and a binary STL with a real triangle count.
- Every printed part fits a 220 mm bed.
- The 83-instance assembly is one compound in a sane bounding box, with the feet on the soil rather than through it.
- Analytical leg IK round-trips to < 0.05 mm, the trot stays inside the joint limits, and stance slip is numerical noise.
- The gripper four-bar assembles at the open and closed stops and the gap grows by more than 25 mm.
- The field-pick target is inside the arm workspace and the joint limits.
- Simulation videos exist and are longer than three seconds.
