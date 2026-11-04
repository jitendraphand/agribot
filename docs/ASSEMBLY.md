# Assembly

Print every PETG part in `export/stl` with 0.2 mm layers, 4 perimeters, and 30% gyroid. The feet are TPU 95A, 3 perimeters, 20% infill. Horn clamps and tie rods are 100% infill. Nothing in the catalogue is longer than 200 mm, so the tub is the largest single piece and it fits a 220 mm bed.

The STEP files share the part frames documented below. `export/agribot_assembly.step` is the stand pose with the arm stowed, 83 instances, if you would rather inspect fit than build it.

## Frames

- Chassis, deck, bezels, electronics: body datum. Origin at the tub centre, +X forward, +Z up.
- Cradles and leg links: proximal joint at the origin. Coxa beam runs to ±Y, femur and tibia run toward −Z.
- Arm links: proximal joint at the origin, link running toward +X.
- Servo envelopes: output face on local Z = 0, case hanging toward −Z.

## Order

1. Melt the ten M3 heat-sets: six in the deck bosses of the tub, four in the yaw-clamp ears.
2. Bolt `arm_yaw_housing` under the deck around the shaft hole. The yaw servo hangs inside the tub, shaft up. Centre it at 1500 µs before the horn goes on.
3. Drop in the pack between the floor rails, the Pi forward, the PCA9685 between them, the UBEC beside the pack. IMU on the deck centreline. Route servo power from the pack through the rear switch and the 30 A fuse. Logic 5 V comes only from the UBEC.
4. Close the deck with six M3 screws. Fit the front bezel (camera, hood, ultrasonic) and the rear panel (switch, XT30, USB).
5. Bolt a hip cradle to each corner, flange against the side wall, mouth facing forward. Slide the abduction servo in from the mouth and pin it with the two cross screws so it cannot walk out. Do not use the servo's own ears; the case is clamped.
6. For each leg, in order: horn on the abduction shaft, coxa, hip servo (case inboard), horn, femur, knee servo (case outboard), horn, tibia. The tenon is a light press into the TPU foot. If it is loose, a drop of CA at the shoulder of the tenon is enough — do not glue the ball.
7. Set every leg servo to the horn-zero angle in `qa/REPORT.md` before tightening the clamp screw. The gait then sits in the middle of the travel.
8. Shoulder bracket on the yaw horn. Shoulder servo, humerus, elbow servo, forearm, wrist hub, micro servo, palm.
9. Gripper: micro servo in the palm pocket, horn at 0° for the closed stop. Tie rods from the horn pins to the crank holes. Finger pivots are M3 screws used as pins, nyloc from below, not clamped hard enough to bind.
10. Confirm the closed gap is about 11 mm and the −35° command opens past 45 mm before the robot walks.

## Bring-up

Command a stand first: feet under the hip-pitch axes, body centre 104 mm up. Then a slow trot (`period` 0.8 s, `step` 24 mm) on a hard floor with the robot on a sling, and only then on soil. The spherical feet will rock; that is the fixed ankle, not a loose horn. If a stance foot skates, the horn zero is off — re-centre that servo rather than adding trim in software until the table matches.
