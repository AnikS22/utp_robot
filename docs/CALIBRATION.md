# Calibration

Nine items. **Do them in this order** — each later one absorbs the residual error of the earlier
ones, so calibrating out of order means calibrating twice.

Every entry has an acceptance criterion. Record the measured number and the criterion outcome in
`EXPERIMENT_LOG.md`; a calibration with no recorded number has not been done.

The thing that ultimately matters: an ADA push plate is **11–15 cm** across, so the press only needs
to land within roughly ±3 cm. That is a forgiving target, and it is why this is tedious rather than
frightening. But the errors below stack, and several are pure offsets that never average out.

---

## ① Riser height — `base_link → link_base`

**Why first.** The xArm sits on a riser left by another student's experiment. This is a pure
vertical offset that propagates into *every* press. Uncalibrated, it looks like a grounding bug, and
you will spend a day on the wrong subsystem.

It is also load-bearing for the science: ADA elevator hall calls are at **1.067 m**, above the sim's
0.70–1.05 m envelope at the flush 0.345 m mount. The riser is what makes the elevator tier possible
at all.

**Method.** Tape measure from the chassis deck reference to the arm's mounting flange. Put the value
in the URDF `base_link → link_base`. Verify: command the tool tip to a marked height on a wall and
measure the error with a rule.

**Accept:** commanded vs measured tip height within **±5 mm**.

## ② Stylus TCP offset

**Why here.** The end effector is a ~0.12 m stylus. Until the arm knows about it, every commanded
position refers to the flange and lands ~12 cm short along the approach axis.

**Method.** Measure the stylus **as mounted** (they get bent and re-seated). Set it with
`set_tcp_offset` on the xArm. Verify by touching a fixed point from two different arm
configurations — the tip should reach the same physical point both times.

**Accept:** same physical point from two configurations within **±5 mm**.

## ③ Lidar mount pose — `base_link → lidar_link`

Currently the design value `[0.25, 0.0, 0.08]` and **unmeasured**. A wrong lidar offset biases every
obstacle by that offset; the resulting map looks entirely plausible while being wrong.

**Method.** Measure x/y/z from the base frame origin to the sensor's optical centre. Update
`config/lidar.yaml`. Verify: park with a flat wall directly ahead, compare the forward scan range to
a tape measure.

**Accept:** scan range vs tape within **±2 cm**.

## ④ Lidar scan direction and zero-angle

**Not a number — a convention check, and the one that silently ruins maps.** A mirrored or rotated
scan publishes perfectly healthy-looking messages: right beam count, right ranges, plausible room
shape. Nothing in the data reveals it. Only a physical test does.

**Method.** Clear a space. Put an object ~1 m **directly in front** (the +x drive direction), run
`python3 bringup/check_scan_geometry.py`, confirm the nearest return is near **0°** and the ASCII
blob is at the top. Repeat with the object on the **left**: REP-103 is x forward, y left, yaw
counter-clockwise, so left must read **positive** (~+90°).

If left reads −90°, the scan is mirrored. Fix it with the driver's `inverted` / `flip_x_axis`
parameter — **never** by negating angles downstream, which fixes the symptom in one consumer and
leaves every other one wrong.

**Accept:** front ≈ 0° ± 5°, left ≈ +90° ± 5°.

## ⑤ Wheel odometry scale

**This matters more than it looks.** The design deliberately runs the entire grounding → approach →
press chain in the **odom** frame, specifically so AMCL's discontinuous jumps stay out of the press
error budget. That only pays off if odom is metrically honest. 4WS bases commonly carry a few
percent systematic scale error.

**Method.**
- *Translation:* mark a start line, drive a measured **5 m** straight, compare `/odom` displacement
  to a tape.
- *Rotation:* mark a heading, spin exactly **360°**, compare `/odom` yaw to 2π.

Repeat each 3 times; systematic error is what you are after, not noise.

**Accept:** translation within **2%**, rotation within **3%**. Worse than that, correct it in the
driver/URDF wheel parameters rather than living with it — a 5% scale error is 25 cm over a 5 m
corridor.

## ⑥ Twist characterisation (GAP 1)

**Not a calibration, a characterisation** — you are confirming what the base actually does with what
you ask, because the driver silently drops twist components (see `HARDWARE_SPECS.md`).

**Method.** Command each in turn and measure the realised motion:

| Command | Expect |
|---|---|
| `linear.x` only | straight, speed matches |
| `angular.z` only | spin in place |
| `linear.y` only | crab sideways |
| `linear.x` + `angular.z` together | **the arc you asked for, or something else?** |
| `linear.x` + `linear.y` + `angular.z` | expect yaw to be dropped |

**Accept:** the truncation table in `HARDWARE_SPECS.md` reproduces. If it does not, re-derive it
before trusting any servo loop — and confirm the approach servo alternates rotate-then-translate
rather than blending.

## ⑦ Camera intrinsics

The D455 is factory-calibrated and `/camera_info` is normally fine — do not re-calibrate by reflex.

**Do confirm depth-to-colour alignment is enabled.** Misalignment presents as "grounding is right
but the 3D point is wrong", which is very easily misdiagnosed as hand-eye error and sends you back
to redo ⑧ for nothing.

**Method.** Point at a target of known size at a known distance. Check the depth at the pixel where
the RGB target centre sits.

**Accept:** depth at the RGB centre within **±2 cm** of tape at 1 m.

## ⑧ Hand-eye — `base_link → mast_cam_optical`

**The critical one.** The camera is fixed to the base on a mast, not mounted on the arm, so this is
a base-to-sensor calibration, not classic eye-in-hand. Do it **after ①, ② and ⑦** so it absorbs only
the residual, rather than silently swallowing the riser and TCP errors.

**Method.**
1. Attach a small visually distinctive marker to the tool tip.
2. Command the arm to **at least 8–10 well-spread points** across the reachable workspace,
   especially at ADA button heights and typical standoff. Vary depth, not just image position — a
   set of coplanar points gives a solution that looks good and extrapolates badly.
3. At each pose, record the tip position in `base_link` from forward kinematics, and detect the
   marker in the camera, lifting to 3D via depth and `K`.
4. Solve the rigid transform (Kabsch / least-squares) for the point pairs.
5. Report the **RMS residual**. That number *is* your press error budget.

**Accept:** RMS residual **< 2 cm**, no single point > 4 cm. Comfortably inside the 11–15 cm plate.
If residuals are large only at the workspace edges, suspect ① or ②, not the solve.

## ⑨ Stopping distance

Validates the slew and deceleration ceilings in `config/safety.yaml`, which were chosen
analytically and have **never been checked against a real chassis**.

**Method.** From max commanded speed, command zero and measure the distance travelled. Repeat with
the arm stowed and — carefully, at reduced speed, with a human on the RC — with it extended, since
the raised CG is the tipping case.

**Accept:** stopping distance consistent with `max_decel_lin`; no visible tipping tendency. If the
robot lurches, lower `max_accel_lin` before anything else.

---

## Then, before any mission

**Map the site** with `slam_toolbox`, driving slowly and closing loops. **Tune AMCL** in the actual
corridor — expect degeneracy along a long featureless corridor, where uncertainty grows along the
travel axis while staying tight laterally. Because of the odom-frame decision, that degrades goal
scoring but cannot cause a missed press. Verify that separation holds in practice rather than
assuming it.

## Summary table

| # | Calibration | Accept | Blocks |
|---|---|---|---|
| ① | Riser `base_link→link_base` | ±5 mm | every press, elevator tier |
| ② | Stylus TCP offset | ±5 mm | every press |
| ③ | Lidar mount pose | ±2 cm | mapping, costmap |
| ④ | Scan direction / zero-angle | ±5° | mapping, navigation |
| ⑤ | Odometry scale | 2% / 3% | servo loops, Nav2 |
| ⑥ | Twist characterisation | table reproduces | approach servo, controller choice |
| ⑦ | Camera intrinsics / alignment | ±2 cm @ 1 m | grounding→3D lift |
| ⑧ | Hand-eye | RMS < 2 cm | **press accuracy** |
| ⑨ | Stopping distance | matches config | safety limits |
