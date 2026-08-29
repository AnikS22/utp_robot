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

**MEASURED 2026-08-21: riser = 391.225 mm** (CAD, cross-checked by tape). Deck is 0.345 m, so
`link_base` sits at **~0.740 m** off the floor, and the arm's base plate is **horizontal** — so
this is a pure translation, as assumed above. Consequence: with 0.764 m of arm from 0.740 m, the
**1.067 m ADA elevator hall call is reachable**. At the flush 0.345 m mount it would not be.

## ② Tool TCP offset — **STILL OPEN, and it is the only thing blocking a press**

**The fitted tool is a GRIPPER, not the stylus these docs assume.** Checked on the arm 2026-08-21:

    tcp_offset = [0, 0, 0, 0, 0, 0]      tcp_load = [0, [0, 0, 0]]

So `get_position()` reports the **flange**, and the arm believes its tool is a bare plate. Two
consequences, and the second is the one that matters for a contact move: every commanded position
is short by the tool length, **and** the collision-detection thresholds are calibrated for no
payload, so the arm will either nuisance-trip on its own gripper or fail to notice a real contact.

Hand-eye (item ⑧) no longer depends on this — it now solves the marker offset for itself. But a
**press** does: the calibration knows where the *marker* is, not where the *fingertips* are.

**What is needed:** flange face → fingertips, along the gripper's pointing direction, ±3 mm. Plus
the gripper's mass for `set_tcp_load`.

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

**MEASURED 2026-08-29** (`bringup/characterise_twist.py --go`, 1 odom publisher, 46 Hz):

| axis | commanded | expected | measured | scale | sign |
|---|---|---|---|---|---|
| linear `vx` | +0.10 m/s, 3 s | +0.300 m | +0.283 m | **0.94** | correct |
| angular `wz` | +0.20 rad/s, 3 s | +34.377 deg | +20.283 deg | **0.59** | correct |

Lateral drift on the straight run was -0.000 m; position drift on the spin was 0.001 m. So the
axes are clean and independent -- this is a SCALE error, not a coupling or a sign error.

**The angular figure is the serious one.** 0.59 means every commanded rotation delivers 59% of
what the stack believes it asked for. A proportional heading controller under-rotates by 41% on
every cycle, so turns take far longer than the tuning assumes, and combined with a steering
target that is recomputed each cycle it can fail to converge at all -- which is what the door
livelock of 2026-08-29 looked like from the outside.

**WHAT THIS MEASUREMENT CANNOT TELL YOU, and it matters before anyone "corrects" it.** It compares
COMMANDED against ODOMETRY. Those disagree by 41%, but the disagreement has two opposite causes:

  * the CHASSIS under-rotates -- the robot really did turn 20 deg. Then the controller is being
    lied to about its authority and a gain correction is right.
  * ODOMETRY under-reports -- the robot really turned 34 deg and only claims 20. Then a gain
    correction makes the robot rotate 1.7x too far, and every recorded waypoint yaw is wrong.

Guessing between them is how you turn a 41% error into a 70% one. Disambiguate with an EXTERNAL
reference: mark the floor under two points on the chassis, command a full turn, and see whether
the robot physically returns to the marks when odom reports 360 deg. `bringup/scan_compass.py`
gives a second, lidar-based view of the same question.

Until that is done, treat both scales as UNCORRECTED and known-wrong. The 0.94 linear figure also
means every leg is ~6% short, which is inside the 15 cm arrival tolerance for a 2 m leg and is not
inside it for a 6 m one.

<!-- original section follows -->
## ⑤ Wheel odometry scale (procedure)

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

**PASS 2026-08-21**, measured with `bringup/check_depth_alignment.py` off a single frame using the
ADA plate's own protrusion: centre offset **−2.8 mm / +4.2 mm** at 0.84 m; diameter 17.0 cm from
depth vs 16.5 cm from RGB. Three traps that each produced a wrong answer before the method settled,
all now handled by that script: a flat depth threshold mistakes a non-fronto-parallel wall for
misalignment; signs and fire alarms also stand proud, so a bounding box over all protruding pixels
measures the widest of them; and the robot's own arm appears at ~0.37 m, half a metre proud.

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

**PASS 2026-08-21. rms 3.0 mm, worst 8.2 mm, 10 poses, rotation spread 63.8°.**

**The method changed, and it no longer needs item ② first.** `cv2.calibrateRobotWorldHandEye`
solves for the camera pose *and* the flange→marker offset together, given the marker's full 6-DoF
pose (solvePnP on the ArUco corners, scaled by the printed 40 mm). So the ruler measurement that
step 1 above asks for comes **out** of the solve instead of into it. Run:

```bash
python3 bringup/handeye_auto.py --go        # drives the arm through 10 bounded joint poses
python3 bringup/handeye_solve_rw.py         # solve; writes calib/handeye.json
python3 bringup/handeye_verify.py --go      # end-to-end placement accuracy, no contact
python3 bringup/check_calib.py              # later: still valid? one frame, 30 s
```

**A residual is not validation.** A systematically wrong calibration fits its own data perfectly.
Two independent checks were run and both should be repeated after any recalibration:

- **Against an outside measurement.** Solved camera x = **−327.6 mm** vs **−324.238 mm** off the
  CAD — 3.4 mm apart, and the solver never saw that number.
- **Leave-one-out cross-validation.** Solve on 9 pairs, predict the 10th: **3.31 mm mean,
  6.16 mm worst**, against an in-sample 3.0 mm. Nearly equal, so it generalises rather than overfits.

**Known weakness: camera *lateral* position is poorly constrained.** Across the ten LOO sub-solves,
camera y spans 55 mm (std 14 mm) while x spans 16 mm — camera-y and marker-y trade off against each
other. Prediction is unaffected (that is what LOO measures), but do not read the reported y as a
physical measurement, and **include more lateral spread** in the next collection.

**End-to-end placement (`handeye_verify.py`, 6 targets over ±50 mm):**

    mean |error| 4.3 mm    worst 9.7 mm    bias dx +2.3 dy +0.5 dz +0.5 mm

That is the number that matters — it includes calibration, arm positioning, detection and depth.
An ADA plate needs ~±30 mm, so there is 3× margin.

### Two hardware constraints found while doing this

**J5 is the binding joint.** Its range is **−97° to +180°**, and in the working pose it sits near
−92° — under 5° of headroom. It stopped collection once as a joint delta (now fixed by reading
`XCONF.Robot.JOINT_LIMITS` and clamping), and again as **error 23** on a *Cartesian* goal 50 mm
lower, where clamping cannot help because the IK picks the joint solution. **Approach in joint
space, or keep Cartesian goals at or above the current height.**

**The arm occludes its own camera.** At a working pose it fills over half the frame, and the marker
leaves view entirely at full extension. So grounding must complete **before** the arm enters the
workspace, and closed-loop visual verification is not available at the moment of contact. This is a
constraint on the pipeline's ordering, not a nuisance — see `docs/PIPELINE.md`.

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
