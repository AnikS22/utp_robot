"""Isaac trial server (py3.11) — the live sim + ROS2 bridge that the pipeline/Nav2 drive.

Built in stages against docs/integration_contract.md (ROS_DOMAIN_ID=42):
  STAGE 1: robot articulation + /odom, /tf (odom->base_link, base_link->sensors),
           and /scan from the RTX lidar — the Nav2 inputs. Verified via `ros2 topic echo`.
  STAGE 2: + D455 camera over the bridge — /mast_cam/color/image_raw (rgb 1280x720),
           /mast_cam/depth/image_rect_raw (32FC1 meters, distance_to_image_plane on the color RP),
           /mast_cam/color/camera_info. frame_id mast_cam_optical.
  STAGE 3: + /cmd_vel -> Ranger4WSController -> wheel joints. ROS2SubscribeTwist
           feeds (vx, vy, wz) each frame into utp/control/ranger_4ws.py (mode policy from
           config/robot.yaml base.nav_mode); steer joints position-driven, wheels velocity-driven.
           Zero cmd or >0.5 s without a message stops the base (message arrival counted via a
           Counter node on the subscriber's execOut).
           SIGN CONVENTION (authority: validation/4ws_report.md Part C): the model's steer joints
           are positive-CLOCKWISE viewed from above (crab test: commanded steer +90 deg + positive
           wheel spin moved the base dy=-1.756, i.e. body-RIGHT; controller convention is y-LEFT).
           So joint steer target = -controller steer. Positive wheel joint velocity rolls the wheel
           forward (forward test: +spin -> dx=+1.755). Controller wheel_speed is already rad/s.
  STAGE 4 (this file, now): + the CONTROL-PLANE:
           /scene/command  (std_msgs/String JSON, generic isaacsim.ros2.bridge.ROS2Subscriber):
               {"cmd":"build","scene_type":"button_door","seed":1} -> tear down the demo walls,
               build the scene at /World/Scene (scene_gen builders), robot to START=(-0.9,0);
               {"cmd":"reset"} -> rebuild the SAME scene/seed.
           /scene/state    (std_msgs/String JSON ~5 Hz, generic ROS2Publisher gated by an
               OnImpulseEvent every 12 frames): contract JSON + robot_pose (world base_link pose,
               an ADDITION to the contract so clients can do world<->base_link transforms without
               TF plumbing) + gt_target.bbox = the button prim's world AABB corners projected
               through the D455 color camera (pinhole fx=fy=focal/hAperture*W, cx=W/2, cy=H/2 —
               cross-checked at startup against the values the CameraInfo helper publishes).
           /arm_reach/goal (geometry_msgs/PointStamped in base_link, generic ROS2Subscriber) ->
               IK reach-and-press with the M1.1 DLS solver (fk + joint limits imported from
               isaac_worker/robot/ik_sweep.py, solved in the ACTUAL link_base frame from live
               transforms): approach a standoff 6 cm before the point along the base-forward
               press axis, advance 4 cm, dwell, retreat, return home. Then /arm_reach/result
               (std_msgs/Bool).
           PRESS->SCENE COUPLING (honest note, also in the contract doc): press detection is
               PROXIMITY-based, not force-based. When the advance completes with the flange
               within tolerance of the commanded press point AND the goal point is within
               PRESS_ACT_TOL of a ground-truth button center, the server calls
               builder.act("press_button", <that target_id>), which physically depresses the
               button cap and (for a working, correct button) opens the door articulation.

Run (pinned, headless, domain 42):
    ROS_DOMAIN_ID=42 env -u DISPLAY CUDA_VISIBLE_DEVICES=<your assigned GPU — check nvidia-smi Display Active first> \
        ~/isaacsim/python.sh isaac_worker/trial_server.py --seconds 900
"""
from __future__ import annotations
import argparse
import json
import math
import os
import sys
from pathlib import Path

# LAPTOP COPY (2026-08-27) of the sim repo's isaac_worker/trial_server.py -- copied, not edited
# in place, per CLAUDE.md. TWO deltas from upstream, both tagged UTP-LAPTOP below:
#   1. REPO points at the sim repo from outside it.
#   2. rep.orchestrator.run() after play: on this Isaac build (pip isaacsim-venv), SDG render
#      products only fill under orchestrator control -- without it every camera frame is the
#      cleared buffer (uniform gray 228, depth all-inf; measured). The repo's own
#      robot/verify_render.py documents the same: "annotators only fill on rep.orchestrator.step()".
REPO = Path.home() / "unlocking-the-path"  # UTP-LAPTOP
ROBOT_USD = REPO / "isaac_worker" / "assets" / "ranger_xarm6_arranged.usda"
sys.path.insert(0, str(REPO))  # utp.control.ranger_4ws is pure numpy -> py3.11-safe

CMD_TIMEOUT_S = 0.5   # no /cmd_vel message for this long (sim time) -> stop
CMD_EPS = 1e-3        # below this the command component is "zero" (matches controller eps)
STEER_SIGN = -1.0     # model steer joints are +CW from above; controller is +CCW (see docstring)

# ---- STAGE 4 layout / control-plane constants ----
# SEAM (seed-driven placement): these are only NOMINAL fallbacks. The corridor scenes now vary the
# robot start + goal per seed (base.SceneBuilder._place_start_goal) and EXPOSE them as
# builder.start_xy / builder.goal_xy. do_build() already reads builder.start_pose()/goal_pose() to
# spawn the robot + place the GoalMarker, but the non-elevator at_goal check below still compares
# against the constant GOAL_XY -> to honor per-seed goals, that branch should read the builder:
#     gx, gy = SC["builder"].goal_xy        # instead of: gx, gy = GOAL_XY
# (Elevator scenes already override goal_pose and are handled by the is_elev branch.) Left as a
# clean one-line seam per the task; wiring it live is a follow-up when per-seed goals are enabled.
START_XY = (-0.9, 0.0)      # scene_gen/assemble.py layout: robot start, facing +X (nominal fallback)
GOAL_XY = (3.0, 0.0)        # goal beyond the blockage (nominal fallback; see SEAM note above)
AT_GOAL_TOL = 0.4           # at_goal = base_link within this XY radius of the goal
STATE_EVERY_N = 12          # 60 Hz sim / 12 -> ~5 Hz /scene/state
IMG_W, IMG_H = 1280, 720    # D455 color render product (contract)
REACH_STANDOFF = 0.06       # EE standoff before the goal point along the press axis (m)
REACH_ADVANCE = 0.04        # press advance from the standoff (m) -> stops 2 cm short of the point
PRESS_OK_TOL = 0.05         # flange within this of the commanded press point = mechanical press ok
# CLOSED-LOOP PRESS (the fix for "arm=miss with a perfect IK solution"). The arm phases below are
# TIME-based open-loop ramps: interpolate the joint command to q2 over 1.0 s, hold 0.5 s, then measure
# the flange. But a position-driven 6-DOF arm does not track that ramp instantly — it lags under
# gravity — so the measurement was taken while the arm was still rising and the press was booked as a
# miss even though the target was reachable and the IK exact.
#   Measured, M1_door__01 / `ours`, 3 runs: `ik_press(ok=True pe=0.0000)` on EVERY attempt, yet
#   err=0.50 / 0.31 / 0.27 vs err=0.015 on the one that passed. The residual was almost PURE Z and
#   always LOW (cmd z=4.102 -> measured 3.792 / 3.829 vs 4.090 on the pass), from near-identical base
#   offsets (base x 0.532 vs 0.544) — the signature of an unconverged joint trajectory, not of a
#   geometry, reach or grounding problem.
# So the dwell now HOLDS the final command and waits for the measured joints to actually arrive before
# scoring. On timeout it still scores honestly from the measured pose, so a genuinely unreachable or
# obstructed press still fails — it just fails after a fair wait instead of being timed out mid-motion.
SETTLE_Q_TOL = 0.02         # rad: per-joint |q_meas - q_cmd| that counts as "the arm arrived"
SETTLE_MAX_S = 6.0          # bounded wait for convergence (sim seconds) before scoring anyway
PRESS_ACT_TOL = 0.12        # goal point within this of a gt button's SURFACE (world AABB) -> act() coupling
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
# ARM GAINS — must be consistent with the joints' REAL torque budget (xArm6: 50/50/32/32/32/20 N*m).
# A PD position drive applies tau = kp*e - kd*qdot, and PhysX then CLIPS that to the joint's max
# effort. The old gains (kp 2.0e4, kd 2.0e3) were ~32x / ~2000x the values authored in the USD
# (625 / 1.0) and wildly outside that budget: at kd=2.0e3 the arm moving at a mere 0.025 rad/s spends
# the ENTIRE 50 N*m on the damping term alone, so the drive saturates and the arm can only creep.
# Measured consequence: presses whose IK solved exactly still ended 0.7-1.4 rad from the commanded
# joints after holding the target 6 s, and successive retries crept closer (flange err 0.50 -> 0.31 ->
# 0.27 m) because each retry resumed where the creep stopped.
# The binding constraint turned out to be DAMPING, not stiffness and not the torque cap. Measured at
# the stalled press with the REAL caps in force: j2 = -31.9 N*m of 50, j3 = -15.1 of 32 -- NOTHING
# saturated, yet j2/j3 held a standing error of 0.124 / 0.199 rad for 6 s. That is a static
# equilibrium: the drive was too SOFT to hold the arm against gravity, so it sagged. Meanwhile the old
# kd=2.0e3 was what made the arm creep (at 0.025 rad/s it alone consumes the whole 50 N*m).
# So: keep the stiffness high enough to hold the pose, and the damping LOW enough that motion is not
# strangled -- with the real torque caps left in place, which PhysX clips to during transients.
ARM_KP, ARM_KD = 2.0e4, 40.0
# Torque-cap override, OFF by default: keep the joints' REAL xArm6 limits.
# History (do not undo without reading): the press used to miss with an exactly-solved IK, and raising
# this to 5.0e3 "fixed" it — but that made the sim arm ~100x stronger than the hardware, which would
# invalidate any real-world-validity claim about actuation. The audit that followed found the true
# cause was the GAINS above, not the caps: the arm's link masses are the real xArm6 URDF values
# (12.05 kg total, verified from the USD) and the caps are the real specs, while kp/kd were ~32x/~2000x
# the authored values and could not be met within that torque budget. With the gains sized to the
# budget the press converges at the REAL limits, so this stays 0 (= leave as authored). Set
# UTP_ARM_MAX_EFFORT to a value in N*m only as a deliberate, documented fidelity tradeoff.
ARM_MAX_EFFORT = float(os.environ.get("UTP_ARM_MAX_EFFORT", "0") or 0)


def _find_prims(stage):
    """Locate base_link, the RTX lidar, and the D455 color camera regardless of exact ref path."""
    base_link = lidar = camera = None
    for p in stage.Traverse():
        sp = str(p.GetPath()); low = sp.lower()
        if base_link is None and low.endswith("/base_link"):
            base_link = sp
        if lidar is None and str(p.GetTypeName()) == "OmniLidar":
            lidar = sp
        if camera is None and "mast_cam" in low and "color" in low and p.GetTypeName() == "Camera":
            camera = sp
    return base_link, lidar, camera


def _select_mode(nav_mode: str, vx: float, vy: float, wz: float, r_char: float):
    """Map a raw twist to (controller_mode, vx, vy, wz) honoring the mode's constraints.

    nav_mode 'spin_traverse' (config/robot.yaml): rotation in SPIN, translation in TRAVERSE, and
    — crucially — a MIXED twist (forward + yaw at once, which is what Regulated Pure Pursuit emits
    on every arc) in ACKERMANN, so the base follows the curve instead of throwing away a component.
    The old policy zeroed either vx or wz on any mixed twist (spin XOR traverse), which made RPP's
    arcs undriveable: the base drove dead-straight ignoring the steer correction, then stopped to
    spin — the stop-straight-spin staircase that veered off path and clipped walls. Now:
        pure rotation (|v|~0, wz!=0) -> spin      (in-place; serves RPP use_rotate_to_heading)
        pure translation (|wz|~0)    -> traverse   (strafe / straight)
        strafe + yaw                 -> omni      (general 4WIS; see below)
        forward + yaw (no strafe)    -> ackermann  (car-like arc)
    nav_mode 'ackermann' passes (vx, 0, wz) car-like. Returns None when the cmd is ~zero -> stop.

    THE STRAFE+YAW CASE (why 'omni' exists). Nav2's MPPI controller runs motion_model "Omni" and
    emits vx, vy AND wz in the SAME twist. Sending that to ackermann DROPS vy, and when vx ~ 0 —
    which is exactly what MPPI commands while centring itself for a doorway or closing the last
    metre to an in-room goal — ackermann's own |vx| < eps branch then yields zero wheel speed too,
    so the whole command becomes a NO-OP: the wheels steer and the base does not move. Measured
    live on M0_open__01 (passive): the base sat 0.92 m short of a free, fully-plannable goal for
    the entire nav budget under cmd=(+0.00,-0.10,+0.09), wiggling. Since the Ranger is
    4-wheel-INDEPENDENT-steer it can physically realise any planar twist, so the honest fix is to
    realise the command instead of discarding part of it (docs/PROJECT_HANDOFF.md 8B).
    """
    lin = math.hypot(vx, vy)
    if lin < CMD_EPS and abs(wz) < CMD_EPS:
        return None
    if nav_mode == "ackermann":
        return ("ackermann", vx, 0.0, wz)
    # spin_traverse, arc-preserving: route by what the twist actually is (never zero the yaw on
    # a moving command — that is what broke path-following).
    if lin < CMD_EPS:
        return ("spin", 0.0, 0.0, wz)        # in-place rotate (rotate_to_heading)
    if abs(wz) < CMD_EPS:
        return ("traverse", vx, vy, 0.0)     # pure translation
    if abs(vy) >= CMD_EPS:
        return ("omni", vx, vy, wz)          # strafe + yaw: realise it all (general 4WIS)
    return ("ackermann", vx, 0.0, wz)        # forward + yaw = an arc


def _quat_wxyz_to_R(q):
    """3x3 rotation from a (w,x,y,z) quaternion (numpy-free caller passes np in)."""
    import numpy as np
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def at_goal_office(x, y, z, gx, gy, start_floor, target_floor, tol, floor_z_fn, floor_tol=0.6):
    """Floor-GATED at_goal predicate for office_usd (spawn_absolute) missions — PURE (Isaac-free).

    ROOT CAUSE (BUG #2, passive-baseline guardrail violation): a PLANAR-only at_goal
    (``math.hypot(x-gx, y-gy) <= tol``) lets a CROSS-FLOOR mission (M5 elevator / M7 chained, i.e.
    target_floor != start_floor) FALSELY trip at_goal. The goal's XY on the DESTINATION floor
    projects onto the SAME XY on the START floor, so a robot that reaches that XY WITHOUT riding the
    elevator satisfies the planar check on the wrong floor -> reached_goal -> passive "succeeds",
    violating docs/experiment_plan.md ("passive must fail every interaction tier").

    Fix: a cross-floor mission additionally requires the robot's world ``z`` to be on the TARGET
    floor (``|z - floor_z_fn(target_floor)| < floor_tol``). Single-floor office missions
    (target_floor == start_floor) are planar-only and therefore UNAFFECTED.

    ``floor_z_fn(floor) -> world z of that floor's spawn/walkable surface`` (office_usd_map.spawn_z);
    ``floor_tol`` mirrors OfficeUsdScene.on_target_floor's 0.6 m tolerance (floors are 3 m apart).
    """
    planar = math.hypot(x - gx, y - gy) <= tol
    if int(target_floor) == int(start_floor):
        return planar
    return planar and abs(z - float(floor_z_fn(int(target_floor)))) < floor_tol


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=900.0)
    ap.add_argument("--gui", action="store_true",
                    help="open the Isaac window (watch the trial). Run on the DISPLAY gpu only.")
    ap.add_argument("--open-usd", default=None,
                    help="KEYSTONE (office_usd): open this USD as the stage and bind to its BAKED "
                         "robot (office_building.usd is a complete world), instead of building the "
                         "procedural robot+demo-walls. scene_type then = a mission id.")
    ap.add_argument("--build", default=None,
                    help="auto-build this scene_type/mission-id once at startup (GUI/debug "
                         "convenience; no external ROS client needed to see the spawn).")
    ap.add_argument("--build-seed", type=int, default=0, help="seed for --build.")
    ap.add_argument("--preview", action="store_true",
                    help="Option B kinematic preview: the robot is held collision-free and moved only "
                         "by `glide` commands (pure visual path preview; never penetrates the car/"
                         "doors or falls). No physics drive-through in this mode.")
    args = ap.parse_args(argv)

    from isaacsim.simulation_app import SimulationApp
    app = SimulationApp({"headless": not args.gui})
    if args.gui:
        # Match the proven office GUI recipe (drive_office.sh): raise the RTX descriptor heap so the
        # 2nd (D455) render product/viewport is stable, and disable multiGpu (avoids the IOMMU P2P
        # hang). Best-effort — set immediately after app init, before any render product is created.
        try:
            import carb
            _s = carb.settings.get_settings()
            _s.set("/rtx/descriptorSets", 1440000)
            _s.set("/rtx/reservedDescriptors", 3600000)
            _s.set("/renderer/multiGpu/enabled", False)
        except Exception as _e:
            print(f"[srv] WARN gui rtx-stability settings failed: {_e!r}", flush=True)
    try:
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.ros2.bridge")
        app.update()

        import numpy as np, omni.usd
        import omni.graph.core as og
        import omni.replicator.core as rep
        from isaacsim.core.api import World
        from pxr import Usd, UsdGeom, UsdLux, Gf

        from pxr import Vt as _Vt
        OFFICE_USD = args.open_usd
        if OFFICE_USD:
            # KEYSTONE office_usd: open office_building.usd AS the stage (a complete world with the
            # baked Ranger+xArm6 robot, D455/lidar, floors, elevators, lights). No procedural robot,
            # no demo walls, no default ground plane — the building already has them, and the baked
            # /World/Robot IS the same asset trial_server binds below (base_link/link6/wheels/camera).
            omni.usd.get_context().open_stage(str(OFFICE_USD))
            for _ in range(10):
                app.update()
            world = World(stage_units_in_meters=1.0)   # attach to the now-open stage
            stage = omni.usd.get_context().get_stage()
            DEMO_WALLS = []
            # bind `robot` to the BAKED /World/Robot (Define on an existing prim just returns it, so
            # the file's reference arc + baked xformOps are preserved).
            robot = UsdGeom.Xform.Define(stage, "/World/Robot")
            # STABILITY FIX: the baked robot has a NESTED rigid body (mast_cam/RSD455 camera under
            # base_link, itself a rigid body) — PhysX warns "multiple RigidBodyAPI's in a hierarchy ->
            # unpredicted results", which makes the base jitter/explode when driven. A mounted camera
            # must be a FIXED child of base_link, not a dynamic body, so disable any rigid body that is
            # nested under another rigid body (non-destructive: authored into the session layer).
            from pxr import UsdPhysics as _UsdPhysics
            rroot = stage.GetPrimAtPath("/World/Robot")
            _rbs = [str(p.GetPath()) for p in Usd.PrimRange(rroot)
                    if p.HasAPI(_UsdPhysics.RigidBodyAPI)]
            _nested = [b for b in _rbs if any(b != o and b.startswith(o + "/") for o in _rbs)]
            for bp in _nested:
                rb = _UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(bp))
                (rb.GetRigidBodyEnabledAttr() or rb.CreateRigidBodyEnabledAttr()).Set(False)
                print(f"[srv] office_usd STABILITY: disabled nested rigid body {bp}", flush=True)
            if args.preview:
                # Option B: the robot is a pure KINEMATIC visual (moved only by glide) — disable ALL
                # its colliders so a glide through the doorway / up the shaft never penetrates the
                # dynamic door-leaf / elevator car (that penetration exploded physics -> nan).
                _n_off = 0
                for _pp in Usd.PrimRange(rroot):
                    if _pp.HasAPI(_UsdPhysics.CollisionAPI):
                        ca = _UsdPhysics.CollisionAPI(_pp)
                        (ca.GetCollisionEnabledAttr() or ca.CreateCollisionEnabledAttr()).Set(False)
                        _n_off += 1
                print(f"[srv] office_usd PREVIEW: robot collision disabled ({_n_off} colliders)",
                      flush=True)
            # FLOOR FIX: the F0 collision floor (BaseFloor, top z=-0.05) sits 5 cm BELOW the VISUAL
            # floor (the room plates at z=0, which carry NO colliders), so the robot's 4WS wheels rest
            # 5 cm INTO the visual floor and contact unevenly -> looks sunk, spawns crooked, and
            # steering the penetrated wheels flings the rig on every turn. Raise BaseFloor so its top
            # aligns with the visual floor at z=0 (session-layer override, non-destructive).
            _bf = stage.GetPrimAtPath("/World/BaseFloor")
            if _bf and _bf.IsValid():
                _bx = UsdGeom.Xformable(_bf)
                _bt = next((o for o in _bx.GetOrderedXformOps()
                            if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
                if _bt is None:
                    _bt = _bx.AddTranslateOp()
                _tv = _bt.Get() or Gf.Vec3d(0, 0, 0)
                _bt.Set(Gf.Vec3d(_tv[0], _tv[1], _tv[2] + 0.05))
                print("[srv] office_usd FLOOR FIX: raised BaseFloor +0.05 (collision top -0.05 -> 0)",
                      flush=True)
            # WHEEL FIX: office_building.usd references the robot's DEFAULT prim but NOT the sibling
            # `/colliders` subtree in ranger_mini_v3_physics.usd, so the 4 wheel_links have rigid
            # bodies (8 kg each) but NO collision + there are NO physics materials (no friction). The
            # wheels therefore can't touch/grip the floor -> the robot slides on nothing, tips, won't
            # drive straight, and falls through. Re-create the wheel cylinder colliders (radius 0.09,
            # height 0.08, axis Z, local z=-0.005 — from the payload) on the wheel rigid bodies + a
            # high-friction physics material bound to wheels and floor (session-layer, non-destructive).
            from pxr import UsdShade as _UsdShade
            _mat = _UsdShade.Material.Define(stage, "/World/Robot/PhysMat_wheel")
            _pm = _UsdPhysics.MaterialAPI.Apply(_mat.GetPrim())
            _pm.CreateStaticFrictionAttr(1.2)
            _pm.CreateDynamicFrictionAttr(1.0)
            _pm.CreateRestitutionAttr(0.0)
            _nwheel = 0
            for _w in ("fl", "fr", "rl", "rr"):
                _wl = stage.GetPrimAtPath(f"/World/Robot/ranger/{_w}_wheel_link")
                if not (_wl and _wl.IsValid()):
                    continue
                _cp = f"/World/Robot/ranger/{_w}_wheel_link/wheel_collider"
                # SPHERE (r=0.10, hub-centered): tip-STABLE for this high-CoM robot (the arm raises the
                # CoM; a cylinder's line contact + steering created a lateral force that tipped it near
                # sharp maneuvers). Orientation-independent so steering never lifts it off the floor.
                # Slight lateral drift vs a real wheel is corrected by Nav2's closed-loop arc control.
                _sph = UsdGeom.Sphere.Define(stage, _cp)
                _sph.CreateRadiusAttr(0.10)
                # Reuse an existing translate op if the prim already has one (Define returns the
                # existing prim when the USD/a prior build already created it) — AddTranslateOp throws
                # "xformOp:translate already exists" otherwise, crashing the whole server on re-open.
                _xf = UsdGeom.Xformable(_sph)
                _tops = [op for op in _xf.GetOrderedXformOps()
                         if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
                (_tops[0] if _tops else _xf.AddTranslateOp()).Set(Gf.Vec3d(0.0, 0.0, 0.0))
                UsdGeom.Imageable(_sph.GetPrim()).CreateVisibilityAttr("invisible")
                _UsdPhysics.CollisionAPI.Apply(_sph.GetPrim())
                _UsdShade.MaterialBindingAPI.Apply(_sph.GetPrim())
                _UsdShade.MaterialBindingAPI(_sph.GetPrim()).Bind(
                    _mat, _UsdShade.Tokens.weakerThanDescendants, "physics")
                _nwheel += 1
            # also bind the friction material to the (now-raised) BaseFloor collider
            if _bf and _bf.IsValid():
                _UsdShade.MaterialBindingAPI.Apply(_bf)
                _UsdShade.MaterialBindingAPI(_bf).Bind(_mat, _UsdShade.Tokens.weakerThanDescendants,
                                                       "physics")
            print(f"[srv] office_usd WHEEL FIX: added {_nwheel} wheel cylinder colliders + friction "
                  f"material (was: rigid bodies with NO collision/friction)", flush=True)
            # SEALED-DOOR FIX: a `sealed` door was PHYSICALLY PASSABLE — nothing stopped the robot.
            # The building models openable doors as an animated `func_Door_*` rig (leaf + collider)
            # and sets their `/Doors/Door_K` cube `invisible` as a placeholder. A sealed door never
            # animates, so it kept the STATIC cube — visible, correctly sized (full 1.2 m opening,
            # full height, 6 cm thick) — but `CollisionAPI` was never applied to it. Until now the
            # occupancy map filled sealed openings as solid wall, so the map was the ONLY thing
            # sealing them; the moment the map stopped doing that (doors are now plannable so the
            # agent must APPROACH and DISCOVER them), nothing did. MEASURED: the robot drove to
            # within 0.08 m of F0 Door_6 and ended at (2.65, 5.06) — inside the doorway span, in the
            # wall plane, collided=False.
            #
            # This matters for the M4/M6b restraint tier, whose whole premise is that the goal is
            # UNREACHABLE. A refusal is only worth measuring if the agent EARNS it by driving up and
            # finding no usable control — and only if the door genuinely does not open. A doorway it
            # can walk through makes `goal_reachable: false` a false statement.
            #
            # The prim already exists and is already visible to the camera/VLM; it just needs to be
            # solid. Applied on the SESSION layer (like the fixes above), never to the canonical USD.
            # Driven off the descriptor so any future sealed door is covered automatically.
            try:
                from isaac_worker.scene_gen import office_usd_map as _Mseal
                _sealed = [d for d in _Mseal.load_descriptor().get("doors", [])
                           if d.get("kind") == "sealed"]
                _nseal = 0
                for _d in _sealed:
                    _sp = stage.GetPrimAtPath(f"/World/Floor_{_d['floor']}/Doors/{_d['usd_name']}")
                    if not (_sp and _sp.IsValid()):
                        print(f"[srv] WARN sealed door prim missing: Floor_{_d['floor']}/Doors/"
                              f"{_d['usd_name']} — that opening stays PASSABLE", flush=True)
                        continue
                    if not _sp.HasAPI(_UsdPhysics.CollisionAPI):
                        _UsdPhysics.CollisionAPI.Apply(_sp)
                        _nseal += 1
                print(f"[srv] office_usd SEALED-DOOR FIX: {_nseal} sealed door(s) made solid "
                      f"(of {len(_sealed)} in the descriptor)", flush=True)
            except Exception as _e:
                print(f"[srv] WARN sealed-door collider fix failed: {_e!r} — sealed doors remain "
                      f"PASSABLE, so the M4/M6b restraint premise is INVALID this run", flush=True)
            # UPPER-FLOOR FLOOR FIX: Floor_1/Floor_2 have NO floor-slab collider (verified: the only
            # broad horizontal colliders in the whole building are BaseFloor@z0 and Roof@z9.2). A
            # physics-driven robot on an upper floor (after an elevator ride) stands on nothing and
            # falls to BaseFloor. Add an invisible BOX collider at each upper floor's surface z
            # (= the min-z of that floor's Walls, i.e. the wall base = walkable surface), spanning
            # the floor footprint. Box colliders are analytic + the most stable floor (session layer).
            try:
                _bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                         ["default", "render", "proxy", "guide"], useExtentsHint=True)
                for _fl in ("Floor_1", "Floor_2"):
                    _wp = stage.GetPrimAtPath(f"/World/{_fl}/Walls")
                    _fp = stage.GetPrimAtPath(f"/World/{_fl}")
                    if not (_wp and _wp.IsValid() and _fp and _fp.IsValid()):
                        continue
                    _surf = _bbc.ComputeWorldBound(_wp).ComputeAlignedRange().GetMin()[2]  # wall base = floor top
                    _fr = _bbc.ComputeWorldBound(_fp).ComputeAlignedRange()
                    _cx = (_fr.GetMin()[0] + _fr.GetMax()[0]) / 2.0
                    _cy = (_fr.GetMin()[1] + _fr.GetMax()[1]) / 2.0
                    _sx = (_fr.GetMax()[0] - _fr.GetMin()[0]) + 2.0
                    _sy = (_fr.GetMax()[1] - _fr.GetMin()[1]) + 2.0
                    # ELEVATOR SHAFT HOLES (found live 2026-07-29): a SINGLE slab spanning the whole
                    # footprint also seals the elevator shafts, so the car — a dynamic rigid body with
                    # real colliders — slams into this invisible floor and CANNOT RISE past it.
                    # Measured: car_floor stayed 0 no matter what the lift drive target was set to, so
                    # no ride could ever happen (and neither could parking the car on another floor).
                    # A USD Cube cannot have a hole, so tile the footprint with boxes that EXCLUDE
                    # every shaft: split the rect along all shaft edges and keep only the cells that
                    # do not overlap a shaft. The robot still has a floor everywhere it can walk; the
                    # shafts are open exactly where the cars travel.
                    _shafts = []
                    try:
                        from isaac_worker.scene_gen import office_usd_map as _M
                        _pad = 0.06          # keep the car from scraping the hole's rim
                        for _e in _M.load_descriptor().get("elevators", []):
                            _a = _e["car_aabb"]
                            _shafts.append((_a[0] - _pad, _a[1] - _pad, _a[3] + _pad, _a[4] + _pad))
                    except Exception as _e2:
                        print(f"[srv] WARN shaft holes unavailable ({_e2!r}) — upper-floor slab will "
                              f"be SOLID and the elevator car cannot move", flush=True)
                    _x0, _y0 = _cx - _sx / 2.0, _cy - _sy / 2.0
                    _x1, _y1 = _cx + _sx / 2.0, _cy + _sy / 2.0
                    _xe = sorted({_x0, _x1} | {v for s in _shafts for v in (s[0], s[2])
                                               if _x0 < v < _x1})
                    _ye = sorted({_y0, _y1} | {v for s in _shafts for v in (s[1], s[3])
                                               if _y0 < v < _y1})
                    _n = 0
                    for _i in range(len(_xe) - 1):
                        for _k in range(len(_ye) - 1):
                            _ax0, _ax1 = _xe[_i], _xe[_i + 1]
                            _ay0, _ay1 = _ye[_k], _ye[_k + 1]
                            if _ax1 - _ax0 < 1e-6 or _ay1 - _ay0 < 1e-6:
                                continue
                            _mx, _my = (_ax0 + _ax1) / 2.0, (_ay0 + _ay1) / 2.0
                            if any(s[0] < _mx < s[2] and s[1] < _my < s[3] for s in _shafts):
                                continue                      # this cell IS a shaft -> leave it open
                            _slab = UsdGeom.Cube.Define(stage, f"/World/{_fl}_CollisionFloor_{_n}")
                            _slab.CreateSizeAttr(1.0)
                            _sxf = UsdGeom.Xformable(_slab.GetPrim())
                            _sxf.AddTranslateOp().Set(Gf.Vec3d(_mx, _my, _surf - 0.05))  # top at _surf
                            _sxf.AddScaleOp().Set(Gf.Vec3f(_ax1 - _ax0, _ay1 - _ay0, 0.10))
                            UsdGeom.Imageable(_slab.GetPrim()).CreateVisibilityAttr("invisible")
                            _UsdPhysics.CollisionAPI.Apply(_slab.GetPrim())
                            _UsdShade.MaterialBindingAPI.Apply(_slab.GetPrim())
                            _UsdShade.MaterialBindingAPI(_slab.GetPrim()).Bind(
                                _mat, _UsdShade.Tokens.weakerThanDescendants, "physics")
                            _n += 1
                    print(f"[srv] office_usd UPPER-FLOOR FIX: {_fl} collision floor at z={_surf:.2f} "
                          f"({_sx:.0f}x{_sy:.0f} m) as {_n} boxes, {len(_shafts)} shaft(s) left OPEN",
                          flush=True)
            except Exception as _e:
                print(f"[srv] WARN upper-floor collision fix failed: {_e!r}", flush=True)
            # PHYSICS STABILITY (anti-shake / anti-tip): the robot is top-heavy (xArm6 mounted high) on
            # hub-centered SPHERE wheels -> 4 point contacts under a high CoM = it rocks (shakes) and
            # tips in sharp maneuvers. The sphere<->cylinder wheel flip-flop never fixed this because
            # the CoM is the real cause. So: (1) pull the CoM down by making base_link a heavy, low
            # chassis that dominates the arm's high mass; (2) turn on PhysX contact stabilization +
            # more velocity-solver iterations; (3) enable CCD (anti fall-through-on-tunnel). Session layer.
            try:
                _ps = stage.GetPrimAtPath("/World/physicsScene")
                if _ps and _ps.IsValid():
                    _a = _ps.GetAttribute("physxScene:enableCCD");           _a and _a.Set(True)
                    _a = _ps.GetAttribute("physxScene:enableStabilization"); _a and _a.Set(True)
                    print("[srv] office_usd PHYSICS: CCD + contact stabilization ON", flush=True)
                _bl = stage.GetPrimAtPath("/World/Robot/ranger/base_link")
                if _bl and _bl.IsValid():
                    _m = _UsdPhysics.MassAPI.Apply(_bl)
                    _m.CreateMassAttr(80.0)                               # heavy chassis dominates the CoM
                    _m.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, -0.22))  # pull CoM down toward the axle
                    _a = _bl.GetAttribute("physxArticulation:solverVelocityIterationCount"); _a and _a.Set(16)
                    print("[srv] office_usd PHYSICS: base_link CoM z=-0.22, mass=80, velIter=16", flush=True)
            except Exception as _e:
                print(f"[srv] WARN physics stability fix failed: {_e!r}", flush=True)
            print(f"[srv] office_usd: opened {OFFICE_USD} (baked robot + building)", flush=True)
        else:
            world = World(stage_units_in_meters=1.0)
            world.scene.add_default_ground_plane()
            stage = omni.usd.get_context().get_stage()
            UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1000.0)
            key = UsdLux.DistantLight.Define(stage, "/World/Key")
            key.CreateIntensityAttr(2600.0)
            UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))

            # robot articulation at origin
            robot = UsdGeom.Xform.Define(stage, "/World/Robot")
            robot.GetPrim().GetReferences().AddReference(str(ROBOT_USD))
            # demo walls around the robot so the 2D lidar has returns BEFORE any scene is built
            # (empty world -> no /scan hits). Torn down by the first /scene/command build.
            DEMO_WALLS = []
            for name, ctr, scl in [("N", (3, 0, 1.05), (0.1, 6, 2.1)), ("S", (-3, 0, 1.05), (0.1, 6, 2.1)),
                                   ("E", (0, 3, 1.05), (6, 0.1, 2.1)), ("W", (0, -3, 1.05), (6, 0.1, 2.1))]:
                c = UsdGeom.Cube.Define(stage, f"/World/Wall{name}"); c.CreateSizeAttr(1.0)
                UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*ctr))
                UsdGeom.Xformable(c.GetPrim()).AddScaleOp().Set(Gf.Vec3f(*scl))
                c.CreateDisplayColorAttr(_Vt.Vec3fArray([(0.8, 0.8, 0.82)]))
                DEMO_WALLS.append(f"/World/Wall{name}")
        for _ in range(10):
            app.update()

        base_link, lidar, camera = _find_prims(stage)
        print(f"[srv] base_link={base_link}\n[srv] lidar={lidar}\n[srv] camera={camera}", flush=True)
        if not base_link or not lidar or not camera:
            print("[SERVER_ERROR] could not find base_link/lidar/camera prims", flush=True); return

        # Apply the REAL RPLIDAR A1M8 profile to the OmniLidar prim itself (the referenced S2E asset
        # names its RTX-lidar config via omni:sensor:modelName -> lidar_configs/SLAMTEC/<name>.json;
        # RPLIDAR_A1M8.json is installed there). The /scan min/max range comes from THIS config, and
        # /scan is what feeds the Nav2 costmap — so without this the costmap uses S2E's 0.05/30 m
        # instead of the A1M8's 0.15/12 m. Must be on the OmniLidar prim (not the parent Xform) and
        # before the lidar render product is created below.
        try:
            from pxr import Sdf as _Sdf
            import yaml as _yl
            _lc = _yl.safe_load((REPO / "config" / "sensors.yaml").read_text())["lidar_front"]
            _lp = stage.GetPrimAtPath(lidar)
            _nmin, _nmax = [float(x) for x in _lc["range_m"]]

            def _seta(n, t, v):
                a = _lp.GetAttribute(n) or _lp.CreateAttribute(n, t)
                a.Set(v)
            _samples = int(_lc.get("samples", 360))
            _scan_hz = max(1, int(round(float(_lc.get("scan_rate_hz", 5.5)))))  # uint field -> 6 (~5.5)
            _report_hz = _scan_hz * _samples          # rays/rev = report/scan -> ~1 deg at 360 samples
            _seta("omni:sensor:modelName", _Sdf.ValueTypeNames.String, "RPLIDAR_A1M8")
            _seta("omni:sensor:Core:nearRangeM", _Sdf.ValueTypeNames.Float, _nmin)
            _seta("omni:sensor:Core:farRangeM", _Sdf.ValueTypeNames.Float, _nmax)
            _seta("omni:sensor:Core:scanRateBaseHz", _Sdf.ValueTypeNames.UInt, _scan_hz)
            _seta("omni:sensor:Core:reportRateBaseHz", _Sdf.ValueTypeNames.UInt, _report_hz)
            print(f"[srv] lidar profile -> RPLIDAR_A1M8, range=[{_nmin},{_nmax}] m, "
                  f"{_scan_hz} Hz, {_samples} rays/rev (~1 deg) — feeds Nav2 costmap", flush=True)
        except Exception as _e:  # noqa — non-fatal; log and continue
            print(f"[srv] WARN lidar A1M8 profile override failed: {_e}", flush=True)

        # arm chain anchor prims (for the reach IK frame math)
        link_base_path = link6_path = None
        for p in stage.Traverse():
            sp = str(p.GetPath())
            if sp.startswith("/World/Robot") and sp.endswith("/link_base") and link_base_path is None:
                link_base_path = sp
            if sp.startswith("/World/Robot") and sp.endswith("/link6") and link6_path is None:
                link6_path = sp
        print(f"[srv] link_base={link_base_path} link6={link6_path}", flush=True)
        if not link_base_path:
            print("[SERVER_ERROR] no arm link_base prim found", flush=True); return

        # ---- STAGE 3: 4WS controller + wheel/steer joints ----
        import yaml as _yaml
        from pxr import UsdPhysics
        from utp.control.ranger_4ws import Ranger4WSController, WHEELS

        robot_cfg = _yaml.safe_load((REPO / "config" / "robot.yaml").read_text())
        nav_mode = str(robot_cfg["base"]["nav_mode"])  # e.g. spin_traverse
        if OFFICE_USD:
            # office_usd keeps spin_traverse: _select_mode is now the ARC-PRESERVING hybrid (mixed
            # vx+wz -> ackermann arc for RPP path-following, pure rotation -> SPIN, pure translation ->
            # traverse). The old code forced pure ackermann here to stop spin_traverse zeroing the arc
            # yaw — but pure ackermann CANNOT rotate in place (vx~0 => 0 steer, 0 speed), which silently
            # broke every _drive_to square-up + the interaction approach/repositioning (the base could
            # never turn to face or line up on a press target). The hybrid gives arcs AND in-place spin.
            print(f"[srv] office_usd: nav_mode={nav_mode} (arc-preserving hybrid _select_mode)",
                  flush=True)
        ctrl4ws = Ranger4WSController(str(REPO / "config" / "ranger_kinematics.yaml"))
        r_char = math.hypot(ctrl4ws.wheelbase / 2.0, ctrl4ws.track / 2.0)
        print(f"[srv] 4ws nav_mode={nav_mode} wheel_radius={ctrl4ws.wheel_radius} r_char={r_char:.3f}",
              flush=True)

        # locate the steer/wheel joint prims (paths depend on the arranged-asset layout)
        joint_paths = {}
        for p in stage.Traverse():
            name = p.GetName()
            for w in WHEELS:
                if name == f"{w}_steering_joint":
                    joint_paths[f"{w}_steer"] = str(p.GetPath())
                elif name == f"{w}_wheel":
                    joint_paths[f"{w}_drive"] = str(p.GetPath())
        print(f"[srv] joint prims: {joint_paths}", flush=True)
        if len(joint_paths) != 8:
            print("[SERVER_ERROR] expected 8 steer/wheel joints, found "
                  f"{len(joint_paths)}", flush=True); return

        # drive gains BEFORE world.reset() so PhysX parses them (validated in
        # validation/drive_capture.py): steer = stiff POSITION drive, wheel = pure VELOCITY drive.
        def _set_drive(joint_path, stiffness, damping):
            drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(joint_path), "angular")
            drive.CreateStiffnessAttr(float(stiffness))
            drive.CreateDampingAttr(float(damping))
            drive.CreateMaxForceAttr(1.0e7)
        for w in WHEELS:
            # steer: softened from 1e5/1e4 -> 3e4/6e3. The very stiff position drive fought every small
            # ground perturbation and injected high-frequency jitter (shake); this still tracks the
            # commanded steer angle but absorbs contact noise instead of amplifying it.
            _set_drive(joint_paths[f"{w}_steer"], 3.0e4, 6.0e3)
            _set_drive(joint_paths[f"{w}_drive"], 0.0, 5.0e3)

        # the base articulation root (the arranged asset may also expose the arm's root)
        art_roots = [str(p.GetPath()) for p in stage.Traverse()
                     if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        print(f"[srv] articulation roots: {art_roots}", flush=True)
        base_roots = [r for r in art_roots if "ranger" in r.lower() or r == base_link]
        if not base_roots:
            print("[SERVER_ERROR] no base articulation root found", flush=True); return
        art_root = base_roots[0]

        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction

        # a render product for the RTX lidar (the LaserScan helper consumes it)
        lidar_rp = rep.create.render_product(lidar, [1, 1], name="scan_rp")
        # D455: ONE render product on the COLOR camera at contract resolution; rgb + depth
        # (distance_to_image_plane -> 32FC1 meters) + camera_info all come off this RP.
        # Near clip 0.4 m = D455 depth min (config/sensors.yaml): without it the RSD455 housing
        # 1.3 cm in front of the lens renders as a huge black occluder (verified from a frame).
        UsdGeom.Camera(stage.GetPrimAtPath(camera)).GetClippingRangeAttr().Set(Gf.Vec2f(0.4, 1000.0))
        cam_rp = rep.create.render_product(camera, [IMG_W, IMG_H], name="mast_cam_rp")

        # optional THIRD-PERSON chase camera for recording a demo (UTP_CHASE_CAM=1): a fixed camera set
        # behind the spawn, above, looking toward the door, published on /chase_cam/color. Off by default.
        chase_rp = None
        if os.environ.get("UTP_CHASE_CAM") and OFFICE_USD:
            _cc = UsdGeom.Camera.Define(stage, "/World/ChaseCam")
            _cc.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 1000.0))
            _view = Gf.Matrix4d(1.0)
            _view.SetLookAt(Gf.Vec3d(5.8, 6.7, 2.1), Gf.Vec3d(16.4, 5.5, 1.05), Gf.Vec3d(0, 0, 1))
            _cxf = UsdGeom.Xformable(_cc.GetPrim())
            _cxf.ClearXformOpOrder(); _cxf.AddTransformOp().Set(_view.GetInverse())
            chase_rp = rep.create.render_product("/World/ChaseCam", [IMG_W, IMG_H], name="chase_rp")
            print("[srv] CHASE CAM enabled -> /chase_cam/color/image_raw", flush=True)

        # ---- ROS2 bridge ActionGraph: odom + tf + scan + camera + cmd_vel ----
        og.Controller.edit(
            {"graph_path": "/TrialGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("Odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ("PubOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("PubTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ("RtxLidar", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                    ("CamRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CamDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("CamInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                    ("Context.outputs:context", "SubTwist.inputs:context"),
                    ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
                    ("Context.outputs:context", "PubClock.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                    ("OnTick.outputs:tick", "Odom.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubOdom.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubOdomTF.inputs:execIn"),
                    ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
                    ("Context.outputs:context", "PubOdom.inputs:context"),
                    ("Context.outputs:context", "PubOdomTF.inputs:context"),
                    ("Context.outputs:context", "PubTF.inputs:context"),
                    ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
                    ("SimTime.outputs:simulationTime", "PubOdomTF.inputs:timeStamp"),
                    ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
                    ("Odom.outputs:linearVelocity", "PubOdom.inputs:linearVelocity"),
                    ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
                    ("Odom.outputs:position", "PubOdom.inputs:position"),
                    ("Odom.outputs:orientation", "PubOdom.inputs:orientation"),
                    ("Odom.outputs:position", "PubOdomTF.inputs:translation"),
                    ("Odom.outputs:orientation", "PubOdomTF.inputs:rotation"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PubClock.inputs:topicName", "clock"),
                    ("SubTwist.inputs:topicName", "cmd_vel"),
                    ("Odom.inputs:chassisPrim", [base_link]),
                    ("PubOdom.inputs:odomFrameId", "odom"),
                    ("PubOdom.inputs:chassisFrameId", "base_link"),
                    ("PubOdom.inputs:topicName", "odom"),
                    ("PubOdomTF.inputs:parentFrameId", "odom"),
                    ("PubOdomTF.inputs:childFrameId", "base_link"),
                    ("PubTF.inputs:parentPrim", [base_link]),
                    ("PubTF.inputs:targetPrims", [p for p in (lidar, camera) if p]),
                    ("RtxLidar.inputs:renderProductPath", lidar_rp.path),
                    ("RtxLidar.inputs:topicName", "scan"),
                    ("RtxLidar.inputs:frameId", "lidar_link"),
                    ("RtxLidar.inputs:type", "laser_scan"),
                    ("CamRGB.inputs:renderProductPath", cam_rp.path),
                    ("CamRGB.inputs:type", "rgb"),
                    ("CamRGB.inputs:topicName", "/mast_cam/color/image_raw"),
                    ("CamRGB.inputs:frameId", "mast_cam_optical"),
                    ("CamDepth.inputs:renderProductPath", cam_rp.path),
                    ("CamDepth.inputs:type", "depth"),
                    ("CamDepth.inputs:topicName", "/mast_cam/depth/image_rect_raw"),
                    ("CamDepth.inputs:frameId", "mast_cam_optical"),
                    ("CamInfo.inputs:renderProductPath", cam_rp.path),
                    ("CamInfo.inputs:topicName", "/mast_cam/color/camera_info"),
                    ("CamInfo.inputs:frameId", "mast_cam_optical"),
                ],
            },
        )
        # Helper nodes wire their own context internally; tick them too
        for helper in ("RtxLidar", "CamRGB", "CamDepth", "CamInfo"):
            og.Controller.connect("/TrialGraph/OnTick.outputs:tick", f"/TrialGraph/{helper}.inputs:execIn")
        if chase_rp is not None:
            og.Controller.create_node("/TrialGraph/CamChase", "isaacsim.ros2.bridge.ROS2CameraHelper")
            og.Controller.attribute("/TrialGraph/CamChase.inputs:renderProductPath").set(chase_rp.path)
            og.Controller.attribute("/TrialGraph/CamChase.inputs:type").set("rgb")
            og.Controller.attribute("/TrialGraph/CamChase.inputs:topicName").set("/chase_cam/color/image_raw")
            og.Controller.attribute("/TrialGraph/CamChase.inputs:frameId").set("chase_optical")
            og.Controller.connect("/TrialGraph/OnTick.outputs:tick", "/TrialGraph/CamChase.inputs:execIn")
            print("[srv] CHASE CAM graph node wired (/chase_cam/color/image_raw)", flush=True)
        print("[srv] bridge graph built (odom/tf/scan/mast_cam rgb+depth+info + cmd_vel sub)", flush=True)

        # ---- STAGE 4 graph: generic control-plane pub/sub nodes ----
        # ROS2Subscriber/ROS2Publisher are the bridge's GENERIC message nodes (verified present in
        # this build: exts/isaacsim.ros2.bridge tests). Dynamic message-field attributes
        # (inputs:data / outputs:data / outputs:point:*) are created a few app updates after
        # messageName is set. /scene/state + /arm_reach/result publishers are gated by
        # OnImpulseEvent nodes so WE decide when a message goes out (state ~5 Hz, result on demand).
        og.Controller.edit(
            "/TrialGraph",
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("SubCmd", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("SubGoal", "isaacsim.ros2.bridge.ROS2Subscriber"),
                    ("PubState", "isaacsim.ros2.bridge.ROS2Publisher"),
                    ("PubResult", "isaacsim.ros2.bridge.ROS2Publisher"),
                    ("StateImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("ResultImpulse", "omni.graph.action.OnImpulseEvent"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("SubCmd.inputs:topicName", "/scene/command"),
                    ("SubCmd.inputs:messagePackage", "std_msgs"),
                    ("SubCmd.inputs:messageSubfolder", "msg"),
                    ("SubCmd.inputs:messageName", "String"),
                    ("SubGoal.inputs:topicName", "/arm_reach/goal"),
                    ("SubGoal.inputs:messagePackage", "geometry_msgs"),
                    ("SubGoal.inputs:messageSubfolder", "msg"),
                    ("SubGoal.inputs:messageName", "PointStamped"),
                    ("PubState.inputs:topicName", "/scene/state"),
                    ("PubState.inputs:messagePackage", "std_msgs"),
                    ("PubState.inputs:messageSubfolder", "msg"),
                    ("PubState.inputs:messageName", "String"),
                    ("PubResult.inputs:topicName", "/arm_reach/result"),
                    ("PubResult.inputs:messagePackage", "std_msgs"),
                    ("PubResult.inputs:messageSubfolder", "msg"),
                    ("PubResult.inputs:messageName", "Bool"),
                ],
            },
        )
        # cross-connections with ABSOLUTE paths (relative names only resolve for nodes created
        # in the SAME edit call — mixing old/new nodes in one CONNECT list fails to parse)
        for src, dst in [
            ("OnTick.outputs:tick", "SubCmd.inputs:execIn"),
            ("OnTick.outputs:tick", "SubGoal.inputs:execIn"),
            ("Context.outputs:context", "SubCmd.inputs:context"),
            ("Context.outputs:context", "SubGoal.inputs:context"),
            ("Context.outputs:context", "PubState.inputs:context"),
            ("Context.outputs:context", "PubResult.inputs:context"),
            ("StateImpulse.outputs:execOut", "PubState.inputs:execIn"),
            ("ResultImpulse.outputs:execOut", "PubResult.inputs:execIn"),
        ]:
            og.Controller.connect(f"/TrialGraph/{src}", f"/TrialGraph/{dst}")
        for _ in range(6):
            app.update()   # let the generic nodes create their dynamic message attributes

        def _attrs_of(node_name):
            return [a.get_name() for a in og.Controller.node(f"/TrialGraph/{node_name}").get_attributes()]
        for n in ("SubCmd", "SubGoal", "PubState", "PubResult"):
            print(f"[srv] {n} attrs: {_attrs_of(n)}", flush=True)

        def _attr(path):
            try:
                return og.Controller.attribute(path)
            except Exception:
                return None

        cmd_data_attr = _attr("/TrialGraph/SubCmd.outputs:data")
        state_data_attr = _attr("/TrialGraph/PubState.inputs:data")
        result_data_attr = _attr("/TrialGraph/PubResult.inputs:data")
        state_impulse = _attr("/TrialGraph/StateImpulse.state:enableImpulse")
        result_impulse = _attr("/TrialGraph/ResultImpulse.state:enableImpulse")
        if not all([cmd_data_attr, state_data_attr, result_data_attr, state_impulse, result_impulse]):
            print("[SERVER_ERROR] control-plane dynamic attributes missing "
                  f"(cmd={cmd_data_attr} state={state_data_attr} res={result_data_attr} "
                  f"imp={state_impulse}/{result_impulse})", flush=True); return

        goal_attr_names = _attrs_of("SubGoal")

        def read_goal_point():
            """PointStamped payload -> [x,y,z], tolerant of how the generic node maps fields."""
            if all(f"outputs:point:{c}" in goal_attr_names for c in "xyz"):
                return [float(og.Controller.attribute(f"/TrialGraph/SubGoal.outputs:point:{c}").get())
                        for c in "xyz"]
            if "outputs:point" in goal_attr_names:
                v = og.Controller.attribute("/TrialGraph/SubGoal.outputs:point").get()
                if isinstance(v, str):
                    j = json.loads(v)
                    return [float(j["x"]), float(j["y"]), float(j["z"])]
                arr = [float(x) for x in list(v)]
                if len(arr) >= 3:
                    return arr[:3]
            print("[srv] WARN cannot read /arm_reach/goal point from node attrs", flush=True)
            return None

        # message-arrival counters on the generic subscribers' execOut (fires per received msg)
        def _wire_counter(sub_name):
            try:
                sub_attrs = _attrs_of(sub_name)
                exec_out = next(n for n in sub_attrs if n.startswith("outputs:") and "exec" in n.lower())
                og.Controller.edit(
                    "/TrialGraph",
                    {og.Controller.Keys.CREATE_NODES: [(f"{sub_name}Count", "omni.graph.action.Counter")]})
                cnt_attrs = _attrs_of(f"{sub_name}Count")
                cnt_exec_in = next(n for n in cnt_attrs if n.startswith("inputs:") and "exec" in n.lower())
                cnt_out = next(n for n in cnt_attrs if n.startswith("outputs:") and "count" in n.lower())
                og.Controller.connect(f"/TrialGraph/{sub_name}.{exec_out}",
                                      f"/TrialGraph/{sub_name}Count.{cnt_exec_in}")
                print(f"[srv] {sub_name} counter wired ({exec_out} -> {cnt_exec_in})", flush=True)
                return og.Controller.attribute(f"/TrialGraph/{sub_name}Count.{cnt_out}")
            except Exception as e:
                print(f"[srv] WARN {sub_name} counter failed ({e!r})", flush=True)
                return None

        cmd_count_attr = _wire_counter("SubCmd")
        goal_count_attr = _wire_counter("SubGoal")
        twist_count_attr = _wire_counter("SubTwist")
        if twist_count_attr is None:
            print("[srv] WARN no cmd_vel counter; TIMEOUT DISABLED (zero-cmd stop still works)", flush=True)

        lin_attr = og.Controller.attribute("/TrialGraph/SubTwist.outputs:linearVelocity")
        ang_attr = og.Controller.attribute("/TrialGraph/SubTwist.outputs:angularVelocity")

        # ---- static relative transforms (USD, before play; column-vector numpy convention) ----
        xfc = UsdGeom.XformCache(Usd.TimeCode.Default())

        def _l2w(path):
            return np.array(xfc.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T

        W_base0 = _l2w(base_link)
        REL_CAM = np.linalg.inv(W_base0) @ _l2w(camera)       # base_link -> camera (rigid mast)
        REL_LB = np.linalg.inv(W_base0) @ _l2w(link_base_path)  # base_link -> arm link_base
        print(f"[srv] base_link z0={W_base0[2,3]:.4f} cam_rel_t={REL_CAM[:3,3].round(3).tolist()} "
              f"lb_rel_t={REL_LB[:3,3].round(3).tolist()}", flush=True)

        # camera intrinsics (must match what the CameraInfo helper publishes: fx=fy=634.086 for
        # the RSD455 color camera at 1280x720 — validation/logs/camera_info_echo.txt)
        camg = UsdGeom.Camera(stage.GetPrimAtPath(camera))
        focal = float(camg.GetFocalLengthAttr().Get())
        hap = float(camg.GetHorizontalApertureAttr().Get())
        vap = float(camg.GetVerticalApertureAttr().Get())
        FX = focal / hap * IMG_W
        FY_FROM_VAP = focal / vap * IMG_H if vap else FX
        CX, CY = IMG_W / 2.0, IMG_H / 2.0
        print(f"[srv] intrinsics: focal={focal:.3f} hap={hap:.3f} vap={vap:.3f} "
              f"-> fx={FX:.3f} (fy_from_vap={FY_FROM_VAP:.3f}; using fy=fx, matches CameraInfo)",
              flush=True)

        # ---- M1.1 IK (reused): FK chain + joint limits from the validated sweep ----
        from isaac_worker.robot.ik_sweep import fk as ik_fk, J_LIMITS

        def _clamp_q(q):
            return np.minimum(np.maximum(q, J_LIMITS[:, 0]), J_LIMITS[:, 1])

        def solve_ik_lb(target_lb, xdes_lb, q_seed=None, seeds=14, iters=150, lam=0.08,
                        pos_tol=0.006, ori_tol_deg=6.0):
            """DLS IK in the link_base frame; press axis = xdes_lb (roll about it left free)."""
            xdes = np.asarray(xdes_lb, dtype=float)
            xdes = xdes / (np.linalg.norm(xdes) or 1.0)
            tgt = np.asarray(target_lb, dtype=float)

            def err(q):
                p, R = ik_fk(q)
                return np.concatenate([tgt - p, np.cross(R[:, 0], xdes)])

            rng = np.random.default_rng(7)
            best = (False, None, 1e9, 1e9)
            for s in range(seeds):
                if s == 0 and q_seed is not None:
                    q = np.asarray(q_seed, dtype=float).copy()
                elif s <= 1:
                    q = np.zeros(6)
                else:
                    q = rng.uniform(J_LIMITS[:, 0].clip(-math.pi, math.pi),
                                    J_LIMITS[:, 1].clip(-math.pi, math.pi))
                for _ in range(iters):
                    e = err(q)
                    J = np.zeros((6, 6))
                    for k in range(6):
                        dq = q.copy(); dq[k] += 1e-6
                        J[:, k] = (err(dq) - e) / 1e-6
                    step = -J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(6), e)
                    q = _clamp_q(q + step)
                e = err(q)
                pe = float(np.linalg.norm(e[:3]))
                x_cur = ik_fk(q)[1][:, 0]
                oe = float(math.acos(max(-1.0, min(1.0, float(x_cur @ xdes)))))
                if pe + 0.1 * oe < best[2] + 0.1 * math.radians(best[3]):
                    best = (pe <= pos_tol and oe <= math.radians(ori_tol_deg), q.copy(),
                            pe, math.degrees(oe))
                if best[0]:
                    break
            return best

        # ---- articulation binding (startup + after every scene rebuild) ----
        import omni.timeline
        timeline = omni.timeline.get_timeline_interface()

        SB = {"art": None, "dof": [], "steer_idx": [], "wheel_idx": [], "arm_idx": [],
              "ndof": 0, "q_home": None, "bind_n": 0}

        def bind_articulation():
            SB["bind_n"] += 1
            art = SingleArticulation(prim_path=art_root, name=f"ranger_base_{SB['bind_n']}")
            art.initialize()
            dof = list(art.dof_names)
            steer_idx = [dof.index(f"{w}_steering_joint") for w in WHEELS]
            wheel_idx = [dof.index(f"{w}_wheel") for w in WHEELS]
            arm_idx = [dof.index(j) for j in ARM_JOINTS]
            ndof = art.num_dof
            try:
                actrl = art.get_articulation_controller()
                kps = np.zeros(ndof, dtype=np.float32); kds = np.zeros(ndof, dtype=np.float32)
                for i in steer_idx:
                    kps[i], kds[i] = 1.0e5, 1.0e4
                for i in wheel_idx:
                    kps[i], kds[i] = 0.0, 5.0e3
                for i in arm_idx:
                    kps[i], kds[i] = ARM_KP, ARM_KD
                actrl.set_gains(kps=kps, kds=kds)
                print(f"[srv] CTRL_SET_GAINS ok (bind {SB['bind_n']}: steer pos, wheel vel, arm pos)",
                      flush=True)
            except Exception as e:
                print(f"[srv] CTRL_SET_GAINS skipped: {e!r}", flush=True)
            # ---- ARM TORQUE HEADROOM (the "reachable, exact IK, still misses" failure) ----
            # Gains alone do not bound a position drive: PhysX ALSO caps the torque it may apply
            # (maxForce / max effort). A cap that cannot hold the arm at full extension makes the drive
            # SATURATE, so the joints stall part-way and never reach the commanded angles — which is
            # what the closed-loop press measured: after HOLDING the target 6 s, per-joint error stayed
            # 0.72-1.44 rad (41-83 deg) with the flange ~0.3 m low, on presses whose IK solved exactly
            # (`ik_press(ok=True pe=0.0000)`). Report the caps and raise the ARM joints only (steer and
            # wheel drives are already tuned) so the drive is not the limiting factor. Fully guarded and
            # purely additive: if the API differs, the caps stay as authored and behaviour is unchanged.
            # NOTE the API lives on the ArticulationController (`actrl`), not on the Articulation.
            try:
                holder = next((o for o in (actrl, art)
                               if hasattr(o, "get_max_efforts") and hasattr(o, "set_max_efforts")),
                              None)
                if holder is None or ARM_MAX_EFFORT <= 0:
                    cur = (np.asarray(holder.get_max_efforts(), dtype=float).reshape(-1)
                           if holder is not None else None)
                    shown = ([round(float(cur[i]), 1) for i in arm_idx] if cur is not None
                             else "unreadable")
                    print(f"[srv] ARM max_efforts: REAL limits kept {shown}", flush=True)
                else:
                    cur = np.asarray(holder.get_max_efforts(), dtype=float).reshape(-1)
                    print(f"[srv] ARM max_efforts BEFORE: "
                          f"{[round(float(cur[i]), 1) for i in arm_idx]}", flush=True)
                    new = cur.copy()
                    for i in arm_idx:
                        new[i] = max(float(new[i]), ARM_MAX_EFFORT)
                    holder.set_max_efforts(np.asarray(new, dtype=np.float32))
                    got = np.asarray(holder.get_max_efforts(), dtype=float).reshape(-1)
                    print(f"[srv] ARM max_efforts AFTER:  "
                          f"{[round(float(got[i]), 1) for i in arm_idx]}", flush=True)
            except Exception as e:
                print(f"[srv] ARM max_efforts NOT raised ({e!r}); left as authored", flush=True)
            q_home = np.array(art.get_joint_positions(), dtype=float)[arm_idx].copy()
            SB.update(art=art, dof=dof, steer_idx=steer_idx, wheel_idx=wheel_idx,
                      arm_idx=arm_idx, ndof=ndof, q_home=q_home)
            print(f"[srv] DOF_NAMES: {dof}", flush=True)
            print(f"[srv] arm q_home: {q_home.round(3).tolist()}", flush=True)

        def base_T():
            p, q = SB["art"].get_world_pose()
            T = np.eye(4)
            T[:3, :3] = _quat_wxyz_to_R(q)
            T[:3, 3] = np.asarray(p, dtype=float)
            return T

        # ---- scene manager ----
        from isaac_worker.scene_gen.scenes import build_scene
        # office_usd per-floor surface Z (Z-stacked building) — feeds the cross-floor at_goal gate.
        from isaac_worker.scene_gen.office_usd_map import spawn_z as office_spawn_z

        SC = {"builder": None, "scene_type": None, "seed": None, "corners": {}, "housing": {},
              "environment": None, "environments": None}

        # the referenced robot root already has a translate op (baked ground lift) — remember it
        rxf = UsdGeom.Xformable(robot.GetPrim())
        rtop = next((o for o in rxf.GetOrderedXformOps()
                     if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
        if rtop is None:
            rtop = rxf.AddTranslateOp()
        ROBOT_T0 = rtop.Get() or Gf.Vec3d(0, 0, 0)

        def _button_corners(prim_path):
            """8 world corners of the prim's world AABB (visible geometry only — the invisible
            collider child is excluded by BBoxCache's visibility handling)."""
            cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                      [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
            rng = cache.ComputeWorldBound(stage.GetPrimAtPath(prim_path)).ComputeAlignedRange()
            lo, hi = np.array(rng.GetMin()), np.array(rng.GetMax())
            return [np.array([x, y, z]) for x in (lo[0], hi[0])
                    for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]

        def project_px(pw):
            """World point -> (u, v) pixels through the live D455 color camera, or None."""
            Tc = base_T() @ REL_CAM
            pc = np.linalg.inv(Tc) @ np.array([pw[0], pw[1], pw[2], 1.0])
            zf = -pc[2]                      # USD camera looks down -Z
            if zf <= 1e-3:
                return None
            return (FX * pc[0] / zf + CX, CY - FX * pc[1] / zf)

        def bbox_px(corners):
            """Project the 8 AABB corners -> clamped [x0,y0,x1,y1] pixel bbox, or None."""
            if not corners:
                return None
            pts = [project_px(c) for c in corners]
            if any(p is None for p in pts):
                return None
            us = [p[0] for p in pts]; vs = [p[1] for p in pts]
            x0, x1 = max(0.0, min(us)), min(float(IMG_W), max(us))
            y0, y1 = max(0.0, min(vs)), min(float(IMG_H), max(vs))
            if x1 - x0 < 1.0 or y1 - y0 < 1.0:
                return None
            return [round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)]

        def do_build(scene_type, seed, environment=None, environments=None):
            GLIDE["active"] = False   # cancel any in-flight preview glide so it can't fight the respawn
            # reset the forensic per-trial fields so a new trial never carries the previous arm pose /
            # collision (FSM is a closure var defined below; late-bound, so this is valid at call time).
            FSM.update(ee_world=None, press_dist=None, press_ok=False, collided=False, n_contacts=0)
            # Real-environment cfg (real-env foundation): when the build command carries an
            # `environment` block ({env, anchor}) we hand build_scene a cfg dict pairing it with
            # the `environments` registry (env -> {env_usd, curate}) so base.py:build() takes its
            # full-env branch (anchors.get_anchor + env_curate.curate_env). When environment is
            # None we pass cfg=None so the procedural white-box path is byte-for-byte unchanged.
            cfg = ({"environment": environment, "environments": environments or {}}
                   if environment else None)
            # office_usd: pass the elevator variable + badge zones through (experimental axis). The
            # `environments` slot doubles as a generic office cfg carrier when opening a USD.
            if OFFICE_USD and isinstance(environments, dict):
                cfg = dict(environments)
            print(f"[srv] BUILD scene_type={scene_type} seed={seed} "
                  f"env={environment} ...", flush=True)
            timeline.stop()
            for _ in range(3):
                app.update()
            for p in DEMO_WALLS + ["/World/Scene", "/World/GoalMarker"]:
                if stage.GetPrimAtPath(p):
                    stage.RemovePrim(p)
            app.update()
            builder = build_scene(scene_type, int(seed), cfg=cfg, root="/World/Scene")
            builder.build(stage)
            # placement hooks: corridor scenes keep START/GOAL; the elevator overrides them
            # (starts on a landing, goal on the target-floor Z). See base.SceneBuilder.
            sx, sy, syaw = builder.start_pose()
            gx, gy, gz = builder.goal_pose()
            goal = UsdGeom.Cylinder.Define(stage, "/World/GoalMarker")
            goal.CreateRadiusAttr(0.25); goal.CreateHeightAttr(0.02); goal.CreateAxisAttr("Z")
            UsdGeom.Xformable(goal.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(gx, gy, gz))
            goal.CreateDisplayColorAttr(_Vt.Vec3fArray([(0.15, 0.85, 0.3)]))
            if getattr(builder, "spawn_absolute", False):
                # office_usd: the building is the world, so spawn the robot at the ABSOLUTE mission
                # start (Z-stacked by floor) and orient its yaw to face the goal (not ROBOT_T0+offset).
                ax, ay, az = builder.start_xyz()
                rtop.Set(Gf.Vec3d(ax, ay, az))
                try:
                    rorient = next((o for o in rxf.GetOrderedXformOps()
                                    if o.GetOpType() == UsdGeom.XformOp.TypeOrient), None)
                    if rorient is not None:
                        cw, sw = math.cos(syaw / 2.0), math.sin(syaw / 2.0)
                        cur = rorient.Get()
                        Quat = type(cur) if cur is not None else Gf.Quatd
                        Imag = type(cur.GetImaginary()) if cur is not None else Gf.Vec3d
                        rorient.Set(Quat(cw, Imag(0.0, 0.0, sw)))
                except Exception as _e:
                    print(f"[srv] WARN office_usd yaw set failed: {_e!r}", flush=True)
                print(f"[srv] office_usd spawn abs=({ax:.2f},{ay:.2f},{az:.2f}) yaw={syaw:+.2f} "
                      f"goal=({gx:.2f},{gy:.2f},{gz:.2f})", flush=True)
            else:
                # robot to the start pose (offset the baked translate, facing +X unchanged)
                rtop.Set(Gf.Vec3d(ROBOT_T0[0] + sx, ROBOT_T0[1] + sy, ROBOT_T0[2]))
            world.reset()
            builder.initialize_physics(world)
            bind_articulation()
            if args.preview and getattr(builder, "spawn_absolute", False):
                # seed the preview HOLD at the spawn (ride height ~az-0.31, since collision-off means
                # the base never settles), so the collision-free robot stays put until the first glide.
                _pp, _qq = SB["art"].get_world_pose()
                GLIDE.update(active=False, t0=None, frm=None,
                             to=(float(ax), float(ay), float(az) - 0.31), orient=_qq)
            corners = {}
            housing = {}
            # the elevator exposes ALL button targets (call + every floor) for bbox capture even
            # though .interactables only ever holds the CURRENT step's press target.
            all_t = list(getattr(builder, "all_targets", builder.interactables))
            # A lone control's plate IS its housing (small plate) -> the box a detector actually
            # returns; on a shared decoy plate the plate holds many buttons, so we must NOT expand
            # the gt to the whole plate (that would reward wrong grounding).
            plate_path = getattr(getattr(builder, "panel", None), "plate_path", None)
            solo = plate_path is not None and len(all_t) == 1
            for it in all_t:
                try:
                    # Box the VISIBLE control (`pose_prim_path`, the front push-plate), not the
                    # actuation prim: on the split ADA buttons `prim_path` is the housing hidden
                    # ~0.24 m behind the door, so projecting it produced a gt box offset from the
                    # plate the detector actually sees and understated grounding_iou (M1_door__00 /
                    # `ours`: 0.304-0.314 against the housing). Procedural scenes leave
                    # pose_prim_path empty -> unchanged behaviour.
                    corners[it.target_id] = _button_corners(
                        getattr(it, "pose_prim_path", "") or it.prim_path)
                    # HOUSING bbox: the button + its immediate labeled housing — what an open-vocab
                    # detector (gdino) reasonably boxes for "the <X> button". Tight cap alone
                    # understated grounding_iou (M2: 0.064, gdino boxed the whole panel). = union of
                    # the cap AABB, its per-button label decal (label_i), and (solo only) the plate.
                    hc = list(corners[it.target_id])
                    label_path = it.prim_path.replace("/button_", "/label_")
                    if "/button_" in it.prim_path and stage.GetPrimAtPath(label_path).IsValid():
                        hc += _button_corners(label_path)
                    if solo and stage.GetPrimAtPath(plate_path).IsValid():
                        hc += _button_corners(plate_path)
                    housing[it.target_id] = hc
                except Exception as e:
                    print(f"[srv] WARN bbox corners for {it.target_id}: {e!r}", flush=True)
            SC.update(builder=builder, scene_type=scene_type, seed=int(seed),
                      corners=corners, housing=housing,
                      environment=environment, environments=environments)
            timeline.play()
            for _ in range(3):
                app.update()
            print(f"SCENE_BUILT type={scene_type} seed={seed} "
                  f"interactables={[it.target_id for it in builder.interactables]} "
                  f"door_open={builder.is_door_open()}", flush=True)

        # ---- arm reach FSM ----
        # ee_world/press_dist: the flange's FINAL world pose + its distance to the commanded press point
        # (the forensic "where the arm actually went / did it hit" fields, published in /scene/state).
        # collided/n_contacts: set by the chassis contact-report callback (below); reset each trial.
        FSM = {"state": "idle", "phases": [], "pi": 0, "t0": 0.0, "q_from": None,
               "press_world": None, "goal_world": None, "press_ok": False,
               "ee_world": None, "press_dist": None,
               # closed-loop press forensics: how long the dwell held for the joints to arrive, the
               # final per-joint tracking error, and whether it converged (vs scored on timeout).
               # These separate "the arm could not get there" from "we measured it too early".
               "press_settle_s": None, "press_q_err": None, "press_converged": None,
               # flange-to-button-SURFACE distance: the physically meaningful press test, since the
               # commanded point can lie inside the door (see the press scoring block).
               "press_surface_dist": None,
               "collided": False, "n_contacts": 0,
               "last_result": None, "result_frames": 0}

        # Option B KINEMATIC PREVIEW: a smooth base glide to a target (x,y,z) over `secs`, driven the
        # SAME way as the elevator ride — SB["art"].set_world_pose each frame with the 4WS controller
        # gated off — so the robot follows the verified route (incl. through doorways / up the shaft)
        # without fighting door-leaf collision. Orientation is held (no yaw change).
        GLIDE = {"active": False, "t0": None, "frm": None, "to": None,
                 "orient": None, "secs": 2.0}

        def publish_result(ok):
            FSM["last_result"] = bool(ok)
            FSM["result_frames"] = 5     # re-impulse a few frames so a late subscriber still gets it
            result_data_attr.set(bool(ok))
            print(f"ARM_RESULT {ok}", flush=True)

        def start_reach(p_base):
            Tb = base_T()
            pw = (Tb @ np.array([p_base[0], p_base[1], p_base[2], 1.0]))[:3]
            a_w = Tb[:3, 0].copy(); a_w[2] = 0.0
            a_w = a_w / (np.linalg.norm(a_w) or 1.0)      # press axis = base forward, horizontal
            standoff_w = pw - REACH_STANDOFF * a_w
            press_w = pw - (REACH_STANDOFF - REACH_ADVANCE) * a_w
            T_lb = Tb @ REL_LB
            T_lb_inv = np.linalg.inv(T_lb)
            t1 = (T_lb_inv @ np.append(standoff_w, 1.0))[:3]
            t2 = (T_lb_inv @ np.append(press_w, 1.0))[:3]
            xdes = T_lb_inv[:3, :3] @ a_w
            ok1, q1, pe1, oe1 = solve_ik_lb(t1, xdes)
            ok2, q2, pe2, oe2 = solve_ik_lb(t2, xdes, q_seed=q1)
            print(f"ARM_GOAL base={np.round(p_base,3).tolist()} world={pw.round(3).tolist()} "
                  f"ik_standoff(ok={ok1} pe={pe1:.4f} oe={oe1:.1f}) "
                  f"ik_press(ok={ok2} pe={pe2:.4f} oe={oe2:.1f})", flush=True)
            if not (ok1 and ok2):
                publish_result(False)
                return
            q_now = np.array(SB["art"].get_joint_positions(), dtype=float)[SB["arm_idx"]]
            FSM.update(state="moving", pi=0, t0=None, q_from=q_now,
                       press_world=press_w, goal_world=pw, press_ok=False,
                       phases=[("approach", q1, 2.5), ("advance", q2, 1.0), ("dwell", q2, 0.5),
                               ("retreat", q1, 1.0), ("home", SB["q_home"].copy(), 2.5)])

        # ---- collision reporting (CHASSIS only) ----
        # base_link should never touch anything while driving (it rides above the floor; the arm is a
        # separate link, so button-presses do NOT fire here). So any base_link contact with a non-self,
        # non-ground prim = a real collision (rammed a wall/door). Fully GUARDED: if the physx contact
        # API differs, collision stays False and the server is unaffected. The live test tunes the floor
        # exclusion; the flag->state->record plumbing is verified headless.
        _collision_sub = [None]

        def _setup_collision_report():
            try:
                from pxr import PhysxSchema, PhysicsSchemaTools
                from omni.physx import get_physx_simulation_interface
                blp = base_link if isinstance(base_link, str) else str(base_link)
                bl_prim = stage.GetPrimAtPath(blp)
                if bl_prim and bl_prim.IsValid():
                    PhysxSchema.PhysxContactReportAPI.Apply(bl_prim)   # report contacts on the chassis
                _EXCLUDE = ("floor", "ground", "basefloor", "/world/robot")   # self + floor substrings

                def _on_contact(headers, data):
                    for h in headers:
                        try:
                            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)).lower()
                            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1)).lower()
                        except Exception:  # noqa
                            continue
                        pair = a0 + "|" + a1
                        if "base_link" not in pair:
                            continue
                        other = a1 if "base_link" in a0 else a0
                        if any(x in other for x in _EXCLUDE):
                            continue                       # floor / self contact -> not a collision
                        FSM["collided"] = True
                        FSM["n_contacts"] += 1

                _collision_sub[0] = get_physx_simulation_interface() \
                    .subscribe_contact_report_events(_on_contact)
                print("[srv] COLLISION report enabled on base_link", flush=True)
            except Exception as e:  # noqa — never let collision setup break the server
                print(f"[srv] COLLISION report NOT enabled ({e}); collided stays False", flush=True)

        _setup_collision_report()

        def _apply_arm(q_cmd):
            jp = np.full(SB["ndof"], np.nan, dtype=np.float32)
            for k, i in enumerate(SB["arm_idx"]):
                jp[i] = q_cmd[k]
            SB["art"].apply_action(ArticulationAction(joint_positions=jp))

        def fsm_step(t_now):
            if FSM["state"] != "moving":
                return
            if FSM["t0"] is None:
                FSM["t0"] = t_now
            name, q_to, dur = FSM["phases"][FSM["pi"]]
            frac = min(1.0, (t_now - FSM["t0"]) / max(dur, 1e-3))
            q_cmd = FSM["q_from"] + frac * (q_to - FSM["q_from"])
            _apply_arm(q_cmd)
            if frac < 1.0:
                return
            # phase complete
            if name == "dwell":
                q_meas = np.array(SB["art"].get_joint_positions(), dtype=float)[SB["arm_idx"]]
                # CLOSED-LOOP HOLD: the ramp is done, but the ARM may not be there yet. Keep the final
                # command applied and wait for the measured joints to converge before scoring (see
                # SETTLE_Q_TOL / SETTLE_MAX_S). Without this the flange was measured mid-rise and a
                # reachable, exactly-solved press was recorded as a miss (pure-z residual, always low).
                q_err = float(np.max(np.abs(q_meas - q_to)))
                held = t_now - FSM["t0"]
                if q_err > SETTLE_Q_TOL and held < SETTLE_MAX_S:
                    _apply_arm(q_to)                    # hold the target; re-enter next tick
                    return
                FSM["press_settle_s"] = round(float(held), 3)
                FSM["press_q_err"] = round(q_err, 5)
                FSM["press_converged"] = bool(q_err <= SETTLE_Q_TOL)
                # WHICH joint is the limit? If a joint sits AT its torque cap with a standing position
                # error, the commanded pose is beyond the arm's static capability at that extension —
                # a real hardware constraint to design the approach around (get the base closer so the
                # arm folds), NOT something to paper over by raising the cap. Guarded + forensic only.
                if not FSM["press_converged"]:
                    try:
                        eff = SB["art"].get_measured_joint_efforts()
                        if eff is not None:
                            e = np.asarray(eff, dtype=float).reshape(-1)
                            per = [(round(float(e[i]), 1), round(float(q_meas[k] - q_to[k]), 3))
                                   for k, i in enumerate(SB["arm_idx"])]
                            print(f"[srv] PRESS UNCONVERGED joint (effort_Nm, q_err_rad): {per}",
                                  flush=True)
                    except Exception as _e:      # noqa
                        print(f"[srv] PRESS effort readback unavailable: {_e!r}", flush=True)
                p_lb, _ = ik_fk(q_meas)
                p_ee_w = ((base_T() @ REL_LB) @ np.append(p_lb, 1.0))[:3]
                err = float(np.linalg.norm(p_ee_w - FSM["press_world"]))
                # proximity coupling: goal point near a gt button's PRESSABLE SURFACE -> act on it.
                # We measure distance to the interactable's world AABB (its bbox-able prim_path), NOT
                # its world_pose CENTER. A deep/elongated button prim (observed live: F2 panel_13_release
                # center sits 0.28 m *behind* the visible cap) has its center far from the pressed point
                # even when the arm hit the cap dead-on (grounding IoU 0.86 vs that same prim's projected
                # bbox proves the bbox covers the visible cap). Center-distance then spuriously exceeded
                # PRESS_ACT_TOL and the door never opened despite a clean press. Surface-distance couples
                # the deep button while a genuine miss (arm nowhere near the prim's box) still fails.
                # base.py Interactable docstring anticipates exactly this ("compute its world bounding
                # box via UsdGeom.BBoxCache"). Falls back to center-distance if the bbox can't be read.
                # Measure to `pose_prim_path` — the prim whose bbox produced the interactable's
                # world_pose, i.e. the geometry the grounder saw and the arm aimed at. For the split
                # ADA push-buttons the actuation prim (`prim_path`, the back housing) sits ~0.24 m
                # BEHIND that front push-plate, on the far side of the door, so measuring to it
                # scored a dead-centre press as a 0.25 m miss and the act coupling was skipped.
                # Measured on M1_door__00/`ours` x3: goal 0.250/0.248/0.248 m from `ada_gateE`
                # (x=17.12) while the plate `ada_gateE_core` (x=16.88) and the commanded goal
                # (x=16.81) were 0.07 m apart — the door never opened despite arm_press_ok=True.
                def _surface_dist(pt, it):
                    try:
                        prim = stage.GetPrimAtPath(
                            getattr(it, "pose_prim_path", "") or it.prim_path)
                        if prim and prim.IsValid():
                            rng = (UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                       [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
                                   .ComputeWorldBound(prim).ComputeAlignedRange())
                            lo, hi = rng.GetMin(), rng.GetMax()
                            return float(np.sqrt(sum(
                                max(lo[k] - pt[k], 0.0, pt[k] - hi[k]) ** 2 for k in range(3))))
                    except Exception:
                        pass
                    return float(np.linalg.norm(np.array(it.world_pose) - pt))

                # ---- did the arm PRESS? Score against the button's front SURFACE, not a point that
                # may lie INSIDE the door. `press_world` is derived from the GROUNDED 3D target, whose
                # depth carries the detector's + the depth camera's error, so it routinely lands a few
                # cm BEHIND the pressable face — a point no flange can ever occupy, because the door
                # stops it. Scoring flange-to-commanded-point then books a clean press as a miss.
                #   Measured (M1_door__01 / `ours`, post torque fix): two presses whose joints had fully
                #   CONVERGED (q_err 0.019 / 0.021 rad) scored err 0.1640 / 0.1526 because the commanded
                #   point sat at x=13.043 / 13.033 while the door plane is x=13.0 — i.e. 3-4 cm inside
                #   the door, with the flange resting correctly ON the surface.
                # The act-coupling below already measures to the interactable's world AABB for exactly
                # this reason ("a deep/elongated button prim ... center sits 0.28 m behind the visible
                # cap"); press_ok now uses the same physical test. We take the interactable nearest the
                # GOAL point (the control we were trying to press) and measure the FLANGE to its
                # surface. A genuine miss — flange stopped in free space away from the control — still
                # fails, because its surface distance stays large. Both numbers are published so the
                # trace can always show which test passed.
                surf_err = None
                if SC["builder"] is not None and SC["builder"].interactables:
                    try:
                        target_it = min(SC["builder"].interactables,
                                        key=lambda it: _surface_dist(FSM["goal_world"], it))
                        surf_err = float(_surface_dist(p_ee_w, target_it))
                    except Exception:      # noqa — never let scoring break the arm FSM
                        surf_err = None
                FSM["press_surface_dist"] = (round(surf_err, 4) if surf_err is not None else None)
                FSM["press_ok"] = bool(err <= PRESS_OK_TOL
                                       or (surf_err is not None and surf_err <= PRESS_OK_TOL))
                FSM["ee_world"] = p_ee_w.round(4).tolist()      # forensic: arm final world pose
                FSM["press_dist"] = round(err, 4)               # forensic: did it hit (distance)
                print(f"[srv] PRESS ee_world={p_ee_w.round(3).tolist()} "
                      f"cmd_press={FSM['press_world'].round(3).tolist()} err={err:.4f} "
                      f"surf_err={FSM['press_surface_dist']} ok={FSM['press_ok']} "
                      f"settle={FSM['press_settle_s']}s q_err={FSM['press_q_err']} "
                      f"converged={FSM['press_converged']}", flush=True)

                if FSM["press_ok"] and SC["builder"] is not None and SC["builder"].interactables:
                    dists = [(_surface_dist(FSM["goal_world"], it), it)
                             for it in SC["builder"].interactables]
                    dmin, itmin = min(dists, key=lambda x: x[0])
                    if dmin <= PRESS_ACT_TOL:
                        # office_usd steps can be press_button / present_fob / call_elevator /
                        # select_floor — drive the RIGHT InteractionController method, not a
                        # hardcoded press. Procedural builders lack current_action_type -> press_button.
                        _bld = SC["builder"]
                        # Execute the action the POLICY chose. Falling back to the mission's
                        # ground-truth action (current_action_type) meant a wrong-action policy still
                        # advanced the world — action SELECTION, the core thing under test, was never
                        # exercised physically. builder.act() already rejects a mismatched action, so
                        # a wrong choice now correctly produces no effect. The GT fallback remains
                        # only for the oracle path (`force_act`), which sets no planned action.
                        act_type = SC.get("planned_action")
                        if not act_type:
                            act_type = (_bld.current_action_type()
                                        if hasattr(_bld, "current_action_type") else "press_button")
                        res = _bld.act(act_type, itmin.target_id)
                        SC["planned_action"] = None   # consume once; never reuse for a later press
                        print(f"[srv] ACT {act_type} {itmin.target_id} (dist {dmin:.3f}) -> "
                              f"success={res.success} detail='{res.detail}' "
                              f"door_open={_bld.is_door_open()}", flush=True)
                    else:
                        print(f"[srv] ACT skipped: goal {dmin:.3f} m from nearest button "
                              f"({itmin.target_id}) > {PRESS_ACT_TOL}", flush=True)
            FSM["pi"] += 1
            FSM["t0"] = t_now
            FSM["q_from"] = q_to.copy()
            if FSM["pi"] >= len(FSM["phases"]):
                FSM["state"] = "idle"
                publish_result(FSM["press_ok"])

        # ---- /scene/state JSON ----
        def state_json(sim_t):
            Tb = base_T()
            x, y, z = float(Tb[0, 3]), float(Tb[1, 3]), float(Tb[2, 3])
            yaw = math.atan2(Tb[1, 0], Tb[0, 0])
            b = SC["builder"]
            is_elev = b is not None and getattr(b, "is_elevator", lambda: False)()
            if getattr(b, "spawn_absolute", False):
                # office_usd: goal from the mission (Z-stacked). This is the SINGLE office_usd at_goal
                # path — checked BEFORE the generic is_elev branch — so the CROSS-FLOOR floor gate is
                # explicit here and no longer depends on is_elevator() routing through on_target_floor.
                # BUG #2 (passive-baseline guardrail): a planar-only check let a cross-floor mission
                # (M5 elevator / M7 chained, target_floor != start_floor) falsely trip at_goal when the
                # robot reached the goal XY on the START floor WITHOUT riding the elevator (the goal XY
                # projects onto the same XY one floor down) -> passive "reached" the goal and PASSED.
                # at_goal_office() floor-gates cross-floor missions; single-floor office missions
                # (target_floor == start_floor) stay planar-only and are UNAFFECTED.
                gx, gy, _gz = b.goal_pose()
                elevator_floor, localized = 1, True
                sf = int(getattr(b, "start_floor", 0))
                tf = int(getattr(b, "_goal_floor", getattr(b, "goal_floor", sf)))
                at_goal = at_goal_office(x, y, z, gx, gy, sf, tf, AT_GOAL_TOL, office_spawn_z)
            elif is_elev:
                # procedural ElevatorScene: goal is on the target floor; at_goal is floor-gated (Z),
                # and the floor + localization come from the builder's live FSM state.
                gx, gy, gz = b.goal_pose()
                bst = b.get_scene_state()
                elevator_floor = int(bst.get("elevator_floor", 1))
                localized = bool(bst.get("localized", True))
                at_goal = (math.hypot(x - gx, y - gy) <= AT_GOAL_TOL) and b.on_target_floor(z)
            else:
                gx, gy = GOAL_XY
                elevator_floor, localized = 1, True
                at_goal = math.hypot(x - gx, y - gy) <= AT_GOAL_TOL
            # the scene's actual START pose (varies per scene: door-family ~-0.9, elevator ~-2.4)
            # so the client's reset-ready check can compare to the RIGHT start, not a hardcoded one.
            try:
                sxy = [round(float(v), 4) for v in b.start_xy] if b is not None else None
            except Exception:  # noqa
                sxy = None
            d = {"scene_type": SC["scene_type"], "seed": SC["seed"],
                 "scene_ready": SC["builder"] is not None,
                 "sim_time": round(sim_t, 2), "elevator_floor": elevator_floor,
                 "localized": localized,
                 "arm_state": FSM["state"], "last_arm_result": FSM["last_result"],
                 # FORENSIC arm + collision (read by fsm.py into each step row):
                 "arm_ee_pose": FSM["ee_world"], "arm_press_dist_m": FSM["press_dist"],
                 "arm_press_ok": bool(FSM["press_ok"]),
                 # closed-loop press forensics (see FSM init): distinguishes "the arm could not reach"
                 # from "the flange was measured before the joints arrived".
                 "arm_press_surface_dist_m": FSM["press_surface_dist"],
                 "arm_press_settle_s": FSM["press_settle_s"],
                 "arm_press_q_err": FSM["press_q_err"],
                 "arm_press_converged": FSM["press_converged"],
                 "collided": bool(FSM["collided"]), "n_contacts": int(FSM["n_contacts"]),
                 "robot_pose": {"xy": [round(x, 4), round(y, 4)], "z": round(z, 4),
                                "yaw": round(yaw, 4)},
                 "start_xy": sxy,
                 "at_goal": at_goal}
            if b is None:
                d.update(step_index=0, pressed=[], door_open=False, door_angle_rad=0.0,
                         gt_target=None, gt_expected_actions=[], interactions_required=0)
                return d
            st = b.get_scene_state()
            d.update(step_index=st.get("step_index", 0), pressed=st.get("pressed", []),
                     door_open=b.is_door_open(), door_angle_rad=round(b.door_angle(), 3),
                     gt_expected_actions=b.gt_expected_actions(),
                     interactions_required=b.gt_interactions_required())
            if getattr(b, "spawn_absolute", False):
                # office_usd: publish the mission goal + corridor-safe route waypoints so the
                # oracle/route-follow driver knows where to drive (client reads these instead of
                # the procedural GOAL_XY), plus goal_reachable for the unreachable (M4/M6b) tasks.
                gxy = b.goal_pose()
                d["goal_xy"] = [round(gxy[0], 4), round(gxy[1], 4)]
                d["route_xy"] = b.route_waypoints()
                d["goal_reachable"] = bool(getattr(b, "goal_reachable", True))
                d["start_floor"] = int(getattr(b, "start_floor", 0))
                d["target_floor"] = int(getattr(b, "_goal_floor", getattr(b, "goal_floor", 0)))
                # CROSS-FLOOR NAV: the robot's CURRENT floor (None mid-ride) is what the Nav2 client
                # keys its per-floor MAP SWITCH off — a cross-floor goal is not on the start floor's
                # map, so the client must reload map_server when this changes. Also publish the
                # elevator's landing standoff + car centre so the client can drive the ride legs
                # (approach -> enter -> exit) without any hardcoded scene geometry.
                if b.is_elevator() and hasattr(b, "floor_of_z"):
                    try:
                        d["robot_floor"] = b.floor_of_z(z)
                        _cz = b.car_world_z()
                        d["car_z"] = None if _cz is None else round(float(_cz), 3)
                        d["car_floor"] = b.car_floor()
                        d["park_floor"] = getattr(b, "_park_floor", None)
                        try:    # the live lift-drive target: proves whether a park/call reached physics
                            _e = b.elevator()
                            _j = b._ctrl.stage.GetPrimAtPath(_e.get("lift_joint", ""))
                            _a = _j.GetAttribute("drive:linear:physics:targetPosition") if _j else None
                            d["lift_target"] = None if _a is None else round(float(_a.Get() or 0.0), 3)
                        except Exception:
                            d["lift_target"] = None
                        d["robot_in_car"] = bool(b.robot_in_car(x, y))
                        d["elevator_tag"] = b.elevator_tag
                        d["elevator_standoff"] = [round(v, 4) for v in b.elevator_standoff()]
                        d["car_centre"] = [round(v, 4) for v in b.car_centre()]
                    except Exception:
                        pass
            if is_elev and hasattr(b, "current_action"):   # procedural ElevatorScene only
                cur = b.current_action()
                d.update(target_floor=int(b.target_floor), current_action=cur)
            tgt = b.gt_correct_target()
            if tgt is None:
                d["gt_target"] = None
            else:
                # bbox = tight cap AABB (kept for the tight-localization view); bbox_housing =
                # cap + label (+ solo plate) = what a detector returns, scored for grounding_iou.
                d["gt_target"] = {"label": tgt["label"], "target_id": tgt["target_id"],
                                  "point3d": [round(v, 4) for v in tgt["point3d"]],
                                  "bbox": bbox_px(SC["corners"].get(tgt["target_id"])),
                                  "bbox_housing": bbox_px(SC["housing"].get(tgt["target_id"]))}
            return d

        # ---- start ----
        world.reset()
        bind_articulation()
        timeline.play()
        rep.orchestrator.run()   # UTP-LAPTOP: start continuous SDG capture (see header)
        if args.gui:
            # frame a 3rd-person view of the corridor so the trial is watchable in the window
            try:
                from isaacsim.core.utils.viewports import set_camera_view
                set_camera_view(eye=[-2.8, -3.0, 2.3], target=[1.4, 0.0, 0.6])
            except Exception:  # noqa — non-fatal; user can orbit manually
                pass
        print("SERVER_UP publishing /clock /odom /tf /scan /mast_cam/{color,depth} /scene/state; "
              "driving on /cmd_vel; control-plane /scene/command /arm_reach/goal", flush=True)

        # --build <mission_id>: GUI/debug convenience (checkpoint 2) — auto-build ONE mission at
        # startup so no external ROS client is needed to see the robot spawn at the start.
        if getattr(args, "build", None):
            try:
                do_build(args.build, int(getattr(args, "build_seed", 0)))
                if args.gui:
                    try:
                        from isaacsim.core.utils.viewports import set_camera_view
                        _b = SC.get("builder")
                        if _b is not None and getattr(_b, "spawn_absolute", False):
                            ax, ay, az = _b.start_xyz()
                            set_camera_view(eye=[ax - 4.5, ay - 4.5, az + 3.5],
                                            target=[ax, ay, az])
                    except Exception:
                        pass
                print(f"AUTO_BUILT {args.build}", flush=True)
            except Exception as _e:
                import traceback as _tb
                _tb.print_exc()
                print(f"[srv] ERROR --build {args.build}: {_e!r}", flush=True)

        dt = float(world.get_physics_dt())
        prev_steer = {w: 0.0 for w in WHEELS}
        last_twist_count = None
        last_cmd_count = 0
        last_goal_count = 0
        last_msg_t = -1.0e9
        mode_err_logged = False
        t = 0.0
        steps = int(args.seconds * 60)
        for i in range(steps):
            # ---- control-plane: scene command ----
            if cmd_count_attr is not None:
                cc = int(cmd_count_attr.get())
                if cc != last_cmd_count:
                    last_cmd_count = cc
                    raw = cmd_data_attr.get() or ""
                    try:
                        msg = json.loads(raw)
                        cmd = msg.get("cmd")
                        if cmd == "build":
                            SC["planned_action"] = None
                            do_build(msg.get("scene_type", "button_door"), msg.get("seed", 0),
                                     msg.get("environment"), msg.get("environments"))
                            prev_steer = {w: 0.0 for w in WHEELS}
                            FSM.update(state="idle", press_ok=False)
                        elif cmd == "glide":
                            # Option B: start a smooth kinematic base glide to (x,y,z) over secs.
                            try:
                                _p, _q = SB["art"].get_world_pose()
                                GLIDE.update(
                                    active=True, t0=None,
                                    frm=(float(_p[0]), float(_p[1]), float(_p[2])),
                                    to=(float(msg["x"]), float(msg["y"]),
                                        float(msg.get("z", _p[2]))),
                                    orient=_q, secs=float(msg.get("secs", 2.0)))
                                print(f"[srv] GLIDE -> ({msg['x']:.2f},{msg['y']:.2f}) "
                                      f"secs={GLIDE['secs']}", flush=True)
                            except Exception as _e:
                                print(f"[srv] WARN glide: {_e!r}", flush=True)
                        elif cmd == "planned_action":
                            # The action the POLICY chose for the next press. Consumed once by the
                            # arm/act coupling; cleared on build/reset so it can never leak across
                            # trials. `force_act` deliberately leaves it unset so the oracle path
                            # still runs the ground-truth action.
                            SC["planned_action"] = msg.get("action")
                        elif cmd == "force_act":
                            # oracle PATH-verification: run the current step's GT interaction directly
                            # (open the blockage without arm positioning). No-op for procedural scenes.
                            _b = SC.get("builder")
                            if _b is not None and hasattr(_b, "force_current_action"):
                                _res = _b.force_current_action()
                                print(f"[srv] FORCE_ACT -> success={getattr(_res,'success',None)} "
                                      f"detail='{getattr(_res,'detail','')}' "
                                      f"door_open={_b.is_door_open()}", flush=True)
                            else:
                                print("[srv] WARN force_act: builder has no force_current_action",
                                      flush=True)
                        elif cmd == "reset":
                            if SC["scene_type"] is None:
                                print("[srv] WARN reset before any build; ignored", flush=True)
                            else:
                                do_build(SC["scene_type"], SC["seed"],
                                         SC["environment"], SC["environments"])
                                prev_steer = {w: 0.0 for w in WHEELS}
                                FSM.update(state="idle", press_ok=False)
                        else:
                            print(f"[srv] WARN unknown scene cmd: {raw!r}", flush=True)
                    except Exception as e:
                        import traceback as _tb
                        print(f"[srv] WARN bad /scene/command {raw!r}: {e!r}\n{_tb.format_exc()}",
                              flush=True)

            # ---- control-plane: arm reach goal ----
            if goal_count_attr is not None:
                gc = int(goal_count_attr.get())
                if gc != last_goal_count:
                    last_goal_count = gc
                    if FSM["state"] != "idle":
                        print("[srv] WARN /arm_reach/goal ignored (arm busy)", flush=True)
                    else:
                        p = read_goal_point()
                        if p is not None:
                            start_reach(p)
                        else:
                            publish_result(False)

            fsm_step(t)

            # office_usd elevator missions: keep landing doors synced to the car's physical height.
            _b = SC.get("builder")
            if _b is not None and hasattr(_b, "tick"):
                try:
                    _b.tick(t)
                except Exception:
                    pass

            # ---- elevator multi-step FSM: enter (nav) / ride (ANIMATED) / exit (nav) ----
            # call_elevator + select_floor are handled by the arm-press proximity coupling in
            # fsm_step (builder.act). Here we advance the NAVIGATION steps by the robot's live world
            # pose, and drive the RIDE as an ANIMATED, LOCKSTEP kinematic move: once the arm is idle
            # and the robot is aboard (quasi-stationary — robot_in_car is a tight box at car centre),
            # the car's /Car xform Z is interpolated over sim steps (accel/cruise/decel) and the SAME
            # per-step Z delta is applied to the robot via set_world_pose, so it rides up smoothly in
            # lockstep with the cabin. The 4WS base controller is GATED OFF while riding (below).
            # X/Y/yaw stay continuous (same map); localized drops at ride start and is re-asserted
            # True on arrival per floor.
            _riding = False
            _b = SC["builder"]

            # ---- office_usd ELEVATOR RIDE: carry the robot in lockstep with the physically-driven car.
            # The baked Elevator_A/B cars move on a REAL UsdPhysics PrismaticJoint whose drive target is
            # set by interaction_controller.call_elevator() when the arm presses a call/panel button —
            # so the car's motion is genuine physics, not a script. The robot standing on the car floor
            # is NOT welded to it though, so we measure the car's per-frame Z delta and apply the SAME
            # delta to the base (the "temporarily attach for the sealed ride" decision, MULTIFLOOR
            # HANDOFF §3.3). Only Z is touched: X/Y/yaw are untouched (the shaft is at one XY on every
            # floor), so /odom and map->odom stay planar-continuous across the ride and only the floor
            # gate sees a change. The 4WS controller is gated off while _riding (below) so the wheels
            # cannot fight the carry. Guarded by robot_in_car() so a robot still in the doorway is
            # never dragged through the frame.
            if (_b is not None and getattr(_b, "spawn_absolute", False)
                    and getattr(_b, "is_elevator", lambda: False)()
                    and hasattr(_b, "car_world_z")):
                try:
                    _cz = _b.car_world_z()
                    _pz = SC.get("_car_z_prev")
                    SC["_car_z_prev"] = _cz
                    if _cz is not None and _pz is not None:
                        _dz = _cz - _pz
                        if abs(_dz) > 1e-6:
                            _Tb2 = base_T()
                            _rx2, _ry2 = float(_Tb2[0, 3]), float(_Tb2[1, 3])
                            if _b.robot_in_car(_rx2, _ry2):
                                # PHYSICS RIDE (not a kinematic carry). The car has a real floor
                                # collider and the robot has wheel colliders + a high-friction
                                # material, so the cabin floor ALREADY lifts the robot. Applying the
                                # car's per-frame dz to the base on top of that DOUBLE-COUNTS the
                                # motion: measured live, the car reached z=3.0 while the base was
                                # flung to z=5.52 — 3.5 m above the cabin floor, punching through the
                                # ceiling and latching `collided`. So we do NOT move the base here.
                                # We only latch `_riding`, which gates the 4WS controller off so the
                                # wheels cannot fight the cabin, and let physics carry the robot.
                                # This is option (a) in docs/ELEVATOR_PLAN.md — and the more honest
                                # one: the robot is genuinely carried by the car, not teleported.
                                _riding = True
                                _p, _q = SB["art"].get_world_pose()
                                _nf = _b.floor_of_z(float(_p[2]))
                                if _nf is not None and _nf != SC.get("_ride_floor"):
                                    SC["_ride_floor"] = _nf
                                    print(f"ELEV_FLOOR robot now on floor={_nf} "
                                          f"z={float(_p[2]):.2f} car_z={_cz:.2f}", flush=True)
                except Exception as _e:
                    print(f"[srv] WARN office_usd ride carry failed: {_e!r}", flush=True)

            # This animated-ride FSM is the PROCEDURAL ElevatorScene interface (current_action/
            # robot_in_car/ride_tick/...). office_usd (spawn_absolute) does NOT implement it — in the
            # Option-B preview the glide carries the robot across floors — so skip it there.
            if (_b is not None and getattr(_b, "is_elevator", lambda: False)()
                    and hasattr(_b, "current_action")):
                _Tb = base_T()
                _rx, _ry, _rz = float(_Tb[0, 3]), float(_Tb[1, 3]), float(_Tb[2, 3])
                _act = _b.current_action()
                if _act == "enter" and _b.robot_in_car(_rx, _ry):
                    if _b.notify_entered():
                        print(f"ELEV_ENTER robot=({_rx:.2f},{_ry:.2f}) doors_closing "
                              f"floor={_b.get_scene_state()['elevator_floor']}", flush=True)
                # START the animated ride: arm idle + robot aboard (well inside the car).
                if (FSM["state"] == "idle" and getattr(_b, "_ride_pending", False)
                        and _b.robot_in_car(_rx, _ry)):
                    if _b.begin_ride_if_pending():
                        bst = _b.get_scene_state()
                        print(f"ELEV_RIDE_START -> floor={_b.target_floor} "
                              f"robot_z={_rz:.2f} localized={bst['localized']}", flush=True)
                # ADVANCE the in-progress ride: move car in Z, carry the robot in LOCKSTEP.
                if _b.ride_active():
                    _riding = True
                    _dz = _b.ride_tick(dt)
                    if _dz is not None:
                        _p, _q = SB["art"].get_world_pose()
                        _newz = float(_p[2]) + float(_dz)
                        SB["art"].set_world_pose(position=[float(_p[0]), float(_p[1]), _newz],
                                                 orientation=_q)
                        SB["art"].apply_action(ArticulationAction(
                            joint_velocities=np.zeros(SB["ndof"], dtype=np.float32)))
                    if not _b.ride_active():          # last step just completed -> arrived
                        bst = _b.get_scene_state()
                        _p, _q = SB["art"].get_world_pose()
                        print(f"ELEV_RIDE_DONE car->floor={bst['elevator_floor']} "
                              f"robot_z={float(_p[2]):.2f} doors_open={bst['door_open']} "
                              f"relocalized={bst['localized']}", flush=True)
                if _act == "exit":
                    _gx, _gy, _gz = _b.goal_pose()
                    if (math.hypot(_rx - _gx, _ry - _gy) <= AT_GOAL_TOL
                            and _b.on_target_floor(_rz)):
                        if _b.notify_exited():
                            print(f"ELEV_EXIT reached goal=({_gx:.2f},{_gy:.2f}) "
                                  f"floor={_b.get_scene_state()['elevator_floor']}", flush=True)

            # ---- Option B kinematic preview: HOLD the (collision-free) robot at GLIDE["to"] every
            # frame — interpolating while a glide is active, holding otherwise — so it is a pure
            # visual carried only by glide commands and never falls / penetrates anything.
            _gliding = False
            if args.preview and GLIDE["to"] is not None:
                _gliding = True                                # controller gated off in preview
                tx, ty, tz = GLIDE["to"]
                if GLIDE["active"]:
                    if GLIDE["t0"] is None:
                        GLIDE["t0"] = t
                    frac = min(1.0, (t - GLIDE["t0"]) / max(GLIDE["secs"], 1e-3))
                    sf = frac * frac * (3.0 - 2.0 * frac)      # smoothstep ease in/out
                    fx, fy, fz = GLIDE["frm"]
                    gx = fx + (tx - fx) * sf
                    gy = fy + (ty - fy) * sf
                    gz = fz + (tz - fz) * sf
                    if frac >= 1.0:
                        GLIDE["active"] = False
                        GLIDE["frm"] = GLIDE["to"]
                else:
                    gx, gy, gz = tx, ty, tz                    # hold at the last target
                try:
                    SB["art"].set_world_pose(position=[gx, gy, gz], orientation=GLIDE["orient"])
                    SB["art"].apply_action(ArticulationAction(
                        joint_velocities=np.zeros(SB["ndof"], dtype=np.float32)))
                except Exception as _e:
                    if i % 120 == 0:
                        print(f"[srv] WARN preview hold skipped: {_e!r}", flush=True)

            # ---- base: /cmd_vel -> 4WS controller (STAGE 3) ----
            # GATED OFF while the elevator carries the robot (_riding) OR a preview glide owns the pose
            # (_gliding): the car / glide drives the pose, so the 4WS controller must NOT also drive the
            # base — hold the wheels stopped. vx/vy/wz are still read so the periodic tick log stays valid.
            lv = lin_attr.get(); av = ang_attr.get()
            vx, vy, wz = float(lv[0]), float(lv[1]), float(av[2])
            if _riding or _gliding:
                fresh = False                      # steer held; wheels already zeroed by the ride/glide
            else:
                if twist_count_attr is not None:
                    cnt = int(twist_count_attr.get())
                    if cnt != last_twist_count:
                        last_twist_count = cnt; last_msg_t = t
                    fresh = (t - last_msg_t) <= CMD_TIMEOUT_S
                else:
                    fresh = True
                sel = _select_mode(nav_mode, vx, vy, wz, r_char) if fresh else None
                if sel is None:
                    cmds = {w: (prev_steer[w], 0.0) for w in WHEELS}   # hold steer, stop wheels
                else:
                    mode, cvx, cvy, cwz = sel
                    try:
                        cmds = ctrl4ws.compute(mode, cvx, cvy, cwz, dt, prev_steer)
                    except Exception as e:
                        if not mode_err_logged:
                            print(f"[srv] WARN controller rejected cmd ({e!r}); stopping", flush=True)
                            mode_err_logged = True
                        cmds = {w: (prev_steer[w], 0.0) for w in WHEELS}
                prev_steer = {w: cmds[w][0] for w in WHEELS}
                jp = np.full(SB["ndof"], np.nan, dtype=np.float32)
                jv = np.full(SB["ndof"], np.nan, dtype=np.float32)
                for k, w in enumerate(WHEELS):
                    s, ws = cmds[w]
                    jp[SB["steer_idx"][k]] = STEER_SIGN * s
                    jv[SB["wheel_idx"][k]] = ws
                try:
                    SB["art"].apply_action(ArticulationAction(joint_positions=jp, joint_velocities=jv))
                except Exception as _e:
                    # physics view transiently unavailable (e.g. right after a failed rebuild) — skip
                    # this frame rather than hard-crash the whole server.
                    if i % 120 == 0:
                        print(f"[srv] WARN base apply_action skipped: {_e!r}", flush=True)

            # ---- /scene/state @ ~5 Hz + pending result re-publishes ----
            if i % STATE_EVERY_N == 0:
                try:
                    state_data_attr.set(json.dumps(state_json(t)))
                    state_impulse.set(True)
                except Exception as e:
                    print(f"[srv] WARN state publish failed: {e!r}", flush=True)
            if FSM["result_frames"] > 0:
                FSM["result_frames"] -= 1
                result_impulse.set(True)

            world.step(render=True)   # render=True so the RTX lidar render product produces /scan
            # UTP-LAPTOP: drive the replicator graph EXPLICITLY. isaac_worker/robot/add_sensors.py:
            # "world.step(render=True) does NOT tick the replicator SDG graph in this kit config, so
            # the annotators only fill when rep.orchestrator.step() is called." Under
            # orchestrator.run() alone the bridge's rgb helper filled but its depth helper published
            # 100% inf on every frame (measured 2026-08-29, 40/40 frames, no finite pixel). The
            # repo's own verification uses step(rt_subframes=8); 2 keeps the RTF tolerable.
            if i % 2 == 0:
                try:
                    rep.orchestrator.step(rt_subframes=2, pause_timeline=False)
                except Exception as _e:
                    if i % 600 == 0:
                        print(f"[srv] WARN orchestrator.step failed: {_e!r}", flush=True)
            t += dt
            if i % 600 == 0:
                print(f"[srv] tick {i}/{steps} cmd=({vx:+.2f},{vy:+.2f},{wz:+.2f}) "
                      f"fresh={fresh} arm={FSM['state']} scene={SC['scene_type']} "
                      f"door_open={SC['builder'].is_door_open() if SC['builder'] else None}",
                      flush=True)
        print("[srv] done", flush=True)
    except Exception:
        import traceback; print("[SERVER_ERROR]", flush=True); traceback.print_exc()
    finally:
        app.close()


if __name__ == "__main__":
    main()
