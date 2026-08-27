"""Build the CORRECT full robot: Ranger Mini V3 + xArm6 + mast + real D455 + real RPLIDAR A1M8,
ground-referenced so the wheels sit on z=0 and everything is attached and at real heights.

Root-cause fix: the official Ranger URDF puts base_link ~0.32 m above the wheel/ground contact
(base_link ≈ chassis top). Earlier code assumed base_link=ground, which sank the rover and put the
arm + sensors ~0.32 m too high. Here we MEASURE the wheel-bottom offset and express every height in
GROUND coordinates, converting to base_link-frame by subtracting that offset; a baked root lift then
makes the saved USD sit on the floor when loaded at the origin.

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim-venv/bin/python sim/build_robot_usd.py
Writes isaac_worker/assets/ranger_xarm6_full.usd and prints VERIFIED world coordinates.

LAPTOP COPY (2026-08-27) of the sim repo's isaac_worker/robot/build_full_robot.py -- copied, not
edited in place, per CLAUDE.md. Two changes only: REPO points at the sim repo from outside it,
and the D455/RPLIDAR assets stream from NVIDIA's public asset CDN instead of the old
workstation's local pack (verified HTTP 200 on both). Everything else is byte-identical.
"""
from __future__ import annotations
from pathlib import Path
import yaml

REPO = Path.home() / "unlocking-the-path"
ASSETS = REPO / "isaac_worker" / "assets"
RANGER = ASSETS / "ranger_mini_v3.usd"
XARM = ASSETS / "xarm6.usd"
OUT = ASSETS / "ranger_xarm6_full.usd"
SENSORS = yaml.safe_load(open(REPO / "config" / "sensors.yaml"))

ISAAC_ASSETS = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
D455_USD = f"{ISAAC_ASSETS}/Isaac/Sensors/Intel/RealSense/rsd455.usd"
RPLIDAR_USD = f"{ISAAC_ASSETS}/Isaac/Sensors/Slamtec/RPLIDAR_S2E/Slamtec_RPLIDAR_S2E.usd"  # A1M8 stand-in mesh

# Desired GROUND heights (m) — what the real robot has.
ARM_BASE_GROUND = 0.36      # arm base on the chassis rails (real flush-ish mount)
CAM_GROUND = [-0.25, 0.0, 1.15]   # mast camera (rear-center, 1.15 m) — x,y,z ground
# RPLIDAR A1M8 (2D) — mounted ON the top rails, in FRONT of the arm (not the underbelly).
LIDAR_GROUND = [0.25, 0.0, 0.37]  # front of the top deck, on the rails, ahead of the arm (x=0)
CAM_PITCH_DEG = -10


def main():
    from isaacsim.simulation_app import SimulationApp
    app = SimulationApp({"headless": True})
    try:
        from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Vt
        import math

        def _place(prim, translate, rotateY=None):
            """Robustly set translate(+optional rotateY) on a prim that may already carry xformOps
            from a referenced USD (AddTranslateOp would throw 'already exists')."""
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
            if rotateY is not None:
                xf.AddRotateYOp().Set(float(rotateY))

        def _apply_a1m8_lidar_profile(prim, lid_cfg):
            """Override the referenced OmniLidar's range/rate to the real RPLIDAR A1M8 from config.

            /scan's reported min/max range and scan rate come from these omni:sensor:Core:*
            attributes on the lidar prim (not from any isaac_worker code path). scanRateBaseHz is a
            USD uint, so a 5.5 Hz nominal is floored to 5; points/revolution = reportRate/scanRate,
            so reportRateBaseHz = scanRateBaseHz * samples keeps ~1 deg resolution."""
            from pxr import Sdf
            nmin, nmax = [float(v) for v in lid_cfg["range_m"]]
            samples = int(lid_cfg.get("samples", 360))
            scan_hz = max(1, int(float(lid_cfg.get("scan_rate_hz", 5.5))))  # uint field -> floor
            report_hz = scan_hz * samples
            def _set(name, tname, val):
                a = prim.GetAttribute(name) or prim.CreateAttribute(name, tname)
                a.Set(val)
            _set("omni:sensor:Core:nearRangeM", Sdf.ValueTypeNames.Float, nmin)
            _set("omni:sensor:Core:farRangeM", Sdf.ValueTypeNames.Float, nmax)
            _set("omni:sensor:Core:scanRateBaseHz", Sdf.ValueTypeNames.UInt, scan_hz)
            _set("omni:sensor:Core:reportRateBaseHz", Sdf.ValueTypeNames.UInt, report_hz)
            _set("omni:sensor:modelName", Sdf.ValueTypeNames.String, "RPLIDAR_A1M8")

        # --- measure the wheel-bottom offset on the assembled base, in base_link authored frame ---
        probe = Usd.Stage.Open(str(RANGER))
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        wheel_min_z = min(
            bb.ComputeWorldBound(p).ComputeAlignedRange().GetMin()[2]
            for p in probe.Traverse() if p.GetName().endswith("_wheel_link") and "steer" not in p.GetName()
        )
        chassis_top_z = bb.ComputeWorldBound(probe.GetPrimAtPath("/ranger_mini_v3/base_link")
                                             ).ComputeAlignedRange().GetMax()[2]
        LIFT = -float(wheel_min_z)        # baked root lift so wheels touch z=0
        # ground -> base_link authored z conversion: authored = ground - LIFT
        def gz(ground_z): return ground_z - LIFT
        print(f"[measure] wheel_min_z={wheel_min_z:.3f} (base_link frame) -> LIFT={LIFT:.3f}; "
              f"chassis_top(base_link)={chassis_top_z:.3f}", flush=True)

        # --- assemble onto a fresh stage, ground-lifted root ---
        stage = Usd.Stage.CreateNew(str(OUT))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, "/robot"); stage.SetDefaultPrim(root.GetPrim())
        UsdGeom.Xformable(root.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, LIFT))  # baked lift

        ranger = UsdGeom.Xform.Define(stage, "/robot/ranger")
        ranger.GetPrim().GetReferences().AddReference(str(RANGER))
        arm = UsdGeom.Xform.Define(stage, "/robot/arm")
        arm.GetPrim().GetReferences().AddReference(str(XARM))
        # The Ranger has top aluminium RAILS above the deck; the arm + lidar mount on the rails via
        # slim plates. rail top + plate height (in base_link/robot frame):
        RAIL_H, PLATE_H = 0.030, 0.012
        rail_top = chassis_top_z + RAIL_H
        mount_z = rail_top + PLATE_H
        arm_z = mount_z          # arm base on its plate on the rails
        _place(arm.GetPrim(), (0, 0, arm_z))

        # one articulation root (base) — strip the arm's second articulation
        for pth in ("/robot/arm/root_joint", "/robot/arm/joints/world_joint", "/robot/arm/world"):
            pr = stage.GetPrimAtPath(pth)
            if pr and pr.IsValid(): pr.SetActive(False)
        for p in stage.Traverse():
            if str(p.GetPath()).startswith("/robot/arm") and p.HasAPI(UsdPhysics.ArticulationRootAPI):
                p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
                if p.HasAPI(PhysxSchema.PhysxArticulationAPI):
                    p.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        base_link = stage.GetPrimAtPath("/robot/ranger/base_link")
        link_base = stage.GetPrimAtPath("/robot/arm/link_base")
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        rel = xc.GetLocalToWorldTransform(link_base) * xc.GetLocalToWorldTransform(base_link).GetInverse()
        mj = UsdPhysics.FixedJoint.Define(stage, "/robot/mount_joint")
        mj.CreateBody0Rel().SetTargets([base_link.GetPath()]); mj.CreateBody1Rel().SetTargets([link_base.GetPath()])
        mj.CreateLocalPos0Attr().Set(Gf.Vec3f(rel.ExtractTranslation())); mj.CreateLocalRot0Attr().Set(Gf.Quatf(rel.ExtractRotationQuat()))
        mj.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0)); mj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

        # --- top RAILS (two aluminium extension rails) + slim mounting PLATES for arm & lidar ---
        def _box(path, center, size, color):
            b = UsdGeom.Cube.Define(stage, path); b.CreateSizeAttr(1.0)
            bx = UsdGeom.Xformable(b.GetPrim()); bx.AddTranslateOp().Set(Gf.Vec3d(*center)); bx.AddScaleOp().Set(Gf.Vec3f(*size))
            b.CreateDisplayColorAttr(Vt.Vec3fArray([color]))
        rail_col = (0.62, 0.64, 0.67)
        for sy in (0.16, -0.16):   # two rails running front->back on the deck top
            _box(f"/robot/ranger/base_link/rail_{'l' if sy>0 else 'r'}",
                 (0.0, sy, chassis_top_z + RAIL_H/2), (0.62, 0.045, RAIL_H), rail_col)
        # slim plate under the arm (spans both rails, at the rotation center)
        _box("/robot/ranger/base_link/arm_plate", (0.0, 0.0, rail_top + PLATE_H/2),
             (0.20, 0.40, PLATE_H), (0.30, 0.31, 0.33))
        # slim plate under the lidar — EXTENDED to span/reach BOTH rails (y), front of the arm
        _box("/robot/ranger/base_link/lidar_plate", (0.25, 0.0, rail_top + PLATE_H/2),
             (0.16, 0.40, PLATE_H), (0.30, 0.31, 0.33))
        # mast BASE plate — connects the camera mast to BOTH rails (rear-center)
        _box("/robot/ranger/base_link/mast_base_plate", (-0.25, 0.0, rail_top + PLATE_H/2),
             (0.14, 0.40, PLATE_H), (0.30, 0.31, 0.33))

        # --- mast (thin frame at the rear-center, from chassis top up to the camera) ---
        cam_bz = [CAM_GROUND[0], CAM_GROUND[1], gz(CAM_GROUND[2])]   # base_link-frame
        # lidar sits ON TOP of its plate (raised so it's not embedded in the plate).
        lid_bz = [0.25, 0.0, mount_z + 0.035]
        mast = UsdGeom.Cylinder.Define(stage, "/robot/ranger/base_link/mast"); mast.CreateRadiusAttr(0.018)
        mast_h = cam_bz[2] - mount_z; mast.CreateHeightAttr(max(0.1, mast_h)); mast.CreateAxisAttr("Z")  # stands on the base plate
        UsdGeom.Xformable(mast.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(cam_bz[0], cam_bz[1], mount_z + mast_h/2))
        mast.CreateDisplayColorAttr(Vt.Vec3fArray([(0.55,0.57,0.6)]))

        # --- REAL Intel RealSense D455 mesh at the camera pose, tilted -10 deg ---
        camx = UsdGeom.Xform.Define(stage, "/robot/ranger/base_link/mast_cam")
        camx.GetPrim().GetReferences().AddReference(str(D455_USD))
        _place(camx.GetPrim(), cam_bz, rotateY=-CAM_PITCH_DEG)

        # --- REAL RPLIDAR mesh (A1M8 stand-in) at the front lidar pose ---
        lidx = UsdGeom.Xform.Define(stage, "/robot/ranger/base_link/lidar_front")
        lidx.GetPrim().GetReferences().AddReference(str(RPLIDAR_USD))
        _place(lidx.GetPrim(), lid_bz)

        # Align the RTX-lidar profile to the REAL RPLIDAR A1M8. The referenced Slamtec_RPLIDAR_S2E
        # asset carries an S2E-class OmniLidar profile (near 0.05 m / far 30 m @ 10 Hz) that drives
        # the /scan min/max range; override its omni:sensor:Core:* attributes from config/sensors.yaml
        # so /scan reports the A1M8's true 0.15-12 m range at ~1 deg / 5.5 Hz. (This keeps config as
        # the single source of truth; the same override is also authored declaratively in
        # ranger_xarm6_arranged.usda so it is effective without rerunning this build.)
        _apply_a1m8_lidar_profile(lidx.GetPrim(), SENSORS["lidar_front"])

        stage.GetRootLayer().Save()

        # --- VERIFY world coordinates (load fresh, baked lift applied) ---
        chk = Usd.Stage.Open(str(OUT)); cbb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        def wz(path):
            p = chk.GetPrimAtPath(path)
            if not p or not p.IsValid(): return None
            r = cbb.ComputeWorldBound(p).ComputeAlignedRange()
            return None if r.IsEmpty() else (round(r.GetMin()[2],3), round(r.GetMax()[2],3))
        print(f"[VERIFY world z]  wheel_fl={wz('/robot/ranger/fl_wheel_link')}  "
              f"chassis(base_link)={wz('/robot/ranger/base_link')}  arm_base={wz('/robot/arm/link_base')}  "
              f"camera={wz('/robot/ranger/base_link/mast_cam')}  lidar={wz('/robot/ranger/base_link/lidar_front')}", flush=True)
        print(f"[OK] wrote {OUT}", flush=True)
    except Exception:
        import traceback, sys
        print("[BUILD_ERROR]", flush=True)
        traceback.print_exc(); sys.stdout.flush(); sys.stderr.flush()
    finally:
        app.close()


if __name__ == "__main__":
    main()
