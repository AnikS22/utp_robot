#!/usr/bin/env bash
# Bring up the mast RealSense and publish /mast_cam/* (+ base_link -> mast_cam_optical).
#
#     bash bringup/camera.sh            # publish streams and the TF
#     bash bringup/camera.sh --no-tf    # streams only (a URDF or the hand-eye solve owns the TF)
#
# Settings, and why:
#   * align_depth.enable:=true -- REQUIRED. Without it the depth pixel under an RGB detection is
#     not the same physical point, and the error looks exactly like a bad hand-eye calibration.
#   * serial_no pinned -- the laptop has a built-in webcam and may later have a second RealSense.
#     Pinning means the node either gets THE camera or fails loudly, instead of silently binding
#     to whatever enumerated first. It is passed as "'$SERIAL'" with INNER QUOTES on purpose:
#     RealSense serials are all digits, so a bare -p serial_no:=261222076248 is parsed as an
#     INTEGER and the driver rejects it ("is of type {string}, setting it to {integer}").
#   * depth at 848x480, the D435's native depth resolution. Asking for 720p depth makes the driver
#     upscale and invent detail it never measured.
#   * initial_reset:=true -- the D4xx USB stack routinely comes back wedged after an unclean exit
#     (frames time out, or the device does not enumerate a stream). A reset on start costs ~2 s and
#     removes a whole class of "the camera is broken" that is really "the last process died badly".
#
#   * the node is NAMED "$NS" at the ROOT namespace, not run inside a /$NS namespace. The
#     realsense driver publishes under <namespace>/<node_name>/..., so -r __ns:=/mast_cam plus a
#     node name gives /mast_cam/<node_name>/color/image_raw -- one segment too deep. The pipeline
#     subscribes to /mast_cam/color/image_raw per docs/HARDWARE_SPECS.md and would just receive
#     nothing, with no error on either side. Naming the node mast_cam at / produces the contract.
#
# FRAMES -- who publishes what, and why it is split this way
#   driver (publish_tf:=true) : mast_cam_link -> mast_cam_{color,depth}_optical_frame, from the
#       device's FACTORY extrinsics. These carry the ROS optical convention (z forward, x right,
#       y down) and the real depth-to-colour offset. Do not hand-write them: an optical frame
#       written with body-frame rpy (x forward) is off by ~90 deg, the images still look perfect,
#       and every deprojected 3D point is wrong in a way no message field reveals.
#   this script : base_link -> mast_cam_link, the MOUNT pose only.
# One edge each, no edge published twice.
#
# base_frame_id is `link`, NOT `mast_cam_link`. The driver PREFIXES every frame it publishes with
# camera_name, so base_frame_id=mast_cam_link yields `mast_cam_mast_cam_link` and the camera's whole
# subtree detaches from base_link. Nothing errors: /tf_static simply carries two disconnected trees,
# the images keep flowing, and every TF lookup from base_link to an optical frame fails. Passing
# `link` gives `mast_cam_link`, which is the edge this script publishes.
#
# NOTE: docs/HARDWARE_SPECS.md names the camera frame `mast_cam_optical`. The driver actually
# stamps `mast_cam_color_optical_frame`. TF lookups follow the header, so this works -- but the
# doc and the pipeline config should be reconciled to the real name deliberately.
#
# The mount TF is the DESIGN pose and is UNMEASURED. It exists so the frame tree resolves during
# bring-up; it is NOT a calibration. CALIBRATION.md item 8 replaces it -- bringup/handeye_collect.py.
#
# PROCESS HYGIENE: same rule as lidar.sh -- no `exec`, children are started with setsid so each is
# a process-group leader, and cleanup kills the whole group. A leaked realsense node holds the USB
# device and the next start fails in a way that looks like hardware failure.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO/bringup/env.sh"

read -r SERIAL NS FRAME PARENT CW CH DW DH FPS MX MY MZ MR MP MYAW <<<"$(python3 - "$REPO/config/camera.yaml" <<'EOF'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1])); m = c["mount"]
print(c["serial_no"], c["namespace"], c["frame_id"], c["parent_frame"],
      c["color_width"], c["color_height"], c["depth_width"], c["depth_height"], c["fps"],
      m["x_m"], m["y_m"], m["z_m"], m["roll_rad"], m["pitch_rad"], m["yaw_rad"])
EOF
)"

PUBLISH_TF=1
for a in "$@"; do [ "$a" = "--no-tf" ] && PUBLISH_TF=0; done

CHILDREN=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${CHILDREN[@]:-}"; do
        [ -n "${pid:-}" ] || continue
        kill -KILL -- "-$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    echo "[camera] children reaped"
}
trap cleanup EXIT INT TERM

if [ "$PUBLISH_TF" = "1" ]; then
    echo "[camera] tf $PARENT -> ${NS}_link  xyz=($MX $MY $MZ) rpy=($MR $MP $MYAW)  [MEASURED HAND-EYE]"
    setsid ros2 run tf2_ros static_transform_publisher \
        --x "$MX" --y "$MY" --z "$MZ" --roll "$MR" --pitch "$MP" --yaw "$MYAW" \
        --frame-id "$PARENT" --child-frame-id "${NS}_link" \
        --ros-args -r __node:=utp_robot_cam_tf &
    CHILDREN+=($!)
fi

echo "[camera] serial=$SERIAL ns=$NS color=${CW}x${CH} depth=${DW}x${DH} @${FPS} ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

setsid ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -r __ns:=/ -r __node:="$NS" \
    -p camera_name:="$NS" \
    -p serial_no:="'$SERIAL'" \
    -p align_depth.enable:=true \
    -p initial_reset:=true \
    -p rgb_camera.color_profile:="${CW}x${CH}x${FPS}" \
    -p depth_module.depth_profile:="${DW}x${DH}x${FPS}" \
    -p enable_color:=true -p enable_depth:=true \
    -p enable_infra1:=false -p enable_infra2:=false \
    -p pointcloud.enable:=false \
    -p publish_tf:=true \
    -p base_frame_id:=link &
NODE_PID=$!
CHILDREN+=("$NODE_PID")

echo "[camera] starting (initial_reset adds ~2 s). Ctrl-C to stop."
wait "$NODE_PID" || true
