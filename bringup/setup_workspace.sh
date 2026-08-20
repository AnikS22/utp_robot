#!/usr/bin/env bash
# Build the ROS 2 driver workspace from scratch. Idempotent — safe to re-run.
#
#     bash bringup/setup_workspace.sh
#
# Clones the three driver repos at PINNED commits, vendors asio, applies our patches, and builds.
# Nothing here needs root. The workspace itself is gitignored: this script is the reproducible
# artifact, so the rover laptop gets an identical build from a fresh clone.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$REPO/ros2_ws"
ROS_DISTRO_SETUP=/opt/ros/jazzy/setup.bash

# Pinned so the rover laptop and the workstation build the same code. Bump deliberately.
RPLIDAR_REPO=https://github.com/Slamtec/rplidar_ros.git
RPLIDAR_SHA=24cc9b6dea97e045bda1408eaa867ce730fd3fc3          # branch ros2
RANGER_REPO=https://github.com/agilexrobotics/ranger_ros2.git
RANGER_SHA=b6ea21a275ca5e7168130cc6470e61474681d679           # branch humble -- see NOTE
UGV_REPO=https://github.com/agilexrobotics/ugv_sdk.git
UGV_SHA=f2704eacdc90357078cd93ec60aae08bb4baab35              # branch main
ASIO_REPO=https://github.com/chriskohlhoff/asio.git
ASIO_TAG=asio-1-28-0
#
# NOTE on the ranger branch: we build the *humble* branch on Jazzy deliberately. The jazzy branch
# does NOT support the Ranger Mini V3 -- its RangerSubType enum stops at kRangerMiniV2 and it ships
# no ranger_mini_v3 launch file. Only humble has kRangerMiniV3. Do not "fix" this by switching to
# the jazzy branch; you would silently lose V3 support.

# --- conda must not shadow ROS's python -------------------------------------------------------
# colcon invokes package_xml_2_cmake.py with whatever python3 is first on PATH. If that is conda's,
# it has no catkin_pkg/ament_package and EVERY package fails at ament_package() with an opaque
# "returned error code 1". Scrub conda for the build only.
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v miniconda | paste -sd:)"
unset PYTHONPATH CONDA_PREFIX || true
echo "python for build : $(command -v python3)"

clone_at() {   # repo sha dest
    local repo="$1" sha="$2" dest="$3"
    if [ ! -d "$dest/.git" ]; then
        echo "cloning $(basename "$dest")"
        git clone -q "$repo" "$dest"
    fi
    git -C "$dest" fetch -q --all
    # reset --hard, not plain checkout: `git checkout <sha>` PRESERVES local modifications, so on a
    # re-run the patch below would be applied on top of itself and fail. This is what makes the
    # script genuinely idempotent.
    git -C "$dest" reset -q --hard "$sha"
    git -C "$dest" clean -qfd
}

mkdir -p "$WS/src" "$WS/third_party"
clone_at "$RPLIDAR_REPO" "$RPLIDAR_SHA" "$WS/src/rplidar_ros"
clone_at "$RANGER_REPO"  "$RANGER_SHA"  "$WS/src/ranger_ros2"
clone_at "$UGV_REPO"     "$UGV_SHA"     "$WS/src/ugv_sdk"

# --- asio: header-only, vendored so no sudo apt is needed ------------------------------------
# ugv_sdk includes <asio.hpp> (standalone asio, not boost::asio). Ubuntu ships it as libasio-dev,
# but that needs root; vendoring the headers avoids blocking on a password.
if [ ! -f "$WS/third_party/asio_src/asio/include/asio.hpp" ]; then
    echo "vendoring asio $ASIO_TAG"
    git clone -q --depth 1 -b "$ASIO_TAG" "$ASIO_REPO" "$WS/third_party/asio_src"
fi
ASIO_INC="$WS/third_party/asio_src/asio/include"

# --- patches ----------------------------------------------------------------------------------
# Applied against a clean checkout each run (checkout above resets the tree), so re-running is safe.
echo "applying patches"
git -C "$WS/src/rplidar_ros" apply "$REPO/patches/rplidar_ros-legacy-scan.patch"

# --- build ------------------------------------------------------------------------------------
# ROS setup.bash references unset vars, so -u must be off while sourcing it.
set +u
# shellcheck disable=SC1090
source "$ROS_DISTRO_SETUP"
set -u
cd "$WS"
colcon build \
    --packages-select rplidar_ros ugv_sdk ranger_msgs ranger_base ranger_bringup \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
                 -DCMAKE_CXX_FLAGS="-I$ASIO_INC"

cat <<EOF

Workspace built. To use it:

    source /opt/ros/jazzy/setup.bash
    source $WS/install/setup.bash

Then:  bash bringup/lidar.sh
EOF
