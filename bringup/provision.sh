#!/usr/bin/env bash
# Provision the rover laptop: ROS 2 Jazzy, device permissions, boot-time CAN. Needs root.
#
#     sudo bash bringup/provision.sh
#
# This is LAPTOP_SETUP.md stages 2-3 made executable and idempotent. It does NOT build the driver
# workspace -- that is bringup/setup_workspace.sh, which deliberately needs no root and must run as
# the normal user, or every build artifact ends up owned by root.
#
# Everything here is additive and reversible; nothing is removed or overwritten in place except
# files this script itself owns (the udev rule and the can0 unit), and those are rewritten only
# when their content differs.
set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "must run as root:  sudo bash $0" >&2
    exit 1
fi

# The user we are provisioning FOR, not the root we are running AS. sudo exports the original.
TARGET_USER="${SUDO_USER:-}"
if [ -z "$TARGET_USER" ] || [ "$TARGET_USER" = "root" ]; then
    echo "cannot determine the non-root user -- run via sudo, not as a root login shell" >&2
    exit 1
fi
echo "provisioning for user: $TARGET_USER"
echo

step() { echo; echo "=== $* ==="; }

# ---------------------------------------------------------------------------------------------
step "1/6  ROS 2 apt repository"
# The keyring+sources pair now ships as a versioned .deb (ros2-apt-source) rather than the old
# curl-a-key-into-trusted.gpg dance. Installing the .deb is what keeps the key rotatable.
if [ -f /etc/apt/sources.list.d/ros2.sources ] || [ -f /etc/apt/sources.list.d/ros2.list ]; then
    echo "ROS 2 apt source already present -- skipping"
else
    DEB=/tmp/ros2-apt-source.deb
    if [ ! -s "$DEB" ]; then
        echo "downloading ros2-apt-source"
        curl -fsSL -o "$DEB" \
          https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.2.0/ros2-apt-source_1.2.0.noble_all.deb
    fi
    apt-get install -y "$DEB"
fi
apt-get update

# ---------------------------------------------------------------------------------------------
step "2/6  ROS 2 Jazzy and build tooling"
# ros-jazzy-desktop pulls rviz2 + demo nodes; the rest are what this stack actually launches.
# can-utils gives candump/cansend, which is how the Ranger's CAN link gets verified BEFORE any
# ROS driver is involved -- a layer that answers before the driver can lie about it.
apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-realsense2-camera \
    ros-jazzy-tf2-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    python3-venv \
    git \
    can-utils

# ---------------------------------------------------------------------------------------------
step "3/6  rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
else
    echo "rosdep already initialised -- skipping init"
fi
# `rosdep update` writes to the invoking user's cache, so it must NOT run as root: doing so creates
# root-owned files in ~/.ros that make every later user-level rosdep call fail on permissions.
sudo -u "$TARGET_USER" rosdep update || echo "warning: rosdep update failed (non-fatal here)"

# ---------------------------------------------------------------------------------------------
step "4/6  serial permissions for the lidar"
# /dev/ttyUSB0 is root:dialout 0660. Without this the RPLIDAR cannot be opened at all, and the
# failure is a bare PermissionError that looks nothing like a hardware problem.
if id -nG "$TARGET_USER" | tr ' ' '\n' | grep -qx dialout; then
    echo "$TARGET_USER already in dialout -- skipping"
else
    usermod -aG dialout "$TARGET_USER"
    echo "added $TARGET_USER to dialout"
    echo "NOTE: existing logins do NOT gain the group. Either log out and back in, or prefix"
    echo "      commands with:  sg dialout -c '<command>'"
fi

# ---------------------------------------------------------------------------------------------
step "5/6  udev rule for a stable lidar name"
# /dev/ttyUSBn REORDERS -- observed live on the workstation, ttyUSB0 -> ttyUSB1 after a re-plug.
# The by-id path already survives that and is what bringup/lidar.sh uses; this symlink is the
# shorter alias documented in LAPTOP_SETUP.md.
RULE=/etc/udev/rules.d/99-utp-robot.rules
WANT='SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"'
if [ -f "$RULE" ] && grep -qF "$WANT" "$RULE"; then
    echo "udev rule already correct -- skipping"
else
    printf '%s\n' "$WANT" > "$RULE"
    udevadm control --reload-rules
    udevadm trigger --subsystem-match=tty
    echo "wrote $RULE"
fi

# ---------------------------------------------------------------------------------------------
step "6/6  can0 at boot (Ranger Mini 3.0)"
# Failing to start with the adapter unplugged is EXPECTED, not a fault -- see LAPTOP_SETUP.md.
UNIT=/etc/systemd/system/can0.service
read -r -d '' UNIT_BODY <<'EOF' || true
[Unit]
Description=Bring up can0 for the Ranger Mini 3.0
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 up type can bitrate 500000
ExecStop=/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target
EOF
if [ -f "$UNIT" ] && [ "$(cat "$UNIT")" = "$UNIT_BODY" ]; then
    echo "can0.service already correct -- skipping"
else
    printf '%s\n' "$UNIT_BODY" > "$UNIT"
    systemctl daemon-reload
    echo "wrote $UNIT"
fi
systemctl enable can0.service >/dev/null 2>&1 || true
# Do not fail the whole script because the adapter is absent.
systemctl start can0.service >/dev/null 2>&1 \
    && echo "can0 up" \
    || echo "can0 did not come up -- expected if the USB-CAN adapter is unplugged"

# ---------------------------------------------------------------------------------------------
echo
echo "=============================================================================="
echo "provisioning done. Next, AS $TARGET_USER (not root):"
echo
echo "    bash ~/utp_robot/bringup/setup_workspace.sh"
echo "    sg dialout -c 'bash ~/utp_robot/bringup/lidar.sh'"
echo
echo "The sg wrapper is only needed until the next login."
echo "=============================================================================="
