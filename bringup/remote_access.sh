#!/usr/bin/env bash
# Make this laptop reachable over SSH while it is bolted to a moving robot with the lid shut.
#
#     sudo bash bringup/remote_access.sh                 # install and configure
#     sudo bash bringup/remote_access.sh --hostname utp  # also rename to utp.local
#
# A robot-mounted laptop has three problems a desk laptop does not, and all three have to be
# solved or "ssh in" fails in a way that needs physical access to fix -- which defeats the point.
#
#   1. THE LID IS SHUT. Default logind suspends on lid close. The machine vanishes mid-run and
#      the only fix is to walk over and open it.
#   2. NOBODY TOUCHES IT. Default idle handling will suspend a machine that is "doing nothing"
#      even while it drives a robot, because driving a robot is not keyboard input.
#   3. IT MOVES. Roaming between access points drops TCP connections. WiFi power-save makes it
#      worse by parking the radio between packets, which on a moving robot reads as random
#      unreachability.
#
# So: sshd, no suspend from any cause, no WiFi power-save, and mosh + tmux so a dropped link
# costs you a reconnect rather than a running job.
#
# SECURITY. This opens SSH on whatever network the robot is on -- here a /16 campus WiFi, which is
# not a private lab LAN. Password auth on such a network is a bad idea, so this REFUSES to enable
# it: set up a key first (instructions printed at the end). If you are locked out you still have
# the keyboard.
set -euo pipefail
[ "$(id -u)" = "0" ] || { echo "run with sudo" >&2; exit 1; }

NEW_HOSTNAME=""
[ "${1:-}" = "--hostname" ] && NEW_HOSTNAME="${2:?--hostname needs a name}"

REAL_USER="${SUDO_USER:-weim}"
REAL_HOME="$(getent passwd "$REAL_USER" | cut -d: -f6)"

echo "== 1/5  packages =="
# mosh: survives roaming and IP changes, which plain ssh does not -- the single biggest quality
# difference when the machine is moving. tmux: so a dropped link never kills a running job.
DEBIAN_FRONTEND=noninteractive apt-get update -qq
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq \
    openssh-server mosh tmux avahi-daemon avahi-utils >/dev/null
echo "   openssh-server, mosh, tmux, avahi"

echo "== 2/5  never suspend =="
# Both halves are needed. logind covers lid/idle; the systemd targets cover everything else that
# might ask (including some desktop power managers).
mkdir -p /etc/systemd/logind.conf.d
cat > /etc/systemd/logind.conf.d/10-utp-robot.conf <<'EOF'
# Robot-mounted: the lid is shut and nobody is at the keyboard. Suspending strands the robot.
[Login]
HandleLidSwitch=ignore
HandleLidSwitchDocked=ignore
HandleLidSwitchExternalPower=ignore
IdleAction=ignore
EOF
systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target >/dev/null 2>&1 || true
echo "   lid=ignore, idle=ignore, sleep targets masked"

echo "== 3/5  WiFi stays awake =="
# Power-save parks the radio between packets. On a stationary desk that is invisible; on a moving
# robot it reads as random unreachability and is very hard to tell from a roaming problem.
mkdir -p /etc/NetworkManager/conf.d
cat > /etc/NetworkManager/conf.d/10-utp-no-powersave.conf <<'EOF'
[connection]
wifi.powersave = 2
EOF
for dev in /sys/class/net/wl*; do
    [ -e "$dev" ] || continue
    iw dev "$(basename "$dev")" set power_save off 2>/dev/null || true
done
echo "   powersave off"

echo "== 4/5  sshd =="
mkdir -p /etc/ssh/sshd_config.d
cat > /etc/ssh/sshd_config.d/10-utp-robot.conf <<'EOF'
# Keys only. This machine sits on a campus /16, not a private lab LAN.
PasswordAuthentication no
KbdInteractiveAuthentication no
PubkeyAuthentication yes
PermitRootLogin no

# Keep sessions alive across brief WiFi gaps while the robot moves between access points.
# Without these a roam looks like a hang and then a dead session.
ClientAliveInterval 15
ClientAliveCountMax 20
TCPKeepAlive yes
EOF
systemctl enable ssh >/dev/null 2>&1 || true
systemctl restart ssh
echo "   enabled, key-auth only"

echo "== 5/5  identity =="
if [ -n "$NEW_HOSTNAME" ]; then
    OLD="$(hostname)"
    hostnamectl set-hostname "$NEW_HOSTNAME"
    grep -q "127.0.1.1" /etc/hosts \
        && sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$NEW_HOSTNAME/" /etc/hosts \
        || echo -e "127.0.1.1\t$NEW_HOSTNAME" >> /etc/hosts
    echo "   $OLD -> $NEW_HOSTNAME"
fi
systemctl enable --now avahi-daemon >/dev/null 2>&1 || true
HN="$(hostname)"

AUTH="$REAL_HOME/.ssh/authorized_keys"
NKEYS=0
[ -f "$AUTH" ] && NKEYS=$(grep -cvE '^\s*(#|$)' "$AUTH" || true)

echo
echo "============================================================================"
echo "  SSH is up.  user=$REAL_USER  host=$HN"
for a in $(hostname -I); do echo "      ssh $REAL_USER@$a"; done
echo "      ssh $REAL_USER@$HN.local        (mDNS; campus networks often block it)"
echo "      mosh $REAL_USER@<addr>           <- USE THIS while the robot is moving"
echo

if [ "$NKEYS" -eq 0 ]; then
cat <<KEYS
  NO AUTHORIZED KEYS YET -- and password auth is deliberately OFF, so you cannot log in
  remotely until you add one. From the machine you will connect FROM:

      ssh-copy-id $REAL_USER@$(hostname -I | awk '{print $1}')

  That will fail with key-auth-only. Either do it from this keyboard:

      mkdir -p $REAL_HOME/.ssh && chmod 700 $REAL_HOME/.ssh
      nano $REAL_HOME/.ssh/authorized_keys        # paste your public key
      chmod 600 $REAL_HOME/.ssh/authorized_keys
      chown -R $REAL_USER:$REAL_USER $REAL_HOME/.ssh

  ...or temporarily allow passwords, copy the key, then remove the override:

      sed -i 's/^PasswordAuthentication no/PasswordAuthentication yes/' \\
          /etc/ssh/sshd_config.d/10-utp-robot.conf && systemctl restart ssh
KEYS
else
    echo "  $NKEYS authorized key(s) present -- you should be able to log straight in."
fi

cat <<'NEXT'

  THE IP WILL CHANGE. Campus DHCP hands out a new lease whenever it feels like it, and the robot
  roams. Two ways to find it again without a monitor:

      bash bringup/whereami.sh          # run here; prints and logs every address
      cat ~/utp_robot/.last_address     # written automatically at every bringup

  ALWAYS WORK INSIDE TMUX. A roam between access points WILL drop your session, and anything not
  in tmux dies with it -- including a mapping run you are 40 minutes into.

      tmux new -s utp        # start
      tmux attach -t utp     # after a drop

  mosh survives roams and IP changes on its own and is the better default while driving; tmux
  still protects you if mosh itself is killed.
============================================================================
NEXT
