#!/usr/bin/env bash
# Where is this machine on the network right now? Run it here; read it from anywhere.
#
#     bash bringup/whereami.sh
#
# Campus DHCP re-leases whenever it likes and the robot roams between access points, so the
# address you noted this morning is not necessarily the address this afternoon. With the laptop
# bolted to a robot and the lid shut there is no screen to check it on -- so it gets written to a
# file that survives, and printed on demand.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$REPO/.last_address"

echo "host   : $(hostname)   ($(hostname).local via mDNS, if the network passes multicast)"
echo "user   : ${SUDO_USER:-$USER}"
echo "time   : $(date '+%Y-%m-%d %H:%M:%S %Z')"
echo
echo "addresses:"
ip -brief addr | grep -v '^lo' | while read -r dev state addrs; do
    [ "$state" = "UP" ] || { printf '  %-16s %s\n' "$dev" "$state"; continue; }
    for a in $addrs; do
        case "$a" in
            fe80:*|*:*) continue ;;          # link-local v6 is not reachable off-link
        esac
        printf '  %-16s %s\n' "$dev" "${a%%/*}"
    done
done

WIFI="$(iwgetid -r 2>/dev/null || true)"
[ -n "$WIFI" ] && echo -e "\nwifi   : SSID '$WIFI'  signal $(awk 'NR==3{print $3}' /proc/net/wireless 2>/dev/null || echo '?')"

# A file beats memory: after a roam or a re-lease this is the only record, and it is readable
# over any connection that still works.
{
    echo "# written $(date -Is) by bringup/whereami.sh"
    echo "host=$(hostname)"
    echo "user=${SUDO_USER:-$USER}"
    for a in $(hostname -I); do echo "addr=$a"; done
} > "$STAMP"
echo -e "\nrecorded in $STAMP"
echo
echo "connect with:"
for a in $(hostname -I); do echo "  mosh ${SUDO_USER:-$USER}@$a        # survives roaming"; done
echo "  ssh  ${SUDO_USER:-$USER}@$(hostname).local"
