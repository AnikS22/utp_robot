"""A rejected search must not reuse old TF and certify the robot's position."""
import subprocess
from pathlib import Path


def test_rejected_search_never_reads_tf_or_certifies_floor():
    source = (Path(__file__).resolve().parents[1] / 'bringup/mission.sh').read_text()
    function = source[source.index('find_self() {'):source.index('\nlocalize_on()')]
    harness = r'''
set -uo pipefail
REPO=/unused; DRY=""; LOCALIZED=""; UTP_RELOC_TRIES=2
say() { :; }
note() { echo "$*"; }
event() { echo "EVENT $*"; }
die() { echo "REFUSED"; exit 23; }
sleep() { :; }
python3() {
    if [ "$1" = '-' ]; then
        local code; code="$(cat)"
        case "$code" in *utp_find_self*) echo 'UNEXPECTED_TF_READ' >&2;; esac
        return 0
    fi
    echo 'rejected: ambiguous scan' >&2
    return 2
}
'''
    result = subprocess.run(['bash', '-c', harness + function + '\nfind_self floor1 1.5'],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 23, result.stdout + result.stderr
    assert result.stdout.count('EVENT localization_rejected') == 2
    assert 'UNEXPECTED_TF_READ' not in result.stderr
    assert 'EVENT localized ' not in result.stdout


def test_empty_keepout_is_omitted_from_ros_launch():
    source = (Path(__file__).resolve().parents[1] / 'bringup/bringup_all.sh').read_text()
    start = source.index('        _keepout_args=()')
    end = source.index('        _nav_deadline=', start)
    fragment = source[start:end]
    harness = 'KEEPOUT=""; REPO=/robot; RUNTIME=/params; start_bg() { printf "%s\\n" "$@"; };\n'
    result = subprocess.run(['bash', '-c', harness + fragment], capture_output=True, text=True, check=True)
    assert 'localization:=slam' in result.stdout
    assert 'keepout:=' not in result.stdout
