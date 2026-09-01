"""Nav2 must consume the MASKED scan, and the params files must not contradict the live chain.

WHY THIS FILE EXISTS. On 2026-09-01 both Nav2 costmaps subscribed to /scan_filtered while the
self-occlusion mask published /scan. There was no error to notice: pointcloud_to_laserscan and
the costmaps are both BEST_EFFORT, so the WRONG data arrived perfectly, at full rate, for hours.

/scan_filtered is the RAW projection and it contains the robot -- the stowed arm and mast return
at a fixed 0.70-0.85 m across |bearing| 74-155 deg (measured, 10 scans, stationary, open floor).
Handed to the obstacle layer those become LETHAL cells wrapped around the footprint, so Nav2
believes it is standing inside an obstacle: it accepts a goal, produces no usable plan, and never
moves, with metres of clear floor ahead. That is exactly what was observed.

The comment in the params files made it worse by being INVERTED -- "rear chassis sector removed;
raw /scan is diagnostic only" described the retired A1M8 chain, where filter_scan.py produced a
cleaned /scan_filtered. On the OS0 chain the cleaning happens on the way to /scan.

A topic name is not a detail here: it decides whether Nav2 sees the room or the robot.
"""
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parent.parent
PARAMS = sorted((REPO / "nav2_bringup").glob("nav2_params*.yaml"))


def _costmap_sources(doc):
    """(where, topic) for every costmap observation source, at any nesting depth.

    Nav2 nests costmaps twice -- global_costmap.global_costmap.ros__parameters -- so a
    fixed-depth walk finds nothing and, worse, an assertion built on it PASSES vacuously.
    Recursing means a layout change breaks the test loudly instead of quietly."""
    found = []

    def walk(node, path):
        if not isinstance(node, dict):
            return
        ob = node.get("obstacle_layer")
        if isinstance(ob, dict):
            for src in str(ob.get("observation_sources", "")).split():
                s = ob.get(src)
                if isinstance(s, dict) and "topic" in s:
                    found.append((".".join(path + [src]), s["topic"]))
        for k, v in node.items():
            if isinstance(v, dict):
                walk(v, path + [k])

    walk(doc, [])
    return found


@pytest.mark.parametrize("path", PARAMS, ids=lambda p: p.name)
def test_every_costmap_observation_source_uses_the_masked_scan(path):
    doc = yaml.safe_load(path.read_text())
    sources = _costmap_sources(doc)
    assert sources, f"{path.name}: no costmap observation source found — did the schema change?"
    for where, topic in sources:
        assert topic == "/scan", (
            f"{path.name}: {where} consumes {topic}. /scan_filtered is the RAW projection and "
            f"contains the robot's own arm and mast; the masked scan is /scan.")


@pytest.mark.parametrize("path", PARAMS, ids=lambda p: p.name)
def test_no_params_file_still_claims_scan_filtered_is_the_clean_one(path):
    """The inverted comment is what made this survive a code review. Kill it wherever it appears."""
    src = path.read_text()
    assert "rear chassis sector removed" not in src, (
        f"{path.name} still carries the A1M8-era comment claiming /scan_filtered is the cleaned "
        f"topic. It is the raw one on this stack.")


def test_the_relay_is_what_publishes_the_masked_topic():
    """Ties the assertion above to the thing that actually does the masking, so a rename of either
    side breaks a test rather than silently splitting the chain again."""
    src = (REPO / "bringup" / "scan_relay.py").read_text()
    assert 'OUT_TOPIC = "/scan"' in src, "scan_relay.py no longer publishes /scan"
    assert 'IN_TOPIC = "/scan_filtered"' in src, "scan_relay.py no longer consumes /scan_filtered"
