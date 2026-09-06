"""Bounded readiness checks, independent of ROS for deterministic tests."""
from __future__ import annotations

import time


def wait_for_transforms(edges, available, spin, budget, clock=time.monotonic):
    """Poll all edges against one deadline, without delaying healthy edges."""
    results = {edge: False for edge in edges}
    deadline = clock() + budget
    while True:
        for edge, good in results.items():
            if not good:
                try:
                    results[edge] = bool(available(*edge))
                except Exception:
                    pass
        if all(results.values()) or clock() >= deadline:
            return results
        spin()
