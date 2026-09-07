"""ROS-independent scan matching. Ambiguous global matches must not become pose seeds."""
import math
import numpy as np


def distinct(a, b):
    """Separate location hypotheses, allowing local refinement around each one."""
    return (math.hypot(a[1] - b[1], a[2] - b[2]) > 0.75 or
            abs(math.atan2(math.sin(a[3] - b[3]), math.cos(a[3] - b[3]))) > math.radians(25))


class ScanMatcher:
    def __init__(self, grid, resolution, origin, ranges, angles):
        self.grid = np.asarray(grid)
        self.res = float(resolution)
        self.ox, self.oy = origin
        self.rs = np.asarray(ranges)
        self.angles = np.asarray(angles)
        if (self.res <= 0 or self.grid.ndim != 2 or self.rs.size < 30 or
                self.rs.shape != self.angles.shape or
                not np.isfinite(self.rs).all() or not np.isfinite(self.angles).all()):
            raise ValueError('need a valid map and at least 30 finite scan returns')
        self.h, self.w = self.grid.shape
        self.occ = self.grid > 50

    def scores(self, xs, ys, yaw):
        # Use the same world-to-cell conversion in the lattice and refinement.
        ii = np.floor((np.asarray(xs)[:, None] - self.ox +
                       self.rs * np.cos(self.angles + yaw)) / self.res).astype(int)
        jj = np.floor((np.asarray(ys)[:, None] - self.oy +
                       self.rs * np.sin(self.angles + yaw)) / self.res).astype(int)
        inside = (ii >= 0) & (ii < self.w) & (jj >= 0) & (jj < self.h)
        return (self.occ[np.clip(jj, 0, self.h-1), np.clip(ii, 0, self.w-1)] & inside).sum(axis=1)

    def fit(self, x, y, yaw):
        return int(self.scores([x], [y], yaw)[0])

    def search(self, initial=None):
        step = max(1, round(0.20 / self.res))
        j, i = np.where(self.grid[::step, ::step] == 0)
        if not len(i):
            raise ValueError('map contains no free search candidates')
        xs = self.ox + (i * step + .5) * self.res
        ys = self.oy + (j * step + .5) * self.res
        candidates = []
        if initial is not None and np.isfinite(initial).all():
            candidates.append((self.fit(*initial), *initial))
        # Keep several spatial peaks per heading, not only one coarse winner.
        for deg in range(0, 360, 5):
            yaw = math.radians(deg)
            scores = self.scores(xs, ys, yaw)
            for _ in range(min(8, len(xs))):
                k = int(scores.argmax())
                if scores[k] < 0:
                    break
                candidates.append((int(scores[k]), xs[k], ys[k], yaw))
                scores[np.hypot(xs-xs[k], ys-ys[k]) <= .75] = -1
        seeds = []
        for p in sorted(candidates, reverse=True):
            if all(distinct(p, q) for q in seeds):
                seeds.append(p)
                if len(seeds) == 16:
                    break
        refined = []
        for best in seeds:
            for span, spacing, yr in ((.20, .05, range(-6, 7)), (.08, .02, range(-3, 4))):
                s, x, y, yaw = best
                offsets = np.arange(-round(span/spacing), round(span/spacing)+1) * spacing
                dx, dy = np.meshgrid(offsets, offsets)
                xx, yy = x + dx.ravel(), y + dy.ravel()
                for delta in yr:
                    angle = yaw + math.radians(delta)
                    scores = self.scores(xx, yy, angle)
                    k = int(scores.argmax())
                    if scores[k] > best[0]:
                        best = (int(scores[k]), xx[k], yy[k], angle)
            refined.append(best)
        ranked = []
        for p in sorted(refined, reverse=True):
            if all(distinct(p, q) for q in ranked):
                ranked.append(p)
        return ranked


def acceptance(hypotheses, beams, min_fit=55.0, min_margin=5.0):
    """Heuristic rejection, not proof of physical localization correctness."""
    if not hypotheses or beams < 30:
        return False, 'insufficient scan/map evidence'
    fit = 100 * hypotheses[0][0] / beams
    if fit < min_fit:
        return False, f'best fit {fit:.1f}% is below {min_fit:.1f}%'
    if len(hypotheses) > 1:
        margin = 100 * (hypotheses[0][0] - hypotheses[1][0]) / beams
        if margin < min_margin:
            return False, f'ambiguous locations: score gap {margin:.1f} points is below {min_margin:.1f}'
    return True, 'fit and competing-location checks passed'
