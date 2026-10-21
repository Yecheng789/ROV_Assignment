import heapq
import math
import numpy as np


def segment_distance(points, a, b):
    points = np.asarray(points).reshape(-1, 2)
    d = np.asarray(b) - a
    t = np.clip((points - a) @ d / max(d @ d, 1e-15), 0, 1)
    return np.linalg.norm(points - (a + t[:, None] * d), axis=1)


def path_projection(position, path):
    a, b = path[:-1], path[1:]
    d = b - a
    lengths = np.linalg.norm(d, axis=1)
    t = np.clip(np.sum((position - a) * d, axis=1) /
                np.maximum(lengths**2, 1e-15), 0, 1)
    projections = a + t[:, None] * d
    dist = np.linalg.norm(projections - position, axis=1)
    j = int(np.argmin(dist))
    arc = np.r_[0.0, np.cumsum(lengths)]
    return float(dist[j]), float(arc[j] + t[j] * lengths[j]), arc


def point_at(path, arc, s):
    return np.array([np.interp(s, arc, path[:, i]) for i in range(2)])


def mission_legs(config):
    legs = []
    length = config["line_length"]
    for i, y in enumerate(config["line_y"]):
        start, end = ((0., y), (length, y)) if i % 2 == 0 else ((length, y), (0., y))
        if legs:
            legs.append((legs[-1][1], start, "connector"))
        legs.append((start, end, f"line_{i + 1}"))
    return legs


class Planner:
    def __init__(self, config):
        self.config = config
        self.radius = config["robot_radius"] + config["obstacle_radius"] + config["safety_margin"]
        self.step = config["grid_resolution"]
        self.low, self.high = -0.4, config["line_length"] + 0.4


    def clear(self, a, b, centers, extra=0.0):
        return not len(centers) or np.all(segment_distance(centers, a, b) > self.radius + extra)

    def plan(self, start, goal, centers, leg_start):
        start, goal, leg_start = map(lambda p: np.asarray(p, dtype=float),
                                    (start, goal, leg_start))
        centers = np.asarray(centers).reshape(-1, 2)
        if self.clear(start, goal, centers):

            d = goal - leg_start
            foot = leg_start + np.clip((start - leg_start) @ d / max(d @ d, 1e-9), 0, 1) * d
            if np.linalg.norm(start-foot) > 0.12 and self.clear(start, foot, centers) and self.clear(foot, goal, centers):
                return np.array([start, foot, goal])
            return np.array([start, goal])
        n = int(round((self.high - self.low) / self.step)) + 1
        axis = self.low + np.arange(n) * self.step
        xx, yy = np.meshgrid(axis, axis, indexing="ij")
        grid = np.stack((xx, yy), axis=-1)
        occupied = np.zeros((n, n), dtype=bool)
        for c in centers:
            occupied |= np.linalg.norm(grid - c, axis=-1) <= self.radius + 0.02
        def index(p):
            return tuple(np.clip(np.rint((p - self.low)/self.step).astype(int), 0, n-1))
        source, target = index(start), index(goal)
        if occupied[source] or occupied[target]:
            raise ValueError("Start or endpoint inside inflated obstacle; holding position.")
        line_penalty = segment_distance(grid.reshape(-1, 2), leg_start, goal).reshape(n, n)
        cost = {source: 0.}
        parent = {}
        queue = [(0., source)]
        visited = set()
        moves = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j]
        while queue:
            _, cell = heapq.heappop(queue)
            if cell in visited:
                continue
            if cell == target:
                cells = [cell]
                while cell != source:
                    cell = parent[cell]
                    cells.append(cell)
                points = [start] + [grid[c] for c in reversed(cells)][1:-1] + [goal]


                simple = [points[0]]
                k = 0
                while k < len(points)-1:
                    j = min(k+5, len(points)-1)
                    while j > k+1 and not self.clear(points[k], points[j], centers, 0.01):
                        j -= 1
                    if not self.clear(points[k], points[j], centers):
                        raise ValueError("Unsafe grid endpoint connection.")
                    simple.append(points[j])
                    k = j
                return np.array(simple)
            visited.add(cell)
            for di, dj in moves:
                nxt = (cell[0]+di, cell[1]+dj)
                if not (0 <= nxt[0] < n and 0 <= nxt[1] < n) or occupied[nxt]:
                    continue
                if di and dj and (occupied[cell[0]+di, cell[1]] or occupied[cell[0], cell[1]+dj]):
                    continue
                new_cost = cost[cell] + math.hypot(di, dj)*self.step*(1 + 4*line_penalty[nxt])
                if new_cost < cost.get(nxt, float("inf")):
                    cost[nxt] = new_cost
                    parent[nxt] = cell
                    heuristic = np.linalg.norm(grid[nxt] - goal)
                    heapq.heappush(queue, (new_cost + heuristic, nxt))
        raise ValueError("No collision-free route to the next survey endpoint.")
