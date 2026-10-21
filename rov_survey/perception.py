from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares


@dataclass
class Detection:
    center: np.ndarray
    hits: int = 1


class SonarMapper:
    def __init__(self, config):
        self.config = config
        self.tracks = []
        self.version = 0
        self.nearest = float("inf")

    @property
    def centers(self):
        return np.array([t.center for t in self.tracks]).reshape(-1, 2)

    def update(self, ranges, angle_min, angle_increment, range_min, range_max,
               position, rotation):

        raw = np.asarray(ranges, dtype=float)
        valid = (np.isfinite(raw) & (raw >= range_min) &
                 (raw < min(range_max - 0.03, self.config["sonar_detection_range"])))
        self.nearest = float(np.min(raw[valid])) if np.any(valid) else float("inf")
        ids = np.flatnonzero(valid)
        if not len(ids):
            return False
        angles = angle_min + ids * angle_increment
        distance = raw[ids] - self.config["sonar_range_bias"]
        rays = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(len(ids))))
        origin = np.asarray(position) + rotation @ np.array(self.config["sonar_offset"])
        points = origin + (rays * distance[:, None]) @ rotation.T

        keep = np.abs(points[:, 2] + self.config["depth"]) < 0.8
        points, ids = points[keep, :2], ids[keep]
        if not len(points):
            return False
        splits = np.flatnonzero((np.diff(ids) > 1) |
                               (np.linalg.norm(np.diff(points, axis=0), axis=1) > 0.6)) + 1
        changed = False
        for group in np.split(points, splits):
            direction = group.mean(axis=0) - origin[:2]
            direction /= max(np.linalg.norm(direction), 1e-9)
            radius = self.config["obstacle_radius"]
            guess = group.mean(axis=0) + direction * radius
            if len(group) >= 2 and np.linalg.norm(group[-1] - group[0]) > 0.03:
                fit = least_squares(lambda c: np.linalg.norm(group - c, axis=1) - radius,
                                    guess, max_nfev=30, loss="soft_l1", f_scale=0.02)
                if np.sqrt(np.mean(fit.fun**2)) < 0.06:
                    guess = fit.x
            if self.tracks:
                distances = np.linalg.norm(self.centers - guess, axis=1)
                j = int(np.argmin(distances))
            else:
                distances, j = [float("inf")], 0
            if distances[j] < 0.65:
                track = self.tracks[j]

                if len(group) >= 2:
                    correction = 0.25 * (guess - track.center)
                    track.center += correction
                    changed |= np.linalg.norm(correction) > 0.015
                track.hits += 1
            else:
                self.tracks.append(Detection(guess.copy()))
                changed = True
        if changed:
            self.version += 1
        return changed
