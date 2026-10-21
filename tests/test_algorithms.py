import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from rov_survey.perception import SonarMapper
from rov_survey.planning import Planner, mission_legs, segment_distance
from rov_survey.control import RobustController, allocation_matrix

ROOT = Path(__file__).resolve().parents[1]
CONFIG = json.loads((ROOT/"config/mission.json").read_text())
MODEL = ROOT/"vendor/bluerov2_gz/models/bluerov2_ping/model.sdf"


def scan_circle(center, radius=0.25):
    angles = np.linspace(-0.14835, 0.14835, 5)
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    center = np.array(center) - np.array(CONFIG["sonar_offset"][:2])
    along = rays @ center
    discriminant = along**2 - center@center + radius**2
    return np.where(discriminant >= 0, along-np.sqrt(np.maximum(0, discriminant)) + 0.1, np.inf)


def test_sparse_scan_detects_cylinder_with_bias_and_mount_offset():
    mapper = SonarMapper(CONFIG)
    mapper.update(scan_circle([2., .03]), -.14835, .2967/4, .2, 10,
                  [0, 0, -5], np.eye(3))
    assert len(mapper.centers) == 1
    assert np.linalg.norm(mapper.centers[0] - [2., .03]) < 0.025


def test_invalid_scan_and_pitch_rejection():
    mapper = SonarMapper(CONFIG)
    mapper.update([float("nan"), float("inf"), 0., -2., 10.], -.1, .05, .2, 10,
                  [0, 0, -5], np.eye(3))
    assert len(mapper.centers) == 0
    mapper.update([3.]*5, -.1, .05, .2, 10, [0, 0, -5],
                  Rotation.from_euler("y", 1.).as_matrix())
    assert len(mapper.centers) == 0


def test_map_rotation_and_persistence_without_returns():
    mapper = SonarMapper(CONFIG)
    mapper.update(scan_circle([2., 0.]), -.14835, .2967/4, .2, 10,
                  [4, 2, -5], Rotation.from_euler("z", np.pi/2).as_matrix())
    assert np.linalg.norm(mapper.centers[0] - [4, 4]) < .03
    mapper.update([float("inf")]*5, -.14835, .2967/4, .2, 10,
                  [4, 2, -5], np.eye(3))
    assert len(mapper.centers) == 1


def test_online_planner_detours_and_rejoins_line_continuously():
    planner = Planner(CONFIG)
    obstacles = np.array([[3., 0.], [5., .8]])
    path = planner.plan([0, 0], [8, 0], obstacles, [0, 0])
    assert np.allclose(path[[0, -1]], [[0, 0], [8, 0]])
    assert max(path[:, 1]) > .9
    for a, b in zip(path[:-1], path[1:]):
        assert np.min(segment_distance(obstacles, a, b)) > planner.radius
    assert np.all(path >= planner.low) and np.all(path <= planner.high)


def test_blocked_endpoint_does_not_return_an_unsafe_route():
    with pytest.raises(ValueError):
        Planner(CONFIG).plan([0, 0], [8, 0], [[8, 0]], [0, 0])


def test_empty_map_and_exact_boustrophedon_order():
    legs = mission_legs(CONFIG)
    assert len(legs) == 9
    assert sum(np.linalg.norm(np.array(b)-a) for a, b, _ in legs) == 48.
    assert [legs[i][1] for i in range(0, 9, 2)] == [
        (8., 0.), (0., 2.), (8., 4.), (0., 6.), (8., 8.)]
    assert np.allclose(Planner(CONFIG).plan([0, 0], [8, 0], [], [0, 0]), [[0, 0], [8, 0]])


def test_allocation_reconstructs_wrench_and_bounds_saturation():
    b = allocation_matrix(MODEL)
    wrench = np.array([4., -2., 1., .1, -.3])
    assert np.linalg.matrix_rank(b) == 5
    assert np.allclose(b@np.linalg.pinv(b)@wrench, wrench)
    controller = RobustController(MODEL)
    thrust = controller.command(np.array([0., 0., -4.]), np.eye(3),
                                np.array([3., -3., 2.]), np.zeros(3),
                                np.array([1., 1.]), 1., 5., .04)
    assert np.max(np.abs(thrust)) <= 18
    assert np.allclose(controller.integral, 0.)
