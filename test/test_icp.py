"""Self check of the student files, one task at a time, without ROS and without the simulator.

    python3 -m pytest test/test_icp.py -q -k task1
    python3 -m pytest test/test_icp.py -q -k task2
    python3 -m pytest test/test_icp.py -q -k extra
"""
import math
import os
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "student"))

import icp                                                    # noqa: E402
import scan_odometry                                          # noqa: E402


def room(step=0.01):
    """The outline of an L-shaped room with a box in it, a point every `step` m."""
    corners = [(0, 0), (6, 0), (6, 3), (3, 3), (3, 5), (0, 5), (0, 0)]
    box = [(1, 1), (1.6, 1), (1.6, 1.4), (1, 1.4), (1, 1)]
    points = []
    for outline in (corners, box):
        for a, b in zip(outline[:-1], outline[1:]):
            n = max(1, int(math.dist(a, b) / step))
            points += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n)]
    return np.array(points, dtype=float)


def scan_msg(ranges, angle_min=0.0, increment=math.pi / 2, range_min=0.05, range_max=8.0):
    return NS(ranges=list(ranges), angle_min=angle_min, angle_increment=increment, range_min=range_min,
              range_max=range_max)


def odom_msg(x, y, theta):
    q = NS(x=0.0, y=0.0, z=math.sin(theta / 2), w=math.cos(theta / 2))
    return NS(pose=NS(pose=NS(position=NS(x=x, y=y, z=0.0), orientation=q)))


def moved(T_true, M, keep=slice(0, None, 3)):
    """D: every third point of M as the robot sees it after it moved by T_true (so T_true maps D onto M).

    The model is three times as dense as the data. ICP still stops up to a point spacing short of the truth:
    there each point of D sits on its neighbour in M (a local minimum), so the tests allow 2 cm and 0.3 deg.
    """
    return icp.transform(np.linalg.inv(T_true), M)[keep]


# ------------------------------------------------------------------ task 1

def test_task1_1_scan_to_points_drops_beams_without_echo():
    # four beams: forward 1 m, left no echo (range_max), back 2 m, right too close (below range_min)
    points = scan_odometry.scan_to_points(scan_msg([1.0, 8.0, 2.0, 0.01]))
    assert np.asarray(points).shape == (2, 2), "keep only range_min < r < range_max"
    assert points == pytest.approx(np.array([[1.0, 0.0], [-2.0, 0.0]]), abs=1e-9)
    points = scan_odometry.scan_to_points(scan_msg([3.0], angle_min=math.radians(30)))
    assert points[0] == pytest.approx([3 * math.cos(math.radians(30)), 1.5])


def test_task1_2_closest_points_brute_force():
    M = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 2.0]])
    D = np.array([[0.9, 0.1], [0.1, 1.5], [-0.2, 0.0], [0.6, 0.0]])
    index, distance = icp.closest_points(D, M)
    assert list(index) == [1, 2, 0, 1]
    assert distance == pytest.approx([math.hypot(0.1, 0.1), math.hypot(0.1, 0.5), 0.2, 0.4]), "not squared"


def test_task1_3_4_best_transform_is_the_exercise_of_the_script():
    # script, section 4.1, practice: M = (5, 4), (6, 2); D = (1.5, 2.7), (2, 0.5)
    M = np.array([[5.0, 4.0], [6.0, 2.0]])
    D = np.array([[1.5, 2.7], [2.0, 0.5]])
    x, y, phi = icp.pose_of(icp.best_transform(M, D))
    assert math.degrees(phi) == pytest.approx(13.76, abs=0.01)
    assert (x, y) == pytest.approx((4.18, 1.03), abs=0.01)


@pytest.mark.parametrize("x, y, deg", [(0.3, -0.2, 7.0), (-1.0, 2.0, -40.0), (0.5, 0.5, 150.0), (0.0, 0.0, -170.0)])
def test_task1_3_4_best_transform_finds_any_motion_from_known_pairs(x, y, deg):
    M = room()
    T_true = icp.se2(x, y, math.radians(deg))
    T = icp.best_transform(M, moved(T_true, M, slice(None)))
    assert T == pytest.approx(T_true, abs=1e-9), "with the right pairs one step is enough; atan2, not atan"


def test_task1_5_icp_finds_a_small_motion_of_the_room():
    M = room()
    T_true = icp.se2(0.15, -0.08, math.radians(4.0))
    T, info = icp.icp(M, moved(T_true, M), max_iter=100, eps=1e-12)
    assert info["iterations"] > 1
    x, y, phi = icp.pose_of(np.linalg.inv(T_true) @ T)
    assert math.hypot(x, y) < 0.02 and abs(math.degrees(phi)) < 0.3


def test_task1_5_icp_starts_where_it_is_told():
    M = room()
    T_true = icp.se2(0.15, -0.08, math.radians(4.0))
    T, info = icp.icp(M, moved(T_true, M), T_init=T_true)
    assert T == pytest.approx(T_true, abs=1e-6) and info["iterations"] <= 2


def test_task1_6_next_pose_puts_the_match_on_top_of_the_pose():
    pose = icp.se2(1.0, 2.0, math.pi / 2)                        # at (1, 2), looking along +y
    new = scan_odometry.next_pose(pose, icp.se2(1.0, 0.0, 0.0))  # one metre forward
    assert icp.pose_of(new) == pytest.approx((1.0, 3.0, math.pi / 2), abs=1e-9), "pose @ T, not T @ pose"


def test_task1_7_pairs_kept():
    distance = np.array([0.1, 0.5, 0.3, 0.31])
    assert list(icp.pairs_kept(distance, 0.3)) == [True, False, True, False]
    assert list(icp.pairs_kept(distance, 0.0)) == [True] * 4


def test_task1_7_d_max_keeps_points_without_partner_out():
    M = room()
    T_true = icp.se2(0.1, 0.05, math.radians(3.0))
    D = moved(T_true, M)
    rng = np.random.default_rng(3)
    D = np.vstack([D, rng.uniform([7.0, 0.0], [9.0, 5.0], (40, 2))])   # 40 points the model has never seen
    errors = []
    for d_max in (0.0, 0.3):
        T, _ = icp.icp(M, D, d_max=d_max, max_iter=100, eps=1e-12)
        errors.append(math.hypot(*icp.pose_of(np.linalg.inv(T_true) @ T)[:2]))
    assert errors[1] < 0.02 and errors[0] > 0.1


# ------------------------------------------------------------------ task 2

@pytest.mark.parametrize("before, now, step", [
    ((1.0, 1.0, math.pi / 2), (1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.0)),            # 1 m forward, heading north
    ((0.0, 0.0, 0.0), (0.0, 0.0, 0.3), (0.0, 0.0, 0.3)),                            # turned on the spot
    ((2.0, 0.0, math.pi), (1.0, 0.5, math.pi), (1.0, -0.5, 0.0)),                   # forward and to the right
])
def test_task2_1_odom_guess_is_the_step_in_the_frame_of_the_robot_before(before, now, step):
    T = scan_odometry.odom_guess(odom_msg(*before), odom_msg(*now))
    assert icp.pose_of(T) == pytest.approx(step, abs=1e-9), "inverse(before) @ now"


def test_task2_the_guess_brings_a_large_step_home():
    M = room()
    T_true = icp.se2(0.5, 0.0, math.radians(25.0))
    D = moved(T_true, M)
    T_alone, _ = icp.icp(M, D, d_max=0.3)
    guess = icp.se2(0.52, 0.02, math.radians(24.0))              # what the odometry would say
    T_odom, _ = icp.icp(M, D, T_init=guess, d_max=0.3, max_iter=100, eps=1e-12)
    error = lambda T: math.hypot(*icp.pose_of(np.linalg.inv(T_true) @ T)[:2])   # noqa: E731
    assert error(T_alone) > 0.1 and error(T_odom) < 0.02


# ------------------------------------------------------------------ extra task

def test_extra_1_kdtree_gives_the_same_pairs():
    pytest.importorskip("scipy")
    from scipy.spatial import cKDTree
    rng = np.random.default_rng(1)
    M, D = rng.uniform(0, 5, (300, 2)), rng.uniform(0, 5, (200, 2))
    index, distance = icp.closest_points_kdtree(D, cKDTree(M))
    brute_index, brute_distance = icp.closest_points(D, M)
    assert list(index) == list(brute_index) and distance == pytest.approx(brute_distance)


def test_extra_1_icp_with_the_kdtree():
    pytest.importorskip("scipy")
    M = room()
    T_true = icp.se2(0.15, -0.08, math.radians(4.0))
    T, _ = icp.icp(M, moved(T_true, M), max_iter=100, eps=1e-12, kdtree=True)
    assert math.hypot(*icp.pose_of(np.linalg.inv(T_true) @ T)[:2]) < 0.02
