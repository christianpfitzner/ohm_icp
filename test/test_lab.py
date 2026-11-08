"""Tests of the lab repository: the config fits the loop and the simulator next door, the given tools work.

    python3 -m pytest test -q          (the launch and node tests need ROS 2 sourced, the rest does not)
"""
import importlib.util
import json
import math
import os
import shutil
import subprocess
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from ohm_icp import evaluate, paths, route                    # noqa: E402


def config():
    with open(paths.lab_file("config", "production.json"), encoding="utf-8") as handle:
        return json.load(handle)


def load(rel):
    spec = importlib.util.spec_from_file_location(os.path.basename(rel)[:-3], os.path.join(ROOT, rel))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_loop_is_closed_and_starts_at_the_spawn():
    points = route.loop()
    assert points[0][:2] == pytest.approx(route.START[:2], abs=0.05)
    steps = [math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:] + points[:1])]
    assert max(steps) < 0.06
    assert sum(steps) == pytest.approx(route.length(), rel=0.01)


def test_the_config_drives_the_loop_in_production():
    cfg = config()
    assert cfg["world"] == route.WORLD == "production" and cfg["debug_truth"] is True
    assert cfg["spawns"] == [list(route.START)]
    assert cfg["odom"]["geometry"] == {"scale_xy": 1.03, "lever_scale": 0.97}


def test_the_loop_keeps_a_metre_from_every_wall_of_the_simulator():
    paths.use_simulator()
    from mecanum_lab import worlds
    from mecanum_lab.types import load_config
    cfg = load_config(paths.lab_file("config", "production.json"))
    world = worlds.apply_config(worlds.load_world(route.WORLD, cfg=cfg), cfg)
    pose = world.spawn_pose(0)
    assert (pose.x, pose.y, pose.theta) == pytest.approx(route.START)

    def clearance(x, y):
        return min(math.hypot(max(w.x0 - x, 0.0, x - w.x1), max(w.y0 - y, 0.0, y - w.y1)) for w in world.walls)

    assert min(clearance(x, y) for x, y, _ in route.loop(0.1)) >= 0.99


def test_the_evaluation_compares_from_the_start_of_the_run():
    run = evaluate.Run(lap=10.0)
    start_truth, start_odom = (5.0, 5.0, 0.0), (6.0, 4.0, 1.0)                  # the odometry drifted before
    assert run.add(0.0, start_truth, start_odom, start_odom) == []
    laps = []
    for i in range(1, 121):                                                      # 12 m straight ahead
        truth = (5.0 + 0.1 * i, 5.0, 0.0)
        odom = evaluate.compose(start_odom, (0.103 * i, 0.0, 0.0))               # 3 % too far, same heading
        laps += run.add(i * 0.2, truth, odom, evaluate.compose(start_odom, (0.1 * i, 0.0, 0.0)))
    assert run.driven["truth"] == pytest.approx(12.0) and run.driven["odom"] == pytest.approx(12.36)
    assert run.error["icp"] == pytest.approx((0.0, 0.0), abs=1e-9)
    assert run.error["odom"][0] == pytest.approx(0.36) and run.turned["odom"] == pytest.approx(0.0, abs=1e-9)
    assert [lap for lap, _, _ in laps] == [1] and laps[0][1][0] == pytest.approx(0.3, abs=0.01)
    assert run.aligned("truth", start_truth) == pytest.approx(start_odom)        # paths start at ICP's start


def test_the_monitor_finds_the_nearest_sample_and_the_plot_reads_its_csv(tmp_path):
    pytest.importorskip("rclpy")
    monitor = load("nodes/monitor_node.py")
    samples = [(0.0, 1.0, 1.0, 0.0), (0.04, 2.0, 1.0, 0.0), (0.1, 3.0, 1.0, 0.0)]
    assert monitor.nearest(samples, 0.05, 0.03) == (2.0, 1.0, 0.0)
    assert monitor.nearest(samples, 0.5, 0.03) is None
    plot = load("nodes/plot_run.py")
    run = tmp_path / "task1.csv"
    rows = ["# " + json.dumps({"task": 1, "every": 10}), ",".join(monitor.COLUMNS)]
    rows += [",".join(str(v) for v in [t, t, 0, 0, t, 0, 0, t, 0, 0, 0, 0, 0, 0, 5, 0.001, 300, 50]) for t in range(4)]
    run.write_text("\n".join(rows) + "\n")
    settings, data = plot.load(str(run))
    assert settings == {"task": 1, "every": 10} and len(data) == 4
    assert plot.driven(data) == pytest.approx([0.0, 1.0, 2.0, 3.0])


def test_the_launch_file_has_its_arguments():
    if not shutil.which("ros2"):
        pytest.skip("ROS 2 is not sourced")
    out = subprocess.run(["ros2", "launch", os.path.join(ROOT, "launch", "icp.launch.py"), "--show-args"],
                         capture_output=True, text=True, timeout=60).stdout
    for name in ("robot", "speed", "rviz", "headless", "seconds"):
        assert f"'{name}'" in out


def test_the_nodes_are_executable():
    for rel in ("student/icp_node.py", "nodes/route_node.py", "nodes/monitor_node.py", "nodes/plot_run.py"):
        path = os.path.join(ROOT, rel)
        assert os.access(path, os.X_OK), rel
        with open(path, encoding="utf-8") as handle:
            assert handle.readline().startswith("#!/usr/bin/env python3"), rel
