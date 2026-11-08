#!/usr/bin/env python3
"""The monitor of the lab (given): how good is the pose? Compares odometry and ICP with the truth.

    ros2 run ohm_icp monitor_node.py      (the launch file starts it)

Every start of icp_node.py is a new run. All three poses (truth, odometry, ICP) are taken at the instants of
the matches and compared from the start of the run on (ohm_icp/evaluate.py). Every `period` s one line:

    driven    length of the true path, of the odometry and of ICP since the start of the run, m
    turned    the sum of all turns, deg (one lap of the loop: 360)
    error     position and heading error of the odometry and of ICP, now

and after every lap of the loop one line with the errors at that moment. Each run is written to
runs/task<task>_every<n>_dmax<d>_<time>.csv for plot_run.py. RViz gets the three paths on
/<robot>/icp_lab/*, all starting where your node started.
"""
import collections
import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import rclpy                                                  # noqa: E402
from geometry_msgs.msg import Point, PoseStamped              # noqa: E402
from nav_msgs.msg import Odometry, Path                       # noqa: E402
from rclpy.executors import ExternalShutdownException         # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from std_msgs.msg import ColorRGBA, String                    # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray        # noqa: E402

from ohm_icp import evaluate, paths, route                    # noqa: E402

COLUMNS = ["t", "x_true", "y_true", "th_true", "x_odom", "y_odom", "th_odom", "x_icp", "y_icp", "th_icp",
           "err_odom", "head_odom", "err_icp", "head_icp", "iterations", "error", "pairs", "ms"]
KEEP = 4000                     # samples of truth and odometry kept (truth 20 Hz, odometry 50 Hz)


class MonitorNode(Node):

    def __init__(self):
        super().__init__("monitor")
        robot = self.declare_parameter("robot", "muster").value
        self.period = self.declare_parameter("period", 10.0).value
        self.truth = collections.deque(maxlen=KEEP)            # (t, x, y, theta)
        self.odom = collections.deque(maxlen=KEEP)
        self.pending = collections.deque()                     # infos newer than the newest truth
        self.run, self.run_id, self.file, self.writer = None, None, None, None
        self.paths = {name: collections.deque(maxlen=KEEP) for name in evaluate.Run.NAMES}
        self.last_report = -1e9
        self.create_subscription(PoseStamped, f"/{robot}/truth", self.on_truth, 50)
        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 100)
        self.create_subscription(String, f"/{robot}/icp/info", self.on_info, 50)
        self.pubs = {name: self.create_publisher(Path, f"/{robot}/icp_lab/{name}_path", 1)
                     for name in evaluate.Run.NAMES}
        self.pub_markers = self.create_publisher(MarkerArray, f"/{robot}/icp_lab/markers", 1)
        self.create_timer(0.5, self.draw)
        self.get_logger().info(f"comparing /{robot}/odom and /{robot}/icp/info with /{robot}/truth; "
                               f"start your node: ros2 run ohm_icp icp_node.py")

    # ------------------------------------------------------------------ inputs

    def on_truth(self, msg: PoseStamped) -> None:
        t = stamp(msg)
        if self.truth and t < self.truth[-1][0]:
            self.truth.clear()                                 # simulator restarted
            self.odom.clear()
        q = msg.pose.orientation
        self.truth.append((t, msg.pose.position.x, msg.pose.position.y, math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)))
        while self.pending and self.pending[0]["t"] <= t:
            self.evaluate(self.pending.popleft())

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.odom.append((stamp(msg), msg.pose.pose.position.x, msg.pose.pose.position.y,
                          math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)))

    def on_info(self, msg: String) -> None:
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if self.truth and info["t"] <= self.truth[-1][0]:
            self.evaluate(info)
        else:
            self.pending.append(info)

    # ------------------------------------------------------------------ evaluation

    def evaluate(self, info: dict) -> None:
        t = info["t"]
        truth, odom = nearest(self.truth, t, 0.06), nearest(self.odom, t, 0.03)
        if truth is None or odom is None:
            return
        if info["run"] != self.run_id:
            self.new_run(info)
        icp = (info["x"], info["y"], info["th"])
        for lap, e_odom, e_icp in self.run.add(t, truth, odom, icp):
            self.get_logger().info(evaluate.lap_line(lap, e_odom, e_icp, info["matches"], info["iterations"],
                                                     info["ms"]))
        for name, pose in zip(evaluate.Run.NAMES, (truth, odom, icp)):
            self.paths[name].append(self.run.aligned(name, pose))
        if self.writer is not None and info["matches"] > 0:
            a = [self.run.aligned(name, pose) for name, pose in zip(evaluate.Run.NAMES, (truth, odom, icp))]
            e = self.run.error
            values = [t, *a[0], *a[1], *a[2], e["odom"][0], e["odom"][1], e["icp"][0], e["icp"][1],
                      info.get("last_iterations", 0), info.get("last_error", 0.0), info.get("last_pairs", 0),
                      info.get("last_ms", 0.0)]
            self.writer.writerow([f"{v:.5f}" if isinstance(v, float) else v for v in values])
        if t - self.last_report >= self.period:
            self.last_report = t
            if info["matches"] > 0:
                self.get_logger().info(self.run.line())

    def new_run(self, info: dict) -> None:
        if self.file:
            self.file.close()
            self.get_logger().info(f"run written to {os.path.relpath(self.file.name)}")
        self.run, self.run_id, self.last_report = evaluate.Run(), info["run"], info["t"]
        for queue in self.paths.values():
            queue.clear()
        name = f"task{info['task']}_every{info['every']}_dmax{info['d_max']}"
        folder = paths.lab_file("runs")
        os.makedirs(folder, exist_ok=True)
        self.file = open(os.path.join(folder, f"{name}_{time.strftime('%H%M%S')}.csv"), "w", newline="")
        self.file.write("# " + json.dumps({k: info[k] for k in ("task", "every", "d_max", "max_iter", "eps",
                                                                 "kdtree")}) + "\n")
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        self.get_logger().info(f"new run of your node at t = {info['t']:.1f} s: task {info['task']}, "
                               f"every {info['every']}, d_max {info['d_max']}")

    # ------------------------------------------------------------------ RViz

    def draw(self) -> None:
        if not self.truth:
            return
        now = self.truth[-1][0]
        for name, pub in self.pubs.items():
            pub.publish(make_path(list(self.paths[name]), now))
        loop = marker(0, "loop", Marker.LINE_STRIP, now, (0.6, 0.6, 0.6, 0.5), 0.02)
        loop.points = [Point(x=x, y=y) for x, y, _ in route.loop(0.25)]
        loop.points.append(loop.points[0])
        self.pub_markers.publish(MarkerArray(markers=[loop]))


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def nearest(samples, t: float, tolerance: float):
    """(x, y, theta) of the sample closest to t, None if none is within the tolerance."""
    best = None
    for s in reversed(samples):
        if best is None or abs(s[0] - t) < abs(best[0] - t):
            best = s
        if s[0] < t - tolerance:
            break
    return best[1:] if best is not None and abs(best[0] - t) <= tolerance else None


def make_path(poses, now: float) -> Path:
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp.sec, path.header.stamp.nanosec = int(now), int((now % 1.0) * 1e9)
    for x, y, th in poses:
        p = PoseStamped()
        p.header.frame_id = "map"
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        path.poses.append(p)
    return path


def marker(ident, ns, kind, now, rgba, size) -> Marker:
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp.sec, m.header.stamp.nanosec = int(now), int((now % 1.0) * 1e9)
    m.ns, m.id, m.type, m.action = ns, ident, kind, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = size
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return m


def main():
    rclpy.init()
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                          # Ctrl-C in the middle of a callback: ROS is already gone
        if rclpy.ok():
            raise
    finally:
        if node.file:
            node.file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
