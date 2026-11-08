#!/usr/bin/env python3
"""Lab "ICP": the node around your ICP — where the robot is, from its laser scans (and its odometry).

    ros2 launch ohm_icp icp.launch.py                                      # terminal 1
    ros2 run ohm_icp icp_node.py --ros-args -p task:=1                     # terminal 2

The node is given; the ICP it runs is yours, in `icp.py` and `scan_odometry.py` next to this file.

    task 1   ICP alone:            every `every`-th scan is matched onto the scan before (the model),
                                   ICP starts at the identity; the pose is the chain of all matches
    task 2   ICP and odometry:     the same, ICP starts at the step the odometry measured

The pose starts where the odometry is when the node starts (at the start of the simulator: the spawn).
Results: /<robot>/icp/pose (the pose after every match), /<robot>/icp/markers (model, data and pairs of
the newest match, for RViz), /<robot>/icp/info (settings and numbers as JSON), every 5 s one line here.
"""
import collections
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.realpath(__file__))              # student/, also via the installed symlink
for path in (HERE, os.path.dirname(HERE)):                     # your files, then the package next door
    if path not in sys.path:
        sys.path.insert(0, path)

import numpy as np                                            # noqa: E402
import rclpy                                                  # noqa: E402
from geometry_msgs.msg import Point, PoseStamped              # noqa: E402
from nav_msgs.msg import Odometry                             # noqa: E402
from rclpy.executors import ExternalShutdownException         # noqa: E402
from rclpy.node import Node                                   # noqa: E402
from rclpy.parameter import Parameter                         # noqa: E402
from sensor_msgs.msg import LaserScan                         # noqa: E402
from std_msgs.msg import ColorRGBA, String                    # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray        # noqa: E402

import icp                                                    # noqa: E402
import scan_odometry                                          # noqa: E402

PARAMETERS = {
    "robot": "muster",          # topics /<robot>/scan, /<robot>/odom and /<robot>/icp/...
    "task": 1,                  # 1 ICP alone, 2 ICP with the odometry as the start
    "every": 10,                # match every n-th scan (20 scans per second): 10 = twice per second
    "d_max": 0.3,               # m: pairs farther apart do not count (STUDENT 1.7); 0 = all count
    "max_iter": 50,             # iterations of ICP at most
    "eps": 1e-7,                # m^2: stop when the error changed by less than this
    "kdtree": False,            # extra task Z.1: step 1 with a kd-tree
}
RESTART = 5.0                   # s: a longer step (or a step back) means the simulator was restarted


class IcpNode(Node):

    def __init__(self):
        super().__init__("icp", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.p = {name: self.declare_parameter(name, value).value for name, value in PARAMETERS.items()}
        robot, self.task = self.p["robot"], int(self.p["task"])
        self.odoms = collections.deque(maxlen=200)               # (t, msg), the newest odometry
        self.reset()
        self.create_subscription(LaserScan, f"/{robot}/scan", self.on_scan, 10)
        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 100)
        self.pub_pose = self.create_publisher(PoseStamped, f"/{robot}/icp/pose", 10)
        self.pub_info = self.create_publisher(String, f"/{robot}/icp/info", 10)
        self.pub_markers = self.create_publisher(MarkerArray, f"/{robot}/icp/markers", 1)
        start = "the identity" if self.task == 1 else "the odometry"
        self.get_logger().info(
            f"task {self.task} (ICP from {start}), every {self.p['every']}. scan, d_max {self.p['d_max']} m, "
            f"max_iter {self.p['max_iter']}, eps {self.p['eps']:g}" + (", kd-tree" if self.p["kdtree"] else "")
            + f"; waiting for /{robot}/scan and /{robot}/odom")

    def reset(self) -> None:
        self.pose, self.model, self.run = None, None, None
        self.first_scan, self.scans, self.last_scan, self.last_log = None, 0, None, -1e9
        self.matches, self.iterations, self.ms, self.pairs, self.error = 0, 0, 0.0, 0, 0.0

    # ------------------------------------------------------------------ callbacks

    def on_odom(self, msg: Odometry) -> None:
        self.odoms.append((stamp(msg), msg))

    def odom_at(self, t: float):
        """The odometry message closest in time to t (the simulator stamps both from one clock)."""
        if not self.odoms:
            return None
        found = min(self.odoms, key=lambda e: abs(e[0] - t))
        return found[1] if abs(found[0] - t) < 0.1 else None

    def on_scan(self, msg: LaserScan) -> None:
        t = stamp(msg)
        if self.last_scan is not None:
            dt = t - self.last_scan
            if dt > RESTART or dt < -RESTART:
                self.get_logger().warn(f"time jumped by {dt:.1f} s: the pose starts again at the odometry")
                self.reset()
                self.odoms.clear()
                return
        self.last_scan = t
        if self.first_scan is None:
            self.first_scan = t
        self.scans += 1
        odom = self.odom_at(t)
        if odom is None:
            return
        if self.model is None:                                  # the first scan: the start
            o = odom.pose.pose
            self.pose = icp.se2(o.position.x, o.position.y, scan_odometry.yaw(o.orientation))
            self.model = (t, scan_odometry.scan_to_points(msg), odom)
            self.run = t
            x, y, th = icp.pose_of(self.pose)
            self.get_logger().info(f"start at the odometry ({x:.2f}, {y:.2f}) m, {math.degrees(th):.0f} deg, "
                                   f"t = {t:.1f} s, {len(self.model[1])} points in the first scan")
            self.publish(t, None, None, None)
            return
        period = (t - self.first_scan) / (self.scans - 1) if self.scans > 1 else 0.05   # mean, from the stamps
        if round((t - self.model[0]) / period) < int(self.p["every"]):
            return                                              # not this one: every n-th scan only
        data = scan_odometry.scan_to_points(msg)
        guess = scan_odometry.odom_guess(self.model[2], odom) if self.task >= 2 else None
        begin = time.perf_counter()
        T, info = icp.icp(self.model[1], data, guess, max_iter=int(self.p["max_iter"]), eps=float(self.p["eps"]),
                          d_max=float(self.p["d_max"]), kdtree=bool(self.p["kdtree"]))
        ms = (time.perf_counter() - begin) * 1e3
        before = self.pose
        self.pose = scan_odometry.next_pose(self.pose, T)
        self.matches += 1
        self.iterations += info["iterations"]
        self.ms += ms
        self.pairs, self.error = info["pairs"], info["error"]
        self.publish(t, before, data, (T, guess, info, ms))
        self.model = (t, data, odom)

    # ------------------------------------------------------------------ results

    def publish(self, t: float, before, data, match) -> None:
        x, y, th = icp.pose_of(self.pose)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp.sec, pose.header.stamp.nanosec = int(t), int((t % 1.0) * 1e9)
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.pub_pose.publish(pose)
        n = max(self.matches, 1)
        info = {"run": self.run, "t": t, "x": x, "y": y, "th": th, "task": self.task,
                "every": int(self.p["every"]), "d_max": self.p["d_max"],
                "max_iter": int(self.p["max_iter"]), "eps": self.p["eps"], "kdtree": bool(self.p["kdtree"]),
                "matches": self.matches, "iterations": self.iterations / n, "ms": self.ms / n}
        if match is not None:
            T, guess, last, ms = match
            info.update({"last_iterations": last["iterations"], "last_error": last["error"],
                         "last_pairs": last["pairs"], "last_points": last["points"], "last_ms": ms})
            self.draw(t, before, data, T, guess)
        self.pub_info.publish(String(data=json.dumps(info)))
        if match is not None and t - self.last_log >= 5.0:
            self.last_log = t
            _, _, last, ms = match
            self.get_logger().info(
                f"t {t:6.1f} s | pose ({x:6.2f}, {y:6.2f}) m {math.degrees(th):7.1f} deg | match {self.matches}: "
                f"{last['iterations']:2d} iterations, error {math.sqrt(max(last['error'], 0.0)) * 1000:4.1f} mm, "
                f"{last['pairs']} of {last['points']} pairs, {ms:5.1f} ms (mean {self.ms / n:5.1f} ms)")

    def draw(self, t, before, data, T, guess) -> None:
        """Model (grey) where the robot was, the data at the start of ICP (amber) and after it (red), in map."""
        model = icp.transform(before, self.model[1])
        start = icp.transform(before @ (np.eye(3) if guess is None else guess), data)
        after = icp.transform(before @ T, data)
        index, distance = icp.closest_points(after, model) if len(model) else ([], [])
        markers = MarkerArray()
        for ident, (ns, points, rgba, size) in enumerate((("model", model, (0.75, 0.75, 0.78, 0.9), 0.06),
                                                          ("data at the start", start, (0.95, 0.65, 0.1, 0.8), 0.05),
                                                          ("data after ICP", after, (0.9, 0.25, 0.2, 1.0), 0.05))):
            m = marker(ident, ns, Marker.POINTS, t, rgba, size)
            m.points = [Point(x=float(px), y=float(py), z=0.02 * ident) for px, py in points]
            markers.markers.append(m)
        pairs = marker(3, "pairs", Marker.LINE_LIST, t, (0.4, 0.8, 1.0, 0.6), 0.01)
        for j in range(0, len(after), 3):
            if float(self.p["d_max"]) <= 0.0 or distance[j] <= float(self.p["d_max"]):
                pairs.points += [Point(x=float(after[j][0]), y=float(after[j][1]), z=0.05),
                                 Point(x=float(model[index[j]][0]), y=float(model[index[j]][1]), z=0.05)]
        markers.markers.append(pairs)
        self.pub_markers.publish(markers)


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def marker(ident, ns, kind, t, rgba, size) -> Marker:
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1.0) * 1e9)
    m.ns, m.id, m.type, m.action = ns, ident, kind, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = size
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return m


def main():
    rclpy.init()
    node = IcpNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                          # Ctrl-C in the middle of a callback: ROS is already gone
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
