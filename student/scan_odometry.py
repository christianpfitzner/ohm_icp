"""From the messages to the points, and from the matches to the pose of the robot.

    scan_to_points(msg)              the points of one scan, in the frame of the laser     STUDENT 1.1
    next_pose(pose, T)               the pose after one match                              STUDENT 1.6
    odom_guess(odom_prev, odom_now)  T as the odometry sees it, the start of ICP           STUDENT 2.1
    new_keyframe(T, key_dist, key_angle)   is the robot far enough from the keyframe?     STUDENT Z.2

Poses are 3 x 3 transforms, as T in icp.py: `pose` maps a point from the frame of the robot into the world
(`map`). The messages are ROS messages: `ros2 interface show <type>` lists their fields.

Replace each `raise NotImplementedError("STUDENT ...")` by your code: the comment above it says in words
what goes there, the task in the handout gives the equation.
"""
import math

import numpy as np

from icp import pose_of, se2


def scan_to_points(msg):
    """The points (N x 2) of one sensor_msgs/msg/LaserScan in the frame of the laser; beams without echo dropped.

    Beam i points at the angle angle_min + i * angle_increment, counter-clockwise from x (forward).
    """
    ranges = np.asarray(msg.ranges, dtype=float)
    angles = msg.angle_min + msg.angle_increment * np.arange(len(ranges))
    # keep the beams with an echo: a range above range_min and below range_max (a beam without echo comes
    # back as range_max); then x = r cos(angle), y = r sin(angle) of the kept beams, one point per row
    # TODO STUDENT 1.1
    raise NotImplementedError("STUDENT 1.1")
    return points


def next_pose(pose, T):
    """The pose of the robot at the new scan, from its pose at the previous scan and the match T.

    T maps the new scan onto the previous one: it is the pose of the robot now, in the frame of the robot
    at the previous scan.
    """
    # from the previous frame into the world, after T from the new frame into the previous one
    # TODO STUDENT 1.6
    raise NotImplementedError("STUDENT 1.6")


def odom_guess(odom_prev, odom_now):
    """The T of the odometry between two nav_msgs/msg/Odometry: the pose now, in the frame of the robot before."""
    a, b = odom_prev.pose.pose, odom_now.pose.pose          # each: position (x, y, z) and orientation
    # each pose as a 3 x 3 transform (se2 and yaw are given); then the pose now as the odometry saw it,
    # seen from the pose before: the inverse of the one before, times the one now
    # TODO STUDENT 2.1
    raise NotImplementedError("STUDENT 2.1")


def new_keyframe(T, key_dist, key_angle):
    """Extra task Z.2: True when the newest scan becomes the keyframe, the model of the next matches.

    T maps the newest scan onto the keyframe: the pose of the robot now, in the frame of the robot at the
    keyframe. A new keyframe when the robot is more than key_dist (m) away from it or turned by more than
    key_angle (rad), either way.
    """
    # how far the robot is from the keyframe and how much it turned (pose_of is given), then the two tests
    # TODO STUDENT Z.2
    raise NotImplementedError("STUDENT Z.2")


def yaw(q):
    """The heading of a quaternion (geometry_msgs/msg/Quaternion) that turns only about z."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
