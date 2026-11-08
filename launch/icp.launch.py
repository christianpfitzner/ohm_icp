"""Lab "ICP": the simulator in the hall `production`, the robot drives a loop, RViz shows the paths and the match.

    ros2 launch ohm_icp icp.launch.py
    ros2 run ohm_icp icp_node.py --ros-args -p task:=1       # second terminal
    ros2 launch launch/icp.launch.py                          # from the checkout

`--show-args` lists the arguments; ohm_icp/launching.py does the work.
"""
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))   # checkout, even via a symlink
if HERE not in sys.path:
    sys.path.append(HERE)
from ohm_icp.launching import describe                         # noqa: E402


def generate_launch_description():
    return describe()
