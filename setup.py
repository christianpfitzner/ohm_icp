"""The lab as a ROS 2 package: `colcon build --symlink-install`, then `ros2 launch ohm_icp …`.

`student/` and `nodes/` are installed into lib/: `ros2 run ohm_icp icp_node.py`, the given nodes next to it.
With --symlink-install they are symlinks to the files a group edits.
"""
from glob import glob
import os

from setuptools import setup

PACKAGE = "ohm_icp"

setup(
    name=PACKAGE,
    version="0.1.0",
    description="Lab 'ICP' for the mecanum-lab simulator",
    author="Praktikum Intelligente Robotik",
    license="MIT",
    python_requires=">=3.10",
    packages=[PACKAGE],
    package_data={PACKAGE: ["hints.dat"]},
    data_files=[
        (os.path.join("share", "ament_index", "resource_index", "packages"), ["resource/" + PACKAGE]),
        (os.path.join("share", PACKAGE), ["package.xml"]),
        (os.path.join("share", PACKAGE, "launch"), glob("launch/*.py")),
        (os.path.join("share", PACKAGE, "config"), glob("config/*.json")),
        (os.path.join("share", PACKAGE, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("lib", PACKAGE), glob("student/*.py") + glob("nodes/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
