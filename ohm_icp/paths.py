"""Where this lab and the simulator live — asked, never guessed.

The lab is a separate repository from the simulator (`ir-mecanum-lab`, ROS package `mecanum_lab`) and
cannot run without it: the robot, the laser scanner and the odometry all come from there. The
simulator is found in this order:

1. `MECANUM_LAB` — a checkout named explicitly,
2. a checkout next door (`../ir-mecanum-lab` or `../mecanum-lab`), which is the layout everybody
   develops in and which wins over an install because a stale build is the commoner mistake,
3. the installed package (`share/mecanum_lab` via `ament_index_python`, or the importable module).

This lab's own files (`config/`, `rviz/`, `student/`, `nodes/`, `runs/`) are found from this file: in the checkout
directly, after `colcon build --symlink-install` through the symlink back into the checkout — so a group
that edits `student/icp.py` runs their edit — and in a plain install in
`share/ohm_icp`.
"""
import os
import sys

PACKAGE = "ohm_icp"
SIBLINGS = ("ir-mecanum-lab", "mecanum-lab")


def lab_root() -> str:
    """The directory holding `config/`, `rviz/`, `student/` and `nodes/` of this lab."""
    here = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    if os.path.isdir(os.path.join(here, "config")):
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PACKAGE)
    except Exception:
        return here


def _is_simulator(directory: str) -> bool:
    return os.path.isfile(os.path.join(directory, "mecanum_lab", "engine.py")) or \
        os.path.isfile(os.path.join(directory, "launch", "lab.launch.py"))


def simulator_root() -> str:
    """The simulator's checkout or share directory; raises with the three places it looked."""
    given = os.environ.get("MECANUM_LAB", "")
    if given and _is_simulator(given):
        return os.path.abspath(given)
    parent = os.path.dirname(lab_root())
    for name in SIBLINGS:
        if _is_simulator(os.path.join(parent, name)):
            return os.path.join(parent, name)
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("mecanum_lab")
    except Exception:
        pass
    raise FileNotFoundError(
        "simulator not found: set MECANUM_LAB=<checkout of ir-mecanum-lab>, clone it next to this "
        f"repository ({' or '.join(os.path.join(parent, n) for n in SIBLINGS)}), or build it and "
        "source install/setup.bash")


def use_simulator() -> str:
    """Make `import mecanum_lab` import the simulator `simulator_root()` names.

    A checkout goes to the *front* of sys.path: a sourced workspace with an older build of the simulator
    would otherwise win, and that build may not have the world files of a lab yet.
    An installed simulator has no module next to its share directory; then the import finds it.
    """
    root = simulator_root()
    if os.path.isfile(os.path.join(root, "mecanum_lab", "engine.py")) and root not in sys.path:
        sys.path.insert(0, root)
    return root


def lab_file(*parts: str) -> str:
    """Absolute path of one file of this lab, e.g. `lab_file("config", "production.json")`."""
    return os.path.join(lab_root(), *parts)
