"""What `launch/icp.launch.py` starts: the simulator in the hall `production`, the driver, the monitor, RViz.

The simulator's own `lab.launch.py` (repository `ir-mecanum-lab`) is included with config/production.json and
with its own RViz switched off: this lab brings an RViz config of its own (`rviz/icp.rviz`). The driver
(nodes/route_node.py) drives the loop, the monitor (nodes/monitor_node.py) prints how good odometry and ICP
are. The students' node is not started here; they run it themselves in a second terminal.
"""
import os
import shutil
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, \
    OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from . import paths

ARGS = [
    ("robot", "muster", "robot name"),
    ("speed", "0.5", "speed of the driver along the loop, m/s"),
    ("rviz", "auto", "RViz 2 with the lab's view: auto | true | false"),
    ("headless", "false", "without the window (SDL dummy driver)"),
    ("seconds", "", "end after N s of simulation time (empty = until Ctrl-C)"),
]
NODE = "ros2 run ohm_icp icp_node.py --ros-args -p task:=1"


def rviz_config(robot: str) -> str:
    """Fill the `{robot}` placeholder of the lab's RViz template; returns the generated file."""
    with open(paths.lab_file("rviz", "icp.rviz"), encoding="utf-8") as handle:
        text = handle.read()
    target = f"/tmp/ohm_icp_{robot}.rviz"
    with open(target, "w", encoding="utf-8") as handle:
        handle.write(text.replace("{robot}", robot))
    return target


def given_node(name: str, robot: str, *parameters: str) -> ExecuteProcess:
    """One of nodes/*.py, started with the Python of this launch, from the checkout or the install."""
    command = [sys.executable, paths.lab_file("nodes", name), "--ros-args", "-p", f"robot:={robot}"]
    for parameter in parameters:
        command += ["-p", parameter]
    return ExecuteProcess(cmd=command, name=name[:-3], output="screen", emulate_tty=True)


def _after_exit(event, _context):
    """The simulator ended (window closed, `seconds:=` reached): driver, monitor and RViz end with it."""
    if event.action.name.startswith("mecanum_sim"):
        return [Shutdown(reason="the simulator ended")]
    return None


def _start(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)   # noqa: E731
    simulator = paths.simulator_root()
    robot = arg("robot")
    forwarded = {"config": paths.lab_file("config", "production.json"), "robot": robot, "rviz": "false",
                 "truth": "true", "headless": arg("headless")}
    if arg("seconds"):
        forwarded["seconds"] = arg("seconds")
    actions = [LogInfo(msg=f"[icp] robot '{robot}' drives the loop in the hall 'production' "
                           f"(simulator {simulator}); start your node in a second terminal: {NODE}"),
               IncludeLaunchDescription(
                   PythonLaunchDescriptionSource(os.path.join(simulator, "launch", "lab.launch.py")),
                   launch_arguments=forwarded.items()),
               given_node("route_node.py", robot, f"speed:={arg('speed')}"),
               given_node("monitor_node.py", robot),
               RegisterEventHandler(OnProcessExit(on_exit=_after_exit))]
    wanted = arg("rviz").lower()
    if wanted == "true" or (wanted == "auto" and shutil.which("rviz2")):
        actions.append(ExecuteProcess(cmd=["rviz2", "--display-config", rviz_config(robot),
                                           "--ros-args", "-p", "use_sim_time:=true"],
                                      name="rviz2", output="screen"))
    elif wanted == "auto":
        actions.append(LogInfo(msg="[icp] rviz2 is not installed: sudo apt install ros-$ROS_DISTRO-rviz2"))
    return actions


def describe() -> LaunchDescription:
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=text)
                              for name, default, text in ARGS]
                             + [OpaqueFunction(function=_start)])
