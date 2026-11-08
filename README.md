# ohm_icp

Lab *ICP* for the simulator [ir-mecanum-lab](https://github.com/christianpfitzner/ir-mecanum-lab): scan matching
after Besl and McKay, first alone, then with the odometry as the start. Handout: `docs/icp.pdf` (`make` in `docs/`).

## Install

```bash
cd ~/ros2_ws/src
git clone git@github.com:christianpfitzner/ir-mecanum-lab.git
git clone git@github.com:christianpfitzner/ohm_icp.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select mecanum_lab_interfaces mecanum_lab ohm_icp
source install/setup.bash
```

## Run

| command | what |
|---|---|
| `ros2 launch ohm_icp icp.launch.py` | simulator in the hall `production`, the robot drives a loop, monitor, RViz |
| `ros2 run ohm_icp icp_node.py --ros-args -p task:=1` | your ICP: task 1 alone, task 2 with the odometry as the start |
| `ros2 run ohm_icp plot_run.py` | plots of the newest run in `runs/` |
| `python3 hint.py 1.3` | the next of three hints for `STUDENT 1.3`; without a block: the hints opened so far |

Your code: `student/icp.py`, `student/scan_odometry.py`. Launch arguments: `speed:=`, `robot:=`, `rviz:=`,
`headless:=`, `seconds:=`.

## Parameters of the node

| parameter | default | |
|---|---|---|
| `task` | 1 | 1 ICP alone · 2 ICP with the odometry as the start |
| `every` | 10 | match every n-th scan (20 scans per second) |
| `d_max` | 0.3 | m, pairs farther apart do not count; 0 = all count |
| `max_iter`, `eps` | 50, 1e-7 | at most this many iterations; stop when the error (m²) changed by less |
| `kdtree` | false | extra task: closest points with a kd-tree |

## Tests

```bash
python3 -m pytest test/test_icp.py -q -k task1
```
