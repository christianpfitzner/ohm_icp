"""How good is a pose estimate? The numbers the monitor prints, the same for nodes/monitor_node.py and tools/.

One `Run` is one start of the students' node. Truth, odometry and the ICP pose are taken at the same
instants (every match) and compared from the start of the run on: all three begin at the same pose, so
an error is what the estimate got wrong since then, not where the odometry already was.

    driven    the length of each path, m
    turned    the sum of every turn, deg: a lap of the loop is 360
    error     position and heading of the odometry and of ICP against the truth, now
"""
import math

from . import route


def compose(a, b):
    """a then b, poses (x, y, theta): b seen from a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Run:

    NAMES = ("truth", "odom", "icp")

    def __init__(self, lap: float = route.length()):
        self.lap = lap
        self.start = None                        # the first pose of truth, odometry and ICP
        self.last = None                         # the last relative poses
        self.driven = dict.fromkeys(self.NAMES, 0.0)
        self.turned = dict.fromkeys(self.NAMES, 0.0)
        self.error = {"odom": (0.0, 0.0), "icp": (0.0, 0.0)}      # m, rad
        self.worst = {"odom": 0.0, "icp": 0.0}                   # largest position error, m
        self.laps, self.t = 0, None

    def add(self, t: float, truth, odom, icp) -> list:
        """One instant; returns the laps that were completed with it, as (lap, odom error, icp error)."""
        poses = dict(zip(self.NAMES, (truth, odom, icp)))
        if self.start is None:
            self.start, self.t0 = poses, t
            self.last = {name: (0.0, 0.0, 0.0) for name in self.NAMES}
            self.t = t
            return []
        self.t = t
        rel = {name: compose(inverse(self.start[name]), poses[name]) for name in self.NAMES}
        for name in self.NAMES:
            self.driven[name] += math.dist(self.last[name][:2], rel[name][:2])
            self.turned[name] += abs(wrap(rel[name][2] - self.last[name][2]))
        self.last = rel
        for name in ("odom", "icp"):
            e = compose(inverse(rel["truth"]), rel[name])
            self.error[name] = (math.hypot(e[0], e[1]), abs(e[2]))
            self.worst[name] = max(self.worst[name], self.error[name][0])
        done = []
        while self.driven["truth"] >= (self.laps + 1) * self.lap:
            self.laps += 1
            done.append((self.laps, self.error["odom"], self.error["icp"]))
        return done

    def aligned(self, name: str, pose):
        """A pose of `name`, moved so that its run starts at the start of ICP (what RViz draws)."""
        return compose(self.start["icp"], compose(inverse(self.start[name]), pose))

    def line(self) -> str:
        d, r, e = self.driven, self.turned, self.error
        return (f"t {self.t:6.1f} s | driven: truth {d['truth']:5.1f} m, odom {d['odom']:5.1f} m, ICP {d['icp']:5.1f} m"
                f" | turned: truth {math.degrees(r['truth']):4.0f}, odom {math.degrees(r['odom']):4.0f}, "
                f"ICP {math.degrees(r['icp']):4.0f} deg | error: odom {e['odom'][0]:.2f} m "
                f"{math.degrees(e['odom'][1]):4.1f} deg, ICP {e['icp'][0]:.2f} m {math.degrees(e['icp'][1]):4.1f} deg")


def lap_line(lap: int, odom, icp, matches: int = 0, iterations: float = 0.0, ms: float = 0.0) -> str:
    text = (f"lap {lap} ({lap * route.length():.0f} m): odom {odom[0]:.2f} m {math.degrees(odom[1]):4.1f} deg | "
            f"ICP {icp[0]:.2f} m {math.degrees(icp[1]):4.1f} deg")
    if matches:
        text += f" | {matches} matches, {iterations:.0f} iterations and {ms:.0f} ms per match"
    return text
