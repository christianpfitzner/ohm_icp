"""The loop the robot drives in the hall `production` of the simulator (20 x 12 m), and its driver.

Counter-clockwise around the lower middle rack (x 8 .. 11.5, y 3.5 .. 5.5), 1 m or more from every rack:

    lower straight   y = 2.5,   x 7.25 -> 11.75   (the start)
    right straight   x = 12.5,  y 3.25 -> 6.0     (the aisle between the two lower racks)
    upper straight   y = 6.75,  x 11.75 -> 7.25   (the main aisle)
    left straight    x = 6.5,   y 6.0 -> 3.25

The spawn is in config/*.json; test/test_lab.py checks that it lies on the loop and that the loop keeps off the walls.
"""
import math

WORLD = "production"
CENTRE = (9.5, 4.625)
SIZE = (6.0, 4.25)                      # m, outer width and height of the loop
RADIUS = 0.75                           # m, corner radius
START = (8.5, 2.5, 0.0)                 # x, y, heading of the spawn, on the lower straight
STEP = 0.05                             # m between two points of the loop


def loop(step: float = STEP) -> list:
    """Points (x, y, heading) every `step` metres along the loop, starting at START."""
    (cx, cy), (w, h), r = CENTRE, SIZE, RADIUS
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    pieces = [("line", (x0 + r, y0), (x1 - r, y0)), ("arc", (x1 - r, y0 + r), -math.pi / 2),
              ("line", (x1, y0 + r), (x1, y1 - r)), ("arc", (x1 - r, y1 - r), 0.0),
              ("line", (x1 - r, y1), (x0 + r, y1)), ("arc", (x0 + r, y1 - r), math.pi / 2),
              ("line", (x0, y1 - r), (x0, y0 + r)), ("arc", (x0 + r, y0 + r), math.pi)]
    points = []
    for kind, a, b in pieces:
        if kind == "line":
            length = math.dist(a, b)
            heading = math.atan2(b[1] - a[1], b[0] - a[0])
            n = max(1, round(length / step))
            points += [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n, heading) for i in range(n)]
        else:
            n = max(1, round(r * math.pi / 2 / step))
            for i in range(n):
                phi = b + math.pi / 2 * i / n
                points.append((a[0] + r * math.cos(phi), a[1] + r * math.sin(phi), phi + math.pi / 2))
    first = min(range(len(points)), key=lambda i: math.dist(points[i][:2], START[:2]))
    return points[first:] + points[:first]


def length() -> float:
    (w, h), r = SIZE, RADIUS
    return 2 * (w + h) - 8 * r + 2 * math.pi * r


class Driver:
    """Follows the loop by the true pose: a point `lookahead` ahead, the heading along the loop.

    `step(t, x, y, theta)` gives the command (vx, vy, omega) in the frame of the robot. It starts with
    `accel` m/s^2 up to `speed` and turns with at most `turn` rad/s.
    """

    def __init__(self, speed: float = 0.5, accel: float = 0.3, lookahead: float = 0.6, turn: float = 1.2):
        self.speed, self.accel, self.lookahead, self.turn = speed, accel, lookahead, turn
        self.path = loop(STEP)
        self.index, self.laps, self.t0 = 0, 0, None

    def step(self, t: float, x: float, y: float, theta: float) -> tuple:
        n = len(self.path)
        if self.t0 is None or t < self.t0:                                 # the start, or the simulator restarted
            self.t0, self.laps = t, 0
            self.index = min(range(n), key=lambda i: math.dist(self.path[i][:2], (x, y)))
        window = range(self.index, self.index + int(2.0 / STEP))           # search 2 m ahead only
        nearest = min(window, key=lambda i: math.dist(self.path[i % n][:2], (x, y))) % n
        if nearest < self.index:
            self.laps += 1
        self.index = nearest
        speed = min(self.speed, 0.05 + self.accel * (t - self.t0))
        ahead = self.path[(nearest + int(self.lookahead / STEP)) % n]
        dx, dy = ahead[0] - x, ahead[1] - y
        d = math.hypot(dx, dy) or 1.0
        vx, vy = speed * dx / d, speed * dy / d                            # towards the point ahead, world
        heading = self.path[(nearest + int(0.5 * self.lookahead / STEP)) % n][2]
        error = math.atan2(math.sin(heading - theta), math.cos(heading - theta))
        omega = max(-self.turn, min(self.turn, 2.5 * error))
        return (math.cos(theta) * vx + math.sin(theta) * vy,             # into the frame of the robot
                -math.sin(theta) * vx + math.cos(theta) * vy, omega)
