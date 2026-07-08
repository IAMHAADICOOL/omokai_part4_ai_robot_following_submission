"""Formation offset geometry -- pure functions, unit-testable without ROS.

Given a center path (points the SQUAD centroid follows) and a formation type,
compute each robot's own path by offsetting relative to the direction of
travel at each segment. Robot index 0 is the lead / center reference.
"""
import math


def _heading(p0, p1):
    return math.atan2(p1[1] - p0[1], p1[0] - p0[0])


def _perp(theta):
    return (-math.sin(theta), math.cos(theta))


def _along(theta):
    return (math.cos(theta), math.sin(theta))


def offsets_for_robot(index, n_robots, formation_type, spacing_m):
    """Return (lateral, longitudinal) offset MULTIPLES of spacing for a
    robot at `index` in a squad of `n_robots`, centered on the path."""
    center = (n_robots - 1) / 2.0
    k = index - center

    if formation_type == "line":
        return (k, 0.0)                 # side by side, across heading
    if formation_type == "column":
        return (0.0, -float(index))     # single file, each directly behind
    if formation_type == "wedge":
        if index == 0:
            return (0.0, 0.0)           # apex on the path
        side = 1 if index % 2 == 1 else -1
        rank = (index + 1) // 2
        return (side * rank, -rank)     # fan out behind the apex
    raise ValueError(f"unknown formation_type {formation_type}")


def robot_path(center_path, index, n_robots, formation_type, spacing_m):
    """Compute one robot's (x,y) path from the squad's center_path."""
    lateral, longitudinal = offsets_for_robot(
        index, n_robots, formation_type, spacing_m)
    out = []
    n = len(center_path)
    for i, (x, y) in enumerate(center_path):
        if i < n - 1:
            theta = _heading(center_path[i], center_path[i + 1])
        else:
            theta = _heading(center_path[i - 1], center_path[i]) if n > 1 else 0.0
        px, py = _perp(theta)
        ax, ay = _along(theta)
        ox = px * lateral * spacing_m + ax * longitudinal * spacing_m
        oy = py * lateral * spacing_m + ay * longitudinal * spacing_m
        out.append((x + ox, y + oy))
    return out
