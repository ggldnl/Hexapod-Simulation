import numpy as np


def _make_circle(radius: float = 1.0, center: tuple = (0, 0, 0), n: int = 64):
    """Builds a set of segments discretizing the desired circle."""

    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = np.stack([
        radius * np.cos(theta),
        radius * np.sin(theta),
        np.zeros_like(theta),
    ], axis=1)

    pts = pts + np.array(center)
    pts_next = np.roll(pts, -1, axis=0)
    return np.stack([pts, pts_next], axis=1)


def _make_line(start: tuple = (0, 0, 0), end: tuple = (1, 0, 0)):
    """Builds a single segment representing the desired line."""

    pts = np.array([start, end], dtype=float)
    return pts.reshape(1, 2, 3)


def _make_point(position: tuple = (0, 0, 0)):
    """Builds a single 3D point."""

    return np.array(position, dtype=float).reshape(1, 3)


def add_circle(
        server,
        name='/circle',

        # Circle
        radius: float = 1.0,
        center: tuple = (0, 0, 0),
        n: int = 64,

        # Style
        line_width=2.0,
        colors=(255, 0, 0)
):
    """Adds the desired circle to the visualization."""

    circle_pts = _make_circle(radius=radius, center=center, n=n)
    server.scene.add_line_segments(
        name,
        points=circle_pts,
        line_width=line_width,
        colors=colors
    )


def add_line(
        server,
        name='/line',

        # Line
        start: tuple = (0, 0, 0),
        end: tuple = (1, 0, 0),

        # Style
        line_width=2.0,
        colors=(255, 0, 0)
):
    """Adds the desired line to the visualization."""

    line_pts = _make_line(start=start, end=end)
    server.scene.add_line_segments(
        name,
        points=line_pts,
        line_width=line_width,
        colors=colors
    )


def add_point(
        server,
        name='/point',

        # Point
        position: tuple = (0, 0, 0),

        # Style
        point_size=5.0,
        colors=(255, 0, 0)
):
    """Adds the desired point to the visualization."""

    point_pts = _make_point(position=position)
    server.scene.add_point_cloud(
        name,
        points=point_pts,
        point_size=point_size,
        colors=colors
    )