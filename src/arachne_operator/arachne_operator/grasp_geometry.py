from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


REACH_X = (0.46, 0.96)
REACH_Y = (-0.55, 0.22)


@dataclass(frozen=True)
class GraspGeometry:
    approach: tuple[float, float, float]
    grasp: tuple[float, float, float]
    lift: tuple[float, float, float]
    center: tuple[float, float, float]
    extent: tuple[float, float, float]
    reachable: bool
    point_count: int


def pointcloud_grasp_geometry(
    points_base: Iterable[Iterable[float]],
    *,
    reach_x: tuple[float, float] = REACH_X,
    reach_y: tuple[float, float] = REACH_Y,
) -> GraspGeometry | None:
    points = np.asarray(list(points_base), dtype=np.float64)
    if points.size == 0:
        return None
    points = points.reshape((-1, 3))
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < 6:
        return None

    low, high = np.percentile(points, [2.0, 98.0], axis=0)
    trimmed = points[np.all((points >= low) & (points <= high), axis=1)]
    if trimmed.shape[0] >= 6:
        points = trimmed

    lower = points.min(axis=0)
    upper = points.max(axis=0)
    center = (lower + upper) * 0.5
    extent = upper - lower

    grasp = (float(center[0]), float(center[1]), float(upper[2] + 0.02))
    approach = (grasp[0] - 0.04, grasp[1], grasp[2] + 0.08)
    lift = (grasp[0], grasp[1], grasp[2] + 0.12)
    reachable = reach_x[0] <= grasp[0] <= reach_x[1] and reach_y[0] <= grasp[1] <= reach_y[1]
    return GraspGeometry(
        approach=approach,
        grasp=grasp,
        lift=lift,
        center=(float(center[0]), float(center[1]), float(center[2])),
        extent=(float(extent[0]), float(extent[1]), float(extent[2])),
        reachable=reachable,
        point_count=int(points.shape[0]),
    )


def _demo() -> None:
    rng = np.random.default_rng(26)
    points = rng.uniform((-0.08, -0.04, 0.0), (0.08, 0.04, 0.06), size=(180, 3))
    points += np.asarray((0.70, -0.12, -0.22), dtype=np.float64)
    result = pointcloud_grasp_geometry(points)
    assert result is not None
    assert result.reachable
    assert math.isclose(result.extent[0], 0.16, abs_tol=0.02)


if __name__ == "__main__":
    _demo()
