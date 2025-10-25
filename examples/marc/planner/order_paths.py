"""Ordering utilities for MARC plotting."""
from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

import numpy as np

from . import Color, OrderedStroke, SampledPath


def _path_endpoints(path: SampledPath) -> Tuple[np.ndarray, np.ndarray]:
    return path.points[0], path.points[-1]


def _path_length(path: SampledPath) -> float:
    diffs = np.diff(path.points, axis=0)
    return float(np.sum(np.linalg.norm(diffs, axis=1)))


def _choose_best_next(
    current_point: np.ndarray,
    candidates: Sequence[SampledPath],
) -> Tuple[int, SampledPath, bool]:
    """Choose the nearest neighbour path, optionally reversing it."""

    best_index = 0
    best_path = candidates[0]
    best_reverse = False
    best_distance = float("inf")

    for idx, candidate in enumerate(candidates):
        start, end = _path_endpoints(candidate)
        dist_start = float(np.linalg.norm(current_point - start))
        dist_end = float(np.linalg.norm(current_point - end))
        if dist_start < best_distance:
            best_index = idx
            best_path = candidate
            best_reverse = False
            best_distance = dist_start
        if dist_end < best_distance:
            best_index = idx
            best_path = candidate
            best_reverse = True
            best_distance = dist_end

    return best_index, best_path, best_reverse


def order_paths_by_colour(
    paths_by_colour: Dict[Color, List[SampledPath]],
    pen_up_threshold: float = 10.0,
) -> List[OrderedStroke]:
    """Order sampled paths colour-by-colour using a greedy heuristic."""

    ordered: List[OrderedStroke] = []

    def _colour_total_length(paths: Sequence[SampledPath]) -> float:
        return float(sum(_path_length(path) for path in paths))

    colour_items = sorted(
        paths_by_colour.items(),
        key=lambda item: _colour_total_length(item[1]),
        reverse=True,
    )

    for colour, paths in colour_items:
        if not paths:
            continue

        remaining = paths.copy()
        ordered_colour_paths: List[SampledPath] = []

        # Start with the path having the longest length as a simple heuristic.
        start_index = max(range(len(remaining)), key=lambda idx: _path_length(remaining[idx]))
        current_path = remaining.pop(start_index)
        ordered_colour_paths.append(current_path)
        current_position = current_path.points[-1]

        while remaining:
            idx, next_path, reverse = _choose_best_next(current_position, remaining)
            next_path = next_path.reversed() if reverse else next_path
            ordered_colour_paths.append(next_path)
            remaining.pop(idx)
            current_position = next_path.points[-1]

        for path in ordered_colour_paths:
            start_point = path.points[0]
            require_pen_up = True
            if ordered:
                prev_end = ordered[-1].points[-1]
                travel = float(np.linalg.norm(prev_end - start_point))
                require_pen_up = travel > pen_up_threshold
            ordered.append(
                OrderedStroke(
                    points=path.points,
                    color=colour,
                    pen_up=require_pen_up or not ordered,
                    attributes=dict(path.attributes),
                )
            )

    return ordered


__all__ = ["order_paths_by_colour"]
