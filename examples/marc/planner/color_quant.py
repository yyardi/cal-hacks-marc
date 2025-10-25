"""Color quantisation helpers for MARC plotting."""
from __future__ import annotations

from typing import Dict, List, Mapping, Sequence, Tuple

from . import Color, SampledPath, color_distance

# Basic set of alcohol marker-like colours (rough approximation).
# Values are expressed in RGB 0-255 space.
MARKER_PALETTE: Dict[str, Color] = {
    "black": (20, 20, 20),
    "cool_gray": (128, 130, 133),
    "warm_gray": (176, 162, 149),
    "blue": (31, 88, 181),
    "sky_blue": (96, 165, 223),
    "green": (34, 139, 34),
    "lime": (120, 190, 32),
    "yellow": (242, 201, 76),
    "orange": (242, 125, 53),
    "red": (214, 45, 57),
    "magenta": (199, 42, 141),
    "purple": (116, 61, 172),
    "brown": (120, 72, 33),
}


def _nearest_palette_colour(color: Color, palette: Mapping[str, Color]) -> Tuple[str, Color, float]:
    min_distance = float("inf")
    min_name = ""
    min_color: Color = (0, 0, 0)
    for name, palette_color in palette.items():
        distance = color_distance(color, palette_color)
        if distance < min_distance:
            min_distance = distance
            min_name = name
            min_color = palette_color
    return min_name, min_color, min_distance


def quantize_paths(
    paths: Sequence[SampledPath],
    palette: Mapping[str, Color] | None = None,
    merge_distance: float = 18.0,
) -> Dict[Color, List[SampledPath]]:
    """Quantise path colours to the supported marker palette.

    Parameters
    ----------
    paths:
        Sampled polylines to quantise.
    palette:
        Mapping of colour names to RGB tuples. If omitted, :data:`MARKER_PALETTE`
        is used.
    merge_distance:
        Maximum Euclidean distance (0-255 RGB space) for two palette entries to
        be considered equivalent. This helps merge near-duplicate hues when the
        marker palette contains similar shades.

    Returns
    -------
    dict
        Mapping from quantised RGB colours to the paths using that colour.
    """

    if palette is None:
        palette = MARKER_PALETTE

    grouped: Dict[Color, List[SampledPath]] = {}

    for path in paths:
        name, palette_color, distance = _nearest_palette_colour(path.color, palette)
        path_color: Color = palette_color
        path.attributes.setdefault("color_name", name)
        path.color = path_color

        # Merge near-identical palette colours into a single bucket.
        matched_key: Color | None = None
        for existing_color in grouped.keys():
            if color_distance(existing_color, path_color) <= merge_distance:
                matched_key = existing_color
                break
        if matched_key is None:
            matched_key = path_color
            grouped[matched_key] = []
        grouped[matched_key].append(path)

    return grouped


__all__ = ["MARKER_PALETTE", "quantize_paths"]
