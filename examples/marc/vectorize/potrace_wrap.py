"""Light-weight helpers around Potrace raster-to-vector conversion."""
from __future__ import annotations

import logging
import shutil
import subprocess
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple

LOGGER = logging.getLogger(__name__)

try:  # pragma: no cover - optional dependency
    import numpy as _np
    import potrace as _potrace
    from PIL import Image as _Image
except Exception:  # pragma: no cover - these are optional, fallback to CLI
    _potrace = None
    _np = None
    _Image = None

SVG_HEADER = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\"" " width=\"{w}\" height=\"{h}\" viewBox=\"0 0 {w} {h}\">"
SVG_FOOTER = "</svg>"


def _threshold_to_blacklevel(threshold: float) -> float:
    if threshold > 1:
        return max(0.0, min(1.0, threshold / 255.0))
    return max(0.0, min(1.0, threshold))


def _trace_with_cli(
    image_path: Path,
    output_path: Path,
    *,
    threshold: float,
    turdsize: int,
    opt_tolerance: float,
    extra_args: Optional[Sequence[str]] = None,
) -> Path:
    potrace_exe = shutil.which("potrace")
    if not potrace_exe:
        raise RuntimeError(
            "Potrace command line tool not found and py_potrace is unavailable."
        )

    cmd = [
        potrace_exe,
        str(image_path),
        "--svg",
        "--output",
        str(output_path),
        "--turdsize",
        str(turdsize),
        "--opttolerance",
        str(opt_tolerance),
        "--blacklevel",
        str(_threshold_to_blacklevel(threshold)),
    ]
    if extra_args:
        cmd.extend(extra_args)

    LOGGER.info("Running potrace: %s", " ".join(cmd))
    subprocess.run(cmd, check=True)
    return output_path


def _curve_to_path_commands(curve) -> Iterable[str]:  # pragma: no cover - requires py_potrace
    commands = [f"M {curve.start_point.x} {curve.start_point.y}"]
    for segment in curve:
        if segment.is_corner:
            c = segment.c
            end = segment.end_point
            commands.append(f"L {c.x} {c.y}")
            commands.append(f"L {end.x} {end.y}")
        else:
            c1, c2 = segment.c1, segment.c2
            end = segment.end_point
            commands.append(
                f"C {c1.x} {c1.y} {c2.x} {c2.y} {end.x} {end.y}"
            )
    commands.append("Z")
    return commands


def _trace_with_py_potrace(
    image_path: Path,
    *,
    threshold: float,
    turdsize: int,
    opt_tolerance: float,
    page_size: Optional[Tuple[int, int]] = None,
) -> str:
    if _potrace is None or _np is None or _Image is None:
        raise RuntimeError("py_potrace is not available")

    grayscale = _Image.open(image_path).convert("L")
    width, height = grayscale.size
    array = _np.array(grayscale, dtype=_np.float32)
    cutoff = threshold if threshold > 1 else threshold * 255.0
    bitmap = _np.where(array > cutoff, 0, 1).astype(_np.uint8)
    bmp = _potrace.Bitmap(bitmap)
    traced = bmp.trace(turdsize=turdsize, alphamax=1.0, opttolerance=opt_tolerance)

    paths = []
    for curve in traced:
        commands = " ".join(_curve_to_path_commands(curve))
        paths.append(f"<path d=\"{commands}\" fill=\"black\"/>")

    w, h = page_size or (width, height)
    svg_content = f"{SVG_HEADER.format(w=w, h=h)}{''.join(paths)}{SVG_FOOTER}"
    return svg_content


def trace_bitmap_to_svg(
    image_path: Path,
    output_path: Optional[Path] = None,
    *,
    threshold: float = 128,
    turdsize: int = 2,
    opt_tolerance: float = 0.2,
    page_size: Optional[Tuple[int, int]] = None,
    backend: Optional[str] = None,
    extra_args: Optional[Sequence[str]] = None,
) -> Path:
    """Trace ``image_path`` into an SVG file and return the output path."""

    image_path = Path(image_path)
    if output_path is None:
        output_path = image_path.with_suffix(".svg")
    else:
        output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    backend = backend or ("py_potrace" if _potrace is not None else "cli")

    if backend == "py_potrace":  # pragma: no cover - optional dependency
        svg_text = _trace_with_py_potrace(
            image_path,
            threshold=threshold,
            turdsize=turdsize,
            opt_tolerance=opt_tolerance,
            page_size=page_size,
        )
        output_path.write_text(svg_text, encoding="utf-8")
    else:
        extra = list(extra_args) if extra_args else []
        if page_size:
            width, height = page_size
            extra.extend(["-W", str(width), "-H", str(height)])
        _trace_with_cli(
            image_path,
            output_path,
            threshold=threshold,
            turdsize=turdsize,
            opt_tolerance=opt_tolerance,
            extra_args=extra,
        )

    LOGGER.info("SVG written to %s", output_path)
    return output_path
