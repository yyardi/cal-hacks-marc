"""Thin wrapper around the Potrace CLI for bitmap vectorisation."""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Iterable, Optional, Sequence, Tuple

import cv2

LOGGER = logging.getLogger(__name__)


def _resolve_binary(name: str) -> str:
    binary = shutil.which(name)
    if not binary:
        raise RuntimeError(
            "Potrace executable not found. Install it (e.g. `brew install potrace`) "
            "and ensure it is on PATH."
        )
    return binary


def _prepare_binary_bitmap(image_path: Path, threshold: float) -> Path:
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Cannot read image: {image_path}")

    if threshold <= 1.0:
        cutoff = threshold * 255.0
    else:
        cutoff = threshold
    cutoff = max(0.0, min(255.0, float(cutoff)))
    _, bw = cv2.threshold(image, cutoff, 255, cv2.THRESH_BINARY)

    fd, tmp_path = tempfile.mkstemp(suffix=".pbm")
    tmp = Path(tmp_path)
    try:
        with os.fdopen(fd, "wb") as handle:
            pass  # fdopen to ensure descriptor is managed on Windows
        cv2.imwrite(str(tmp), bw)
    except Exception:
        tmp.unlink(missing_ok=True)
        raise
    return tmp


def _build_cli_command(
    potrace_bin: str,
    bitmap_path: Path,
    output_path: Path,
    *,
    turdsize: int,
    opt_tolerance: float,
    page_size: Optional[Tuple[float, float]],
    extra_args: Optional[Sequence[str]],
) -> list[str]:
    cmd: list[str] = [
        potrace_bin,
        str(bitmap_path),
        "--svg",
        "--output",
        str(output_path),
        "--turdsize",
        str(int(turdsize)),
        "--opttolerance",
        str(float(opt_tolerance)),
    ]
    if page_size is not None:
        width, height = page_size
        cmd.extend(["--width", f"{float(width)}", "--height", f"{float(height)}"])
    if extra_args:
        cmd.extend(extra_args)
    return cmd


def trace_bitmap_to_svg(
    image_path: str | Path,
    output_path: str | Path | None = None,
    *,
    threshold: float = 128.0,
    turdsize: int = 2,
    opt_tolerance: float = 0.2,
    page_size: Optional[Tuple[float, float]] = None,
    potrace_binary: str = "potrace",
    extra_args: Optional[Sequence[str]] = None,
) -> Path:
    """Trace ``image_path`` into an SVG via the Potrace executable.

    Parameters
    ----------
    image_path:
        Input PNG (or any OpenCV-readable image). The bitmap is binarised prior
        to tracing.
    output_path:
        Destination path for the SVG file. Defaults to the source stem with
        ``.svg`` suffix.
    threshold:
        Grayscale threshold used when binarising the image. Values in ``[0, 1]``
        are interpreted as fractions of 255.
    turdsize, opt_tolerance:
        Directly forwarded to Potrace. Larger ``turdsize`` removes tiny blobs;
        smaller ``opttolerance`` produces more detailed curves.
    page_size:
        Optional (width, height) forwarded via ``--width``/``--height`` so the
        resulting SVG coordinates match the intended physical page size.
    potrace_binary:
        Name or path of the Potrace executable to run.
    extra_args:
        Additional CLI flags to append when launching Potrace.
    """

    image_path = Path(image_path)
    destination = Path(output_path) if output_path is not None else image_path.with_suffix(".svg")
    destination.parent.mkdir(parents=True, exist_ok=True)

    bitmap_path = _prepare_binary_bitmap(image_path, threshold)
    binary = _resolve_binary(potrace_binary)
    cmd = _build_cli_command(
        binary,
        bitmap_path,
        destination,
        turdsize=turdsize,
        opt_tolerance=opt_tolerance,
        page_size=page_size,
        extra_args=extra_args,
    )

    LOGGER.info("Running Potrace: %s", " ".join(cmd))
    try:
        subprocess.run(cmd, check=True)
    finally:
        bitmap_path.unlink(missing_ok=True)

    LOGGER.info("SVG written to %s", destination)
    return destination


def bitmap_to_svg(
    in_png: str | Path,
    out_svg: str | Path,
    *,
    threshold: int = 180,
    potrace_opts: Optional[Iterable[str]] = None,
) -> Path:
    """Compatibility wrapper mirroring the legacy helper signature."""

    return trace_bitmap_to_svg(
        in_png,
        out_svg,
        threshold=threshold,
        turdsize=2,
        opt_tolerance=0.2,
        page_size=None,
        extra_args=list(potrace_opts) if potrace_opts is not None else None,
    )


__all__ = ["trace_bitmap_to_svg", "bitmap_to_svg"]
