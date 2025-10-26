"""Helper to download the SO101 URDF used by the MARC drawing pipeline."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
from urllib.request import urlopen

SO101_URDF_URL = (
    "https://raw.githubusercontent.com/TheRobotStudio/"
    "SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf"
)
# Hash of the upstream URDF at commit 4b4d1a7 (2024-04-20)
SO101_URDF_SHA256 = (
    "e9a7295a56d8a3d6d1fcf1f3dcd89e5cd9c27972f4d5556b06d66b6ad72037c8"
)


def compute_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def download_urdf(dest: Path, overwrite: bool = False) -> Path:
    dest.parent.mkdir(parents=True, exist_ok=True)

    if dest.exists():
        existing_hash = compute_sha256(dest)
        if existing_hash == SO101_URDF_SHA256:
            return dest
        if not overwrite:
            raise FileExistsError(
                f"Destination {dest} already exists with a different SHA256 hash. "
                "Pass --overwrite if you want to replace it."
            )

    with urlopen(SO101_URDF_URL) as response:
        data = response.read()

    tmp_path = dest.with_suffix(dest.suffix + ".tmp")
    tmp_path.write_bytes(data)

    downloaded_hash = compute_sha256(tmp_path)
    if downloaded_hash != SO101_URDF_SHA256:
        tmp_path.unlink(missing_ok=True)
        raise RuntimeError(
            "Downloaded URDF hash mismatch. "
            f"Expected {SO101_URDF_SHA256}, got {downloaded_hash}."
        )

    tmp_path.replace(dest)
    return dest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("examples/marc/SO101/so101_new_calib.urdf"),
        help="Where to save the URDF (default: examples/marc/SO101/so101_new_calib.urdf)",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace the destination file if it already exists with a different hash.",
    )
    args = parser.parse_args(argv)

    try:
        dest = download_urdf(args.output, overwrite=args.overwrite)
    except Exception as exc:  # pragma: no cover - defensive user-facing CLI errors
        parser.error(str(exc))
    else:
        print(f"Downloaded URDF to {dest}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
