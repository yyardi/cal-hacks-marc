"""Helper to download the SO101 URDF and mesh assets for the MARC pipeline."""

from __future__ import annotations

import argparse
import hashlib
import io
import shutil
import zipfile
from pathlib import Path
from urllib.request import urlopen

SO101_ARCHIVE_URL = (
    "https://codeload.github.com/TheRobotStudio/"
    "SO-ARM100/zip/refs/heads/main"
)
SO101_ARCHIVE_SUBDIR = "Simulation/SO101"
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


def _clear_path(path: Path) -> None:
    if not path.exists():
        return
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def download_so101_package(dest_dir: Path, overwrite: bool = False) -> Path:
    dest_dir = dest_dir.resolve()

    if dest_dir.exists():
        existing_contents = list(dest_dir.iterdir()) if dest_dir.is_dir() else [dest_dir]
        if existing_contents and not overwrite:
            raise FileExistsError(
                f"Destination {dest_dir} already exists. Pass --overwrite to replace it."
            )
        for entry in existing_contents:
            _clear_path(entry)

    dest_dir.mkdir(parents=True, exist_ok=True)

    with urlopen(SO101_ARCHIVE_URL) as response:
        archive_data = response.read()

    with zipfile.ZipFile(io.BytesIO(archive_data)) as archive:
        namelist = archive.namelist()
        if not namelist:  # pragma: no cover - defensive
            raise RuntimeError("Downloaded archive is empty")
        root_prefix = namelist[0].split("/", 1)[0]
        target_prefix = f"{root_prefix}/{SO101_ARCHIVE_SUBDIR.strip('/')}/"

        members = [name for name in namelist if name.startswith(target_prefix)]
        if not members:
            raise RuntimeError(
                "Unable to locate Simulation/SO101 inside the downloaded archive."
            )

        for member in members:
            if member.endswith("/"):
                continue
            relative_name = member[len(target_prefix) :]
            if not relative_name:
                continue
            target_path = dest_dir / relative_name
            target_path.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(member) as src, target_path.open("wb") as dst:
                shutil.copyfileobj(src, dst)

    urdf_path = dest_dir / "so101_new_calib.urdf"
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"URDF was not found inside the extracted archive at {urdf_path}"
        )

    downloaded_hash = compute_sha256(urdf_path)
    if downloaded_hash != SO101_URDF_SHA256:
        raise RuntimeError(
            "Downloaded URDF hash mismatch. "
            f"Expected {SO101_URDF_SHA256}, got {downloaded_hash}."
        )

    return urdf_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("examples/marc/SO101"),
        help="Directory where the URDF and mesh assets will be stored",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace the destination directory if it already exists.",
    )
    args = parser.parse_args(argv)

    try:
        dest = download_so101_package(args.output_dir, overwrite=args.overwrite)
    except Exception as exc:  # pragma: no cover - defensive user-facing CLI errors
        parser.error(str(exc))
    else:
        print(f"Downloaded SO101 assets to {dest.parent}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
