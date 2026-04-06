"""
Build script to create a standalone executable using PyInstaller.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import PyInstaller.__main__


PROJECT_ROOT = Path(__file__).resolve().parent
ENTRY_SCRIPT = PROJECT_ROOT / "current-analyser.py"
DIST_DIR = PROJECT_ROOT / "dist"
BUILD_DIR = PROJECT_ROOT / "build"
APP_NAME = "RobotCurrentAnalyser"


def build_executable() -> None:
    """Build the standalone executable and copy supporting docs."""
    if not ENTRY_SCRIPT.exists():
        raise FileNotFoundError(f"Could not find entry script: {ENTRY_SCRIPT}")

    if DIST_DIR.exists():
        shutil.rmtree(DIST_DIR)
    if BUILD_DIR.exists():
        shutil.rmtree(BUILD_DIR)

    args = [
        str(ENTRY_SCRIPT),
        "--onefile",
        f"--name={APP_NAME}",
        "--windowed",
        "--clean",
        "--noconfirm",
    ]

    PyInstaller.__main__.run(args)

    readme_source = PROJECT_ROOT / "README.md"
    if readme_source.exists():
        shutil.copy2(readme_source, DIST_DIR / "README.md")

    print("Build complete! Executable created in the 'dist' folder.")
    print(f"Output: {DIST_DIR / f'{APP_NAME}.exe'}")


if __name__ == "__main__":
    build_executable()
