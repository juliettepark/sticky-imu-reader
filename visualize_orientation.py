#!/usr/bin/env python3
"""Play a colored cube rotating through an orientation CSV.

The cube stays at the origin. Its faces follow qw, qx, qy, qz. Up in the view
is the same up used to build the starting quaternion. Pass --save to also write
a GIF beside the CSV.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

QUAT_COLUMNS = ("qw", "qx", "qy", "qz")
# About 30 frames per second, played on the sticky's clock.
FRAME_INTERVAL_MS = 33
# Equal sides, so the drawn object is a cube.
BOX_EXTENTS = np.array([1.0, 1.0, 1.0])
FACE_COLORS = (
    "#13496e",  # -z
    "#1f77b4",  # +z, up
    "#1b661b",  # -y
    "#2ca02c",  # +y
    "#8c1d1d",  # -x
    "#d62728",  # +x
)


# Read time and quaternion columns. Time is shifted so the first row is 0.
def load_orientations(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in ("t_fw_ms", *QUAT_COLUMNS)):
            raise ValueError(f"{path} is missing t_fw_ms or the quaternion columns")
        times = []
        quaternions = []
        for row in reader:
            times.append(float(row["t_fw_ms"]))
            quaternions.append([float(row[name]) for name in QUAT_COLUMNS])
    if not times:
        raise ValueError(f"{path} has no orientation rows")
    time_s = (np.array(times) - times[0]) / 1000.0
    return time_s, np.array(quaternions)


# Rotation matrix for a (w, x, y, z) quaternion. Columns are the box axes in the view.
def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


# Six faces of the box, each as four corners, in the box's own axes.
def box_faces() -> np.ndarray:
    x, y, z = BOX_EXTENTS
    corners = np.array(
        [
            [-x, -y, -z],
            [x, -y, -z],
            [x, y, -z],
            [-x, y, -z],
            [-x, -y, z],
            [x, -y, z],
            [x, y, z],
            [-x, y, z],
        ]
    )
    face_indices = (
        (0, 1, 2, 3),  # -z
        (4, 5, 6, 7),  # +z
        (0, 1, 5, 4),  # -y
        (3, 2, 6, 7),  # +y
        (0, 3, 7, 4),  # -x
        (1, 2, 6, 5),  # +x
    )
    return np.array([corners[list(indices)] for indices in face_indices])


# Keep every sample that lands nearest each playback frame.
def playback_frames(time_s: np.ndarray) -> np.ndarray:
    if len(time_s) == 1:
        return np.array([0])
    duration_s = time_s[-1] - time_s[0]
    frame_times = np.arange(0.0, duration_s, FRAME_INTERVAL_MS / 1000.0)
    if frame_times.size == 0 or frame_times[-1] < duration_s:
        frame_times = np.append(frame_times, duration_s)
    return np.searchsorted(time_s, frame_times, side="left").clip(0, len(time_s) - 1)


# Build the rotating cube. Playback follows the sticky clock at about 30 frames per second.
def animate(time_s: np.ndarray, quaternions: np.ndarray) -> FuncAnimation:
    faces = box_faces()
    frames = playback_frames(time_s)
    figure = plt.figure(figsize=(6, 6))
    axes = figure.add_subplot(111, projection="3d")
    collection = Poly3DCollection(
        faces, facecolors=FACE_COLORS, edgecolors="black", linewidths=0.4
    )
    axes.add_collection3d(collection)
    limit = float(np.max(BOX_EXTENTS) * 1.4)
    axes.set_xlim(-limit, limit)
    axes.set_ylim(-limit, limit)
    axes.set_zlim(-limit, limit)
    axes.set_box_aspect((1, 1, 1))
    axes.set_xlabel("x")
    axes.set_ylabel("y")
    axes.set_zlabel("z (up)")

    def update(frame_index: int):
        sample_index = int(frames[frame_index])
        rotation = quaternion_to_matrix(quaternions[sample_index])
        # Row vectors rotate by R transpose when R is the matrix for column vectors.
        rotated = faces @ rotation.T
        collection.set_verts(list(rotated))
        axes.set_title(f"t = {time_s[sample_index]:.2f} s")
        return (collection,)

    animation = FuncAnimation(
        figure, update, frames=len(frames), interval=FRAME_INTERVAL_MS, blit=False
    )
    return animation


# Save the animation next to the CSV, with the same name and a .gif suffix.
def save_animation(animation: FuncAnimation, csv_path: Path) -> Path:
    gif_path = csv_path.with_suffix(".gif")
    animation.save(gif_path, writer=PillowWriter(fps=round(1000 / FRAME_INTERVAL_MS)))
    return gif_path


# Accept the path of a CSV written by orient_imu.py.
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Animate a cube from an orientation CSV.")
    parser.add_argument("csv_path", type=Path, help="CSV written by orient_imu.py")
    parser.add_argument(
        "--save",
        action="store_true",
        help="Also write a GIF beside the CSV.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    path = args.csv_path
    try:
        time_s, quaternions = load_orientations(path)
    except (OSError, ValueError, csv.Error) as error:
        print(error, file=sys.stderr)
        return 1
    animation = animate(time_s, quaternions)
    if args.save:
        gif_path = save_animation(animation, path)
        print(f"Writing {gif_path.resolve()}")
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
