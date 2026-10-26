#!/usr/bin/env python3
"""Play a colored cube rotating through an orientation CSV or a dual recording.

The cube stays at the origin. Its faces follow qw, qx, qy, qz. Up in the view
is the same up used to build the starting quaternion. Pass a recording folder
to show the ball and the hand side by side on the laptop clock. Pass --save to
also write a GIF.
"""

from __future__ import annotations

import argparse
import csv
import sys
from datetime import datetime
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


# Read combined.csv. Time is seconds from the first laptop timestamp.
# Empty quaternion cells stay missing until that sticky's hold has ended.
def load_combined(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        required = ("t_host", "ball_qw", "ball_qx", "ball_qy", "ball_qz", "hand_qw", "hand_qx", "hand_qy", "hand_qz")
        if reader.fieldnames is None or any(name not in reader.fieldnames for name in required):
            raise ValueError(f"{path} is missing t_host or the ball and hand quaternion columns")
        times = []
        ball = []
        hand = []
        for row in reader:
            times.append(datetime.fromisoformat(row["t_host"]))
            ball.append([_optional_float(row[f"ball_{name}"]) for name in QUAT_COLUMNS])
            hand.append([_optional_float(row[f"hand_{name}"]) for name in QUAT_COLUMNS])
    if not times:
        raise ValueError(f"{path} has no rows")
    time_s = np.array([(stamp - times[0]).total_seconds() for stamp in times])
    ball_quaternions = _fill_orientation(np.array(ball), _starting_quaternion(path.parent, "ball"))
    hand_quaternions = _fill_orientation(np.array(hand), _starting_quaternion(path.parent, "hand"))
    return time_s, ball_quaternions, hand_quaternions


def _optional_float(value: str) -> float:
    if value == "":
        return np.nan
    return float(value)


# The hold rows have no quaternion yet. Use the starting quaternion from
# calibration.csv for those rows, then keep the latest orientation across a gap.
def _fill_orientation(quaternions: np.ndarray, initial: np.ndarray | None) -> np.ndarray:
    filled = quaternions.copy()
    if initial is not None:
        for index in range(len(filled)):
            if np.isnan(filled[index, 0]):
                filled[index] = initial
            else:
                break
    latest = None
    for index in range(len(filled)):
        if np.isnan(filled[index, 0]):
            if latest is not None:
                filled[index] = latest
        else:
            latest = filled[index].copy()
    return filled


def _starting_quaternion(folder: Path, sticky: str) -> np.ndarray | None:
    path = folder / "calibration.csv"
    if not path.exists():
        return None
    with path.open(newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            if row.get("sticky") == sticky and row.get("qw"):
                return np.array([float(row[name]) for name in QUAT_COLUMNS])
    return None


# Keep every sample that lands nearest each playback frame.
def playback_frames(time_s: np.ndarray) -> np.ndarray:
    if len(time_s) == 1:
        return np.array([0])
    duration_s = time_s[-1] - time_s[0]
    frame_times = np.arange(0.0, duration_s, FRAME_INTERVAL_MS / 1000.0)
    if frame_times.size == 0 or frame_times[-1] < duration_s:
        frame_times = np.append(frame_times, duration_s)
    return np.searchsorted(time_s, frame_times, side="left").clip(0, len(time_s) - 1)


def _add_cube(axes, title: str) -> tuple[Poly3DCollection, np.ndarray]:
    faces = box_faces()
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
    axes.set_title(title)
    return collection, faces


def _set_orientation(collection: Poly3DCollection, faces: np.ndarray, quaternion: np.ndarray) -> None:
    if np.isnan(quaternion[0]):
        return
    rotation = quaternion_to_matrix(quaternion)
    # Row vectors rotate by R transpose when R is the matrix for column vectors.
    collection.set_verts(list(faces @ rotation.T))


# Build the rotating cube. Playback follows the sticky clock at about 30 frames per second.
def animate(time_s: np.ndarray, quaternions: np.ndarray) -> FuncAnimation:
    frames = playback_frames(time_s)
    figure = plt.figure(figsize=(6, 6))
    axes = figure.add_subplot(111, projection="3d")
    collection, faces = _add_cube(axes, "")

    def update(frame_index: int):
        sample_index = int(frames[frame_index])
        _set_orientation(collection, faces, quaternions[sample_index])
        axes.set_title(f"t = {time_s[sample_index]:.2f} s")
        return (collection,)

    animation = FuncAnimation(
        figure, update, frames=len(frames), interval=FRAME_INTERVAL_MS, blit=False
    )
    return animation


# Ball and hand share one laptop clock. Each frame uses the combined.csv row
# nearest that host time, so the two cubes stay lined up.
def animate_pair(
    time_s: np.ndarray, ball_quaternions: np.ndarray, hand_quaternions: np.ndarray
) -> FuncAnimation:
    frames = playback_frames(time_s)
    figure = plt.figure(figsize=(10, 5))
    ball_axes = figure.add_subplot(121, projection="3d")
    hand_axes = figure.add_subplot(122, projection="3d")
    ball_collection, ball_faces = _add_cube(ball_axes, "ball")
    hand_collection, hand_faces = _add_cube(hand_axes, "hand")

    def update(frame_index: int):
        sample_index = int(frames[frame_index])
        _set_orientation(ball_collection, ball_faces, ball_quaternions[sample_index])
        _set_orientation(hand_collection, hand_faces, hand_quaternions[sample_index])
        figure.suptitle(f"t = {time_s[sample_index]:.2f} s")
        return (ball_collection, hand_collection)

    return FuncAnimation(
        figure, update, frames=len(frames), interval=FRAME_INTERVAL_MS, blit=False
    )


# A recording folder plays both cubes. A single orientation CSV plays one.
def load_view(path: Path) -> tuple[FuncAnimation, Path]:
    if path.is_dir():
        combined = path / "combined.csv"
        time_s, ball, hand = load_combined(combined)
        return animate_pair(time_s, ball, hand), path / "orientation.gif"
    with path.open(newline="") as csv_file:
        fieldnames = csv.DictReader(csv_file).fieldnames
    if fieldnames is not None and "ball_qw" in fieldnames:
        time_s, ball, hand = load_combined(path)
        return animate_pair(time_s, ball, hand), path.with_suffix(".gif")
    time_s, quaternions = load_orientations(path)
    return animate(time_s, quaternions), path.with_suffix(".gif")


# Accept an orientation CSV, a combined CSV, or a dual-recording folder.
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Animate orientation from a CSV or a recording folder.")
    parser.add_argument(
        "path",
        type=Path,
        help="Orientation CSV, combined CSV, or a folder that contains combined.csv",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Also write a GIF beside the CSV, or orientation.gif inside a folder.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        animation, gif_path = load_view(args.path)
    except (OSError, ValueError, csv.Error) as error:
        print(error, file=sys.stderr)
        return 1
    if args.save:
        animation.save(gif_path, writer=PillowWriter(fps=round(1000 / FRAME_INTERVAL_MS)))
        print(f"Writing {gif_path.resolve()}")
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
