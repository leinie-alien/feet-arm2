#!/usr/bin/env python3
"""
Verify dog camera extrinsics by comparing board pose from two chains.

Procedure:
  1. Place ChArUco board at a NEW position (different from calibration).
  2. Run arm capture node, capture 3-5 frames → save as a mini dataset YAML.
  3. Take dog camera images of the SAME board position.
  4. Run this script:

     python3 tools/verify_dog_camera.py \
       --arm-dataset  recordings/verify/arm_verify.yml \
       --arm-extrinsics recordings/handeye_charuco_capture/handeye_result.yml \
       --dog-camera-result recordings/dog_camera/dog_camera_result.yml \
       --dog-images   recordings/verify/dog_*.png \
       --dog-intrinsics recordings/handeye_charuco_capture/device_intrinsics.yml

If calibration is correct, "pos error" should be < 20mm and "angle error" < 5 deg.
"""

from __future__ import annotations

import argparse
import glob
import math
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# SE(3) helpers (same as solver)
# ---------------------------------------------------------------------------

def compose(r_ab, t_ab, r_bc, t_bc):
    return r_ab @ r_bc, r_ab @ t_bc.reshape(3, 1) + t_ab.reshape(3, 1)


def invert_tf(r, t):
    r_inv = r.T
    return r_inv, -r_inv @ t.reshape(3, 1)


def angle_between_rotations(r1: np.ndarray, r2: np.ndarray) -> float:
    """Angle in degrees between two rotation matrices."""
    r_diff = r1.T @ r2
    trace = float(np.trace(r_diff))
    cos_angle = (trace - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def node_to_matrix(node: cv2.FileNode) -> np.ndarray:
    mat = node.mat()
    if mat is None:
        raise RuntimeError(f"Missing matrix node: {node.name()}")
    return np.asarray(mat, dtype=np.float64)


def load_arm_extrinsics(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open: {path}")
    r = node_to_matrix(fs.getNode("link4_T_camera_rotation"))
    t = node_to_matrix(fs.getNode("link4_T_camera_translation")).reshape(3, 1)
    fs.release()
    return r, t


def load_dog_camera_result(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open: {path}")
    r = node_to_matrix(fs.getNode("world_T_dog_camera_rotation"))
    t = node_to_matrix(fs.getNode("world_T_dog_camera_translation")).reshape(3, 1)
    fs.release()
    return r, t


def load_intrinsics(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open: {path}")
    cam_mat = node_to_matrix(fs.getNode("camera_matrix"))
    dist = node_to_matrix(fs.getNode("dist_coeffs")).flatten()
    fs.release()
    return cam_mat, dist


def load_arm_world_T_board(dataset_path: Path,
                            r_link4_cam: np.ndarray,
                            t_link4_cam: np.ndarray,
                            ) -> List[Tuple[np.ndarray, np.ndarray]]:
    fs = cv2.FileStorage(str(dataset_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open: {dataset_path}")
    samples = fs.getNode("samples")
    results = []
    for i in range(samples.size()):
        s = samples.at(i)
        t2c = s.getNode("camera_T_board")
        g2b = s.getNode("world_T_link4")
        if t2c.empty() or g2b.empty():
            continue
        r_cam_board = node_to_matrix(t2c.getNode("rotation"))
        t_cam_board = node_to_matrix(t2c.getNode("translation")).reshape(3, 1)
        r_world_link4 = node_to_matrix(g2b.getNode("rotation"))
        t_world_link4 = node_to_matrix(g2b.getNode("translation")).reshape(3, 1)
        r_link4_board, t_link4_board = compose(r_link4_cam, t_link4_cam,
                                                r_cam_board, t_cam_board)
        r_world_board, t_world_board = compose(r_world_link4, t_world_link4,
                                                r_link4_board, t_link4_board)
        results.append((r_world_board, t_world_board))
    fs.release()
    return results


def detect_charuco_pose(image: np.ndarray,
                         board,
                         aruco_dict,
                         cam_mat: np.ndarray,
                         dist: np.ndarray,
                         min_corners: int = 8,
                         ) -> Optional[Tuple[np.ndarray, np.ndarray, int]]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image.copy()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
    if ids is None or len(ids) == 0:
        return None
    _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board)
    if charuco_ids is None or len(charuco_ids) < min_corners:
        return None
    ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, cam_mat, dist, None, None)
    if not ok:
        return None
    r_mat, _ = cv2.Rodrigues(rvec)
    return r_mat, tvec.reshape(3, 1), int(len(charuco_ids))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--arm-dataset", required=True)
    p.add_argument("--arm-extrinsics", required=True)
    p.add_argument("--dog-camera-result", required=True)
    p.add_argument("--dog-images", required=True, nargs="+")
    p.add_argument("--dog-intrinsics", required=True)
    p.add_argument("--squares-x", type=int, default=7)
    p.add_argument("--squares-y", type=int, default=5)
    p.add_argument("--square-length", type=float, default=0.03)
    p.add_argument("--marker-length", type=float, default=0.022)
    p.add_argument("--dict", default="DICT_4X4_50")
    return p.parse_args()


def main():
    args = parse_args()

    r_link4_cam, t_link4_cam = load_arm_extrinsics(
        Path(args.arm_extrinsics).expanduser().resolve())
    r_world_dog, t_world_dog = load_dog_camera_result(
        Path(args.dog_camera_result).expanduser().resolve())
    dog_cam_mat, dog_dist = load_intrinsics(
        Path(args.dog_intrinsics).expanduser().resolve())

    dict_id = getattr(cv2.aruco, args.dict)
    aruco_dict = cv2.aruco.Dictionary_get(dict_id)
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y,
        args.square_length, args.marker_length, aruco_dict)

    # --- Chain 1: arm ---
    arm_samples = load_arm_world_T_board(
        Path(args.arm_dataset).expanduser().resolve(),
        r_link4_cam, t_link4_cam)
    if not arm_samples:
        print("[ERROR] No valid arm samples")
        return 1

    t_arm_boards = np.array([t.flatten() for _, t in arm_samples])
    t_arm_mean = t_arm_boards.mean(axis=0)
    t_arm_std  = t_arm_boards.std(axis=0)
    r_arm_mean = arm_samples[0][0]  # approximate; use first sample rotation

    print(f"=== Chain 1 (arm camera) ===")
    print(f"  samples          : {len(arm_samples)}")
    print(f"  world_T_board pos: {t_arm_mean.tolist()}")
    print(f"  pos std          : {t_arm_std.tolist()}  "
          f"({'OK' if t_arm_std.max() < 0.01 else 'WARN: large spread'})")

    # --- Chain 2: dog camera ---
    dog_image_paths = []
    for pattern in args.dog_images:
        expanded = sorted(glob.glob(pattern))
        dog_image_paths.extend(Path(p) for p in (expanded if expanded else [pattern]))

    dog_world_T_boards = []
    print(f"\n=== Chain 2 (dog camera) ===")
    for img_path in dog_image_paths:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  [skip] cannot read: {img_path}")
            continue
        result = detect_charuco_pose(img, board, aruco_dict, dog_cam_mat, dog_dist)
        if result is None:
            print(f"  [skip] {img_path.name}: detection failed")
            continue
        r_dog_board, t_dog_board, n = result
        # world_T_board = world_T_dog_cam × dog_cam_T_board
        r_wb, t_wb = compose(r_world_dog, t_world_dog, r_dog_board, t_dog_board)
        dog_world_T_boards.append((r_wb, t_wb))
        print(f"  [ok] {img_path.name}: corners={n} "
              f"pos={t_wb.flatten().tolist()}")

    if not dog_world_T_boards:
        print("[ERROR] No valid dog camera detections")
        return 1

    t_dog_boards = np.array([t.flatten() for _, t in dog_world_T_boards])
    t_dog_mean = t_dog_boards.mean(axis=0)
    r_dog_mean = dog_world_T_boards[0][0]

    print(f"\n  mean pos         : {t_dog_mean.tolist()}")

    # --- Compare ---
    pos_error_m = np.linalg.norm(t_arm_mean - t_dog_mean)
    angle_error_deg = angle_between_rotations(r_arm_mean, r_dog_mean)

    print(f"\n=== Comparison ===")
    print(f"  arm   world_T_board pos : {t_arm_mean.tolist()}")
    print(f"  dog   world_T_board pos : {t_dog_mean.tolist()}")
    print(f"  pos error               : {pos_error_m*1000:.1f} mm  "
          f"({'OK' if pos_error_m < 0.02 else 'WARN: > 20mm'})")
    print(f"  angle error             : {angle_error_deg:.2f} deg  "
          f"({'OK' if angle_error_deg < 5.0 else 'WARN: > 5 deg'})")

    if pos_error_m < 0.02 and angle_error_deg < 5.0:
        print("\n[PASS] Calibration looks correct.")
    else:
        print("\n[FAIL] Large error — calibration may be wrong.")
        print("  Possible causes:")
        print("  - Board moved between arm capture and dog capture")
        print("  - Hand-eye calibration (handeye_result.yml) is inaccurate")
        print("  - Too few arm samples with low diversity of poses")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
