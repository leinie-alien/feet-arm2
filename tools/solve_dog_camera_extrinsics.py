#!/usr/bin/env python3
"""
Solve world -> dog_camera_link extrinsics.

Chain:
  world_T_board  = world_T_link4 x link4_T_camera x camera_T_board
                   (arm FK)         (handeye_result)  (arm dataset)
  world_T_dog_cam = world_T_board x inv(dog_cam_T_board)
                                      (dog images, ChArUco)

Usage:
  python3 tools/solve_dog_camera_extrinsics.py \
    --arm-dataset  recordings/handeye_charuco_capture/dataset.yml \
    --arm-extrinsics recordings/handeye_charuco_capture/handeye_result.yml \
    --dog-images   recordings/dog_camera/*.png \
    --dog-intrinsics recordings/handeye_charuco_capture/device_intrinsics.yml \
    --output       recordings/dog_camera/dog_camera_result.yml
"""

from __future__ import annotations

import argparse
import glob
import math
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Solve world -> dog_camera_link from arm dataset + dog camera images"
    )
    p.add_argument("--arm-dataset", required=True,
                   help="Path to arm handeye dataset.yml")
    p.add_argument("--arm-extrinsics", required=True,
                   help="Path to handeye_result.yml (link4 -> camera)")
    p.add_argument("--dog-images", required=True, nargs="+",
                   help="Dog camera image paths or glob pattern(s)")
    p.add_argument("--dog-intrinsics", required=True,
                   help="Dog camera intrinsics YAML (camera_matrix + dist_coeffs)")
    p.add_argument("--output", default=None,
                   help="Write result to this YAML file")
    p.add_argument("--min-corners", type=int, default=16,
                   help="Min ChArUco corners to accept a dog image (default: 16)")
    p.add_argument("--min-blur", type=float, default=50.0,
                   help="Min Laplacian-variance blur score for dog images (default: 50)")
    p.add_argument("--squares-x", type=int, default=7)
    p.add_argument("--squares-y", type=int, default=5)
    p.add_argument("--square-length", type=float, default=0.03)
    p.add_argument("--marker-length", type=float, default=0.022)
    p.add_argument("--dict", default="DICT_4X4_50")
    return p.parse_args()


# ---------------------------------------------------------------------------
# SE(3) helpers
# ---------------------------------------------------------------------------

def compose(r_ab: np.ndarray, t_ab: np.ndarray,
            r_bc: np.ndarray, t_bc: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Compose a->b then b->c into a->c."""
    return r_ab @ r_bc, r_ab @ t_bc.reshape(3, 1) + t_ab.reshape(3, 1)


def invert_tf(r: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    r_inv = r.T
    return r_inv, -r_inv @ t.reshape(3, 1)


def quaternion_xyzw_from_rotation(r: np.ndarray) -> np.ndarray:
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s, (r[1,0]-r[0,1])/s
    elif r[0,0] > r[1,1] and r[0,0] > r[2,2]:
        s = math.sqrt(1.0 + r[0,0] - r[1,1] - r[2,2]) * 2.0
        w, x, y, z = (r[2,1]-r[1,2])/s, 0.25*s, (r[0,1]+r[1,0])/s, (r[0,2]+r[2,0])/s
    elif r[1,1] > r[2,2]:
        s = math.sqrt(1.0 + r[1,1] - r[0,0] - r[2,2]) * 2.0
        w, x, y, z = (r[0,2]-r[2,0])/s, (r[0,1]+r[1,0])/s, 0.25*s, (r[1,2]+r[2,1])/s
    else:
        s = math.sqrt(1.0 + r[2,2] - r[0,0] - r[1,1]) * 2.0
        w, x, y, z = (r[1,0]-r[0,1])/s, (r[0,2]+r[2,0])/s, (r[1,2]+r[2,1])/s, 0.25*s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def rotation_from_quaternion_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def average_transforms(rot_trans_list: List[Tuple[np.ndarray, np.ndarray]]
                        ) -> Tuple[np.ndarray, np.ndarray]:
    """Average a list of (R, t) using quaternion eigenvector method for rotation."""
    quats = np.array([quaternion_xyzw_from_rotation(r) for r, _ in rot_trans_list])
    trans = np.array([t.flatten() for _, t in rot_trans_list])

    # Ensure quaternion sign consistency (flip if dot with first < 0)
    for i in range(1, len(quats)):
        if quats[i] @ quats[0] < 0:
            quats[i] = -quats[i]

    # Eigenvector averaging
    M = sum(np.outer(q, q) for q in quats) / len(quats)
    eigvals, eigvecs = np.linalg.eigh(M)
    q_avg = eigvecs[:, np.argmax(eigvals)]
    if q_avg[3] < 0:
        q_avg = -q_avg
    q_avg /= np.linalg.norm(q_avg)

    r_avg = rotation_from_quaternion_xyzw(q_avg)
    t_avg = trans.mean(axis=0).reshape(3, 1)
    return r_avg, t_avg


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def node_to_matrix(node: cv2.FileNode) -> np.ndarray:
    mat = node.mat()
    if mat is None:
        raise RuntimeError(f"Missing matrix node: {node.name()}")
    return np.asarray(mat, dtype=np.float64)


def load_arm_extrinsics(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Load link4_T_camera from handeye_result.yml."""
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open arm extrinsics: {path}")
    r = node_to_matrix(fs.getNode("link4_T_camera_rotation"))
    t = node_to_matrix(fs.getNode("link4_T_camera_translation")).reshape(3, 1)
    fs.release()
    return r, t


def load_arm_world_T_board(dataset_path: Path,
                            r_link4_cam: np.ndarray,
                            t_link4_cam: np.ndarray,
                            ) -> List[Tuple[np.ndarray, np.ndarray]]:
    """
    For each arm sample compute world_T_board:
      world_T_board = world_T_link4 x link4_T_camera x camera_T_board
    """
    fs = cv2.FileStorage(str(dataset_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open arm dataset: {dataset_path}")

    samples = fs.getNode("samples")
    results: List[Tuple[np.ndarray, np.ndarray]] = []
    skipped = 0

    for i in range(samples.size()):
        s = samples.at(i)
        t2c_node = s.getNode("camera_T_board")
        g2b_node = s.getNode("world_T_link4")
        if t2c_node.empty() or g2b_node.empty():
            skipped += 1
            continue

        r_cam_board = node_to_matrix(t2c_node.getNode("rotation"))
        t_cam_board = node_to_matrix(t2c_node.getNode("translation")).reshape(3, 1)
        r_world_link4 = node_to_matrix(g2b_node.getNode("rotation"))
        t_world_link4 = node_to_matrix(g2b_node.getNode("translation")).reshape(3, 1)

        r_link4_board, t_link4_board = compose(r_link4_cam, t_link4_cam,
                                                r_cam_board, t_cam_board)
        r_world_board, t_world_board = compose(r_world_link4, t_world_link4,
                                                r_link4_board, t_link4_board)
        results.append((r_world_board, t_world_board))

    fs.release()
    if skipped:
        print(f"[warn] skipped {skipped} arm samples (missing nodes)")
    return results


def load_intrinsics(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open intrinsics: {path}")
    cam_mat = node_to_matrix(fs.getNode("camera_matrix"))
    dist = node_to_matrix(fs.getNode("dist_coeffs")).flatten()
    fs.release()
    return cam_mat, dist


def blur_score(image: np.ndarray) -> float:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def detect_charuco_pose(image: np.ndarray,
                         board,
                         aruco_dict,
                         cam_mat: np.ndarray,
                         dist: np.ndarray,
                         min_corners: int,
                         ) -> Optional[Tuple[np.ndarray, np.ndarray, int, float]]:
    """Returns (R, t, n_corners, reproj_px) in camera_T_board convention, or None."""
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

    # compute reprojection error manually
    obj_pts = np.array([board.chessboardCorners[j] for j in charuco_ids.flatten()],
                       dtype=np.float32).reshape(-1, 1, 3)
    projected, _ = cv2.projectPoints(obj_pts, rvec, tvec, cam_mat, dist)
    reproj = float(np.mean(np.linalg.norm(
        charuco_corners.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)))

    return r_mat, tvec.reshape(3, 1), int(len(charuco_ids)), reproj


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    args = parse_args()

    # Expand glob patterns
    dog_image_paths: List[Path] = []
    for pattern in args.dog_images:
        expanded = sorted(glob.glob(pattern))
        dog_image_paths.extend(Path(p) for p in (expanded if expanded else [pattern]))

    arm_dataset_path = Path(args.arm_dataset).expanduser().resolve()
    arm_extrinsics_path = Path(args.arm_extrinsics).expanduser().resolve()
    dog_intrinsics_path = Path(args.dog_intrinsics).expanduser().resolve()

    # 1. Arm extrinsics: link4_T_camera
    print(f"arm extrinsics : {arm_extrinsics_path}")
    r_link4_cam, t_link4_cam = load_arm_extrinsics(arm_extrinsics_path)

    # 2. world_T_board from arm dataset
    print(f"arm dataset    : {arm_dataset_path}")
    arm_samples = load_arm_world_T_board(arm_dataset_path, r_link4_cam, t_link4_cam)
    print(f"arm samples    : {len(arm_samples)}")
    if not arm_samples:
        raise RuntimeError("No valid arm samples — cannot solve")

    r_world_board, t_world_board = average_transforms(arm_samples)
    trans_std = np.array([t.flatten() for _, t in arm_samples]).std(axis=0)
    print(f"world_T_board pos : {t_world_board.flatten().tolist()}")
    print(f"world_T_board std : {trans_std.tolist()}  "
          f"({'OK' if trans_std.max() < 0.01 else 'WARN: large spread, was board moving?'})")

    # 3. Dog camera intrinsics + board
    cam_mat, dist = load_intrinsics(dog_intrinsics_path)
    dict_id = getattr(cv2.aruco, args.dict)
    aruco_dict = cv2.aruco.Dictionary_get(dict_id)
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y,
        args.square_length, args.marker_length, aruco_dict)

    # 4. Detect ChArUco in dog images
    print(f"\ndog images     : {len(dog_image_paths)}")
    dog_samples: List[Tuple[np.ndarray, np.ndarray]] = []
    for img_path in dog_image_paths:
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  [skip] cannot read: {img_path}")
            continue
        bs = blur_score(img)
        if bs < args.min_blur:
            print(f"  [skip] {img_path.name}: blur={bs:.1f} < {args.min_blur}")
            continue
        result = detect_charuco_pose(img, board, aruco_dict, cam_mat, dist, args.min_corners)
        if result is None:
            print(f"  [skip] {img_path.name}: detection failed or corners < {args.min_corners}")
            continue
        r_dog, t_dog, n_corners, reproj = result
        print(f"  [ok]   {img_path.name}: corners={n_corners} reproj={reproj:.3f}px blur={bs:.1f}")
        dog_samples.append((r_dog, t_dog))

    if not dog_samples:
        raise RuntimeError("No valid dog camera detections — cannot solve")

    # 5. Solve: world_T_dog_cam = world_T_board x inv(dog_cam_T_board)
    r_dog_board_avg, t_dog_board_avg = average_transforms(dog_samples)
    r_board_dog, t_board_dog = invert_tf(r_dog_board_avg, t_dog_board_avg)
    r_result, t_result = compose(r_world_board, t_world_board, r_board_dog, t_board_dog)
    quat_result = quaternion_xyzw_from_rotation(r_result)

    tx, ty, tz = t_result.flatten()
    qx, qy, qz, qw = quat_result

    print(f"\ndog samples used : {len(dog_samples)}")
    print("world -> dog_camera_link translation [m]:")
    print(f"  [{tx:.6f}, {ty:.6f}, {tz:.6f}]")
    print("world -> dog_camera_link quaternion [x, y, z, w]:")
    print(f"  [{qx:.8f}, {qy:.8f}, {qz:.8f}, {qw:.8f}]")
    print("\nparams.yaml snippet (fill into arrm/arm/src/arm2_task/config/params.yaml):")
    print("dog_camera_extrinsics:")
    print("  parent_frame: world")
    print("  child_frame:  dog_camera_link")
    print(f"  pos:  [{tx:.6f}, {ty:.6f}, {tz:.6f}]")
    print(f"  quat: [{qx:.8f}, {qy:.8f}, {qz:.8f}, {qw:.8f}]")

    if args.output:
        out_path = Path(args.output).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fs = cv2.FileStorage(str(out_path), cv2.FILE_STORAGE_WRITE)
        if not fs.isOpened():
            raise RuntimeError(f"Cannot open output for write: {out_path}")
        fs.write("arm_samples_used", int(len(arm_samples)))
        fs.write("dog_samples_used", int(len(dog_samples)))
        fs.write("world_T_dog_camera_rotation", r_result.astype(np.float64))
        fs.write("world_T_dog_camera_translation", t_result.reshape(3, 1).astype(np.float64))
        fs.write("world_T_dog_camera_quaternion_xyzw", quat_result.reshape(4, 1))
        fs.release()
        print(f"wrote: {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
