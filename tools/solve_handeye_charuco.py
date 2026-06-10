#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np


METHODS: Dict[str, int] = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Solve eye-in-hand extrinsics from handeye_charuco_capture dataset.yml"
    )
    parser.add_argument("--dataset", required=True, help="Path to dataset.yml")
    parser.add_argument(
        "--method",
        default="park",
        choices=sorted(METHODS.keys()),
        help="OpenCV hand-eye solver method",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional YAML file to write the solved Link_4 -> camera transform",
    )
    parser.add_argument(
        "--min-corners",
        type=int,
        default=16,
        help="Reject samples with fewer detected ChArUco corners",
    )
    parser.add_argument(
        "--max-reproj",
        type=float,
        default=0.60,
        help="Reject samples above this reprojection error in pixels",
    )
    parser.add_argument(
        "--min-blur",
        type=float,
        default=90.0,
        help="Reject samples with Laplacian-variance blur score below this threshold",
    )
    return parser.parse_args()


def node_to_matrix(node: cv2.FileNode) -> np.ndarray:
    mat = node.mat()
    if mat is None:
        raise RuntimeError(f"Missing matrix at node {node.name()}")
    return np.asarray(mat, dtype=np.float64)


def blur_score(image_path: Path) -> float:
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return float("nan")
    return float(cv2.Laplacian(image, cv2.CV_64F).var())


def load_samples(
    dataset_path: Path,
    min_corners: int,
    max_reproj: float,
    min_blur: float,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray], List[Dict[str, object]], List[Dict[str, object]]]:
    fs = cv2.FileStorage(str(dataset_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open dataset: {dataset_path}")

    samples_node = fs.getNode("samples")
    if samples_node.empty():
        raise RuntimeError("Dataset has no 'samples' node")

    r_gripper2base: List[np.ndarray] = []
    t_gripper2base: List[np.ndarray] = []
    r_target2cam: List[np.ndarray] = []
    t_target2cam: List[np.ndarray] = []
    kept_info: List[Dict[str, object]] = []
    rejected_info: List[Dict[str, object]] = []

    for i in range(samples_node.size()):
        sample = samples_node.at(i)
        g2b = sample.getNode("world_T_link4")
        t2c = sample.getNode("camera_T_board")
        sample_index = int(sample.getNode("sample_index").real())
        image_rel = sample.getNode("image_path").string()
        image_path = dataset_path.parent / image_rel
        corners = int(sample.getNode("charuco_corner_count").real())
        reproj = float(sample.getNode("reprojection_error_px").real())
        blur = blur_score(image_path)
        reject_reasons: List[str] = []
        if corners < min_corners:
            reject_reasons.append(f"corners<{min_corners} ({corners})")
        if reproj > max_reproj:
            reject_reasons.append(f"reproj>{max_reproj:.2f} ({reproj:.3f})")
        if not np.isfinite(blur) or blur < min_blur:
            reject_reasons.append(f"blur<{min_blur:.1f} ({blur:.1f})")

        info = {
            "sample_index": sample_index,
            "image_path": image_rel,
            "corners": corners,
            "reproj": reproj,
            "blur": blur,
        }
        if reject_reasons:
            info["reasons"] = reject_reasons
            rejected_info.append(info)
            continue

        r_gripper2base.append(node_to_matrix(g2b.getNode("rotation")))
        t_gripper2base.append(node_to_matrix(g2b.getNode("translation")).reshape(3, 1))
        r_target2cam.append(node_to_matrix(t2c.getNode("rotation")))
        t_target2cam.append(node_to_matrix(t2c.getNode("translation")).reshape(3, 1))
        kept_info.append(info)

    fs.release()
    return r_gripper2base, t_gripper2base, r_target2cam, t_target2cam, kept_info, rejected_info


def invert_transform(r_ab: np.ndarray, t_ab: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    r_ba = r_ab.T
    t_ba = -r_ba @ t_ab.reshape(3, 1)
    return r_ba, t_ba


def quaternion_xyzw_from_rotation(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat


def write_output(
    output_path: Path,
    rotation: np.ndarray,
    translation: np.ndarray,
    method: str,
    sample_count: int,
    min_corners: int,
    max_reproj: float,
    min_blur: float,
) -> None:
    quat = quaternion_xyzw_from_rotation(rotation)
    fs = cv2.FileStorage(str(output_path), cv2.FILE_STORAGE_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open output for write: {output_path}")
    fs.write("solver_method", method)
    fs.write("sample_count", int(sample_count))
    fs.write("filter_min_corners", int(min_corners))
    fs.write("filter_max_reproj_px", float(max_reproj))
    fs.write("filter_min_blur", float(min_blur))
    fs.write("link4_T_camera_rotation", rotation.astype(np.float64))
    fs.write("link4_T_camera_translation", translation.reshape(3, 1).astype(np.float64))
    fs.write("link4_T_camera_quaternion_xyzw", quat.reshape(4, 1))
    fs.release()


def main() -> int:
    args = parse_args()
    dataset_path = Path(args.dataset).expanduser().resolve()
    r_g2b, t_g2b, r_t2c, t_t2c, kept_info, rejected_info = load_samples(
        dataset_path,
        min_corners=args.min_corners,
        max_reproj=args.max_reproj,
        min_blur=args.min_blur,
    )
    sample_count = len(r_g2b)
    if sample_count < 3:
        raise RuntimeError(f"Need at least 3 valid samples, got {sample_count}")

    method_flag = METHODS[args.method]
    r_link4_camera, t_link4_camera = cv2.calibrateHandEye(
        r_g2b,
        t_g2b,
        r_t2c,
        t_t2c,
        method=method_flag,
    )
    quat = quaternion_xyzw_from_rotation(r_link4_camera)

    print(f"dataset: {dataset_path}")
    print(f"method: {args.method}")
    print(
        f"filters: min_corners>={args.min_corners}, max_reproj<={args.max_reproj:.3f}, min_blur>={args.min_blur:.1f}"
    )
    print(f"samples kept: {sample_count}")
    print(f"samples rejected: {len(rejected_info)}")
    if rejected_info:
        for item in rejected_info:
            print(
                "reject:",
                item["sample_index"],
                item["image_path"],
                f"corners={item['corners']}",
                f"reproj={item['reproj']:.3f}",
                f"blur={item['blur']:.1f}",
                ",".join(item["reasons"]),
            )
    print("Link_4 -> camera translation [m]:")
    print(f"  [{t_link4_camera[0,0]:.6f}, {t_link4_camera[1,0]:.6f}, {t_link4_camera[2,0]:.6f}]")
    print("Link_4 -> camera quaternion [x, y, z, w]:")
    print(f"  [{quat[0]:.8f}, {quat[1]:.8f}, {quat[2]:.8f}, {quat[3]:.8f}]")
    print("params.yaml snippet:")
    print("camera_extrinsics:")
    print("  parent_frame: Link_4")
    print("  child_frame: camera_link")
    print(f"  pos: [{t_link4_camera[0,0]:.6f}, {t_link4_camera[1,0]:.6f}, {t_link4_camera[2,0]:.6f}]")
    print(f"  quat: [{quat[0]:.8f}, {quat[1]:.8f}, {quat[2]:.8f}, {quat[3]:.8f}]")

    if args.output:
        output_path = Path(args.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        write_output(
            output_path,
            r_link4_camera,
            t_link4_camera,
            args.method,
            sample_count,
            args.min_corners,
            args.max_reproj,
            args.min_blur,
        )
        print(f"wrote: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
