#!/usr/bin/env python3
"""
Debug dog camera calibration by reprojecting board corners.

If calibration is correct, the projected corners should overlap with detected corners.

Usage:
  python3 tools/debug_dog_camera.py \
    --arm-dataset  recordings/handeye_charuco_capture/dataset.yml \
    --arm-extrinsics recordings/handeye_charuco_capture/handeye_result.yml \
    --dog-image    recordings/dog_camera/dog_0006.png \
    --dog-intrinsics recordings/handeye_charuco_capture/device_intrinsics.yml \
    --dog-result   recordings/dog_camera/dog_camera_result.yml \
    --output       /tmp/debug_reproj.png
"""
from __future__ import annotations
import argparse
from pathlib import Path
import cv2
import numpy as np


def node_to_mat(node):
    return np.asarray(node.mat(), dtype=np.float64)

def compose(r_ab, t_ab, r_bc, t_bc):
    return r_ab @ r_bc, r_ab @ t_bc.reshape(3,1) + t_ab.reshape(3,1)

def invert_tf(r, t):
    ri = r.T
    return ri, -ri @ t.reshape(3,1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--arm-dataset",   required=True)
    p.add_argument("--arm-extrinsics",required=True)
    p.add_argument("--dog-image",     required=True)
    p.add_argument("--dog-intrinsics",required=True)
    p.add_argument("--dog-result",    required=True)
    p.add_argument("--output",        default="/tmp/debug_dog_reproj.png")
    p.add_argument("--arm-sample",    type=int, default=-1,
                   help="which arm sample to use (-1 = average all)")
    args = p.parse_args()

    # ── load intrinsics ──────────────────────────────────────────────────────
    fs = cv2.FileStorage(args.dog_intrinsics, cv2.FILE_STORAGE_READ)
    cam_mat = node_to_mat(fs.getNode("camera_matrix"))
    dist    = node_to_mat(fs.getNode("dist_coeffs")).flatten()
    fs.release()

    # ── ChArUco board ────────────────────────────────────────────────────────
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    board = cv2.aruco.CharucoBoard_create(7, 5, 0.03, 0.022, aruco_dict)

    # ── load arm extrinsics (link4_T_camera) ─────────────────────────────────
    fs = cv2.FileStorage(args.arm_extrinsics, cv2.FILE_STORAGE_READ)
    r_l4c = node_to_mat(fs.getNode("link4_T_camera_rotation"))
    t_l4c = node_to_mat(fs.getNode("link4_T_camera_translation")).reshape(3,1)
    fs.release()
    print(f"link4_T_camera  t = {t_l4c.flatten().tolist()}")

    # ── load world_T_dog_cam ─────────────────────────────────────────────────
    fs = cv2.FileStorage(args.dog_result, cv2.FILE_STORAGE_READ)
    r_wdc = node_to_mat(fs.getNode("world_T_dog_camera_rotation"))
    t_wdc = node_to_mat(fs.getNode("world_T_dog_camera_translation")).reshape(3,1)
    fs.release()
    print(f"world_T_dog_cam t = {t_wdc.flatten().tolist()}")

    # ── compute world_T_board from arm dataset ───────────────────────────────
    fs = cv2.FileStorage(args.arm_dataset, cv2.FILE_STORAGE_READ)
    samples = fs.getNode("samples")
    boards = []
    for i in range(samples.size()):
        s = samples.at(i)
        g2b = s.getNode("world_T_link4")
        t2c = s.getNode("camera_T_board")
        if g2b.empty() or t2c.empty():
            continue
        r_wl4 = node_to_mat(g2b.getNode("rotation"))
        t_wl4 = node_to_mat(g2b.getNode("translation")).reshape(3,1)
        r_cb  = node_to_mat(t2c.getNode("rotation"))
        t_cb  = node_to_mat(t2c.getNode("translation")).reshape(3,1)
        r_l4b, t_l4b = compose(r_l4c, t_l4c, r_cb,  t_cb)
        r_wb,  t_wb  = compose(r_wl4, t_wl4, r_l4b, t_l4b)
        boards.append((r_wb, t_wb, i))
    fs.release()

    if args.arm_sample >= 0 and args.arm_sample < len(boards):
        r_wb, t_wb, idx = boards[args.arm_sample]
        print(f"\nUsing arm sample {idx}  world_T_board t = {t_wb.flatten().tolist()}")
    else:
        # average translation, use first rotation (approximate)
        ts = np.array([t.flatten() for _,t,_ in boards])
        t_wb = ts.mean(axis=0).reshape(3,1)
        r_wb = boards[0][0]
        print(f"\nUsing average of {len(boards)} arm samples")
        print(f"world_T_board t = {t_wb.flatten().tolist()}")
        print(f"world_T_board std = {ts.std(axis=0).tolist()}")

    # ── detect board in dog image ─────────────────────────────────────────────
    img = cv2.imread(args.dog_image)
    assert img is not None, f"Cannot read {args.dog_image}"
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict)
    assert ids is not None, "No ArUco markers detected in dog image"
    _, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    assert ci is not None and len(ci) >= 8, "Too few ChArUco corners"
    ok, rvec_det, tvec_det = cv2.aruco.estimatePoseCharucoBoard(
        cc, ci, board, cam_mat, dist, None, None)
    assert ok, "Pose estimation failed"
    r_dc_det, _ = cv2.Rodrigues(rvec_det)
    t_dc_det = tvec_det.reshape(3,1)
    print(f"\ndog_cam_T_board (detected) t = {t_dc_det.flatten().tolist()}")

    # ── compute world_T_dog_cam from this single image ────────────────────────
    r_bd, t_bd = invert_tf(r_dc_det, t_dc_det)
    r_wdc_check, t_wdc_check = compose(r_wb, t_wb, r_bd, t_bd)
    print(f"world_T_dog_cam (single-image check) t = {t_wdc_check.flatten().tolist()}")

    # ── project board corners via arm chain into dog camera ───────────────────
    # dog_cam_T_board = inv(world_T_dog_cam) × world_T_board
    r_dcw, t_dcw = invert_tf(r_wdc, t_wdc)          # dog_cam_T_world
    r_proj, t_proj = compose(r_dcw, t_dcw, r_wb, t_wb)  # dog_cam_T_board (projected)
    rvec_proj, _ = cv2.Rodrigues(r_proj)
    tvec_proj = t_proj
    print(f"\ndog_cam_T_board (from arm chain + dog result) t = {t_proj.flatten().tolist()}")
    print(f"dog_cam_T_board (detected)                     t = {t_dc_det.flatten().tolist()}")
    diff = t_proj.flatten() - t_dc_det.flatten()
    print(f"difference = {diff.tolist()}  norm = {np.linalg.norm(diff)*100:.1f} cm")

    # ── draw both poses on image ──────────────────────────────────────────────
    vis = img.copy()
    # detected pose → GREEN axes
    cv2.drawFrameAxes(vis, cam_mat, dist, rvec_det,  tvec_det,  0.06)
    cv2.aruco.drawDetectedCornersCharuco(vis, cc, ci)

    # projected pose (from arm chain) → RED axes
    # draw projected board corners manually in red
    obj_corners = np.array([board.chessboardCorners[j] for j in ci.flatten()],
                           dtype=np.float32).reshape(-1,1,3)
    img_proj, _ = cv2.projectPoints(obj_corners, rvec_proj, tvec_proj, cam_mat, dist)
    for pt in img_proj.reshape(-1,2):
        cv2.circle(vis, (int(pt[0]), int(pt[1])), 6, (0,0,255), 2)

    # legend
    cv2.putText(vis, "GREEN = dog detected",     (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,200,0),2)
    cv2.putText(vis, "RED   = arm chain project",(10,60), cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
    err_cm = np.linalg.norm(diff)*100
    cv2.putText(vis, f"t_diff = {err_cm:.1f} cm", (10,90), cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,100,0),2)

    cv2.imwrite(args.output, vis)
    print(f"\nWrote: {args.output}")
    print("GREEN axes/corners = detected by dog camera")
    print("RED   circles      = projected from arm chain")
    print("If calibration is correct, they should overlap.")


if __name__ == "__main__":
    main()
