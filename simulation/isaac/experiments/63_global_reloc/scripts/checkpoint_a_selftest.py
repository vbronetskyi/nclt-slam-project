#!/usr/bin/env python3
"""Checkpoint A: run the matcher offline against the teach recording

Purpose: catch matcher bugs before any repeat run.  Expectation: matching
teach-run frames against teach-run landmarks should give near-identity
anchor poses (< 0.3 m offset from teach VIO) for ≥ 90% of sampled ticks.

  --landmarks   path to south_landmarks.pkl
  --bag-dir     teach recording dir (contains camera_rgb/, camera_depth/)
  --teach-traj  teach trajectory CSV with (ts, gt_x, gt_y, ...) - we read
                the teach VIO position at each frame ts for the "expected"
                pose.  NOTE: the teach recorder's landmarks already encode
                camera poses, so we treat the landmark's own pose as
                ground truth for this self-test.
  --out-dir     directory for summary + per-tick CSV

  checkpoint_a_summary.txt
  checkpoint_a_matches.csv
"""
import argparse
import glob
import math
import os
import pickle
import sys

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from visual_landmark_matcher import (
    FX, FY, CX, CY, K, DIST,
    MIN_MATCHES, LOWE_RATIO, RANSAC_REPROJ_PX, RANSAC_ITERATIONS,
    MIN_INLIERS, REPROJ_MAX_PX, CONSISTENCY_M, MAX_CANDIDATES, CANDIDATE_RADIUS_M,
    quat_to_rot, rot_to_quat,
)


def run_matcher_self(lm, frame_rgb_path, frame_depth_path, vio_xy, landmarks_all, xy):
    """Run one matcher attempt for a single frame; return dict of result."""
    rgb = cv2.imread(frame_rgb_path)
    if rgb is None:
        return {'outcome': 'rgb_read_fail'}
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(nfeatures=500)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    kpts_curr, desc_curr = orb.detectAndCompute(gray, None)
    if desc_curr is None or len(kpts_curr) < MIN_MATCHES:
        return {'outcome': 'curr_no_features'}
    pts_curr_2d = np.array([k.pt for k in kpts_curr], dtype=np.float32)

    # Candidates   
    dxy = xy - np.array(vio_xy)
    d = np.linalg.norm(dxy, axis=1)
    idx_sorted = np.argsort(d)
    cand_idx = [i for i in idx_sorted[:MAX_CANDIDATES] if d[i] < CANDIDATE_RADIUS_M]
    if not cand_idx:
        return {'outcome': 'no_candidates'}

    best = None
    for li in cand_idx:
        lm_t = landmarks_all[li]
        desc_t = lm_t['descriptors']
        if desc_t is None or len(desc_t) < MIN_MATCHES:
            continue
        try:
            knn = bf.knnMatch(desc_curr, desc_t, k=2)
        except cv2.error:
            continue
        good = [m for m, n in knn if (m.distance < LOWE_RATIO * n.distance)]
        if len(good) < MIN_MATCHES:
            continue
        obj_pts = np.array([lm_t['keypoints_3d_cam'][m.trainIdx] for m in good],
                           dtype=np.float32)
        img_pts = np.array([pts_curr_2d[m.queryIdx] for m in good],
                           dtype=np.float32)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts, img_pts, K, DIST,
            iterationsCount=RANSAC_ITERATIONS,
            reprojectionError=RANSAC_REPROJ_PX,
            flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inliers is None or len(inliers) < MIN_INLIERS:
            continue
        proj, _ = cv2.projectPoints(obj_pts[inliers[:, 0]], rvec, tvec, K, DIST)
        err = float(np.linalg.norm(
            proj.reshape(-1, 2) - img_pts[inliers[:, 0]], axis=1).mean())
        if err > REPROJ_MAX_PX:
            continue
        R_cur_teach, _ = cv2.Rodrigues(rvec)
        R_teach_cur = R_cur_teach.T
        t_teach_cur = -R_teach_cur @ tvec.reshape(3)
        teach_pose = lm_t['pose']
        R_world_teach = quat_to_rot(*teach_pose[3:7])
        R_world_cur = R_world_teach @ R_teach_cur
        t_world_cur = (np.array(teach_pose[:3]) + R_world_teach @ t_teach_cur)
        if best is None or len(inliers) > best['n_in']:
            best = {
                'n_in': len(inliers),
                'reproj': err,
                'world_cur_xy': (float(t_world_cur[0]), float(t_world_cur[1])),
                'teach_idx': li,
            }
    if best is None:
        return {'outcome': 'no_pnp_accept', 'n_cand': len(cand_idx)}
    return {
        'outcome': 'ok',
        'n_cand': len(cand_idx),
        'n_in': best['n_in'],
        'reproj': best['reproj'],
        'anchor_xy': best['world_cur_xy'],
        'teach_idx': best['teach_idx'],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--landmarks', required=True)
    ap.add_argument('--bag-dir', required=True,
                    help='teach recording dir with camera_rgb/ and camera_depth/')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--sample-every', type=int, default=3,
                    help='pick every Nth landmark frame to test')
    args = ap.parse_args()

    with open(args.landmarks, 'rb') as f:
        data = pickle.load(f)
    lms = data['landmarks']
    xy = np.array([[lm['pose'][0], lm['pose'][1]] for lm in lms])
    print(f'Loaded {len(lms)} landmarks')

    rgb_files = sorted(glob.glob(os.path.join(args.bag_dir, 'camera_rgb', '*.jpg')))
    if not rgb_files:
        print(f'ERROR: no RGB frames in {args.bag_dir}/camera_rgb/')
        return 1
    print(f'Found {len(rgb_files)} RGB frames in bag')

    # Map teach landmark timestamp -> nearest RGB frame
    import csv
    os.makedirs(args.out_dir, exist_ok=True)
    log_path = os.path.join(args.out_dir, 'checkpoint_a_matches.csv')
    with open(log_path, 'w') as lf:
        lf.write('landmark_idx,teach_x,teach_y,outcome,n_cand,n_in,reproj,'
                 'anchor_x,anchor_y,offset_m\n')

        n_ok = 0
        offsets = []
        attempts = 0
        for li in range(0, len(lms), args.sample_every):
            lm = lms[li]
            # For each teach landmark, the RGB frame with ts closest to lm['ts'] is
            # our test image.  Pairing: filenames are sim_time floats.
            ts = lm['ts']
            # Find nearest rgb file
            tstamps = np.array([float(os.path.basename(p).replace('.jpg', '')) for p in rgb_files])
            idx = int(np.argmin(np.abs(tstamps - ts)))
            rgb_path = rgb_files[idx]
            # Depth path (same timestamp dir camera_depth/)
            depth_path = rgb_path.replace('camera_rgb', 'camera_depth').replace('.jpg', '.png')
            if not os.path.exists(depth_path):
                continue

            vio_xy = (lm['pose'][0], lm['pose'][1])
            res = run_matcher_self(lm, rgb_path, depth_path, vio_xy, lms, xy)
            attempts += 1

            if res['outcome'] == 'ok':
                ax, ay = res['anchor_xy']
                offset = math.hypot(ax - lm['pose'][0], ay - lm['pose'][1])
                offsets.append(offset)
                if offset < 0.3:
                    n_ok += 1
                lf.write(f"{li},{lm['pose'][0]:.3f},{lm['pose'][1]:.3f},"
                         f"{res['outcome']},{res['n_cand']},{res['n_in']},"
                         f"{res['reproj']:.2f},{ax:.3f},{ay:.3f},{offset:.3f}\n")
            else:
                lf.write(f"{li},{lm['pose'][0]:.3f},{lm['pose'][1]:.3f},"
                         f"{res['outcome']},{res.get('n_cand','')},,,,,\n")
            if attempts % 20 == 0:
                print(f'  progress: {attempts} / {len(lms) // args.sample_every}  '
                      f'ok={n_ok}')

    summary = os.path.join(args.out_dir, 'checkpoint_a_summary.txt')
    pass_frac = n_ok / attempts if attempts > 0 else 0.0
    with open(summary, 'w') as sf:
        sf.write(f'Checkpoint A self-match\n'
                 f'  landmarks total:     {len(lms)}\n'
                 f'  attempts:            {attempts}\n'
                 f'  accepted matches:    {sum(1 for x in offsets)}\n'
                 f'  near-identity (<0.3m): {n_ok}  '
                 f'({100*pass_frac:.1f}%)\n'
                 f'  mean offset:         {np.mean(offsets):.3f} m\n' if offsets
                 else '  mean offset:         n/a\n'
                 f'  median offset:       {np.median(offsets):.3f} m\n' if offsets
                 else '')
        sf.write(f'  PASS: {"YES" if pass_frac >= 0.9 else "NO (< 90%)"}\n')
    print(open(summary).read())
    return 0 if pass_frac >= 0.9 else 1


if __name__ == '__main__':
    sys.exit(main())
