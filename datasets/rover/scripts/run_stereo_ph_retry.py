#!/usr/bin/env python3
"""retry all failed/missing Stereo PinHole experiments on ROVER dataset

runs up to MAX_ATTEMPTS per recording since ORB-SLAM3 crashes are
non-deterministic (Sophus/g2o singularity). also includes
campus_large_night-light which was never tested

Usage:
    python3 scripts/run_stereo_ph_retry.py 2>&1 | tee /workspace/datasets/rover/stereo_ph_retry.log
"""

import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# paths
DATA_DIR = "/workspace/data/rover"
RESULTS_DIR = "/workspace/datasets/rover/results"
CONFIGS_DIR = "/workspace/datasets/rover/configs"
ORBSLAM3_DIR = "/workspace/third_party/ORB_SLAM3"
VOCAB = os.path.join(ORBSLAM3_DIR, "Vocabulary", "ORBvoc.txt")
CONFIG = os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo.yaml")
EXECUTABLE = os.path.join(ORBSLAM3_DIR, "Examples", "Stereo", "stereo_euroc")

MAX_ATTEMPTS = 3
TIMEOUT = 2700  # 45 minutes (longer than default 30 min)

# recordings that need Stereo PH results (failed or missing)
FAILED_RECORDINGS = [
    "garden_large_dusk_2024-05-29_2",
    "garden_large_night-light_2024-05-30_2",
    "garden_large_summer_2023-08-18",
    "park_day_2024-05-08",
    "park_night-light_2024-05-24_2",
    "campus_large_day_2024-09-25",
    "campus_large_dusk_2024-09-24_2",
    "campus_large_night-light_2024-09-24_4",  # Never tested
]

# T265 calibration
CAM_LEFT_K = np.array([
    [280.4362476646957, 0.0, 434.5911290024899],
    [0.0, 279.5757903173993, 395.3741210501516],
    [0.0, 0.0, 1.0]
])
CAM_LEFT_D = np.array([[-0.011532772136434897], [0.0501515488043061],
                        [-0.05041450901368907], [0.012741893876582578]])
CAM_RIGHT_K = np.array([
    [280.311263999059, 0.0, 431.35302371548494],
    [0.0, 279.5434630904508, 388.5071222043099],
    [0.0, 0.0, 1.0]
])
CAM_RIGHT_D = np.array([[-0.011950967309164085], [0.0530642563172375],
                         [-0.049469178559530994], [0.011573768486635416]])
ORIG_SIZE = (848, 800)
HFOV = 110
OUT_SIZE = (640, 480)


def log(msg):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def extract_timestamp(filename):
    name = filename.replace('.png', '')
    if '_' in name:
        parts = name.split('_')
        for p in reversed(parts):
            try:
                val = float(p)
                if val > 1e9:
                    return p
            except ValueError:
                continue
    return name


def ensure_pinhole_euroc(rec_name):
    """Ensure pinhole EuRoC data exists, create if not"""
    import cv2

    output_dir = Path(DATA_DIR) / f"{rec_name}_pinhole_euroc"
    if (output_dir / "times.txt").exists():
        n = sum(1 for _ in open(output_dir / "times.txt"))
        log(f"  Pinhole EuRoC exists: {n} frames")
        # gt_tum.txt might be missing if pinhole euroc already exists
        if not (output_dir / "gt_tum.txt").exists():
            rec_dir = Path(DATA_DIR) / rec_name
            gt_file = rec_dir / "groundtruth.txt"
            if not gt_file.exists():
                gt_file = rec_dir / "groundtruth"
            if gt_file.exists():
                gt_out = output_dir / "gt_tum.txt"
                with open(gt_file, 'r') as fin, open(gt_out, 'w') as fout:
                    for line in fin:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        parts = line.split()
                        if len(parts) >= 4:
                            try:
                                ts_s = float(parts[0])
                                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                                fout.write(f"{ts_s:.6f} {x} {y} {z} 0 0 0 1\n")
                            except ValueError:
                                continue
                log(f"  GT copied (was missing)")
            else:
                log(f"  WARNING: No GT file found for {rec_name}")
        return True

    rec_dir = Path(DATA_DIR) / rec_name
    left_dir = rec_dir / "realsense_T265" / "cam_left"
    right_dir = rec_dir / "realsense_T265" / "cam_right"
    imu_file = rec_dir / "realsense_T265" / "imu" / "imu.txt"
    gt_file = rec_dir / "groundtruth.txt"
    if not gt_file.exists():
        gt_file = rec_dir / "groundtruth"  # Some recordings omit .txt

    if not left_dir.exists():
        log(f"  ERROR: {left_dir} not found")
        return False

    out_w, out_h = OUT_SIZE
    fx = (out_w / 2.0) / math.tan(math.radians(HFOV / 2.0))
    new_K = np.array([[fx, 0, out_w / 2.0], [0, fx, out_h / 2.0], [0, 0, 1]], dtype=np.float64)

    R_identity = np.eye(3)
    map_left_x, map_left_y = cv2.fisheye.initUndistortRectifyMap(
        CAM_LEFT_K, CAM_LEFT_D, R_identity, new_K, OUT_SIZE, cv2.CV_32FC1)
    map_right_x, map_right_y = cv2.fisheye.initUndistortRectifyMap(
        CAM_RIGHT_K, CAM_RIGHT_D, R_identity, new_K, OUT_SIZE, cv2.CV_32FC1)

    cam0_dir = output_dir / "mav0" / "cam0" / "data"
    cam1_dir = output_dir / "mav0" / "cam1" / "data"
    imu_out_dir = output_dir / "mav0" / "imu0"
    cam0_dir.mkdir(parents=True, exist_ok=True)
    cam1_dir.mkdir(parents=True, exist_ok=True)
    imu_out_dir.mkdir(parents=True, exist_ok=True)

    left_files = sorted([f for f in os.listdir(left_dir) if f.endswith('.png')])
    right_files = sorted([f for f in os.listdir(right_dir) if f.endswith('.png')])

    left_ts_map = {extract_timestamp(f): f for f in left_files}
    right_ts_map = {extract_timestamp(f): f for f in right_files}
    common_ts = sorted(set(left_ts_map.keys()) & set(right_ts_map.keys()))
    log(f"  Undistorting {rec_name}: {len(common_ts)} stereo pairs...")

    times_list = []
    t0 = time.time()
    for i, ts in enumerate(common_ts):
        ts_ns = str(int(float(ts) * 1e9))

        img_l = cv2.imread(str(left_dir / left_ts_map[ts]), cv2.IMREAD_GRAYSCALE)
        img_r = cv2.imread(str(right_dir / right_ts_map[ts]), cv2.IMREAD_GRAYSCALE)
        if img_l is None or img_r is None:
            continue

        und_l = cv2.remap(img_l, map_left_x, map_left_y, cv2.INTER_LINEAR)
        und_r = cv2.remap(img_r, map_right_x, map_right_y, cv2.INTER_LINEAR)

        cv2.imwrite(str(cam0_dir / f"{ts_ns}.png"), und_l)
        cv2.imwrite(str(cam1_dir / f"{ts_ns}.png"), und_r)
        times_list.append(ts_ns)

        if (i + 1) % 3000 == 0:
            elapsed = time.time() - t0
            rate = (i + 1) / elapsed
            eta = (len(common_ts) - i - 1) / rate
            log(f"    {i + 1}/{len(common_ts)} ({rate:.1f} fps, ETA {eta:.0f}s)")

    elapsed = time.time() - t0
    log(f"  Undistorted {len(times_list)} pairs in {elapsed:.0f}s")

    with open(output_dir / "times.txt", 'w') as f:
        for ts_ns in times_list:
            f.write(f"{ts_ns}\n")

    # convert IMU
    if imu_file.exists():
        with open(imu_file, 'r') as fin, \
             open(imu_out_dir / "data.csv", 'w', newline='') as fout:
            writer = csv.writer(fout)
            writer.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]",
                             "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]",
                             "a_RS_S_x [m s^-2]", "a_RS_S_y [m s^-2]",
                             "a_RS_S_z [m s^-2]"])
            count = 0
            for row in csv.reader(fin):
                if len(row) < 7:
                    continue
                try:
                    ts_s = float(row[0])
                except ValueError:
                    continue
                ts_ns = int(ts_s * 1e9)
                ax, ay, az = float(row[1]), float(row[2]), float(row[3])
                gx, gy, gz = float(row[4]), float(row[5]), float(row[6])
                writer.writerow([ts_ns, gx, gy, gz, ax, ay, az])
                count += 1
        log(f"  IMU: {count} samples")

    # copy ground truth
    if gt_file.exists():
        gt_out = output_dir / "gt_tum.txt"
        with open(gt_file, 'r') as fin, open(gt_out, 'w') as fout:
            for line in fin:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) >= 4:
                    try:
                        ts_s = float(parts[0])
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        fout.write(f"{ts_s:.6f} {x} {y} {z} 0 0 0 1\n")
                    except ValueError:
                        continue
        log(f"  GT copied")

    return True


def evaluate_trajectory(traj_path, gt_path, out_dir, rec_name, total_frames, max_diff=0.5):
    """evaluate trajectory against GT"""
    # load trajectory
    traj = []
    with open(traj_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    ts = float(parts[0])
                    if ts > 1e15:
                        ts = ts / 1e9
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    traj.append((ts, np.array([x, y, z])))
                except ValueError:
                    continue

    # load GT
    gt = []
    with open(gt_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                try:
                    ts = float(parts[0])
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    gt.append((ts, np.array([x, y, z])))
                except ValueError:
                    continue

    if len(traj) < 10:
        result = {
            "error": "Too few estimated poses",
            "recording": rec_name,
            "mode": "stereo_pinhole",
            "num_estimated": len(traj)
        }
        with open(os.path.join(out_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    # associate by nearest timestamp
    gt_ts = np.array([g[0] for g in gt])
    matched = []
    for t_ts, t_pos in traj:
        idx = np.argmin(np.abs(gt_ts - t_ts))
        if abs(gt_ts[idx] - t_ts) < max_diff:
            matched.append((t_pos, gt[idx][1]))

    if len(matched) < 10:
        result = {
            "error": "Too few matched poses",
            "recording": rec_name,
            "mode": "stereo_pinhole",
            "num_estimated": len(traj),
            "num_matched": len(matched)
        }
        with open(os.path.join(out_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    est_pts = np.array([m[0] for m in matched])
    gt_pts = np.array([m[1] for m in matched])

    # sim3 alignment (with scale)
    est_c = est_pts.mean(axis=0)
    gt_c = gt_pts.mean(axis=0)
    est_centered = est_pts - est_c
    gt_centered = gt_pts - gt_c

    H = est_centered.T @ gt_centered
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T

    scale = np.sum(S * np.diag(D)) / np.sum(est_centered ** 2)
    t = gt_c - scale * R @ est_c

    aligned = scale * (R @ est_pts.T).T + t
    errors = np.linalg.norm(aligned - gt_pts, axis=1)

    ate_rmse = float(np.sqrt(np.mean(errors ** 2)))
    ate_mean = float(np.mean(errors))
    ate_median = float(np.median(errors))
    ate_std = float(np.std(errors))
    ate_max = float(np.max(errors))

    # SE3 (no scale)
    est_c2 = est_pts.mean(axis=0)
    gt_c2 = gt_pts.mean(axis=0)
    H2 = (est_pts - est_c2).T @ (gt_pts - gt_c2)
    U2, S2, Vt2 = np.linalg.svd(H2)
    d2 = np.linalg.det(Vt2.T @ U2.T)
    R2 = Vt2.T @ np.diag([1, 1, d2]) @ U2.T
    t2 = gt_c2 - R2 @ est_c2
    aligned_se3 = (R2 @ est_pts.T).T + t2
    errors_se3 = np.linalg.norm(aligned_se3 - gt_pts, axis=1)

    # rpe
    rpe_errors = []
    for i in range(1, len(matched)):
        dt_est = est_pts[i] - est_pts[i - 1]
        dt_gt = gt_pts[i] - gt_pts[i - 1]
        rpe_errors.append(np.linalg.norm(scale * R @ dt_est - dt_gt))

    tracking_pct = round(100.0 * len(traj) / total_frames, 1) if total_frames > 0 else 0

    result = {
        "recording": rec_name,
        "mode": "stereo_pinhole",
        "num_estimated": len(traj),
        "num_gt": len(gt),
        "num_matched": len(matched),
        "tracking_rate_pct": tracking_pct,
        "sim3_scale": round(float(scale), 6),
        "ate_sim3": {
            "rmse": round(ate_rmse, 4),
            "mean": round(ate_mean, 4),
            "median": round(ate_median, 4),
            "std": round(ate_std, 4),
            "max": round(ate_max, 4)
        },
        "ate_se3": {
            "rmse": round(float(np.sqrt(np.mean(errors_se3 ** 2))), 4),
            "mean": round(float(np.mean(errors_se3)), 4),
            "median": round(float(np.median(errors_se3)), 4)
        },
        "rpe": {
            "rmse": round(float(np.sqrt(np.mean(np.array(rpe_errors) ** 2))), 4),
            "mean": round(float(np.mean(rpe_errors)), 4)
        }
    }

    with open(os.path.join(out_dir, "eval_results.json"), 'w') as f:
        json.dump(result, f, indent=2)

    # plot
    try:
        fig, ax = plt.subplots(figsize=(10, 8))
        ax.plot(gt_pts[:, 0], gt_pts[:, 1], 'g-', label='Ground Truth', linewidth=2)
        ax.plot(aligned[:, 0], aligned[:, 1], 'r-', label=f'Stereo PH (ATE={ate_rmse:.2f}m, s={scale:.3f})', linewidth=1.5)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'{rec_name}: Stereo PinHole')
        ax.legend()
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(os.path.join(out_dir, "trajectory_comparison.png"), dpi=150)
        plt.close()
    except Exception as e:
        log(f"  Plot error: {e}")

    return result


def run_single_attempt(rec_name, attempt, out_dir):
    """Run a single ORB-SLAM3 attempt. Returns result dict or None on failure"""
    euroc_dir = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc")
    times_file = os.path.join(euroc_dir, "times.txt")
    gt_path = os.path.join(euroc_dir, "gt_tum.txt")

    if not os.path.exists(times_file):
        log(f"  ERROR: No pinhole EuRoC for {rec_name}")
        return None

    with open(times_file) as f:
        total_frames = sum(1 for _ in f)

    output_name = f"rover_{rec_name}_stereo_pinhole_attempt{attempt}"
    cmd = [EXECUTABLE, VOCAB, CONFIG, euroc_dir, times_file, output_name]

    log(f"  Attempt {attempt}/{MAX_ATTEMPTS}: {total_frames} frames, timeout={TIMEOUT}s")

    # clean up stale trajectory files
    for pattern in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
                    "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(pattern):
            os.remove(pattern)

    env = os.environ.copy()
    env["DISPLAY"] = ":99"
    env["QT_QPA_PLATFORM"] = "offscreen"
    t0 = time.time()

    try:
        ret = subprocess.run(
            ["xvfb-run", "-a", "--server-args=-screen 0 1024x768x24"] + cmd,
            capture_output=True, text=True, timeout=TIMEOUT, env=env, cwd="/tmp")
        elapsed = time.time() - t0

        # save log
        log_path = os.path.join(out_dir, f"orbslam3_log_attempt{attempt}.txt")
        with open(log_path, 'w') as f:
            f.write(f"CMD: {' '.join(cmd)}\n")
            f.write(f"ELAPSED: {elapsed:.1f}s\n")
            f.write(f"RETURN_CODE: {ret.returncode}\n\n")
            f.write("=== STDOUT (first 5000) ===\n")
            f.write(ret.stdout[:5000] if len(ret.stdout) > 5000 else ret.stdout)
            f.write(f"\n\n=== STDOUT (last 5000) ===\n")
            f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else "")
            f.write(f"\n\n=== STDERR ===\n")
            f.write(ret.stderr[-3000:] if len(ret.stderr) > 3000 else ret.stderr)

        if ret.returncode != 0:
            log(f"  Attempt {attempt}: CRASHED (exit code {ret.returncode}, {elapsed:.0f}s)")
            # check for segfault
            if ret.returncode == -11 or ret.returncode == 139:
                log(f"  -> SEGFAULT (Sophus/g2o bug, will retry)")
            return None

    except subprocess.TimeoutExpired as e:
        elapsed = time.time() - t0
        log(f"  Attempt {attempt}: TIMEOUT ({elapsed:.0f}s)")
        # kill lingering processes
        try:
            subprocess.run(["pkill", "-f", output_name], capture_output=True, timeout=5)
        except Exception:
            pass
        return None

    # find trajectory
    traj_file = None
    for pattern in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt"]:
        if os.path.exists(pattern):
            traj_file = pattern
            break

    if traj_file is None:
        for candidate in ["/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
            if os.path.exists(candidate):
                traj_file = candidate
                break

    if traj_file is None:
        log(f"  Attempt {attempt}: NO TRAJECTORY ({elapsed:.0f}s)")
        return None

    # copy trajectory
    traj_dst = os.path.join(out_dir, "trajectory_stereo_pinhole.txt")
    shutil.copy2(traj_file, traj_dst)

    # also save as attempt-specific
    shutil.copy2(traj_file, os.path.join(out_dir, f"trajectory_attempt{attempt}.txt"))

    # clean up
    for pattern in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
                    "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(pattern):
            os.remove(pattern)

    # evaluate
    result = evaluate_trajectory(traj_dst, gt_path, out_dir, rec_name,
                                 total_frames=total_frames)

    ate = result.get('ate_sim3', {}).get('rmse', '?')
    scale = result.get('sim3_scale', '?')
    log(f"  Attempt {attempt}: SUCCESS, ATE={ate}m, scale={scale} ({elapsed:.0f}s)")

    return result


def main():
    log("ROVER Stereo PinHole: Retry Failed/Missing Recordings")
    log(f"  Recordings to process: {len(FAILED_RECORDINGS)}")
    log(f"  Max attempts per recording: {MAX_ATTEMPTS}")
    log(f"  Timeout per attempt: {TIMEOUT}s ({TIMEOUT/60:.0f} min)")

    total_start = time.time()
    summary = []

    for i, rec in enumerate(FAILED_RECORDINGS):
        log("")
        log(f"[{i+1}/{len(FAILED_RECORDINGS)}] {rec}")

        # check pinhole EuRoC data exists
        if not ensure_pinhole_euroc(rec):
            summary.append({"recording": rec, "status": "NO_DATA"})
            continue

        out_dir = os.path.join(RESULTS_DIR, rec, "stereo_pinhole")
        os.makedirs(out_dir, exist_ok=True)

        # remove old failed eval
        old_eval = os.path.join(out_dir, "eval_results.json")
        if os.path.exists(old_eval):
            os.remove(old_eval)

        best_result = None
        for attempt in range(1, MAX_ATTEMPTS + 1):
            result = run_single_attempt(rec, attempt, out_dir)
            if result and "error" not in result:
                best_result = result
                break  # Success, no need for more attempts
            # brief delay before retry
            if attempt < MAX_ATTEMPTS:
                log(f"  Waiting 5s before retry...")
                time.sleep(5)

        if best_result:
            summary.append({
                "recording": rec,
                "status": "OK",
                "ate": best_result["ate_sim3"]["rmse"],
                "scale": best_result["sim3_scale"],
                "attempts": attempt
            })
        else:
            summary.append({
                "recording": rec,
                "status": "FAILED_ALL_ATTEMPTS",
                "attempts": MAX_ATTEMPTS
            })
            # save failure result
            fail_result = {
                "error": f"Failed after {MAX_ATTEMPTS} attempts",
                "recording": rec,
                "mode": "stereo_pinhole"
            }
            with open(os.path.join(out_dir, "eval_results.json"), 'w') as f:
                json.dump(fail_result, f, indent=2)

    # final summary
    total_elapsed = time.time() - total_start

    log("")
    log("FINAL SUMMARY")

    n_ok = sum(1 for s in summary if s["status"] == "OK")
    n_fail = sum(1 for s in summary if s["status"] == "FAILED_ALL_ATTEMPTS")
    n_nodata = sum(1 for s in summary if s["status"] == "NO_DATA")

    log(f"Success: {n_ok}/{len(FAILED_RECORDINGS)}")
    log(f"Failed:  {n_fail}/{len(FAILED_RECORDINGS)}")
    if n_nodata:
        log(f"No data: {n_nodata}/{len(FAILED_RECORDINGS)}")
    log("")

    for s in summary:
        if s["status"] == "OK":
            log(f"  OK   {s['recording']}: ATE={s['ate']:.2f}m, scale={s['scale']:.3f} (attempt {s['attempts']})")
        elif s["status"] == "FAILED_ALL_ATTEMPTS":
            log(f"  FAIL {s['recording']}: all {s['attempts']} attempts failed")
        else:
            log(f"  SKIP {s['recording']}: no data")

    log("")
    log(f"Total time: {total_elapsed/60:.1f} min ({total_elapsed/3600:.1f} hours)")

    # regenerate heatmap
    log("Regenerating heatmap...")
    try:
        subprocess.run([sys.executable, "scripts/generate_heatmap.py"],
                       cwd="/workspace/datasets/rover", timeout=60)
    except Exception as e:
        log(f"Heatmap generation failed: {e}")

    # save summary JSON
    summary_path = os.path.join(RESULTS_DIR, "stereo_ph_retry_summary.json")
    with open(summary_path, 'w') as f:
        json.dump(summary, f, indent=2)
    log(f"Summary saved to {summary_path}")


if __name__ == "__main__":
    main()
