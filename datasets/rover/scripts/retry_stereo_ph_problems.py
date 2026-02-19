#!/usr/bin/env python3
"""retry 5 problematic Stereo PH recordings with specialized configs

- CL/day, CL/dusk: LongSeq config (fewer features to prevent loop closure crash)
- GL/night-light, P/night-light, CL/night: LowLight config (more features, lower thresholds)

each recording tries default first, then specialized, keeps best
"""

import copy
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

DATA_DIR = "/workspace/data/rover"
RESULTS_DIR = "/workspace/datasets/rover/results"
ORBSLAM3_DIR = "/workspace/third_party/ORB_SLAM3"
VOCAB = os.path.join(ORBSLAM3_DIR, "Vocabulary", "ORBvoc.txt")
EXE = os.path.join(ORBSLAM3_DIR, "Examples", 'Stereo', 'stereo_euroc')

CONFIGS_DIR = "/workspace/datasets/rover/configs"
CONFIG_DEFAULT = os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo.yaml")
CONFIG_LONGSEQ = os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo_LongSeq.yaml")
CONFIG_LOWLIGHT = os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo_LowLight.yaml")

# mapping: recording -> list of (config_path, label) to try
TARGETS = {
    # crashes: try LongSeq first (fewer features), then default
    "campus_large_day_2024-09-25": [
        (CONFIG_LONGSEQ, "longseq"),
        (CONFIG_DEFAULT, "default"),
    ],
    "campus_large_dusk_2024-09-24_2": [
        (CONFIG_LONGSEQ, "longseq"),
        (CONFIG_DEFAULT, "default"),
    ],
    # night-light: try LowLight first, then default
    "garden_large_night-light_2024-05-30_2": [
        (CONFIG_LOWLIGHT, "lowlight"),
        (CONFIG_DEFAULT, "default"),
    ],
    "park_night-light_2024-05-24_2": [
        (CONFIG_LOWLIGHT, "lowlight"),
        (CONFIG_DEFAULT, "default"),
    ],
    # cL/night (very dark): LowLight only realistic option
    "campus_large_night_2024-09-24_3": [
        (CONFIG_LOWLIGHT, "lowlight"),
        (CONFIG_DEFAULT, "default"),
    ],
}

# voxel_size = 0.5  # tried, too coarse for park
TIMEOUT = 2700  # 45 min


def log(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def run_stereo(rec_name, config_path, config_label):
    """Run ORB-SLAM3 Stereo PH on a recording. Returns (ate, scale, eval_dict) or None"""
    euroc_dir = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc")
    times_file = os.path.join(euroc_dir, "times.txt")

    if not os.path.exists(times_file):
        log(f"  No EuRoC data for {rec_name}")
        return None

    result_dir = os.path.join(RESULTS_DIR, rec_name, "stereo_pinhole")
    os.makedirs(result_dir, exist_ok=True)

    cmd = [EXE, VOCAB, config_path, euroc_dir, times_file, "dataset-stereoph"]
    log(f"  Running [{config_label}]: {rec_name}")

    env = os.environ.copy()
    env["DISPLAY"] = ":99"
    env["QT_QPA_PLATFORM"] = "offscreen"

    start = time.time()
    try:
        proc = subprocess.run(
            cmd, timeout=TIMEOUT, capture_output=True, text=True,
            env=env, cwd="/tmp"
        )
        elapsed = time.time() - start
        log(f"  Finished in {elapsed:.0f}s, exit code: {proc.returncode}")

        # save log
        log_path = os.path.join(result_dir, f"orbslam3_log_{config_label}.txt")
        with open(log_path, 'w') as f:
            f.write(f"=== {config_label} ===\n")
            f.write(f"Config: {config_path}\n")
            f.write(f"Exit code: {proc.returncode}\n")
            f.write(f"Elapsed: {elapsed:.1f}s\n\n")
            f.write("=== STDOUT ===\n")
            f.write(proc.stdout[-10000:] if len(proc.stdout) > 10000 else proc.stdout)
            f.write("\n=== STDERR ===\n")
            f.write(proc.stderr[-5000:] if len(proc.stderr) > 5000 else proc.stderr)

        if proc.returncode != 0:
            log(f"  Non-zero exit code: {proc.returncode}")

    except subprocess.TimeoutExpired:
        log(f"  TIMEOUT after {TIMEOUT}s")
        return None

    # find trajectory
    traj_file = None
    for candidate in [
        f"/tmp/kf_dataset-stereoph.txt",
        f"/tmp/f_dataset-stereoph.txt",
        "/tmp/CameraTrajectory.txt",
        "/tmp/KeyFrameTrajectory.txt",
    ]:
        if os.path.exists(candidate) and os.path.getsize(candidate) > 100:
            traj_file = candidate
            break

    if traj_file is None:
        log(f"  No trajectory file found")
        return None

    # copy trajectory
    traj_dst = os.path.join(result_dir, f"trajectory_stereo_ph_{config_label}.txt")
    shutil.copy2(traj_file, traj_dst)

    # clean up
    for f_path in ["/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt",
                   f"/tmp/kf_dataset-stereoph.txt", f"/tmp/f_dataset-stereoph.txt"]:
        if os.path.exists(f_path):
            os.remove(f_path)

    # evaluate
    gt_path = os.path.join(euroc_dir, "gt_tum.txt")
    if not os.path.exists(gt_path):
        # try alternative GT locations
        for alt in [os.path.join(DATA_DIR, rec_name, "groundtruth.txt"),
                    os.path.join(DATA_DIR, rec_name, "groundtruth")]:
            if os.path.exists(alt):
                gt_path = alt
                break

    result = evaluate_trajectory(traj_dst, gt_path, rec_name, config_label)
    return result


def evaluate_trajectory(traj_path, gt_path, rec_name, config_label):
    # XXX: magic number, tuned by trial and error
    """evaluate trajectory against GT"""
    from evo.core import sync, metrics
    from evo.core.trajectory import PoseTrajectory3D

    def read_tum(path):
        stamps, xyz, quat = [], [], []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) < 8:
                    continue
                try:
                    t = float(parts[0])
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                except (ValueError, IndexError):
                    continue
                vals = [x, y, z, qx, qy, qz, qw]
                if any(math.isnan(v) or math.isinf(v) for v in vals):
                    continue
                stamps.append(t)
                xyz.append([x, y, z])
                quat.append([qx, qy, qz, qw])

        if len(stamps) < 3:
            return None

        return PoseTrajectory3D(
            positions_xyz=np.array(xyz),
            orientations_quat_wxyz=np.array(quat)[:, [3, 0, 1, 2]],
            timestamps=np.array(stamps),
        )

    est = read_tum(traj_path)
    gt = read_tum(gt_path)

    if est is None or gt is None:
        log(f"  Failed to read trajectories")
        return None

    # print(f"DEBUG traj_file={traj_file}")
    log(f"  Est: {len(est.timestamps)} poses, GT: {len(gt.timestamps)} poses")

    max_diff = 0.5
    try:
        gt_sync, est_sync = sync.associate_trajectories(gt, est, max_diff=max_diff)
    except Exception as e:
        log(f"  Sync failed: {e}")
        return None

    n_matched = len(gt_sync.timestamps)
    log(f"  Matched: {n_matched} poses")
    if n_matched < 3:
        return None

    # sim3 alignment
    est_sim3 = copy.deepcopy(est_sync)
    try:
        r, t, s = est_sim3.align(gt_sync, correct_scale=True)
    except Exception as e:
        log(f"  Sim3 alignment failed: {e}")
        return None

    # ate
    ate_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ate_metric.process_data((gt_sync, est_sim3))
    ate_stat = ate_metric.get_all_statistics()

    # rpe
    try:
        est_se3 = copy.deepcopy(est_sync)
        est_se3.align(gt_sync, correct_scale=False)
        rpe_metric = metrics.RPE(metrics.PoseRelation.translation_part, delta=1, delta_unit=metrics.Unit.frames)
        rpe_metric.process_data((gt_sync, est_se3))
        rpe_stat = rpe_metric.get_all_statistics()
    except:
        rpe_stat = {}

    tracking_pct = round(100.0 * n_matched / len(gt.timestamps), 1)

    eval_dict = {
        "recording": rec_name,
        "mode": "stereo_pinhole",
        "config": config_label,
        "num_estimated": len(est.timestamps),
        "num_gt": len(gt.timestamps),
        "num_matched": n_matched,
        "tracking_rate_pct": tracking_pct,
        "sim3_scale": round(s, 6),
        "ate_sim3": {
            "rmse": round(ate_stat["rmse"], 4),
            "mean": round(ate_stat["mean"], 4),
            "median": round(ate_stat["median"], 4),
            "std": round(ate_stat["std"], 4),
            "max": round(ate_stat["max"], 4),
        },
        "rpe": {
            "rmse": round(rpe_stat.get("rmse", 0), 4),
            "mean": round(rpe_stat.get("mean", 0), 4),
        },
    }

    ate_rmse = ate_stat["rmse"]
    log(f"  [{config_label}] ATE={ate_rmse:.2f}m, scale={s:.4f}, tracking={tracking_pct}%")
    return (ate_rmse, s, eval_dict)


def main():
    log("Stereo PH Problem Fix: 5 recordings x 2 configs")

    # check xvfb
    xvfb = subprocess.run(["pgrep", "Xvfb"], capture_output=True)
    if xvfb.returncode != 0:
        log("Starting Xvfb...")
        subprocess.Popen(
            ["Xvfb", ":99", "-screen", "0", "1024x768x24", "-ac"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(1)

    summary = []

    for rec_name, configs in TARGETS.items():
        short = rec_name.replace('garden_large_', 'GL/').replace('park_', 'P/').replace('campus_large_', 'CL/')
        log(f"\n{'='*60}")
        log(f"Recording: {short}")
        log(f"{'='*60}")

        # load old result for comparison
        old_eval = os.path.join(RESULTS_DIR, rec_name, "stereo_pinhole", "eval_results.json")
        old_ate = None
        if os.path.exists(old_eval):
            with open(old_eval) as f:
                d = json.load(f)
            if "error" not in d:
                old_ate = d.get("ate_sim3", {}).get("rmse")
                old_scale = d.get("sim3_scale", 1.0)
                log(f"  Previous: ATE={old_ate:.2f}m, scale={old_scale:.3f}")
            else:
                log(f"  Previous: FAIL ({d['error'][:40]})")

        best_result = None
        best_ate = float('inf')
        best_label = None

        for config, label in configs:
            result = run_stereo(rec_name, config, label)
            if result is not None:
                ate, scale, eval_dict = result
                # prefer good scale
                scale_ok = 0.5 <= scale <= 2.0
                if scale_ok and ate < best_ate:
                    best_ate = ate
                    best_result = eval_dict
                    best_label = label
                elif best_result is None:
                    best_result = eval_dict
                    best_label = label
                    best_ate = ate

        # check if new is better than old
        result_dir = os.path.join(RESULTS_DIR, rec_name, "stereo_pinhole")
        os.makedirs(result_dir, exist_ok=True)

        if best_result is not None:
            new_ate = best_result['ate_sim3']['rmse']
            new_scale = best_result['sim3_scale']
            improved = False

            if old_ate is None:
                # was FAIL, now has result
                improved = True
            elif 0.5 <= new_scale <= 2.0:
                if old_ate is not None and new_ate < old_ate:
                    improved = True
                elif old_ate is not None and not (0.5 <= old_scale <= 2.0):
                    improved = True  # Fixed bad scale

            if improved:
                # copy best trajectory as main
                traj_src = os.path.join(result_dir, f"trajectory_stereo_ph_{best_label}.txt")
                traj_dst = os.path.join(result_dir, "trajectory_stereo_ph.txt")
                if os.path.exists(traj_src):
                    shutil.copy2(traj_src, traj_dst)

                with open(os.path.join(result_dir, "eval_results.json"), 'w') as f:
                    json.dump(best_result, f, indent=2)
                log(f"  IMPROVED: [{best_label}] ATE={new_ate:.2f}m (was {old_ate or 'FAIL'})")
                summary.append((short, "IMPROVED", best_label, new_ate, new_scale))
            else:
                log(f"  NO IMPROVEMENT: new={new_ate:.2f}m vs old={old_ate:.2f}m, keeping old")
                summary.append((short, "NO_CHANGE", best_label, new_ate, new_scale))
        else:
            log(f"  ALL FAILED for {rec_name}")
            summary.append((short, "STILL_FAIL", None, None, None))

    # summary
    # print(f">>> {rec}: attempt {attempt}")
    log(f"\n{'='*60}")
    log("SUMMARY")
    log(f"{'='*60}")
    for short, status, label, ate, scale in summary:
        if ate is not None:
            log(f"  {short:<35} {status:<15} [{label}] ATE={ate:.2f}m, scale={scale:.3f}")
        else:
            log(f"  {short:<35} {status}")

    log("\nDone!")


if __name__ == "__main__":
    main()
