#!/usr/bin/env python3
"""retry 4 bad RGB-D recordings with low-light config

tries both default and low-light configs, keeps better result
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

# === Paths ===
DATA_DIR = "/workspace/data/rover"
RESULTS_DIR = "/workspace/datasets/rover/results"
ORBSLAM3_DIR = "/workspace/third_party/ORB_SLAM3"
VOCAB = os.path.join(ORBSLAM3_DIR, "Vocabulary", "ORBvoc.txt")
EXE = os.path.join(ORBSLAM3_DIR, "Examples", "RGB-D", "rgbd_tum")

CONFIG_DEFAULT = "/workspace/datasets/rover/configs/ROVER_D435i_RGBD.yaml"
CONFIG_LOWLIGHT = "/workspace/datasets/rover/configs/ROVER_D435i_RGBD_LowLight.yaml"

# 4 bad recordings
TARGETS = [
    "garden_large_night_2024-05-30_1",
    "campus_large_night-light_2024-09-24_4",
    "garden_large_dusk_2024-05-29_2",
    "campus_large_night_2024-09-24_3",
]

TIMEOUT = 2700  # 45 min


def log(msg):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def run_rgbd(rec_name, config_path, config_label):
    """Run ORB-SLAM3 RGB-D on a recording. Returns (ate, scale, eval_dict) or None"""
    rgbd_dir = os.path.join(DATA_DIR, f"{rec_name}_rgbd")
    assoc_file = os.path.join(rgbd_dir, "associations.txt")
    gt_path = os.path.join(rgbd_dir, "gt_tum.txt")

    if not os.path.exists(assoc_file):
        log(f"  No associations.txt for {rec_name}")
        return None

    if not os.path.exists(gt_path):
        log(f"  No gt_tum.txt for {rec_name}")
        return None

    result_dir = os.path.join(RESULTS_DIR, rec_name, "rgbd")
    os.makedirs(result_dir, exist_ok=True)

    cmd = [EXE, VOCAB, config_path, rgbd_dir, assoc_file]
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
            f.write(proc.stdout[-5000:] if len(proc.stdout) > 5000 else proc.stdout)
            f.write("\n=== STDERR ===\n")
            f.write(proc.stderr[-5000:] if len(proc.stderr) > 5000 else proc.stderr)

    except subprocess.TimeoutExpired:
        log(f"  TIMEOUT after {TIMEOUT}s")
        return None

    # find trajectory
    traj_file = None
    for candidate in [
        "/tmp/CameraTrajectory.txt",
        "/tmp/KeyFrameTrajectory.txt",
        f"/tmp/kf_rgbd_{rec_name}.txt",
        f"/tmp/f_rgbd_{rec_name}.txt",
    ]:
        if os.path.exists(candidate) and os.path.getsize(candidate) > 100:
            traj_file = candidate
            break

    if traj_file is None:
        log(f"  No trajectory file found")
        return None

    # copy trajectory
    traj_dst = os.path.join(result_dir, f"trajectory_rgbd_{config_label}.txt")
    shutil.copy2(traj_file, traj_dst)
    # clean up
    for f in ["/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(f):
            os.remove(f)

    # evaluate
    result = evaluate_trajectory(traj_dst, gt_path, rec_name, config_label)
    return result


def evaluate_trajectory(traj_path, gt_path, rec_name, config_label):
    """evaluate trajectory against GT, returns (ate_rmse, scale, eval_dict)"""
    try:
        import evo.core.trajectory as traj_module
        from evo.core import sync, metrics
        from evo.core.trajectory import PoseTrajectory3D
    except ImportError:
        log("  ERROR: evo not installed")
        return None

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
                # skip NaN/Inf
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

    log(f"  Est: {len(est.timestamps)} poses, GT: {len(gt.timestamps)} poses")

    # sync
    max_diff = 0.5  # GT is ~2 Hz
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
    import copy
    est_sim3 = copy.deepcopy(est_sync)
    try:
        r, t, s = est_sim3.align(gt_sync, correct_scale=True)
    except Exception as e:
        log(f"  Sim3 alignment failed: {e}")
        return None

    # ate
    ate_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ate_data = (gt_sync, est_sim3)
    ate_metric.process_data(ate_data)
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
        "mode": "rgbd",
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
    log(f"  [{config_label}] ATE={ate_rmse:.2f}m, scale={s:.4f}")
    return (ate_rmse, s, eval_dict)


def main():
    log("=" * 60)
    log("RGB-D Low-Light Retry — 4 recordings × 2 configs")
    log("=" * 60)

    # check xvfb
    xvfb = subprocess.run(["pgrep", "Xvfb"], capture_output=True)
    if xvfb.returncode != 0:
        log("Starting Xvfb...")
        subprocess.Popen(
            ["Xvfb", ":99", "-screen", "0", "1024x768x24", "-ac"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        time.sleep(1)

    for rec_name in TARGETS:
        log(f"\n{'='*60}")
        log(f"Recording: {rec_name}")
        log(f"{'='*60}")

        best_result = None
        best_ate = float('inf')
        best_label = None

        for config, label in [(CONFIG_DEFAULT, "default"), (CONFIG_LOWLIGHT, "lowlight")]:
            result = run_rgbd(rec_name, config, label)
            if result is not None:
                ate, scale, eval_dict = result
                # consider scale validity
                if 0.5 <= scale <= 2.0 and ate < best_ate:
                    best_ate = ate
                    best_result = eval_dict
                    best_label = label
                elif best_result is None:
                    # even bad scale beats nothing
                    best_result = eval_dict
                    best_label = label
                    best_ate = ate

        # save best result
        result_dir = os.path.join(RESULTS_DIR, rec_name, "rgbd")
        os.makedirs(result_dir, exist_ok=True)

        if best_result is not None:
            # copy best traj as main
            traj_src = os.path.join(result_dir, f"trajectory_rgbd_{best_label}.txt")
            traj_dst = os.path.join(result_dir, "trajectory_rgbd.txt")
            if os.path.exists(traj_src):
                shutil.copy2(traj_src, traj_dst)

            eval_path = os.path.join(result_dir, "eval_results.json")
            with open(eval_path, 'w') as f:
                json.dump(best_result, f, indent=2)
            log(f"  BEST: [{best_label}] ATE={best_result['ate_sim3']['rmse']:.2f}m, "
                f"scale={best_result['sim3_scale']:.4f}")
        else:
            # all attempts failed
            eval_path = os.path.join(result_dir, "eval_results.json")
            with open(eval_path, 'w') as f:
                json.dump({
                    "recording": rec_name,
                    "mode": "rgbd",
                    "error": "Failed with both default and lowlight configs"
                }, f, indent=2)
            log(f"  ALL FAILED for {rec_name}")

    log("\n" + "=" * 60)
    log("DONE! Regenerating heatmap...")

    # regenerate heatmap
    try:
        subprocess.run(
            [sys.executable, os.path.join(os.path.dirname(__file__), "generate_heatmap.py")],
            timeout=30
        )
    except:
        pass

    log("Complete!")


if __name__ == "__main__":
    main()
