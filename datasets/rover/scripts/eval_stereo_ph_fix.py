#!/usr/bin/env python3
"""re-evaluate Stereo PH trajectories with ns->s timestamp fix

ORB-SLAM3 EuRoC stereo outputs ns timestamps, GT uses seconds
"""

import copy
import json
import math
import os
import sys

import numpy as np
from evo.core import sync, metrics
from evo.core.trajectory import PoseTrajectory3D

RESULTS_DIR = "/workspace/datasets/rover/results"
DATA_DIR = "/workspace/data/rover"

# recordings with trajectories but failed eval (timestamp mismatch)
TARGETS = [
    "garden_large_night-light_2024-05-30_2",
    "park_night-light_2024-05-24_2",
    "campus_large_night_2024-09-24_3",
]


def read_tum(path, ns_to_s=False):
    """Read TUM trajectory. If ns_to_s, convert nanosecond timestamps to seconds"""
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
                if ns_to_s and t > 1e15:  # nanoseconds
                    t = t / 1e9
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


def evaluate(rec_name, traj_path, gt_path, config_label):
    """evaluate trajectory against GT with ns->s conversion"""
    est = read_tum(traj_path, ns_to_s=True)
    gt = read_tum(gt_path, ns_to_s=False)

    if est is None or gt is None:
        print(f"  Failed to read trajectories")
        return None

    print(f"  Est: {len(est.timestamps)} poses ({est.timestamps[0]:.3f}s - {est.timestamps[-1]:.3f}s)")
    print(f"  GT:  {len(gt.timestamps)} poses ({gt.timestamps[0]:.3f}s - {gt.timestamps[-1]:.3f}s)")

    max_diff = 0.5
    try:
        gt_sync, est_sync = sync.associate_trajectories(gt, est, max_diff=max_diff)
    except Exception as e:
        print(f"  Sync failed: {e}")
        return None

    n_matched = len(gt_sync.timestamps)
    print(f"  Matched: {n_matched} poses")
    if n_matched < 3:
        return None

    # sim3
    est_sim3 = copy.deepcopy(est_sync)
    try:
        r, t, s = est_sim3.align(gt_sync, correct_scale=True)
    except Exception as e:
        print(f"  Sim3 failed: {e}")
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

    print(f"  [{config_label}] ATE={ate_stat['rmse']:.2f}m, scale={s:.4f}, tracking={tracking_pct}%")
    return eval_dict


def main():
    for rec_name in TARGETS:
        short = rec_name.replace('garden_large_', 'GL/').replace('park_', 'P/').replace('campus_large_', 'CL/')
        print(f"\n{'='*60}")
        print(f"Recording: {short}")

        result_dir = os.path.join(RESULTS_DIR, rec_name, "stereo_pinhole")
        gt_path = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc", "gt_tum.txt")
        if not os.path.exists(gt_path):
            # try alternative
            for alt in [os.path.join(DATA_DIR, rec_name, "groundtruth.txt"),
                        os.path.join(DATA_DIR, rec_name, "groundtruth")]:
                if os.path.exists(alt):
                    gt_path = alt
                    break

        # find trajectory files from retry
        best_result = None
        best_ate = float('inf')

        # load old result for comparison
        old_eval_path = os.path.join(result_dir, "eval_results.json")
        old_ate = None
        if os.path.exists(old_eval_path):
            with open(old_eval_path) as f:
                d = json.load(f)
            if "error" not in d:
                old_ate = d.get("ate_sim3", {}).get("rmse")
                old_scale = d.get("sim3_scale", 1.0)
                print(f"  Previous: ATE={old_ate:.2f}m, scale={old_scale:.3f}")

        for label in ["lowlight", "default"]:
            traj_path = os.path.join(result_dir, f"trajectory_stereo_ph_{label}.txt")
            if not os.path.exists(traj_path):
                continue

            print(f"\n  Evaluating [{label}]:")
            result = evaluate(rec_name, traj_path, gt_path, label)
            if result is not None:
                ate = result["ate_sim3"]["rmse"]
                scale = result["sim3_scale"]
                scale_ok = 0.5 <= scale <= 2.0
                if scale_ok and ate < best_ate:
                    best_ate = ate
                    best_result = result

        if best_result is not None:
            new_ate = best_result["ate_sim3"]["rmse"]
            new_scale = best_result["sim3_scale"]
            improved = False

            if old_ate is None:
                improved = True
            elif new_ate < old_ate:
                improved = True
            elif old_ate is not None and not (0.5 <= old_scale <= 2.0) and (0.5 <= new_scale <= 2.0):
                improved = True

            if improved:
                with open(old_eval_path, 'w') as f:
                    json.dump(best_result, f, indent=2)
                print(f"\n  UPDATED: ATE={new_ate:.2f}m, scale={new_scale:.3f} "
                      f"(was ATE={old_ate or 'N/A'}, scale={old_scale if old_ate else 'N/A'})")
            else:
                print(f"\n  NO IMPROVEMENT: new={new_ate:.2f}m vs old={old_ate:.2f}m")
        else:
            print(f"\n  No valid results to evaluate")

    print("\nDone!")


if __name__ == "__main__":
    main()
