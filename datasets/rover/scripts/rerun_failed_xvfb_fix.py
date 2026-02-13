#!/usr/bin/env python3
"""re-run all failed ORB-SLAM3 experiments with proper Xvfb display

root cause: xvfb-run creates auth-protected displays that conflict with
OpenCV's Qt plugins. fix: use pre-started Xvfb with -ac (no auth)

Prerequisites:
    # kill old Xvfb instances and start clean one:
    kill $(pgrep Xvfb) 2>/dev/null; sleep 1
    Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &

Usage:
    DISPLAY=:99 python3 -u rerun_failed_xvfb_fix.py 2>&1 | tee /workspace/datasets/rover/rerun_fix.log
"""

import json
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# === Paths ===
DATA_DIR = "/workspace/data/rover"
RESULTS_DIR = "/workspace/datasets/rover/results"
CONFIGS_DIR = "/workspace/datasets/rover/configs"
ORBSLAM3_DIR = "/workspace/third_party/ORB_SLAM3"
VOCAB = os.path.join(ORBSLAM3_DIR, "Vocabulary", "ORBvoc.txt")

CONFIGS = {
    "stereo_pinhole": os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo.yaml"),
    "stereo_inertial_pinhole": os.path.join(CONFIGS_DIR, "ROVER_T265_PinHole_Stereo_Inertial.yaml"),
    "rgbd": os.path.join(CONFIGS_DIR, "ROVER_D435i_RGBD.yaml"),
}

EXECUTABLES = {
    "stereo_pinhole": os.path.join(ORBSLAM3_DIR, "Examples", "Stereo", "stereo_euroc"),
    "stereo_inertial_pinhole": os.path.join(ORBSLAM3_DIR, "Examples", "Stereo-Inertial", "stereo_inertial_euroc"),
    "rgbd": os.path.join(ORBSLAM3_DIR, "Examples", "RGB-D", "rgbd_tum"),
}

ALL_RECORDINGS = [
    "garden_large_autumn_2023-12-21",
    "garden_large_day_2024-05-29_1",
    "garden_large_dusk_2024-05-29_2",
    "garden_large_night-light_2024-05-30_2",
    "garden_large_night_2024-05-30_1",
    "garden_large_spring_2024-04-11",
    "garden_large_summer_2023-08-18",
    "garden_large_winter_2024-01-13",
    "park_autumn_2023-11-07",
    "park_day_2024-05-08",
    "park_dusk_2024-05-13_1",
    "park_night-light_2024-05-24_2",
    "park_night_2024-05-13_2",
    "park_spring_2024-04-14",
    "park_summer_2023-07-31",
]


def log(msg):
    """print timestamped log msg"""
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def has_valid_result(rec_name, mode):
    """check if valid eval_results.json exists"""
    eval_json = os.path.join(RESULTS_DIR, rec_name, mode, "eval_results.json")
    if not os.path.exists(eval_json):
        return False
    try:
        with open(eval_json) as f:
            r = json.load(f)
        ate = r.get("ate_sim3", {}).get("rmse")
        return ate is not None and r.get("num_estimated", 0) > 100
    except Exception:
        return False


# === Evaluation (same as before) ===

def load_tum_trajectory(path):
    data = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            data.append([float(p) for p in parts[:8]])
    if not data:
        return np.array([]), np.array([]), np.array([])
    data = np.array(data)
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def load_orbslam_trajectory(path):
    ts, pos, quat = load_tum_trajectory(path)
    if len(ts) > 0 and ts[0] > 1e15:
        ts = ts / 1e9
    return ts, pos, quat


def associate_trajectories(ts_est, ts_gt, max_diff=0.5):
    idx_est, idx_gt = [], []
    gt_idx = 0
    for i, te in enumerate(ts_est):
        while gt_idx < len(ts_gt) - 1 and ts_gt[gt_idx + 1] <= te:
            gt_idx += 1
        best_j = gt_idx
        best_diff = abs(te - ts_gt[gt_idx])
        if gt_idx + 1 < len(ts_gt):
            d = abs(te - ts_gt[gt_idx + 1])
            if d < best_diff:
                best_j = gt_idx + 1
                best_diff = d
        if best_diff <= max_diff:
            idx_est.append(i)
            idx_gt.append(best_j)
    return np.array(idx_est), np.array(idx_gt)


def umeyama_alignment(src, dst, with_scale=True):
    n, d = src.shape
    mu_src, mu_dst = src.mean(0), dst.mean(0)
    src_c, dst_c = src - mu_src, dst - mu_dst
    var_src = np.sum(src_c ** 2) / n
    H = (dst_c.T @ src_c) / n
    U, S, Vt = np.linalg.svd(H)
    D_mat = np.eye(d)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D_mat[d - 1, d - 1] = -1
    R = U @ D_mat @ Vt
    s = np.trace(np.diag(S) @ D_mat) / var_src if with_scale else 1.0
    t = mu_dst - s * R @ mu_src
    return R, t, s


def evaluate_trajectory(traj_path, gt_path, output_dir, mode_name, rec_name,
                        total_frames=0, max_diff=0.5):
    os.makedirs(output_dir, exist_ok=True)

    ts_est, pos_est, quat_est = load_orbslam_trajectory(traj_path)
    ts_gt, pos_gt, quat_gt = load_tum_trajectory(gt_path)

    if len(ts_est) < 3:
        result = {"error": "Too few estimated poses", "recording": rec_name,
                  "mode": mode_name, "num_estimated": int(len(ts_est))}
        with open(os.path.join(output_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    idx_e, idx_g = associate_trajectories(ts_est, ts_gt, max_diff=max_diff)
    if len(idx_e) < 3:
        result = {"error": "Too few matched pairs", "recording": rec_name,
                  "mode": mode_name, "num_estimated": int(len(ts_est)),
                  "num_matched": int(len(idx_e))}
        with open(os.path.join(output_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    pos_e, pos_g = pos_est[idx_e], pos_gt[idx_g]
    ts_matched = ts_est[idx_e]

    R, t, s = umeyama_alignment(pos_e, pos_g, with_scale=True)
    pos_aligned = s * (R @ pos_e.T).T + t
    ate_errors = np.linalg.norm(pos_aligned - pos_g, axis=1)

    R_se3, t_se3, _ = umeyama_alignment(pos_e, pos_g, with_scale=False)
    pos_se3 = (R_se3 @ pos_e.T).T + t_se3
    ate_se3 = np.linalg.norm(pos_se3 - pos_g, axis=1)

    rpe = []
    for i in range(len(pos_aligned) - 1):
        dp_est = pos_aligned[i + 1] - pos_aligned[i]
        dp_gt = pos_g[i + 1] - pos_g[i]
        rpe.append(np.linalg.norm(dp_gt - dp_est))
    rpe = np.array(rpe) if rpe else np.array([0.0])

    tracking_rate = len(ts_est) / total_frames * 100 if total_frames > 0 else -1

    results = {
        "recording": rec_name,
        "mode": mode_name,
        "num_estimated": int(len(ts_est)),
        "num_gt": int(len(ts_gt)),
        "num_matched": int(len(idx_e)),
        "tracking_rate_pct": round(tracking_rate, 1) if tracking_rate > 0 else "N/A",
        "sim3_scale": round(float(s), 6),
        "ate_sim3": {
            "rmse": round(float(np.sqrt(np.mean(ate_errors**2))), 4),
            "mean": round(float(np.mean(ate_errors)), 4),
            "median": round(float(np.median(ate_errors)), 4),
            "std": round(float(np.std(ate_errors)), 4),
            "max": round(float(np.max(ate_errors)), 4),
        },
        "ate_se3": {
            "rmse": round(float(np.sqrt(np.mean(ate_se3**2))), 4),
            "mean": round(float(np.mean(ate_se3)), 4),
            "median": round(float(np.median(ate_se3)), 4),
        },
        "rpe": {
            "rmse": round(float(np.sqrt(np.mean(rpe**2))), 4),
            "mean": round(float(np.mean(rpe)), 4),
        },
    }

    with open(os.path.join(output_dir, "eval_results.json"), 'w') as f:
        json.dump(results, f, indent=2)

    try:
        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        ax = axes[0]
        ax.plot(pos_g[:, 0], pos_g[:, 1], 'b-', lw=1, alpha=0.7, label='GT')
        ax.plot(pos_aligned[:, 0], pos_aligned[:, 1], 'r-', lw=1, alpha=0.7,
                label=f'{mode_name} (s={s:.3f})')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'Trajectory: {rec_name}')
        ax.legend(fontsize=8); ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        ax = axes[1]
        sc = ax.scatter(pos_aligned[:, 0], pos_aligned[:, 1], c=ate_errors,
                        cmap='hot', s=3, vmin=0)
        plt.colorbar(sc, ax=ax, label='ATE (m)')
        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
        ax.set_title(f'ATE Heatmap (RMSE={results["ate_sim3"]["rmse"]:.2f}m)')
        ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

        ax = axes[2]
        t_rel = ts_matched - ts_matched[0]
        ax.plot(t_rel, ate_errors, 'r-', lw=0.5, alpha=0.7)
        ax.axhline(results['ate_sim3']['rmse'], color='b', ls='--', alpha=0.5,
                    label=f'RMSE={results["ate_sim3"]["rmse"]:.2f}m')
        ax.set_xlabel('Time (s)'); ax.set_ylabel('ATE (m)')
        ax.set_title(f'{mode_name} ATE Over Time')
        ax.legend(); ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, "trajectory_comparison.png"), dpi=120)
        plt.close()
    except Exception as e:
        log(f"  Plot error: {e}")

    return results


def run_orbslam3(rec_name, mode):
    """run ORB-SLAM3 directly, uses pre-started Xvfb"""
    out_dir = os.path.join(RESULTS_DIR, rec_name, mode)
    os.makedirs(out_dir, exist_ok=True)

    exe = EXECUTABLES[mode]
    config = CONFIGS[mode]

    if mode in ("stereo_pinhole", "stereo_inertial_pinhole"):
        euroc_dir = os.path.join(DATA_DIR, f"{rec_name}_pinhole_euroc")
        times_file = os.path.join(euroc_dir, "times.txt")
        gt_path = os.path.join(euroc_dir, "gt_tum.txt")
        if not os.path.exists(times_file):
            return {"error": "no pinhole data", "recording": rec_name, "mode": mode}
        with open(times_file) as f:
            total_frames = sum(1 for _ in f)
        output_name = f"rover_{rec_name}_{mode}"
        cmd = [exe, VOCAB, config, euroc_dir, times_file, output_name]

    elif mode == "rgbd":
        rgbd_dir = os.path.join(DATA_DIR, f"{rec_name}_rgbd")
        assoc_file = os.path.join(rgbd_dir, "associations.txt")
        gt_path = os.path.join(rgbd_dir, "gt_tum.txt")
        if not os.path.exists(assoc_file):
            return {"error": "no rgbd data", "recording": rec_name, "mode": mode}
        with open(assoc_file) as f:
            total_frames = sum(1 for _ in f)
        output_name = f"rover_{rec_name}_rgbd"
        cmd = [exe, VOCAB, config, rgbd_dir, assoc_file]

    log(f"  RUN {rec_name}/{mode} ({total_frames} frames)")
    log_path = os.path.join(out_dir, "orbslam3_log.txt")

    t0 = time.time()
    env = os.environ.copy()
    # use existing DISPLAY from clean Xvfb
    # remove XAUTHORITY to avoid auth issues
    env.pop("XAUTHORITY", None)
    # remove QT_PLUGIN_PATH to avoid OpenCV Qt conflict
    env.pop("QT_PLUGIN_PATH", None)

    try:
        ret = subprocess.run(
            cmd, capture_output=True, text=True, timeout=1800,
            env=env, cwd="/tmp")
        elapsed = time.time() - t0

        with open(log_path, 'w') as f:
            f.write(f"CMD: {' '.join(cmd)}\n")
            f.write(f"ELAPSED: {elapsed:.1f}s\n")
            f.write(f"RETURN_CODE: {ret.returncode}\n\n")
            f.write("=== STDOUT (last 5000 chars) ===\n")
            f.write(ret.stdout[-5000:] if len(ret.stdout) > 5000 else ret.stdout)
            f.write("\n=== STDERR (last 5000 chars) ===\n")
            f.write(ret.stderr[-5000:] if len(ret.stderr) > 5000 else ret.stderr)

        if ret.returncode != 0:
            log(f"  WARN {rec_name}/{mode}: rc={ret.returncode} ({elapsed:.0f}s)")

    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log(f"  TIMEOUT {rec_name}/{mode} (>{elapsed:.0f}s)")
        return {"error": "timeout", "recording": rec_name, "mode": mode}

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
        log(f"  NO TRAJECTORY {rec_name}/{mode} (rc={ret.returncode})")
        result = {"error": "no trajectory file", "recording": rec_name, "mode": mode}
        with open(os.path.join(out_dir, "eval_results.json"), 'w') as f:
            json.dump(result, f, indent=2)
        return result

    traj_dst = os.path.join(out_dir, f"trajectory_{mode}.txt")
    shutil.copy2(traj_file, traj_dst)

    # clean up /tmp
    for p in [f"/tmp/kf_{output_name}.txt", f"/tmp/f_{output_name}.txt",
              "/tmp/CameraTrajectory.txt", "/tmp/KeyFrameTrajectory.txt"]:
        if os.path.exists(p):
            os.remove(p)

    result = evaluate_trajectory(
        traj_dst, gt_path, out_dir, mode, rec_name,
        total_frames=total_frames, max_diff=0.5)

    ate = result.get('ate_sim3', {}).get('rmse', '?')
    scale = result.get('sim3_scale', '?')
    log(f"  DONE {rec_name}/{mode}: ATE={ate}m, scale={scale} ({elapsed:.0f}s)")
    return result


def make_summary():
    """generate summary from all results"""
    all_results = []
    modes = ["stereo_pinhole", "stereo_inertial_pinhole", "rgbd"]

    for rec in ALL_RECORDINGS:
        for mode in modes:
            eval_json = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
            if os.path.exists(eval_json):
                with open(eval_json) as f:
                    all_results.append(json.load(f))

    rows = []
    for r in all_results:
        if "error" in r:
            rows.append({"recording": r.get("recording", "?"), "mode": r.get("mode", "?"),
                          "ate_rmse": None, "error": r["error"]})
        else:
            rows.append({"recording": r["recording"], "mode": r["mode"],
                          "ate_rmse": r["ate_sim3"]["rmse"],
                          "ate_mean": r["ate_sim3"]["mean"],
                          "ate_median": r["ate_sim3"]["median"],
                          "scale": r["sim3_scale"], "matched": r["num_matched"]})

    with open(os.path.join(RESULTS_DIR, "all_results_v3.json"), 'w') as f:
        json.dump(all_results, f, indent=2)

    with open(os.path.join(RESULTS_DIR, "summary_v3.txt"), 'w') as f:
        f.write("ROVER ORB-SLAM3 Results (v3: Xvfb fix)\n")
        f.write("=" * 100 + "\n\n")
        f.write(f"{'Recording':<45} {'Mode':<25} {'ATE RMSE':>10} {'Scale':>8} {'Matched':>8}\n")
        f.write("-" * 100 + "\n")
        for r in sorted(rows, key=lambda x: (x["recording"], x["mode"])):
            if r.get("ate_rmse") is not None:
                f.write(f"{r['recording']:<45} {r['mode']:<25} "
                        f"{r['ate_rmse']:>10.4f} {r['scale']:>8.4f} {r['matched']:>8}\n")
            else:
                f.write(f"{r['recording']:<45} {r['mode']:<25} "
                        f"{'FAILED':>10} {'':>8} {'':>8}  ({r['error']})\n")

        f.write("\n" + "=" * 100 + "\n")
        f.write("Summary per mode:\n")
        for mode in modes:
            vals = [r["ate_rmse"] for r in rows if r["mode"] == mode and r.get("ate_rmse") is not None]
            n_fail = sum(1 for r in rows if r["mode"] == mode and r.get("ate_rmse") is None)
            if vals:
                f.write(f"  {mode:<25}: mean={np.mean(vals):.4f}m, "
                        f"median={np.median(vals):.4f}m, "
                        f"success={len(vals)}/{len(vals)+n_fail}\n")

    log(f"Summary saved to {RESULTS_DIR}/summary_v3.txt")

    # plots
    try:
        mode_labels = ["Stereo PH", "Stereo-Inertial PH", "RGB-D"]
        recordings = sorted(set(r["recording"] for r in rows))

        fig, ax = plt.subplots(figsize=(max(16, len(recordings)), 8))
        x = np.arange(len(recordings))
        width = 0.25
        for i, (mode, label) in enumerate(zip(modes, mode_labels)):
            vals = []
            for rec in recordings:
                r = [row for row in rows if row["recording"] == rec and row["mode"] == mode]
                vals.append(r[0]["ate_rmse"] if r and r[0].get("ate_rmse") else 0)
            ax.bar(x + i * width, vals, width, label=label, alpha=0.8)
        ax.set_xlabel('Recording'); ax.set_ylabel('ATE RMSE (m)')
        ax.set_title('ROVER ORB-SLAM3: All 3 Modes (v3 Xvfb fix)')
        ax.set_xticks(x + width)
        ax.set_xticklabels([r.replace('garden_large_', 'GL/').replace('park_', 'P/')
                            for r in recordings], rotation=45, ha='right', fontsize=8)
        ax.legend(); ax.grid(True, alpha=0.3, axis='y')
        plt.tight_layout()
        plt.savefig(os.path.join(RESULTS_DIR, "comparison_bar_v3.png"), dpi=150)
        plt.close()

        fig, ax = plt.subplots(figsize=(8, max(8, len(recordings) * 0.5)))
        data = np.zeros((len(recordings), len(modes)))
        for i, rec in enumerate(recordings):
            for j, mode in enumerate(modes):
                r = [row for row in rows if row["recording"] == rec and row["mode"] == mode]
                data[i, j] = r[0]["ate_rmse"] if r and r[0].get("ate_rmse") else np.nan
        vmax = np.nanpercentile(data, 90) if not np.all(np.isnan(data)) else 5
        im = ax.imshow(data, cmap='RdYlGn_r', aspect='auto', vmin=0, vmax=vmax)
        ax.set_xticks(range(len(modes))); ax.set_xticklabels(mode_labels)
        ax.set_yticks(range(len(recordings)))
        ax.set_yticklabels([r.replace('garden_large_', 'GL/').replace('park_', 'P/')
                            for r in recordings], fontsize=8)
        median_val = np.nanmedian(data)
        for i in range(len(recordings)):
            for j in range(len(modes)):
                v = data[i, j]
                if not np.isnan(v):
                    ax.text(j, i, f'{v:.2f}', ha='center', va='center', fontsize=7,
                            color='white' if v > median_val else 'black')
                else:
                    ax.text(j, i, 'X', ha='center', va='center', fontsize=7, color='gray')
        plt.colorbar(im, label='ATE RMSE (m)')
        ax.set_title('ORB-SLAM3 ATE RMSE Heatmap (v3)')
        plt.tight_layout()
        plt.savefig(os.path.join(RESULTS_DIR, "comparison_heatmap_v3.png"), dpi=150)
        plt.close()
        log("Plots saved")
    except Exception as e:
        log(f"Plot error: {e}")


def main():
    log("=" * 70)
    log("ROVER Re-run Failed Experiments (Xvfb fix)")
    log("=" * 70)

    # verify DISPLAY is set
    display = os.environ.get("DISPLAY")
    if not display:
        log("ERROR: DISPLAY not set! Start Xvfb first:")
        log("  Xvfb :99 -screen 0 1024x768x24 -ac &")
        log("  export DISPLAY=:99")
        sys.exit(1)
    log(f"Using DISPLAY={display}")

    total_start = time.time()
    results_log = []

    # collect experiments that need re-running
    experiments = []
    for rec in ALL_RECORDINGS:
        for mode in ["stereo_pinhole", "rgbd"]:
            if not has_valid_result(rec, mode):
                experiments.append((rec, mode))
        # also re-run SI failures
        if not has_valid_result(rec, "stereo_inertial_pinhole"):
            experiments.append((rec, "stereo_inertial_pinhole"))

    log(f"Experiments to re-run: {len(experiments)}")
    for rec, mode in experiments:
        log(f"  {rec}/{mode}")

    # run all
    for i, (rec, mode) in enumerate(experiments):
        log(f"\n[{i+1}/{len(experiments)}] {rec}/{mode}")

        # remove old failed result
        old_eval = os.path.join(RESULTS_DIR, rec, mode, "eval_results.json")
        if os.path.exists(old_eval):
            os.remove(old_eval)

        result = run_orbslam3(rec, mode)
        results_log.append(result)

    # generate summary
    log("\nGenerating summary...")
    make_summary()

    total_elapsed = time.time() - total_start
    log(f"\nALL DONE! Total time: {total_elapsed/60:.1f} min ({total_elapsed/3600:.1f} hours)")

    n_ok = sum(1 for r in results_log if r.get("ate_sim3", {}).get("rmse") is not None)
    n_fail = len(results_log) - n_ok
    log(f"Results: {n_ok} succeeded, {n_fail} failed out of {len(results_log)} experiments")

    for r in results_log:
        ate = r.get("ate_sim3", {}).get("rmse")
        if ate is not None:
            log(f"  OK  {r['recording']}/{r['mode']}: ATE={ate:.3f}m, scale={r['sim3_scale']:.3f}")
        else:
            log(f"  FAIL {r.get('recording', '?')}/{r.get('mode', '?')}: {r.get('error', '?')}")


if __name__ == "__main__":
    import sys
   