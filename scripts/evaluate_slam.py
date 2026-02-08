#!/usr/bin/env python3
"""Evaluate SLAM pipeline on NCLT dataset.

Runs LiDAR odometry with optional loop closure and evaluates
against ground truth trajectories.

Usage:
    python scripts/evaluate_slam.py \\
        --dataset-config configs/dataset_config.yaml \\
        --slam-config configs/slam_config.yaml \\
        --session 2012-01-08

    # Odometry only (no loop closure)
    python scripts/evaluate_slam.py \\
        --dataset-config configs/dataset_config.yaml \\
        --slam-config configs/slam_config.yaml \\
        --session 2012-01-08 \\
        --no-loop-closure
"""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

import numpy as np
from tqdm import tqdm

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def run_odometry(
    dataset,
    slam_config: dict,
    max_frames: int | None = None,
) -> tuple[np.ndarray, list[int]]:
    """Run LiDAR odometry on a dataset session.

    Args:
        dataset: NCLTDataset instance for a single session.
        slam_config: SLAM configuration dictionary.
        max_frames: Optional limit on number of frames to process.

    Returns:
        Tuple of (estimated_poses, keyframe_indices).
    """
    from src.slam.lidar_odometry import LiDARodometry, OdometryConfig

    odom_config = OdometryConfig.from_dict(slam_config["slam"]["odometry"])
    odometry = LiDARodometry(odom_config)

    n_frames = len(dataset)
    if max_frames:
        n_frames = min(n_frames, max_frames)

    logger.info("Running odometry on %d frames...", n_frames)
    t0 = time.time()

    for i in tqdm(range(n_frames), desc="Odometry"):
        sample = dataset[i]
        points = sample["point_cloud"]
        if isinstance(points, np.ndarray):
            pass
        else:
            points = points.numpy()
        timestamp = float(sample.get("timestamp", i))
        odometry.process_frame(points, timestamp)

    elapsed = time.time() - t0
    logger.info(
        "Odometry complete: %d frames in %.1fs (%.1f FPS)",
        n_frames, elapsed, n_frames / elapsed,
    )

    result = odometry.get_result()
    poses = np.array(result.poses)
    return poses, result.keyframe_indices


def run_loop_closure(
    dataset,
    estimated_poses: np.ndarray,
    keyframe_indices: list[int],
    slam_config: dict,
) -> np.ndarray:
    """Apply loop closure to refine trajectory.

    Args:
        dataset: NCLTDataset instance.
        estimated_poses: (N, 4, 4) odometry poses.
        keyframe_indices: Indices of keyframes.
        slam_config: SLAM configuration.

    Returns:
        Optimized (N, 4, 4) poses.
    """
    from src.models.place_recognition import load_model
    from src.slam.pose_graph import PoseGraphConfig, SLAMPoseGraph

    lc_config = slam_config["slam"]["loop_closure"]
    pg_config = PoseGraphConfig.from_dict(slam_config["slam"]["pose_graph"])

    # Build pose graph from odometry
    pose_graph = SLAMPoseGraph(pg_config)
    pose_graph.build_from_odometry(estimated_poses)

    # Load place recognition model
    model_path = Path(lc_config["model_path"])
    if not model_path.exists():
        logger.warning("Loop closure model not found at %s. Skipping.", model_path)
        return estimated_poses

    model = load_model(model_path, device="cpu")
    logger.info("Loaded loop closure model from %s", model_path)

    # Extract descriptors for keyframes
    logger.info("Extracting descriptors for %d keyframes...", len(keyframe_indices))
    descriptors = []
    for idx in tqdm(keyframe_indices, desc="Descriptors"):
        sample = dataset[idx]
        points = sample["point_cloud"]
        if not isinstance(points, np.ndarray):
            points = points.numpy()

        import torch
        with torch.no_grad():
            desc = model([points[:, :3]])
        descriptors.append(desc.cpu().numpy().flatten())

    descriptors = np.array(descriptors)
    positions = estimated_poses[keyframe_indices, :3, 3]

    # Find loop closures
    search_radius = lc_config.get("search_radius", 50.0)
    min_time_diff = lc_config.get("min_time_diff", 30.0)
    sim_threshold = lc_config.get("similarity_threshold", 0.7)
    num_closures = 0

    for i in range(len(keyframe_indices)):
        for j in range(i + 1, len(keyframe_indices)):
            # Temporal constraint
            if abs(keyframe_indices[j] - keyframe_indices[i]) < min_time_diff:
                continue

            # Spatial constraint
            dist = np.linalg.norm(positions[i] - positions[j])
            if dist > search_radius:
                continue

            # Descriptor similarity
            sim = float(descriptors[i] @ descriptors[j])
            if sim > sim_threshold:
                # Compute relative transform from ICP
                relative = np.linalg.inv(estimated_poses[keyframe_indices[i]]) @ \
                           estimated_poses[keyframe_indices[j]]
                pose_graph.add_loop_closure(
                    keyframe_indices[i], keyframe_indices[j],
                    relative, confidence=sim,
                )
                num_closures += 1

    logger.info("Found %d loop closures.", num_closures)

    if num_closures > 0:
        optimized = pose_graph.optimize()
        return optimized

    return estimated_poses


def main() -> None:
    """Main evaluation entry point."""
    parser = argparse.ArgumentParser(description="Evaluate SLAM pipeline")
    parser.add_argument("--dataset-config", type=Path,
                        default=PROJECT_ROOT / "configs" / "dataset_config.yaml")
    parser.add_argument("--slam-config", type=Path,
                        default=PROJECT_ROOT / "configs" / "slam_config.yaml")
    parser.add_argument("--session", type=str, default="2012-01-08",
                        help="Session to evaluate on")
    parser.add_argument("--max-frames", type=int, default=None,
                        help="Maximum frames to process")
    parser.add_argument("--no-loop-closure", action="store_true",
                        help="Disable loop closure")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="Output directory for results")
    args = parser.parse_args()

    from src.datasets.nclt_dataset import NCLTDataset
    from src.evaluation.metrics import compute_ate, compute_rpe
    from src.evaluation.visualization import plot_trajectory_2d
    from src.utils.io_utils import load_config, save_results, save_trajectory

    slam_config = load_config(args.slam_config)
    output_dir = args.output_dir or Path(
        slam_config["slam"]["output"]["output_dir"]
    ) / args.session
    output_dir.mkdir(parents=True, exist_ok=True)

    # Load dataset
    dataset = NCLTDataset(config_path=args.dataset_config, split="test")
    session_dataset = dataset.get_session_dataset(args.session)
    logger.info("Loaded session %s with %d frames", args.session, len(session_dataset))

    # Run odometry
    estimated_poses, keyframe_indices = run_odometry(
        session_dataset, slam_config, args.max_frames,
    )

    # Loop closure
    if not args.no_loop_closure:
        estimated_poses = run_loop_closure(
            session_dataset, estimated_poses, keyframe_indices, slam_config,
        )

    # Get ground truth
    gt_poses = []
    for i in range(len(session_dataset)):
        sample = session_dataset[i]
        pose = sample["pose"]
        if not isinstance(pose, np.ndarray):
            pose = pose.numpy()
        gt_poses.append(pose)
    gt_poses = np.array(gt_poses)

    # Trim to same length
    n = min(len(estimated_poses), len(gt_poses))
    estimated_poses = estimated_poses[:n]
    gt_poses = gt_poses[:n]

    # Evaluate
    ate = compute_ate(estimated_poses, gt_poses)
    rpe = compute_rpe(estimated_poses, gt_poses)

    logger.info("=== Results for %s ===", args.session)
    logger.info("ATE RMSE: %.3f m", ate["rmse"])
    logger.info("ATE Mean: %.3f m", ate["mean"])
    logger.info("RPE Trans RMSE: %.3f m", rpe["trans_rmse"])
    logger.info("RPE Rot RMSE: %.3f deg", rpe["rot_rmse"])

    # Save results
    results = {"session": args.session, "ate": ate, "rpe": rpe}
    save_results(results, output_dir / "metrics.json")

    # Save trajectory
    traj_format = slam_config["slam"]["output"].get("trajectory_format", "tum")
    save_trajectory(estimated_poses, output_dir / f"estimated.{traj_format}")
    save_trajectory(gt_poses, output_dir / f"ground_truth.{traj_format}")

    # Plot
    fig = plot_trajectory_2d(
        {"Ground Truth": gt_poses, "Estimated": estimated_poses},
        output_path=output_dir / "trajectory.png",
        title=f"SLAM Trajectory - {args.session}",
    )

    logger.info("Results saved to %s", output_dir)


if __name__ == "__main__":
    main()
