"""Pose graph optimization for SLAM.

Builds and optimizes a pose graph with odometry edges and
loop closure constraints using Open3D's pose graph optimizer.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np

logger = logging.getLogger(__name__)

try:
    import open3d as o3d

    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
    logger.warning("Open3D not installed. Pose graph optimization unavailable.")


@dataclass
class PoseGraphConfig:
    """Configuration for pose graph optimization.

    Attributes:
        optimizer: Optimization method ("levenberg_marquardt" or "gauss_newton").
        max_iterations: Maximum optimization iterations.
        odometry_information_scale: Scale for odometry edge information matrix.
        loop_information_scale: Scale for loop closure edge information matrix.
    """

    optimizer: str = "levenberg_marquardt"
    max_iterations: int = 100
    odometry_information_scale: float = 1.0
    loop_information_scale: float = 0.5

    @classmethod
    def from_dict(cls, config: dict) -> PoseGraphConfig:
        """Create config from dictionary."""
        return cls(**{k: v for k, v in config.items() if k in cls.__dataclass_fields__})


class SLAMPoseGraph:
    """Pose graph for SLAM with odometry and loop closure edges.

    Manages nodes (poses) and edges (constraints) and provides
    global optimization via Open3D.

    Args:
        config: Pose graph configuration.
    """

    def __init__(self, config: PoseGraphConfig | None = None) -> None:
        if not HAS_OPEN3D:
            raise RuntimeError("Open3D is required for pose graph optimization.")

        self.config = config or PoseGraphConfig()
        self._pose_graph = o3d.pipelines.registration.PoseGraph()
        self._num_nodes = 0
        self._loop_closures: list[tuple[int, int, np.ndarray, float]] = []

        logger.info("Pose graph initialized.")

    def add_node(self, pose: np.ndarray) -> int:
        """Add a pose node to the graph.

        Args:
            pose: 4x4 SE3 pose matrix.

        Returns:
            Index of the added node.
        """
        node = o3d.pipelines.registration.PoseGraphNode(pose.copy())
        self._pose_graph.nodes.append(node)
        node_idx = self._num_nodes
        self._num_nodes += 1
        return node_idx

    def add_odometry_edge(
        self,
        source_idx: int,
        target_idx: int,
        transform: np.ndarray,
        information: np.ndarray | None = None,
        uncertain: bool = False,
    ) -> None:
        """Add an odometry (sequential) edge between two nodes.

        Args:
            source_idx: Source node index.
            target_idx: Target node index.
            transform: 4x4 relative transform from source to target.
            information: 6x6 information matrix. If None, uses identity
                scaled by config.
            uncertain: Whether this edge is uncertain (e.g., from
                failed registration).
        """
        if information is None:
            scale = self.config.odometry_information_scale
            information = np.eye(6, dtype=np.float64) * scale

        edge = o3d.pipelines.registration.PoseGraphEdge(
            source_node_id=source_idx,
            target_node_id=target_idx,
            transformation=transform.copy(),
            information=information,
            uncertain=uncertain,
        )
        self._pose_graph.edges.append(edge)

    def add_loop_closure(
        self,
        source_idx: int,
        target_idx: int,
        transform: np.ndarray,
        confidence: float = 1.0,
        information: np.ndarray | None = None,
    ) -> None:
        """Add a loop closure edge between two nodes.

        Args:
            source_idx: Source node index.
            target_idx: Target node index.
            transform: 4x4 relative transform from source to target.
            confidence: Confidence score for this loop closure (0-1).
            information: 6x6 information matrix. If None, uses identity
                scaled by config and confidence.
        """
        if information is None:
            scale = self.config.loop_information_scale * confidence
            information = np.eye(6, dtype=np.float64) * scale

        edge = o3d.pipelines.registration.PoseGraphEdge(
            source_node_id=source_idx,
            target_node_id=target_idx,
            transformation=transform.copy(),
            information=information,
            uncertain=True,
        )
        self._pose_graph.edges.append(edge)
        self._loop_closures.append(
            (source_idx, target_idx, transform.copy(), confidence)
        )

        logger.info(
            "Loop closure added: %d -> %d (confidence=%.3f)",
            source_idx, target_idx, confidence,
        )

    def optimize(self) -> np.ndarray:
        """Optimize the pose graph.

        Returns:
            Optimized poses as (N, 4, 4) array.
        """
        logger.info(
            "Optimizing pose graph with %d nodes, %d edges (%d loop closures)",
            self._num_nodes,
            len(self._pose_graph.edges),
            len(self._loop_closures),
        )

        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=1.0,
            edge_prune_threshold=0.25,
            reference_node=0,
        )

        if self.config.optimizer == "levenberg_marquardt":
            method = o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt()
        else:
            method = o3d.pipelines.registration.GlobalOptimizationGaussNewton()

        convergence = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
        convergence.max_iteration = self.config.max_iterations

        o3d.pipelines.registration.global_optimization(
            self._pose_graph, method, convergence, option,
        )

        optimized = self.get_poses()
        logger.info("Pose graph optimization complete.")
        return optimized

    def get_poses(self) -> np.ndarray:
        """Get current poses from the graph.

        Returns:
            (N, 4, 4) array of SE3 poses.
        """
        poses = np.array([
            node.pose for node in self._pose_graph.nodes
        ])
        return poses

    def get_num_nodes(self) -> int:
        """Get number of nodes in the graph."""
        return self._num_nodes

    def get_num_edges(self) -> int:
        """Get number of edges in the graph."""
        return len(self._pose_graph.edges)

    def get_num_loop_closures(self) -> int:
        """Get number of loop closure edges."""
        return len(self._loop_closures)

    def build_from_odometry(
        self,
        poses: np.ndarray,
        timestamps: np.ndarray | None = None,
    ) -> None:
        """Build pose graph from a sequence of odometry poses.

        Adds all poses as nodes and creates sequential odometry edges.

        Args:
            poses: (N, 4, 4) array of SE3 odometry poses.
            timestamps: Optional (N,) array of timestamps.
        """
        n = len(poses)
        if n == 0:
            return

        # Add nodes
        for i in range(n):
            self.add_node(poses[i])

        # Add sequential edges
        for i in range(n - 1):
            relative = np.linalg.inv(poses[i]) @ poses[i + 1]
            self.add_odometry_edge(i, i + 1, relative)

        logger.info("Built pose graph from %d odometry poses.", n)
