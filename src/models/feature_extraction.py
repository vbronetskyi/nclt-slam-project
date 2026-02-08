"""Feature extraction backbone for 3D point clouds.

Provides sparse 3D convolutional feature extraction using MinkowskiEngine
for place recognition on LiDAR point clouds.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import torch
import torch.nn as nn

logger = logging.getLogger(__name__)

try:
    import MinkowskiEngine as ME

    HAS_MINKOWSKI = True
except ImportError:
    HAS_MINKOWSKI = False
    logger.warning(
        "MinkowskiEngine not installed. Sparse conv models unavailable. "
        "Install with: pip install MinkowskiEngine"
    )


class MinkFPN(nn.Module):
    """Minkowski Feature Pyramid Network for 3D point cloud features.

    A U-Net style sparse convolutional network that extracts multi-scale
    features from voxelized point clouds.

    Args:
        in_channels: Number of input feature channels (1 for occupancy,
            4 for x,y,z,intensity).
        out_channels: Output feature dimension.
        num_top_down: Number of top-down (decoder) layers.
        conv0_kernel_size: Kernel size for the initial convolution.
        block_channels: Channel sizes for each encoder block.
    """

    def __init__(
        self,
        in_channels: int = 1,
        out_channels: int = 256,
        num_top_down: int = 2,
        conv0_kernel_size: int = 5,
        block_channels: tuple[int, ...] = (32, 64, 64),
    ) -> None:
        super().__init__()

        if not HAS_MINKOWSKI:
            raise RuntimeError(
                "MinkowskiEngine is required for MinkFPN. "
                "Install with: pip install MinkowskiEngine"
            )

        self.in_channels = in_channels
        self.out_channels = out_channels

        # Encoder (bottom-up)
        self.conv0 = ME.MinkowskiConvolution(
            in_channels, block_channels[0],
            kernel_size=conv0_kernel_size, dimension=3,
        )
        self.bn0 = ME.MinkowskiBatchNorm(block_channels[0])
        self.relu = ME.MinkowskiReLU(inplace=True)

        self.encoder_blocks = nn.ModuleList()
        self.encoder_pools = nn.ModuleList()
        for i in range(len(block_channels) - 1):
            self.encoder_blocks.append(
                self._make_block(block_channels[i], block_channels[i + 1])
            )
            self.encoder_pools.append(
                ME.MinkowskiConvolution(
                    block_channels[i + 1], block_channels[i + 1],
                    kernel_size=2, stride=2, dimension=3,
                )
            )

        # Decoder (top-down)
        self.decoder_blocks = nn.ModuleList()
        self.decoder_ups = nn.ModuleList()
        for i in range(min(num_top_down, len(block_channels) - 1)):
            ch = block_channels[-(i + 1)]
            ch_skip = block_channels[-(i + 2)]
            self.decoder_ups.append(
                ME.MinkowskiConvolutionTranspose(
                    ch, ch_skip, kernel_size=2, stride=2, dimension=3,
                )
            )
            self.decoder_blocks.append(
                self._make_block(ch_skip * 2, ch_skip)
            )

        # Final projection
        final_ch = block_channels[0]
        self.final = ME.MinkowskiConvolution(
            final_ch, out_channels, kernel_size=1, dimension=3,
        )

    def _make_block(self, in_ch: int, out_ch: int) -> nn.Sequential:
        """Create a residual-style conv block."""
        return nn.Sequential(
            ME.MinkowskiConvolution(in_ch, out_ch, kernel_size=3, dimension=3),
            ME.MinkowskiBatchNorm(out_ch),
            ME.MinkowskiReLU(inplace=True),
            ME.MinkowskiConvolution(out_ch, out_ch, kernel_size=3, dimension=3),
            ME.MinkowskiBatchNorm(out_ch),
            ME.MinkowskiReLU(inplace=True),
        )

    def forward(self, x: Any) -> Any:
        """Forward pass.

        Args:
            x: MinkowskiEngine SparseTensor with coordinates and features.

        Returns:
            SparseTensor with per-point features of dimension out_channels.
        """
        # Initial conv
        out = self.relu(self.bn0(self.conv0(x)))

        # Encoder
        skip_connections = [out]
        for block, pool in zip(self.encoder_blocks, self.encoder_pools):
            out = block(out)
            skip_connections.append(out)
            out = pool(out)

        # Decoder
        for up, block in zip(self.decoder_ups, self.decoder_blocks):
            out = up(out)
            skip = skip_connections.pop()
            out = ME.cat(out, skip)
            out = block(out)

        out = self.final(out)
        return out


def pointcloud_to_sparse_tensor(
    points: np.ndarray,
    voxel_size: float = 0.1,
    device: torch.device | str = "cpu",
) -> Any:
    """Convert numpy point cloud to MinkowskiEngine SparseTensor.

    Args:
        points: Point cloud array, shape (N, 3) or (N, 4).
        voxel_size: Voxel size for coordinate quantization.
        device: Target device.

    Returns:
        MinkowskiEngine SparseTensor.

    Raises:
        RuntimeError: If MinkowskiEngine is not installed.
    """
    if not HAS_MINKOWSKI:
        raise RuntimeError("MinkowskiEngine is required.")

    coords = np.floor(points[:, :3] / voxel_size).astype(np.int32)
    # Use occupancy features (1.0 for each voxel)
    feats = np.ones((len(coords), 1), dtype=np.float32)

    # Remove duplicate voxels
    coords_tensor = torch.from_numpy(coords).int()
    feats_tensor = torch.from_numpy(feats).float()

    sparse_tensor = ME.SparseTensor(
        features=feats_tensor,
        coordinates=ME.utils.batched_coordinates([coords_tensor]),
        device=device,
    )
    return sparse_tensor


class PointNetSimple(nn.Module):
    """Simple PointNet-style feature extractor (no MinkowskiEngine needed).

    Lightweight alternative for development/testing without sparse conv
    dependencies.

    Args:
        in_channels: Input point dimension (3 for xyz, 4 with intensity).
        out_channels: Output feature dimension.
    """

    def __init__(self, in_channels: int = 3, out_channels: int = 256) -> None:
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_channels, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, out_channels),
        )

    def forward(self, points: torch.Tensor) -> torch.Tensor:
        """Forward pass.

        Args:
            points: (B, N, C) point cloud tensor.

        Returns:
            (B, out_channels) global descriptor.
        """
        point_features = self.mlp(points)  # (B, N, out_channels)
        global_feat = point_features.max(dim=1)[0]  # (B, out_channels)
        return global_feat
