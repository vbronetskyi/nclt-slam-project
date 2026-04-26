# Exp 43: ORB-SLAM3 + Encoder Fusion

Test ORB-SLAM3 RGBD localization fused with encoder odometry as a position
source for Nav2 navigation. Compare with encoder+compass (exp 42) baseline.

## результат

**SLAM+Encoder fusion achieves 95% completion with obstacles - matching
encoder+compass.** Proper SE(3)->SE(2) frame conversion + stale detection makes
SLAM viable.

| Localization | No obstacles | With obstacles |
|---|---|---|
| GT (exp 41) | 43/43 (100%) | 41/43 (95%) |
| Encoder+Compass (exp 42) | 43/43 (100%) | 41/43 (95%) |
| **SLAM+Encoder (this exp)** | **42/43 (98%), 402s** | **41/43 (95%), 460s** |
| SLAM+Encoder (broken v1) | n/a | 23/43 (53%) |
