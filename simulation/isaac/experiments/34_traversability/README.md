# exp 34: Traversability - South Forest Render/Collision Desync

Render-only vegetation (leaves, grass, ferns, fallen trees) blocks depth camera
but has no collision - robot drives thorugh physically but depth navigator sees 89% blocked.

## результат

| Config | blk% | Anchor | Distance | Key issue |
|---|---|---|---|---|
| Dense forest, basic depth | 89% | 10 | 20m | vegetation blocks |
| Rebuilt (no ferns) | 50-67% | 9-13 | 27m | vegetation blocks |
| Traversability filter | = raw | 9-13 | 27m | filter ineffective |
| No cover, narrow cone | 0% | 13 | 27m | cone oscillation |
| No cover, wide cone | 0% | 4-5 | 10m | odom_yaw drift |
| **No cover + GT yaw** | **0-64%** | **19** | **39m** | gap oscillation in dense zone |

Best non-GT: **39m (anchor 19)** with GT yaw + no cover + gap nav.

## проблеми

Render-only vegetation (leaves, grass, ferns, fallen trees) blocks depth camera
but has no collision - robot drives through physically but depth navigator sees 89% blocked.
