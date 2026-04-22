# exp 03: Nav2 with ORB-SLAM3 localization mode (pre-built atlas)

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 03_slam_localization*


## Goal

Improve over exp 02 (mapping mode, drift) by giving SLAM a pre-built reference map. ORB-SLAM3 localization mode loads an `.osa` atlas built offline from a clean mapping run on the same route.

## setup

- Atlas built offline: 167 MB, 519 keyframes from `tum_road` recording (3651 frames, ATE 0.49 m). Built once and reused.
- ORB-SLAM3 binary: `rgbd_live` with config that has `System.LoadAtlasFromFile: "husky_forest_atlas"`
- Same Nav2 + DWB + obstacle layer as exp 01-02
- Pose pipeline: same as exp 02 (relay reads `/tmp/slam_pose.txt`)

## result

**110 m, 60 % route, cones bypassed.** 28/1665 lost frames (1.7 %).

| Metric | Localization (exp 03) | Mapping (exp 02) |
|---|---|---|
| distance | 110 m (60 %) | 106 m (58 %) |
| Y drift | 3-5 m | 5-10 m |
| lost frames | 1.7 % | 0.1-28 % |
| stuck at | x=-23 (after tent) | x=-23 (after tent) |

Stuck at the same world location as exp 02 (x=-23, y=9) after the tent bypass.

## What we learned

1. Atlas helps with feature matching - 1.7 % lost vs up to 28 % in mapping mode. The atlas provides reference features so the live SLAM has something to localize against.
2. **Atlas does NOT fix lateral drift after maneuvers.** Once the robot leaves the mapped trajectory (obstacle bypass takes it 5+ m off the original path), it sees new views that aren't in the atlas -> SLAM falls back to feature tracking from current frame -> drift accumulates similarly to mapping mode.
3. Same failure mode at the same location. Both exp 02 and exp 03 stuck at x=-23 because the tent forces a large lateral detour.

## Conclusion

Atlas is necessary but not sufficient. The fundamental problem isn't feature matching - it's that obstacle bypass takes the robot off the mapped trajectory and into territory the atlas doesn't cover. This drove the SLAM-frame navigation idea in exp 09 (use the SAME drift on both mapping and navigation by working in SLAM coordinates).

## Files

- `config/rgbd_d435i_v2_localization.yaml` - adds `System.LoadAtlasFromFile`
- Atlas: `/root/bags/husky_real/tum_road/husky_forest_atlas.osa` (167 MB)
- `results/trajectory_plot.png`
