# Exp 55 - visual teach-and-repeat landmark matching for drift-free VIO

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 55_visual_teach_repeat*


## Problem (from exp 53 analysis)

Exp 53 run 2 reached 37/94 WPs, 539 m before SLAM drift ran away (32 m err
by 400 m) in feature-sparse forest. Cause: only 2 VIBA events total ->
IMU bias walked freely over 5 400 s -> pure yaw-bias signature in position
drift.

Exp 54 experiments with adaptive SLAM/encoder blend made things worse
(wedge bounces near cones corrupt encoder dead-reckoning too).

**The methodologically correct fix for GPS-denied outdoor teach-and-repeat
is visual landmark anchoring against the teach map** - the classical
Oxford MRG / Clearpath VT&R pattern. The teach run lays down a sparse
map of visual landmarks (ORB features with 3D positions). The repeat run
continuously matches current frames against nearby teach landmarks,
PnP-solves for the current camera pose in the teach-map frame, and
injects this pose as a correction to the VIO estimate.

No GPS. No LiDAR. No ORB-SLAM3 binary changes (keeps `loopClosing: 0`
intact). No direct GT access anywhere in the pipeline.

## Architecture

```
TEACH run:
  Isaac Sim  ->  /camera/color, /camera/depth, GT pose
                                   │
                                   ▼
      visual_landmark_recorder.py  (every ~2 m of displacement)
         - ORB features, back-projected via depth
         - stores (pose, descriptors, 3D keypoints) per landmark
         - pickles south_landmarks.pkl at shutdown

REPEAT run:
  Isaac Sim  ->  /camera/color, /camera/depth, VIO pose
                                   │
                                   ▼
      visual_landmark_matcher.py (1-2 Hz)
         - loads south_landmarks.pkl
         - candidate teach landmarks within 8 m of current VIO pose
         - BFMatcher + Lowe ratio + PnP-RANSAC (reproj < 2 px, ≥ 30 inliers)
         - consistency check (≤ 5 m from current VIO)
         - publishes /anchor_correction  (PoseWithCovarianceStamped)
                                   │
                                   ▼
      tf_wall_clock_relay_v55.py
         - baseline: 95 % VIO + 5 % encoder  (no_anchor regime)
         - ok     :  75 % VIO + 20 % anchor + 5 % encoder
         - strong : 55 % VIO + 40 % anchor + 5 % encoder
         - hysteresis: 2 consecutive strong anchors needed before switching
         - drops to baseline on any rejection or staleness > 3 s
```

## Files added vs exp 53

```
scripts/visual_landmark_recorder.py     # teach-time, ~220 lines
scripts/visual_landmark_matcher.py      # repeat-time, ~260 lines
scripts/tf_wall_clock_relay_v55.py      # base relay + /anchor_correction subscriber + regime blend
scripts/checkpoint_a_selftest.py        # offline self-match validator
scripts/run_exp55_teach.sh
scripts/run_exp55_repeat.sh
teach/south_landmarks.pkl               # written by teach run
teach/south_landmarks_debug.png         # landmark density heatmap
```

All other scripts (PP follower with v53 plan-based proximity + proactive
projection, costmap_snapshotter, plan_logger, turnaround_supervisor, Nav2
configs) are byte-identical to exp 53.

## Camera & feature params

| Param | Value |
|-------|-------|
| Intrinsics | fx=fy=320, cx=320, cy=240, 640×480 |
| Teach sampling distance | 2.0 m of VIO displacement |
| ORB features | 500 per frame |
| Depth filter | 0.5–5.0 m + local 3×3 std < 0.1 m |
| Candidate search radius | 8 m (tolerates up to ~5 m VIO drift) |
| Matches per candidate | Lowe ratio 0.75, ≥ 30 good |
| PnP-RANSAC | 3 px reproj threshold, 200 iter |
| Acceptance | ≥ 30 inliers AND mean reproj < 2 px |
| Consistency | anchor within 5 m of current VIO |

## hard prohibitions (methodological rules)

- No direct GT access in matcher, recorder, or tf_relay's anchor logic.
  Only ROS topics + landmark pickle + VIO pose file.
- No ORB-SLAM3 binary modifications. `loopClosing: 0` stays.
- No `/tmp/gazebo_models.json` reads. No scene introspection for
  localisation purposes.

## Staged validation protocol

### checkpoint A - offline self-match (must pass before any repeat run)

```bash
python3 scripts/checkpoint_a_selftest.py \
    --landmarks teach/south_landmarks.pkl \
    --bag-dir /root/bags/husky_real/isaac_slam_<teach-ts>/ \
    --out-dir results/checkpoint_a_selftest/
```

Matches teach-run RGB frames against teach-run landmarks. Expected:
≥ 90 % of sampled ticks produce near-identity anchor (offset < 0.3 m).
If this fails, the matcher has a bug - fix before running repeat.

### Checkpoint B - short repeat run (first 200 m)

Launch a repeat run, stop manually near x ≈ -50. Inspect:

- match rate in `anchor_matches.csv` (target ≥ 30 %)
- any consistency rejections (should be < 20 % of accepted PnP)
- `tf_slam.log` `regime=` field - should switch between no_anchor / ok /
  strong at least a few times

### Checkpoint C - full repeat run

Only after A and B clean. Full 770 m roundtrip + 4 obstacles.

## success criteria

- Teach: 300–500 landmarks, mean 100+ valid 3D keypoints each
- Checkpoint A: ≥ 90 % near-identity
- Checkpoint B (first 200 m): ≥ 30 % match rate, at least one `strong`
  regime transition
- Full repeat run:
  - Full 770 m roundtrip reached
  - SLAM err vs distance:
    - 100 m: < 1 m (exp 53 baseline was 0.81 m - match or slightly better)
    - 300 m: < 3 m (exp 53 was ~16 m -> target 5× better)
    - 770 m: < 5 m (exp 53 never reached)
  - All 4 obstacles cleared without contact
  - ≥ 70 / 94 WPs reached
  - No false-match-induced navigation failure (consistency check works)

## reproducing

```bash
# 1. Teach run (once)
bash scripts/run_exp55_teach.sh

# 2. Checkpoint A
python3 scripts/checkpoint_a_selftest.py \
    --landmarks teach/south_landmarks.pkl \
    --bag-dir $(ls -dt /root/bags/husky_real/isaac_slam_* | head -1) \
    --out-dir results/checkpoint_a_selftest/

# 3. Short repeat run (Checkpoint B)
REPEAT_OUT_DIR=results/checkpoint_b_200m \
bash scripts/run_exp55_repeat.sh
# - stop manually when robot reaches x ≈ -50

# 4. Full repeat run (Checkpoint C)
REPEAT_OUT_DIR=results/repeat_run \
bash scripts/run_exp55_repeat.sh
```

## Fallbacks if things go sideways

- Match rate near 0 %: relax Lowe ratio to 0.85, increase ORB features
  to 1000. If still bad - check per-region match rate to localise the
  problem (open forest vs house-near vs deep forest).
- High false-match rate: tighten RANSAC to 2 px, raise MIN_INLIERS to 40.
- Regime oscillation: bump `ANCHOR_HYSTERESIS_N` from 2 -> 5.
- Scale mismatch between teach and repeat VIO: estimate global scale
  from first 20 accepted matches, apply to subsequent PnP outputs.
- Matcher too slow: drop to 0.5 Hz or 300 features. Keep BFMatcher for
  debuggability.

## what this proves (if it works)

A complete GPS-free, LiDAR-free outdoor UGV navigation stack: ORB-SLAM3
VIO for short-term motion, visual teach-and-repeat anchoring for
drift-free long-horizon pose, depth camera + obstacle_layer for reactive
avoidance, Nav2 planner + custom PP follower with proximity limiter and
proactive WP projection. Full 770 m roundtrip with 4 obstacles on an
unmodified ORB-SLAM3 binary. Methodologically defensible - only RGB-D +
IMU + encoder.

## Results - run 3 (scene-fixed, 0.8 m/s, v54 fallback + wedge-recovery)

| Metric                   | Value |
|--------------------------|-------|
| Distance driven          | **477 m** |
| Reached max x            | 83 (past turnaround x=60) |
| WPs reached              | 19 / 94 |
| WPs timeout              | 4 |
| WPs skipped (plan failed)| 20 |
| Wedge recoveries         | 23 |
| Anchor match rate        | 13.1 % (153 / 1164) |
| SLAM err mean / med / max| 4.98 / 3.79 / 18.69 m |
| Encoder err max          | 9.29 m |
| Final drift              | 9.09 m at 482 m |

![trajectory](results/run3_success_outbound_drift_return/trajectory_map.png)
![err vs distance + regime timeline](results/run3_success_outbound_drift_return/err_and_regime.png)

### Highlights

- **Reached turnaround** (GT x = 82.8, past x=60). Outbound complete.
- Cone groups 0, 1, 2 and the tent were all traversed without the robot
  getting stuck permanently (23 wedge-recoveries resolved contacts).
- Scene fixes removed "floating shrubs" bug -> visuals match terrain.
- v54 adaptive SLAM+encoder fallback kicked in when SLAM lost tracking
  (encoder-only mode at end, drift capped at ~9 m).
- Skip-on-plan-fail (5 consecutive failures) prevented 20 deadlocks.

### What didn't work

- Anchor match rate only 13 % overall. Near spawn: 96 %. In cone/tent
  zones and deep forest (rows with d_median > 15 m): near 0 %.
- With no anchor for 27 min on outbound, drift grew to 18 m before SLAM
  lost tracking (GT x=82). After SLAM loss, adaptive fallback switched to
  encoder, total error stayed ~9 m for return.
- Robot did not complete return to spawn - stopped near turnaround.
- Plans sometimes used 2000+ poses for 20 m targets - obstacle_layer
  growing with cumulative drift; plans routed very long detours.

### root issues for next iteration (-> exp 56)

1. Matcher silent in cone zones because teach frames don't include
   cones - different foreground objects. Possible fix: train matcher
   to match predominantly GROUND features (below horizon).
2. Projection moves WPs aggressively -> robot takes different path than
   teach -> camera view diverges -> fewer matches. Less-aggressive
   projection should keep route closer to teach path.
3. No "relocalisation when lost" - once SLAM dies (mode=ENC), the only
   recovery is encoder. If matcher fired there, could relocalise SLAM.
   Currently anchor only influences the fused nav, not SLAM's internal
   state.
