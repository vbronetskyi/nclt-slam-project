# exp 57 - visual T&R v3 (depth cross-correlation channel - negative result)

*[thesis root](../../../../README.md) > [simulation](../../../README.md) > isaac > experiments > 57_depth_correlation*


## Goal

Add a second anchor channel (depth-based cross-correlation against the
teach occupancy map) to complement the ORB-based visual_landmark_matcher
from exp 55/56.  Hypothesis: geometry-only matching would fire where
visual matching fails (cone zones, feature-poor forest) and close the
drift gap that ORB alone cannot.

## Architecture (as implemented)

Kept everything from exp 56 (projection cap, wedge-recovery, adaptive
SLAM/encoder fallback, scene fixes).  Added one parallel node:

```
            ┌─────────────────────────────┐
/depth_points ─-> │ depth_correlation_matcher │ ─-> /anchor_correction
            └─────────────────────────────┘         (same topic as ORB)
            (teach_occ loaded at startup, dilated 1 cell)

/camera/color ─-> visual_landmark_matcher (ORB) ────-> /anchor_correction
                      (pkl with 128 landmarks)
```

`tf_wall_clock_relay_v55` accepts whichever `/anchor_correction` arrived
most recently; both nodes compete.

### how the depth correlation matcher works

1. Load teach occupancy grid from `south_teach_map.pgm`, dilate by 1
   cell (0.1 m) to widen the hit zone.
2. At 1 Hz: take the latest `/depth_points` cloud, filter to z ∈ [0.2,
   2.0] m (matches teach depth-mapper filter), range ≤ 15 m.
3. Transform points to world frame via VIO pose + static base->camera
   offset, quantise to 0.1 m cells.
4. Grid search: for every (dx, dy) in ±3 m at 0.2 m step (961 offsets),
   count how many current-scan cells land on a teach-occupied cell.
5. Accept the best offset if hits > MIN_HITS and hit-fraction >
   MIN_FRACTION and best-score / 2nd-best-score > MIN_PEAK_RATIO.
6. Publish `/anchor_correction` at `vio_pose + (dx, dy)`.

**Implementation detail:** despite the file name carrying "ICP" in the
initial version, this is **not ICP** - no iterative nearest-neighbour
association, just grid-based template matching.  File and class were
renamed to `depth_correlation_matcher` / `DepthCorrelationMatcher` for
accuracy.  See docstring in
`scripts/depth_correlation_matcher.py` for the design rationale
(template matching chosen over ICP because VIO drift up to 8 m would
trap ICP in local minima, and cones-as-new-cells are naturally ignored
by "count only hits on teach-occupied").

## Result - channel does NOT fire

After multiple treshold relaxations and a dilation tune, the
correlation matcher **failed to publish a single anchor** in the full
repeat run.

Why: in a dense forest, the teach occupancy grid and the current-frame
scan both contain ~100-200 cells each, spread over a broad tree canopy.
Cross-correlation across the ±3 m search window produces a **flat
score surface** - most offsets give 70-90 % hit fraction (every offset
lands on some tree cell), peak-ratio stays 1.00-1.05.

Concretely, from run 1 traces:

| Offset | hits | fraction |
|--------|-----:|---------:|
| best   | 150  | 0.88 |
| 2nd    | 148  | 0.87 |
| 10th   | 140  | 0.82 |

No sharp peak -> no reliable anchor.  Dilation 0 cells gave
discriminating peaks (some offsets with 5 hits, others with 50) but too
few hits overall to pass MIN_HITS.  Dilation ≥ 1 cell swamped the
plateau with uniform 80 % overlap everywhere within the search
window.  No intermediate setting produced a meaningful
signal-to-noise ratio.

## what we learned

The issue isn't the matcher implementation - it's the data density.
Our teach scene has uniformly distributed tree points at ~1 m spacing.
At the robot's scale (camera sees 5-15 m into the scene), every
positional offset within ±3 m still aligns most of the view onto
*some* tree.  Cross-correlation needs geometric uniqueness at the
matching scale.  Trees are too regular; only distinctive structures
(buildings, rocks) would give a sharp peak, and those are sparse in
our scene.

For cross-correlation to work in this environment we'd need either:

- A much narrower scan (only trunks within 3 m radius, not canopy at
  10 m) - but that gives too few points.
- A cost function that rewards *unique* spatial patterns - e.g.
  discounting by tree density map, or correlation of edge/roughness
  maps rather than raw occupancy.
- A different feature altogether - building corners, the road strip.

## Main run result (partial - Isaac died mid-run)

| Metric            | Exp 57 run 1 |
|-------------------|-------------:|
| WPs reached       | 36 / 94 |
| Distance          | 267 m (robot at GT (60, 2) past turnaround) |
| SLAM err (final)  | 5.49 m |
| Encoder err       | 3.07 m |
| ORB anchors       | 83 / 1 389 (6 %) |
| **Correlation anchors** | **0 / ~300** |
| Wedges            | 10 |

Isaac crashed ~45 min into the repeat (unclear cause - no traceback in
isaac.log tail; likely out-of-memory from long-running NVIDIA process).
Robot had made it past the turnaround (x = 60) when tf_relay/Isaac
died simultaneously - can't compare WP count 1:1 with exp 56's 83/94
since the run was shorter.

The architectural baseline (ORB anchor + adaptive fallback +
projection cap + wedge-recovery + scene fixes) from exp 56 remains
the best-performing stack: 1 390 m / 83 WPs at 14 % ORB anchor
rate.

## what to try next

Not in this experiment:

1. **Learned global descriptor per teach keyframe**
   (NetVLAD-style place descriptor, single vector per landmark).
   Robust to small viewpoint changes - might lift ORB anchor rate
   significantly.
- Active matcher prior - feed adaptive-fallback's encoder-based
   pose to the matcher's candidate search so it starts from the
   encoder-corrected position rather than drifted VIO.  Could
   reduce false-negative rejections on large-drift runs.
3. **Teach landmarks with cones** - redo teach with cones in the
   scene.  Hybridises T&R: "teach" isn't strictly obstacle-free, but
   landmarks now match repeat view content.  Violates the "clean
   teach" paradigm but probably fixes the cone-zone matcher silence.

## files

```
57_depth_correlation/
├── scripts/
│   └── depth_correlation_matcher.py   (new; produces no useful anchors in this scene)
├── teach/                              (copied verbatim from exp 56)
└── results/repeat_run/
    ├── depth_correlation_matcher.log   (logs)
    ├── depth_icp_matches.csv           (every attempt: all rejected)
    └── anchor_matches.csv              (ORB results, same as exp 56)
```
