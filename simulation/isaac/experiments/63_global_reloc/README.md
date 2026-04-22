# Exp 63 - Global Relocalisation Fallback

Target the drift root cause identified accross exps 60-62:

- Once VIO drifts > 20 m, the matcher's local pose-filter search
  (`CANDIDATE_RADIUS_M = 8 m`) returns zero candidates -> no anchor ->
  drift grows forever. Exp 62 ran for 40 min with 6-23 min anchor
  droughts; mean drift was 16.9 m.
- This is a kidnapped-robot failure: the robot has perfectly good
  camera images of recognisable teach landmarks nearby, but the
  matcher is told to only look at landmarks near its (incorrect)
  position estimate.

Fix: when the matcher is silent for `RELOC_AGE_S = 20 s`, fall back to a
global search - drop the pose filter, rank all teach landmarks by
descriptor similarity, try the top 25 with strict PnP thresholds.

## Changes vs exp 62

| Area | Exp 62 | Exp 63 |
|---|---|---|
| Global relocalisation fallback | - | **added** |
| Relocalisation trigger | - | matcher silent ≥ 20 s |
| Relocalisation candidate pool | - | all landmarks with heading within 90° |
| Relocalisation strict PnP | - | MIN_INLIERS 10->18, REPROJ 2.0->1.5 px |
| Consistency check | reject if shift > 5 m | **skipped when relocating** |
| `tf_relay` jump regime | - | **hard-set nav to anchor if `std < 0.08` and `shift > 3 m`** |

Everything else carried over from exp 62:
- Tight detour ring 2–4 m
- Anchor sanity gate (small-shift-large-disagreement rejection)
- Precise finisher for turnaround + end (empty-Path + DR to GT ≤ 0.5 m)
- 10 cm clearance spec, Nav2 `robot_radius=0.6`, `inflation_radius=1.0`

## Global relocalisation flow

```
every matcher tick:
  cand = landmarks within 8 m of VIO AND heading ≤ 90°
  if cand is empty AND (now - last_anchor_ts) > 20 s:
      relocating = True
      cand = all landmarks with heading ≤ 90°, sorted by descriptor match count
      cand = cand[:25]
  for li in cand:
      PnP on landmark's 3D points vs current 2D keypoints
      if relocating:  inliers ≥ 18, reproj < 1.5 px
      else:           inliers ≥ 10, reproj < 2.0 px
      keep best
  if best:
      consistency check (skipped when relocating)
      publish PoseWithCovariance
```

## Jump regime in `tf_wall_clock_relay_v55.py`

Before exp 63, the strongest anchor only got 40 % weight in the fused
pose (`nav_x = 0.40·anchor + 0.55·slam + 0.05·enc`). For a kidnapped-
robot correction that needs to move nav by 20 m, 40 % is useless.

New regime: when the incoming anchor has `std ≤ 0.08` AND is ≥ 3 m
away from current SLAM belief, `nav_x` is hard-set to the anchor and
the encoder baseline is re-zeroed. SLAM's internal state is not
reset - it will keep tracking from its drifted origin - but the tf
consumers (Nav2, pp_follower) immediately see the corrected pose,
and the next anchor will land close so strong/ok regimes take over.

## How to run

```bash
bash scripts/run_exp63_repeat.sh
# Results -> results/repeat_run/
```

## Success criteria (vs exp 62)

1. `GLOBAL-RELOC jump` events fire at least once during the no-anchor
   stretches past the tent.
2. `JUMP applied` events visible in `tf_slam.log`, aligning nav back
   to GT within 3 m.
3. Mean drift ≤ 3 m (exp 60 baseline) - requires the jump regime
   actually recovering.
4. PRECISE TURNAROUND succeeds (robot arrives within 0.5 m of
   teach GT turnaround). This was the hard failure in exp 62.
5. PRECISE END succeeds (already worked in exp 62 at 28 cm).
