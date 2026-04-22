# Navigation Experiments - Husky Visual Teach-and-Repeat

*[thesis root](../../../README.md) > [simulation](../../README.md) > isaac > experiments*


Reproducible experiment configs for Husky visual teach-and-repeat (T&R)
navigation on the south forest roundtrip (≈ 391 m, 3 cone groups +
tent on the route).

## Final ranking summary

**Exps 52..62 ranked by a composite score across six sub-metrics**
(safety, precision, efficiency, completion, speed, smoothness).

![composite ranking](results/ranking_plots.png)

| Rank | Exp | Composite (outdoor UGV) | One-line verdict |
|---|---|---|---|
| 1 | **59** wp_lookahead_detour | **0.712** | lookahead detour, best loop closure 5.95 m, smoothest |
| 2 | **60** final_approach_10cm | **0.657** | clean tent bypass (+151 cm), most efficient path 1.20× |
| 3 | 56 projection_cap_1m | 0.563 | tight TRACK but 3.55× path ratio and 44 min runtime |
| 4 | 58 route_sanitize_accum | 0.529 | 268 wedge recoveries -> 2.7 h runtime |
| 5 | 62 tight_detour_anchor_sanity | 0.449 | "88/95 REACHED" was belief; physical arrival p95 = 21.9 m |
| 6 | 61 dense_anchors_precise | 0.421 | aggressive accumulator polluted map -> drift runaway |
| 7 | 55 visual_teach_repeat | 0.326 | first T&R prototype, did not close loop |
| 8 | 53 proactive_reroute | 0.275 | proactive re-project, never finished roundtrip |
| 9 | 52 obstacles_v9 r1 | 0.181 | early prototype, stopped at 98 m |
| 10 | 52 obstacles_v9 r4 | 0.142 | same, with tent grazing |

Top 3 stable accross outdoor / research / production weightings - ranking is robust.

See [analysis/experiment_ranking/](../analysis/experiment_ranking/) for full methodology, CSVs, and per-experiment parallel-coordinates plot.

## Top 3 GT trajectories

![top 3 trajectories](results/top3_trajectories.png)

- exp 59 (orange dotted): tight detour geometry, returns closest to spawn (5.95 m)
- **exp 60** (blue dashed): cleanest tent bypass (+1.51 m body-edge clearance)
- exp 56 (green): follows teach tightly but circled the route multiple times (×3.55 path)

## Conclusions - what we learned

### 1. "REACHED count" is not a navigation success metric

Exp 62 reported 88/95 REACHED. Physical arrival p95 = 21.9 m - the
robot believed it was at the waypoint but was 22 m off. The REACH check
was using SLAM pose (belief), not GT. Result: a belief-based high
score with no physical correspondence.

**Take-away:** always evaluate T&R with physical metrics - `arrival_err`
at each REACHED event (compared to GT), not `d_to_goal` in the planner frame.

### 2. Drift is not the right physical metric either

Exp 56 had the lowest TRACK p50 (0.69 m) but the worst path ratio
(×3.55) and a 44-minute runtime for a 6-minute teach. TRACK only
measures distance to the *nearest* teach point - circling around a
waypoint keeps TRACK low while burning time and battery.

**Take-away:** TRACK p50/p95 is necessary but not sufficient. Combine
with path ratio, runtime ratio, and loop closure residual.

### 3. Obstacle clearance is a distribution, not a worst-case

Exp 60's tent +151 cm is famous, but cone clearance was -58 cm - a
body contact. Single worst-case numbers miss this. The full clearance
profile (p05, p50, near-miss count) told the complete story.

**Take-away:** report CLEARANCE p05 and contact_count, not just
`min_clearance`.

### 4. Detour ring geometry is tightly coupled to localisation quality

Exp 61 widened the detour ring 4–7 m (from 2–5 m) to "stay safe".
Effect: robot lost the teach visual corridor, matcher failed, drift
exploded to 17 m. The wider ring was intended to improve safety and
did the opposite.

**Take-away:** in visual T&R, any manoeuvre that takes the robot off
teach visuals costs localisation. Tight detours beat wide detours
unless the teach corridor is exceptionally feature-rich.

### 5. Accumulator aggression has a reinforcement failure mode

Exp 61 lowered accumulator silence 5 s -> 2 s and min-dist 5 m -> 2 m,
expecting more landmark coverage. Instead, landmarks recorded from
already-drifted poses re-matched to themselves, locking the drift in.
Three hours of runtime, 16 m mean drift.

**Take-away:** accumulate only when confident (`last_anchor std < 0.1`
and `enc_err < 1 m`). Never blindly accumulate during drift windows.

### 6. Anchor sanity gating is necessary but not sufficient

Exp 62's `reject if enc_disagree > 2 m and anchor_shift < 0.5 m and
std < 0.15` gate fired 3 times per 40-minute run - caught obvious
self-confirm bugs but didn't prevent the underlying 17 m drift. Need
a more proactive mechanism (global relocalisation fallback with
stricter PnP) - exp 63 attempted this and created new issues.

**Take-away:** sanity gates are defensive filters; the offensive fix
for kidnapped-robot drift is global relocalisation.

### 7. Non-finishers are under-reported in most papers

Exps 52/53/55 all had decent instantaneous metrics (TRACK 1-4 m,
ARRIVAL 3-4 m) at the sample time they were measured - but none
closed the roundtrip. Loop residual 60-161 m. A thesis that reports
only per-WP metrics would rank these as healthy.

**Take-away:** always gate on "did the robot return to spawn within
X m". The composite score does this via the `completion` sub-metric
and a non-finisher ×0.5 cap.

## Best configurations going forward

Two Pareto-optimal configurations - either can be the baseline for
the next experiment:

- **Exp 59 (WP look-ahead detour)** - tight detour geometry, best loop
  closure, smoothest trajectory. Weakness: body contacts with obstacles
  (tent -22 cm, cone -75 cm). Next step: tighten known-obstacle
  clearance treshold without widening detour ring.
- **Exp 60 (final-approach 10 cm spec)** - best tent bypass, cleanest
  path ratio, STOP fix works. Weakness: cone contact -58 cm (detour
  ring couldn't find wider path). Next step: reduce robot_radius +
  inflation together, allow tighter paths through cone gaps.

Hybrid (exp 59 geometry + exp 60 clearance budget) is the natural
candidate for exp 64 if/when drift-control is solidified.

## Experiments directory

```
experiments/
  results/                      - aggregated top-level plots (this README)
  README.md                     - this file
  plot_trajectory.py            - general plotting (symlink)
  52_obstacles_v9/              - first run with obstacles
  53_proactive_reroute/         - per-costmap WP re-projection
  55_visual_teach_repeat/       - first T&R with anchor matcher
  56_projection_cap_1m/         - cap projection shift at 1 m
  58_route_sanitize_accum/      - offline WP shift + continuous accumulation
  59_wp_lookahead_detour/       - runtime look-ahead + 2–5 m ring (RANK 1)
  60_final_approach_10cm/       - 10 cm clearance spec + STOP fix (RANK 2)
  61_dense_anchors_precise_endpoints/  - aggressive accum + precise finisher
  62_tight_detour_anchor_sanity/       - tight detour + anchor sanity gate
  63_global_reloc/              - global relocalisation (regression)
  ...
```

Each experiment directory contains `config/`, `scripts/`, `results/`
and a README with per-experiment details.

## Reproducing the ranking analysis

```bash
cd /workspace/simulation/isaac/analysis/experiment_ranking
python3 compute_metrics.py   # -> metrics_full.csv, composite_scores.csv
python3 make_plots.py        # -> ranking_plots.png
```

Outputs: `metrics_full.csv`, `metrics_normalized.csv`,
`composite_scores.csv`, `ranking_plots.png`, `ranking_table.md`,
`RANKING_ANALYSIS.md`.
