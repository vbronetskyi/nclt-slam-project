# Exp 73 - Stock Nav2 NavigateThroughPoses baseline

## результат

(run_09, 2026-04-22)

| Metric | routes/09 custom | **exp 73 stock** |
|---|---|---|
| Reached / total | 36 / 36 (100 %) | **3 / 36 (8 %)** |
| Skipped | 0 | 33 (BT Aborted) |
| Duration | 689 s | **85 s** (aborted) |
| Final WP reached | x | ✗ (aborted at WP 4) |
| Failure mode | - | RPP collision ahead loop -> clear-costmap recovery -> replan -> same collision -> BT Aborted |
