# exp 12: Revert to exp 09 config (reproducibility check)

Reproduce exp 09's 141 m result with the original (simple) waypoint logic, after exp 10-11 made things worse with aggressive skip rules.

## результат

**118 m, 71 % route, traveled 240 m total (with backups).**

| Metric | exp 12 | exp 09 (original) | exp 11 (skip) |
|---|---|---|---|
| Distance | 118 m | 141 m | +-50 m |
| Route % | 71 % | 85 % | 28 % |
| Mean lateral drift | 1.1 m | +-1 m | varies |
| Max lateral drift | 3.9 m | +-4 m | varies |
