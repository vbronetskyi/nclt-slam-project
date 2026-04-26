# exp 53 - proactive re-route + proximity speed limit

## результат

| Max cost ahead | Speed cap | Why |
|----------------|-----------|-----|
| < 30           | `MAX_VEL` (0.25 m/s) | Free |
| 30–69          | 0.15 m/s  | Near inflation edge - start slowing |
| 70–98          | 0.08 m/s  | Inside inflation gradient - crawl |
| >= 99 or `unknown` | 0.03 m/s | Near stop - contact imminent |

## проблеми

in run 2

SLAM err grew with distance even without any single dramatic break:

| Distance | SLAM err |
|----------|----------|
