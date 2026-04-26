# exp 62 - Tight Detour + Anchor Sanity

## результат

| Area | Exp 60 | Exp 62 |
|---|---|---|
| Detour ring radii | 2 / 3 / 4 / 5 m | **2 / 2.5 / 3 / 3.5 / 4 m** |
| Anchor sanity gate | - | **reject if `enc_disagree > 2 m` AND `anchor_shift < 0.5 m` AND `std < 0.15`** |
| Precise finisher (turnaround + end) | - | **added - stops pp via empty Path, DR to GT, 0.5 m tol** |
