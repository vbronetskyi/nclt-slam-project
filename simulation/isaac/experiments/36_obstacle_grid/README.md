# Exp 36: Local Obstacle Grid

Replace single-frame reactive depth detection with an ego-centric obstacle
grid that accumulates depth hits over time. Expected benefits:
1. Earlier detection (5-6m vs 2-3m from single frame)
2. Cone groups merge into continuous obstacles
3. Stable obstacle representation

## результат

| Version | DECAY | HIT_VALUE | OBSTACLE_HITS | MAX_DEPTH | SECTORS |
|---------|-------|-----------|---------------|-----------|---------|
| v1 | 0.95 | 1.0 | 3.0 | 6.0m | 40 |
| v2 | 0.92 | 1.5 | 3.0 | 4.0m | 80 |
