# exp 28: Nav2+SLAM reproducibility diagnosis - recovery from exps 25-27 drift

Exps 25–27 kept adding complexity (forward-regression watchdog, lateral
corridor, current_idx snap, full-path search, SLAM stale detection,   
cmd_vel angular clamp, smaller BackUp) to protect RGBD SLAM tracking
during obstacle avoidance. Each change was individually reasonable but
the stack capped road-with-obstacle navigation at **+-29% of route**   
(vs exp 13's reported 87%).

This experiment is a diagnosis + revert cycle consolidating 6 seperate

## результат

| Attempt | Config | Max x (GT) | Distance | % | Collisions | Verdict |
|---|---|---|---|---|---|---|
| GT baseline | simple config, world wps, `--use-gt` | -50.7m | 44m | 27% | 1 (cone2) | wedged in cone inflation |
| Simple + SLAM | exp 13 params, SLAM | -45.6m | 49m | 29% | 0 | lucky south bypass, stuck |
| **Exp 13 verbatim - run 1** | exp 13 scripts/configs/waypoints | **-10.2m** | **85m** | **51%** | 0 | **new best after regression** |
| Exp 13 verbatim - run 2 | same | -10.7m | 84m | 50% | 0 | deterministic plateau |
| Exp 13 verbatim - run 3 | same | -11.3m | 84m | 50% | 0 | deterministic plateau |
