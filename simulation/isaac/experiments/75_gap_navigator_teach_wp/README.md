# exp 75 - Gap Navigator driven by teach waypoints

## результат

| Component | Source | Role |
|---|---|---|
| Isaac Sim + obstacles | simulator | scene + props |
| ORB-SLAM3 RGB-D-I VIO | 3rd party | camera pose |
| `tf_wall_clock_relay_v55` | ours | publishes map->base_link from VIO+encoder |
| `visual_landmark_matcher` | ours | anchor correction against teach landmarks |
| `turnaround_supervisor` | ours | FIRE obstacle removal at 10 m from final |
