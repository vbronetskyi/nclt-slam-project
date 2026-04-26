# exp 11: SLAM frame + improved waypoint skip logic

Fix cascading waypoint skip from exp 10. Add safeguards to prevent the skip logic from accidentally finishing the route prematurely.

## результат

| variant | distance | what happened |
|---|---|---|
| 11a | 50 m | cone area, stuck |
| 11b | 80 m | false positive flooding |
| 11c | 80 m | retry loops |
| 11d | 50 m | over-aggressive skip |

None better than exp 09 (141 m). The skip logic improvements were neccessary safeguards but didn't actually improve route progress.
