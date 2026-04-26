# exp 45: South Forest with Nav2 SmacPlanner2D

## результат

**47/99 REACHED / 27 SKIPPED - peak 93m (vs exp 44's 89m with NavFn).**

SmacPlanner2D accounts for robot footprint when searching the costmap,
so it refuses 1-cell non-lethal gaps thorugh inflation halos that NavFn
would pick but the 0.7m-wide robot physically can't fit thorugh.
Marginally better route progress, but same fundamental stall point ~(−5, −9)
in the NE forest cluster where local costmap halo hits 55-65% inflated
and RegulatedPurePursuit's velocity scaling collapses to near-zero.
