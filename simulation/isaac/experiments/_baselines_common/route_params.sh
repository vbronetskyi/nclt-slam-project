# Route -> (Isaac --route arg, spawn args, supervisor args) mapping.
# Sourced by baseline run scripts and the orchestrator.

declare -A RP_ISAAC_ROUTE RP_SPAWN_ARGS RP_SUPERVISOR_ARGS RP_TEACH_SUBDIR

# Old-API supervisor (--turnaround-x/--past-margin), default Isaac spawn.
RP_ISAAC_ROUTE[01_road]=road
RP_SPAWN_ARGS[01_road]=""
RP_SUPERVISOR_ARGS[01_road]="--turnaround-x 70.0 --past-margin 2.0"
RP_TEACH_SUBDIR[01_road]=01_road/teach/teach_outputs

RP_ISAAC_ROUTE[02_north_forest]=north_forest
RP_SPAWN_ARGS[02_north_forest]=""
RP_SUPERVISOR_ARGS[02_north_forest]="--final-x 70.4 --final-y -2.3 --near-radius 10.0"
RP_TEACH_SUBDIR[02_north_forest]=02_north_forest/teach/teach_outputs

RP_ISAAC_ROUTE[03_south]=south
RP_SPAWN_ARGS[03_south]=""
RP_SUPERVISOR_ARGS[03_south]="--turnaround-x 60.0 --past-margin 2.0"
RP_TEACH_SUBDIR[03_south]=03_south/teach/teach_outputs

# New-API supervisor (--final-x/y/--near-radius), explicit spawn per route.
RP_ISAAC_ROUTE[04_nw_se]=04_nw_se
RP_SPAWN_ARGS[04_nw_se]="--spawn-x -90.00 --spawn-y 35.00 --spawn-yaw 0.0236"
RP_SUPERVISOR_ARGS[04_nw_se]="--final-x 65.0 --final-y -35.0 --near-radius 10.0"
RP_TEACH_SUBDIR[04_nw_se]=04_nw_se/teach/teach_outputs

RP_ISAAC_ROUTE[05_ne_sw]=05_ne_sw
RP_SPAWN_ARGS[05_ne_sw]="--spawn-x 65.00 --spawn-y 35.00 --spawn-yaw -2.6204"
RP_SUPERVISOR_ARGS[05_ne_sw]="--final-x -90.0 --final-y -35.0 --near-radius 10.0"
RP_TEACH_SUBDIR[05_ne_sw]=05_ne_sw/teach/teach_outputs

RP_ISAAC_ROUTE[06_nw_ne]=06_nw_ne
RP_SPAWN_ARGS[06_nw_ne]="--spawn-x -90.00 --spawn-y 35.00 --spawn-yaw 0.0236"
RP_SUPERVISOR_ARGS[06_nw_ne]="--final-x 65.0 --final-y 35.0 --near-radius 10.0"
RP_TEACH_SUBDIR[06_nw_ne]=06_nw_ne/teach/teach_outputs

RP_ISAAC_ROUTE[07_se_sw]=07_se_sw
RP_SPAWN_ARGS[07_se_sw]="--spawn-x 65.00 --spawn-y -35.00 --spawn-yaw 3.1416"
RP_SUPERVISOR_ARGS[07_se_sw]="--final-x -90.0 --final-y -35.0 --near-radius 10.0"
RP_TEACH_SUBDIR[07_se_sw]=07_se_sw/teach/teach_outputs

RP_ISAAC_ROUTE[08_nw_sw]=08_nw_sw
RP_SPAWN_ARGS[08_nw_sw]="--spawn-x -90.00 --spawn-y 35.00 --spawn-yaw -2.0921"
RP_SUPERVISOR_ARGS[08_nw_sw]="--final-x -90.0 --final-y -35.0 --near-radius 10.0"
RP_TEACH_SUBDIR[08_nw_sw]=08_nw_sw/teach/teach_outputs

RP_ISAAC_ROUTE[09_se_ne]=09_se_ne
RP_SPAWN_ARGS[09_se_ne]="--spawn-x 65.00 --spawn-y -35.00 --spawn-yaw 1.0496"
RP_SUPERVISOR_ARGS[09_se_ne]="--final-x 65.0 --final-y 35.0 --near-radius 10.0"
RP_TEACH_SUBDIR[09_se_ne]=09_se_ne/teach/teach_outputs

ALL_ROUTES=(01_road 02_north_forest 03_south 04_nw_se 05_ne_sw 06_nw_ne 07_se_sw 08_nw_sw 09_se_ne)
