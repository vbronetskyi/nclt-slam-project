# Route -> (Isaac --route arg, spawn args, supervisor args) mapping.
# Sourced by baseline run scripts and the orchestrator

declare -A RP_ISAAC_ROUTE RP_SPAWN_ARGS RP_SUPERVISOR_ARGS RP_TEACH_SUBDIR

# Old-API supervisor (--turnaround-x/--past-margin), default Isaac spawn
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

# New-API supervisor (--final-x/y/--near-radius), explicit spawn per route
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

# 10-15: mid-range routes (+-150-220 m roundtrip)
# spawn_yaw aimed at waypoint 5 so the robot's depth camera is already facing
# into the route at startup (helps VIO init, keeps near-spawn obstacles in view)
RP_ISAAC_ROUTE[10_nmid_smid]=10_nmid_smid
RP_SPAWN_ARGS[10_nmid_smid]="--spawn-x -20.00 --spawn-y 30.00 --spawn-yaw 0.2600"
RP_SUPERVISOR_ARGS[10_nmid_smid]="--final-x 24.75 --final-y -31.69 --near-radius 10.0"
RP_TEACH_SUBDIR[10_nmid_smid]=10_nmid_smid/teach/teach_outputs

RP_ISAAC_ROUTE[11_nw_mid]=11_nw_mid
RP_SPAWN_ARGS[11_nw_mid]="--spawn-x -90.00 --spawn-y 35.00 --spawn-yaw 0.1349"
RP_SUPERVISOR_ARGS[11_nw_mid]="--final-x -24.32 --final-y -12.61 --near-radius 10.0"
RP_TEACH_SUBDIR[11_nw_mid]=11_nw_mid/teach/teach_outputs

RP_ISAAC_ROUTE[12_ne_mid]=12_ne_mid
RP_SPAWN_ARGS[12_ne_mid]="--spawn-x 65.00 --spawn-y 35.00 --spawn-yaw -2.5907"
RP_SUPERVISOR_ARGS[12_ne_mid]="--final-x -20.90 --final-y -1.84 --near-radius 10.0"
RP_TEACH_SUBDIR[12_ne_mid]=12_ne_mid/teach/teach_outputs

RP_ISAAC_ROUTE[13_cross_nws]=13_cross_nws
RP_SPAWN_ARGS[13_cross_nws]="--spawn-x -30.00 --spawn-y 20.00 --spawn-yaw 0.8761"
RP_SUPERVISOR_ARGS[13_cross_nws]="--final-x 27.42 --final-y -15.53 --near-radius 10.0"
RP_TEACH_SUBDIR[13_cross_nws]=13_cross_nws/teach/teach_outputs

RP_ISAAC_ROUTE[14_se_mid]=14_se_mid
RP_SPAWN_ARGS[14_se_mid]="--spawn-x 65.00 --spawn-y -35.00 --spawn-yaw 2.3562"
RP_SUPERVISOR_ARGS[14_se_mid]="--final-x -0.47 --final-y 17.48 --near-radius 10.0"
RP_TEACH_SUBDIR[14_se_mid]=14_se_mid/teach/teach_outputs

RP_ISAAC_ROUTE[15_wmid_smid]=15_wmid_smid
RP_SPAWN_ARGS[15_wmid_smid]="--spawn-x -61.50 --spawn-y 8.50 --spawn-yaw -1.1485"
RP_SUPERVISOR_ARGS[15_wmid_smid]="--final-x 25.50 --final-y -31.55 --near-radius 10.0"
RP_TEACH_SUBDIR[15_wmid_smid]=15_wmid_smid/teach/teach_outputs

ALL_ROUTES=(01_road 02_north_forest 03_south 04_nw_se 05_ne_sw 06_nw_ne 07_se_sw 08_nw_sw 09_se_ne \
            10_nmid_smid 11_nw_mid 12_ne_mid 13_cross_nws 14_se_mid 15_wmid_smid)
