#!/bin/bash
# Record north and south forest routes, then run ORB-SLAM3 on both.
# Usage: bash record_and_slam.sh [north|south|both]

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SLAM_DIR="/workspace/third_party/ORB_SLAM3"
BAGS_DIR="/root/bags/husky_real"
CONFIG="$BAGS_DIR/rgbd_d435i_v2.yaml"

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/isaac-sim-6.0.0/exts/isaacsim.ros2.core/jazzy/lib
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ROUTES="${1:-both}"

record_route() {
    local route=$1
    echo "============================================"
    echo "  Recording $route forest route..."
    echo "============================================"

    /opt/isaac-sim-6.0.0/python.sh "$SCRIPT_DIR/run_jackal_forest.py" \
        --route "$route" --duration 600 2>&1 | tee "/tmp/record_${route}.log"

    # Find the latest recording directory
    local rec_dir=$(ls -td "$BAGS_DIR"/isaac_slam_* | head -1)
    echo "Recording saved to: $rec_dir"
    echo "$rec_dir" > "/tmp/latest_${route}_rec.txt"
}

build_tum() {
    local route=$1
    local rec_dir=$(cat "/tmp/latest_${route}_rec.txt")
    local tum_dir="$BAGS_DIR/tum_${route}"

    echo "Building TUM dataset for $route from $rec_dir..."

    python3 << PYEOF
import os, hashlib, math
import numpy as np

SRC = "$rec_dir"
OUT = "$tum_dir"

for d in [f'{OUT}/rgb', f'{OUT}/depth']:
    os.makedirs(d, exist_ok=True)

gt = {}
for l in open(f'{SRC}/groundtruth.csv').readlines()[1:]:
    parts = l.strip().split(',')
    t = round(float(parts[0]), 4)
    gt[t] = (float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))

spawn_x, spawn_y = -95.0, -6.0
move_start = None
for t in sorted(gt.keys()):
    x, y, z, yaw = gt[t]
    if math.hypot(x - spawn_x, y - spawn_y) > 0.3:
        move_start = t; break
keep_from = (move_start - 3.0) if move_start else min(gt.keys())

src_rgb = sorted(os.listdir(f'{SRC}/camera_rgb'), key=lambda f: float(f.replace('.jpg','')))
src_dep = sorted(os.listdir(f'{SRC}/camera_depth'), key=lambda f: float(f.replace('.png','')))
rgb_by_ts = {round(float(f.replace('.jpg','')),4): f for f in src_rgb}
dep_by_ts = {round(float(f.replace('.png','')),4): f for f in src_dep}
gt_keys = np.array(sorted(gt.keys()))

CAM_FWD, CAM_UP = 0.5, 0.48
prev_hash = None
kept = 0
n_stale = 0
assoc, gt_out = [], []

for ts in sorted(rgb_by_ts.keys()):
    if ts < keep_from: continue
    if ts not in dep_by_ts: continue
    dep_path = f'{SRC}/camera_depth/{dep_by_ts[ts]}'
    h = hashlib.md5(open(dep_path,'rb').read()).hexdigest()
    if h == prev_hash: n_stale += 1; continue
    prev_hash = h
    idx = np.argmin(np.abs(gt_keys - ts))
    if abs(gt_keys[idx] - ts) > 0.1: continue
    x, y, z, yaw = gt[gt_keys[idx]]
    cx = x + CAM_FWD * math.cos(yaw)
    cy = y + CAM_FWD * math.sin(yaw)
    cz = z + CAM_UP
    qw, qz_ = math.cos(yaw/2), math.sin(yaw/2)
    ts_str = f"{ts:.4f}"
    os.symlink(os.path.abspath(f'{SRC}/camera_rgb/{rgb_by_ts[ts]}'), f'{OUT}/rgb/{ts_str}.jpg')
    os.symlink(os.path.abspath(dep_path), f'{OUT}/depth/{ts_str}.png')
    assoc.append(f"{ts_str} rgb/{ts_str}.jpg {ts_str} depth/{ts_str}.png\n")
    gt_out.append(f"{ts_str} {cx:.6f} {cy:.6f} {cz:.6f} 0.000000 0.000000 {qz_:.6f} {qw:.6f}\n")
    kept += 1

open(f'{OUT}/associations.txt','w').writelines(assoc)
open(f'{OUT}/groundtruth.txt','w').writelines(gt_out)
ts_list = [float(l.split()[0]) for l in assoc]
dts = np.diff(ts_list)
xs = [float(l.split()[1]) for l in gt_out]
ys = [float(l.split()[2]) for l in gt_out]
dist = sum(math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]) for i in range(1,len(xs)))
print(f"TUM {route}: {kept} frames, stale={n_stale}, {(kept-1)/(ts_list[-1]-ts_list[0]):.1f}Hz, dist={dist:.0f}m")
PYEOF

    echo "$tum_dir" > "/tmp/tum_${route}_dir.txt"
}

run_slam() {
    local route=$1
    local tum_dir="$BAGS_DIR/tum_${route}"

    echo "Running ORB-SLAM3 on $route..."
    cd "$SLAM_DIR"
    ./Examples/RGB-D/rgbd_tum \
        Vocabulary/ORBvoc.txt \
        "$CONFIG" \
        "$tum_dir" \
        "$tum_dir/associations.txt" 2>&1 | tee "/tmp/slam_${route}.log"

    # Copy results
    cp CameraTrajectory.txt "$tum_dir/CameraTrajectory.txt"
    cp KeyFrameTrajectory.txt "$tum_dir/KeyFrameTrajectory.txt"

    echo "Evaluating $route..."
    python3 << PYEOF
import numpy as np, math

slam = np.array([[float(p) for p in l.split()] for l in open('$tum_dir/CameraTrajectory.txt')])
gt = np.array([[float(p) for p in l.split()] for l in open('$tum_dir/groundtruth.txt')])
gt_dict = {round(r[0],4): r[1:4] for r in gt}
ms, mg = [], []
for s in slam:
    ts = round(s[0],4)
    if ts in gt_dict: ms.append(s[1:4]); mg.append(gt_dict[ts])
ms, mg = np.array(ms), np.array(mg)
sd = np.sum(np.sqrt(np.sum(np.diff(ms,axis=0)**2,axis=1)))
gd = np.sum(np.sqrt(np.sum(np.diff(mg,axis=0)**2,axis=1)))
mu_m, mu_d = ms.mean(0), mg.mean(0)
mz, dz = ms-mu_m, mg-mu_d
n = len(ms); H = mz.T@dz/n; U,D,Vt = np.linalg.svd(H)
S = np.eye(3)
if np.linalg.det(U)*np.linalg.det(Vt)<0: S[2,2]=-1
R = Vt.T@S@U.T; c = np.trace(np.diag(D)@S)*n/np.sum(mz**2)
t = mu_d - c*R@mu_m
aligned = c*(ms@R.T)+t
errors = np.sqrt(np.sum((aligned-mg)**2, axis=1))
resets = sum(1 for l in open('/tmp/slam_${route}.log') if 'Creation of new map' in l) - 2
tracked = len(slam)
total = len(gt)

print(f"\n{'='*50}")
print(f"  $route ROUTE RESULTS")
print(f"{'='*50}")
print(f"  Tracked: {tracked}/{total} ({tracked/total*100:.0f}%)")
print(f"  Map resets: {max(0, resets)}")
print(f"  Travel: SLAM={sd:.1f}m, GT={gd:.1f}m, ratio={sd/gd:.4f}")
print(f"  Scale: {c:.4f}")
print(f"  ATE RMSE: {np.sqrt(np.mean(errors**2)):.3f}m ({np.sqrt(np.mean(errors**2))/gd*100:.2f}%)")
print(f"  ATE Max: {errors.max():.3f}m")
print(f"{'='*50}")
PYEOF
}

if [ "$ROUTES" = "north" ] || [ "$ROUTES" = "both" ]; then
    record_route north
    build_tum north
    run_slam north
fi

if [ "$ROUTES" = "south" ] || [ "$ROUTES" = "both" ]; then
    record_route south
    build_tum south
    run_slam south
fi

echo ""
echo "All done!"
