#!/usr/bin/env python3
"""Split teach landmarks.pkl into outbound + return subsets by x-turnaround.

Exp 66 road teach captured a roundtrip. landmarks.pkl stores 51 anchors
with camera pose [x, y, z, qx, qy, qz, qw]. The turnaround is detected as
the max-x landmark; anchors up to (and including) that index are
'outbound', the rest are 'return'.

This file is called once by the orchestrator at startup. It produces two
pkl files with the same top-level keys as the input (intrinsics +
base_to_cam_* + landmarks list), allowing visual_landmark_matcher.py
to consume either one identically.
"""
import argparse, pickle
from pathlib import Path


def split(src: Path, out_dir: Path):
    data = pickle.load(open(src, 'rb'))
    lms = data['landmarks']
    if not lms:
        raise SystemExit(f'no landmarks in {src}')

    xs = [lm['pose'][0] for lm in lms]
    i_peak = max(range(len(xs)), key=lambda i: xs[i])
    outbound = lms[:i_peak + 1]
    retrn    = lms[i_peak + 1:]

    out_dir.mkdir(parents=True, exist_ok=True)
    for name, subset in [('outbound', outbound), ('return', retrn)]:
        d = {k: v for k, v in data.items() if k != 'landmarks'}
        d['landmarks'] = subset
        path = out_dir / f'landmarks_{name}.pkl'
        with open(path, 'wb') as f:
            pickle.dump(d, f)
        print(f'{name:9} anchors: {len(subset):3d}   -> {path}')
    print(f'turnaround @ anchor idx {i_peak}   x={xs[i_peak]:+.1f} m')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, help='input landmarks.pkl from teach')
    ap.add_argument('--out-dir', required=True, help='where to write *_outbound.pkl / *_return.pkl')
    split(Path(ap.parse_args().src), Path(ap.parse_args().out_dir))
