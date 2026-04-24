# Oxford RobotCar + RobotCar Seasons pipeline

*[thesis root](../../README.md) > robotcar*

> **3.91 m ATE over 834 m** from ORB-SLAM3 Stereo on Bumblebee XB3.  plus hloc cross-season localisation at **64.5 %** (ALIKED + LightGlue + OpenIBL)

visual localization (hloc benchmark on RobotCar Seasons) and visual SLAM
(ORB-SLAM3 Stereo on raw RobotCar stereo) evaluations.

## Dataset

Oxford RobotCar is a **Nissan LEAF** driven along the same +-10 km route
thorugh central Oxford 100+ times between Nov 2014 and Nov 2015. all seasons,
weather, lighting. for this project i only use:
- Bumblebee XB3 stereo (1280x960 @ 16 Hz, 24 cm baseline) for ORB-SLAM3
- the 3 monocular Grasshopper cameras (1024x1024, undistorted to pinhole
  fx=fy=400) for the RobotCar Seasons benchmark
- the INS navigation stream as ground truth (RobotCar doesn't publish raw
  accel/gyro - only the fused INS solution)

| Sensor | Model | Parameters |
|--------|-------|------------|
| 3 monocular cams | Point Grey Grasshopper2 | 1024x1024, 11.1 Hz, fisheye 180 deg FoV, left/rear/right |
| Stereo | Point Grey Bumblebee XB3 | 1280x960, 16 Hz, 24 cm baseline, forward-facing |
| GPS/INS | NovAtel SPAN-CPT | 50 Hz, dual-antenna, GPS + GLONASS |
| 2D LiDAR | 2x SICK LMS-151 | 270 deg FoV, 50 Hz, 50 m |

### RobotCar Seasons benchmark

RobotCar Seasons uses the 3 mono cameras undistorted + cropped to pinhole.
one overcast reference drive (Nov 2014) builds a 3D map, 9 query sessions
(dawn / dusk / night / night-rain / overcast-summer / overcast-winter / rain
/ snow / sun) are localised against it

evaluation thresholds from [visuallocalization.net](https://www.visuallocalization.net/):   

| level | trans | rot |
|-------|-------|-----|
| high precision | < 0.25 m | < 2 deg |
| medium | < 0.5 m | < 5 deg |
| coarse | < 5 m | < 10 deg |

training split: 1906 images with poses, used here for offline evaluation.

## Experiments ran here

- **0.7**: hloc sweep on RobotCar Seasons (7 detector/matcher/retrieval combos).
  best = ALIKED + LightGlue + OpenIBL, **64.5%** at 0.5m/5deg. see CHANGELOG
- **0.8**: ORB-SLAM3 Stereo on the full overcast-reference drive (no IMU).
  **3.91 m ATE RMSE** over 834 m, 72.7% tracking. first working visual SLAM
  in this project

## Pipeline

```bash
# download (needs account at mrgdatashare.robots.ox.ac.uk)
python3 scripts/download_robotcar_full.py --username USER --password PASS --phase 1
# phase 1 = GPS/INS + VO (+-100 MB)
# phase 2 = stereo images (+-120 GB, careful)

# convert stereo + make GT from INS
python3 scripts/prepare_stereo_euroc.py
python3 scripts/make_ground_truth.py

# run ORB-SLAM3 Stereo
python3 scripts/evaluate_robotcar_orbslam3.py

# RobotCar Seasons hloc sweep (7 configs)
python3 scripts/run_full_benchmark.py
```

## Limitations

- **no raw IMU** - Stereo-Inertial not viable without the raw accel/gyro
  stream. a pseudo-IMU from differentiating INS was tried and failed (too
  smooth for ORB-SLAM3's VIBA init)
- 24 cm stereo baseline is a bit narrow for distant landmarks
- night queries stay hard for all methods (+-40% vs 70%+ daytime)



## Content map

- [`README.md`](README.md) - this file.  dataset overview + experiments planned/ran
- [`CHANGELOG.md`](CHANGELOG.md) - experiment log, 0.7 (hloc) + 0.8 (ORB-SLAM3 Stereo) incl. the stereo-inertial failure walkthrough
- [`configs/`](configs/) - ORB-SLAM3 yaml configs (Stereo + SI attempt)
- [`scripts/`](scripts/) - download, stereo prep, ground truth, synthesise IMU, hloc benchmark
- [`notebooks/`](notebooks/) - hloc results analysis notebook
- [`results/`](results/) - robotcar_orbslam3/ + robotcar_seasons_hloc/ full sweep

## Where to read next

- **hloc 7-config sweep numbers**: `CHANGELOG.md` experiment 0.7 table
- **why Stereo-Inertial fails on RobotCar**: `CHANGELOG.md` experiment 0.8 - explains the pseudo-IMU derivation problem
- **how 4Seasons solves the IMU gap**: [`../4seasons/README.md`](../4seasons/README.md)
- **the hloc benchmark plots**: `results/robotcar_seasons_hloc/plots/`

cross-dataset: RobotCar 3.91 m stereo-only is 4x worse than 4Seasons 0.93 m stereo-inertial - this is the IMU matters evidence

## References

- **Oxford RobotCar** - Maddern et al., 2017, IJRR - [paper](https://doi.org/10.1177/0278364916679498) - [website](https://robotcar-dataset.robots.ox.ac.uk)
- **RobotCar Seasons benchmark** - Sattler et al., 2018, CVPR - [paper](https://arxiv.org/abs/1707.09092) - [website](https://www.visuallocalization.net)
- **ORB-SLAM3** - Campos et al., 2021 - [paper](https://arxiv.org/abs/2007.11898) - [code](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **hloc** - Sarlin et al., 2019, CVPR - [code](https://github.com/cvg/Hierarchical-Localization)
- **SuperPoint / SuperGlue / LightGlue** - DeTone 2018 / Sarlin 2020 / Lindenberger 2023
- **ALIKED** - Zhao et al., 2023 - [paper](https://arxiv.org/abs/2304.03608)
- **OpenIBL / NetVLAD** - global descriptors used for image retrieval
