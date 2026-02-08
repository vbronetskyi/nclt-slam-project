# NCLT SLAM Project

LiDAR-based SLAM with learned place recognition for loop closure, targeting outdoor UGV deployment on edge hardware.

## Overview

This project implements a SLAM pipeline using the [NCLT dataset](http://robots.engin.umich.edu/nclt/) with:
- **LiDAR odometry** via ICP/GICP registration
- **Place recognition** via MinkLoc3D (sparse 3D convolutions)
- **Pose graph optimization** for globally consistent trajectories
- **Edge deployment** targeting Jetson Orin Nano / RPI5 + Hailo

## Quick Start

```bash
# Clone and install
git clone <repo-url>
cd nclt-slam-project
pip install -e .

# Download sample data
python scripts/download_nclt_sample.py --source kaggle

# Run evaluation
python scripts/evaluate_slam.py --config configs/slam_config.yaml
```

## Project Structure

```
nclt-slam-project/
├── configs/           # YAML configuration files
├── data/              # Dataset storage (git-ignored)
├── src/
│   ├── datasets/      # NCLT data loading and pair generation
│   ├── models/        # Place recognition and feature extraction
│   ├── slam/          # LiDAR odometry and pose graph
│   ├── evaluation/    # Metrics (ATE, RPE, Recall@K)
│   └── utils/         # Point cloud processing, I/O
├── scripts/           # Training and evaluation scripts
├── notebooks/         # Jupyter notebooks for Kaggle
└── tests/             # Unit tests
```

## Training on Kaggle

1. Fork the notebook `notebooks/02_kaggle_training.ipynb`
2. Upload to Kaggle and attach the [NCLT dataset](https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23)
3. Run all cells

## Dataset

Using the preprocessed NCLT dataset from Kaggle with 10 sessions spanning January–December 2012. See `data/README.md` for setup instructions.

## License

MIT
