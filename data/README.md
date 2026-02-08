# Data Setup

## Option 1: Kaggle (Recommended for training)

The preprocessed NCLT dataset is available on Kaggle:
https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23

When running on Kaggle, data is automatically available at:
```
/kaggle/input/nclt-iprofi-hack-23/NCLT_preprocessed/
```

## Option 2: Local Development

### Using Kaggle API
```bash
pip install kaggle
# Place your kaggle.json in ~/.kaggle/
python scripts/download_nclt_sample.py --source kaggle
```

### Manual Download
1. Go to https://www.kaggle.com/datasets/creatorofuniverses/nclt-iprofi-hack-23
2. Download and extract to `data/NCLT_preprocessed/`

## Expected Structure

```
data/
└── NCLT_preprocessed/
    ├── train.csv
    ├── val.csv
    ├── test.csv
    └── sessions/
        ├── 2012-01-08/
        │   ├── velodyne/          # .bin point cloud files
        │   ├── images_small/      # camera images
        │   └── track.csv          # ground truth poses
        ├── 2012-01-22/
        └── ...
```

## Dataset Details

- **Source**: University of Michigan North Campus Long-Term (NCLT) Dataset
- **Sessions**: 10 sessions spanning Jan 2012 - Dec 2012
- **Sensors**: Velodyne HDL-32E LiDAR, Ladybug3 camera, IMU, GPS
- **Ground truth**: Survey-grade poses from RTK GPS + IMU fusion
