"""NCLT SLAM Pipeline config"""
import os

# data pathsNCLT_DATA_ROOT = '/workspace/nclt_data'
VELODYNE_DATA_DIR = os.path.join(NCLT_DATA_ROOT, 'velodyne_data')
SENSOR_DATA_DIR = os.path.join(NCLT_DATA_ROOT, 'sensor_data')
GROUND_TRUTH_DIR = os.path.join(NCLT_DATA_ROOT, 'ground_truth')
HOKUYO_DATA_DIR = os.path.join(NCLT_DATA_ROOT, 'hokuyo_data')
IMAGES_DIR = os.path.join(NCLT_DATA_ROOT, 'images')

# available sessionsSESSIONS = ['2012-01-08', '2012-04-29', '2012-08-04', '2012-10-28']
SESSIONS_WITH_IMAGES = ['2012-04-29', '2012-08-04']

# Velodyne HDL-32E specifications
VELODYNE_SCALING = 0.005  # 5mm per unit
VELODYNE_OFFSET = -100.0
VELODYNE_NUM_LASERS = 32

# Ladybug3 camera specifications
LADYBUG_NUM_CAMERAS = 6

# file patternsVELODYNE_SYNC_PATTERN = '{session}/velodyne_sync/*.bin'
GROUND_TRUTH_FILE = 'groundtruth_{session}.csv'
