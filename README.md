## 🚀 Stereo Depth Estimation with LiDAR-Based Baseline Calibration (ROS2)

This repository implements a full stereo depth estimation pipeline using **OpenCV SGBM**, enhanced with a **one-time LiDAR-based baseline calibration** step to improve stereo depth accuracy. After calibration, the system runs **fully camera-only**, with no machine learning models.

---

## 🚀 Features

- Stereo depth estimation (SGBM + WLS)
- LiDAR-assisted stereo baseline correction  
- Single calibration step → no LiDAR needed afterward  
- Temporal filtering & CLAHE preprocessing
- Left–Right consistency checking
- Real-time ROS2 execution
- 3D point cloud publishing
- Built-in evaluation metrics:
  - RMSE, Median, Chamfer distance
  - Precision / Recall / F1
  - GraphSIM, PointSSIM, PCA curvature features

---

## 📦 Project Structure

```
depth_estimation/
├── config
│   └── config.yaml
├── depth_estimation
│   ├── depth_estimation.py
│   ├── __init__.py
│   ├── __pycache__
│   │   ├── depth_estimation.cpython-310.pyc
│   │   ├── __init__.cpython-310.pyc
│   │   └── utils.cpython-310.pyc
│   └── utils.py
├── launch
│   └── depth_estimation.launch.py
├── package.xml
├── resource
│   └── depth_estimation
├── rviz
│   └── depth_results_viz.rviz
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py

```

---

## 🛠 Installation

### Dependencies

- ROS2 Humble / Foxy
- Python 3.8+
- OpenCV + contrib
- Open3D
- SciPy
- scikit-learn
- message_filters
- tf2_ros

### Install ROS dependencies:

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge \
                 ros-${ROS_DISTRO}-image-transport \
                 ros-${ROS_DISTRO}-tf2-ros \
                 ros-${ROS_DISTRO}-message-filters

```

### Python dependencies:

pip install opencv-python opencv-contrib-python
pip install open3d scipy scikit-learn

## ▶️ Running the Node

Build:

```bash
colcon build --symlink-install
```

Run:

```
source install/setup.bash
ros2 run depth_estimation depth_estimation_node
```

##📡 Subscribed Topics
```
Topic	Type	Description
/left/image/compressed	CompressedImage	Left image
/right/image/compressed	CompressedImage	Right image
/left/camera_info	CameraInfo	Left camera calibration
/right/camera_info	CameraInfo	Right camera calibration
/velodyne_points	PointCloud2	LiDAR cloud (used only for calibration)
```

## 📤 Published Topics

```
Topic	Type	Description
/camera/depth/disparity	Image	Disparity visualization
/stereo/depth_points	PointCloud2	Stereo-based point cloud
/point_cloud/ground_truth	PointCloud2	LiDAR ground truth cloud
```

## 🧠 How the Calibration Works

## The calibration process runs only once:

    Compute stereo disparity

    Reproject disparity → 3D using the initial Q matrix

    Transform LiDAR → camera frame

    Find LiDAR ↔ stereo 3D correspondences

    Solve for scale factor a using:

        RANSAC

        Huber-loss least-squares refinement

    Update stereo baseline:
    bnew=a⋅bold
    bnew​=a⋅bold​

    Recompute Q matrix

    Continue inference using only stereo

This removes long-term dependence on LiDAR and "corrects" the camera depth scale.

## 📊 Example Evaluation Log Output

Running LiDAR-based stereo scale calibration...
Estimated scale factor: 0.9973
Baseline updated: old=0.5323, new=0.5309

Eval: RMSE=0.456m
Median=0.120m
F1_scores={0.2: 0.50, 0.5: 0.78, 0.9: 0.89}
Chamfer=0.351m

## ⚙️ Parameter Configuration

All parameters can be changed via YAML or inline ROS commands:

ros2 param set /depth_estimation_node sgbm.block_size 7

Categories:
```
    sgbm: stereo matcher tuning

    wls: disparity filtering

    preprocessing: CLAHE

    postprocessing: cropping, temporal filtering

    evaluation: RMSE, F1, Chamfer

    debug: save point clouds, logs
```

## 📷 Visualization
<img width="1088" height="858" alt="rviz_screenshot_2025_11_18-23_14_19" src="https://github.com/user-attachments/assets/43d4fd59-1ce1-4d94-a2ce-8ca5c87b39e4" />
<img width="1088" height="858" alt="rviz_screenshot_2025_11_18-23_16_34" src="https://github.com/user-attachments/assets/e34f3472-c745-4532-badf-1178484c29a1" />
<img width="1088" height="858" alt="rviz_screenshot_2025_11_18-23_16_19" src="https://github.com/user-attachments/assets/b9bb55ec-f2bb-49df-8a71-2d0ef2e3c003" />
<img width="1088" height="858" alt="rviz_screenshot_2025_11_18-23_15_44" src="https://github.com/user-attachments/assets/f4d088a8-9f1c-4ec5-a8a2-9a91d2022da2" />
<img width="1088" height="858" alt="rviz_screenshot_2025_11_18-23_15_28" src="https://github.com/user-attachments/assets/176a84e9-acc6-47f3-a7f7-9d0115b4cb26" />


## 🤝 Contributions

Contributions, issues, and pull requests are welcome!

## 📄 License

MIT License 

