# 🚀 Depth Estimation with StereoSGBM and LiDAR Calibration (ROS 2)

This ROS 2 package, **`depth_estimation`**, implements real-time depth estimation from a rectified stereo camera pair using the **StereoSGBM** algorithm, enhanced with **Weighted Least Squares (WLS) filtering** and **Contrast Limited Adaptive Histogram Equalization (CLAHE)**.

A key feature of this package is the **automatic scale calibration** of the stereo baseline using ground truth depth from an accompanying **LiDAR sensor (Velodyne)**, ensuring metric accuracy of the estimated point clouds.

---

## 📦 Package Structure

depth_estimation/ ├── config/ │ └── config.yaml # Main configuration for the depth_estimation_node ├── depth_estimation/ │ ├── depth_estimation.py # ROS 2 Node implementing the core logic │ └── utils.py # Utility functions for point cloud processing and evaluation ├── launch/ │ └── depth_estimation.launch.py # Launch file to start the node with parameters └── ... (standard ROS 2 files)


---

## ✨ Features

* **Stereo Matching:** Uses OpenCV's **StereoSGBM** (Semi-Global Block Matching) for efficient disparity computation.
* **Quality Filtering:** Includes **WLS (Weighted Least Squares)** post-filtering for disparity refinement and **Left-Right Consistency (LRC)** checks.
* **Preprocessing:** Implements **CLAHE** for robust feature matching, especially in low-contrast areas.
* **Temporal Filtering:** Applies an adaptive temporal filter to the disparity map, using a quality score derived from LRC and valid pixel fraction to stabilize depth estimates over time.
* **LiDAR-Stereo Calibration:** Automatically estimates and applies a **scale factor** to the stereo baseline ($B$) using matched LiDAR (ground truth) and stereo point correspondences. The scale estimation uses **RANSAC** and **Huber regression** for robustness.
* **Comprehensive Evaluation:** Computes standard point cloud metrics (RMSE, Median, Chamfer Distance, Precision/Recall/F-Score) and advanced structural similarity metrics (**GraphSIM**, **PointSSIM**, **Curvature Similarity**) against ground truth LiDAR data.
* **ROS 2 Integration:** Synchronizes five inputs (`left_image`, `right_image`, `left_info`, `right_info`, `velo_points`) using `ApproximateTimeSynchronizer`.

---

## 🛠️ Implementation Details

### 1. Disparity and Reprojection

The core depth estimation uses the standard stereo pipeline:

1.  **Image Preprocessing:** Grayscale conversion and optional CLAHE application.
2.  **Disparity Calculation:**
    $$D_{left} = \text{StereoSGBM}(\text{Left}, \text{Right})$$
    $$D_{right} = \text{RightMatcher}(\text{Right}, \text{Left})$$
3.  **WLS Filtering:** Refines the left disparity map using the right one:
    $$D = \text{WLSFilter}(D_{left}, \text{Left Image}, D_{right})$$
4.  **Temporal Filtering:** An adaptive filter stabilizes the disparity based on frame quality:
    $$\alpha = \alpha_{\max} - (\alpha_{\max} - \alpha_{\min}) \times \text{Quality}$$
    $$D_t = \alpha D_{t-1} + (1-\alpha) D_{t}$$
5.  **3D Reprojection:** The final disparity map ($D$) is reprojected to a 3D point cloud using the **Q-matrix** derived from camera intrinsics ($K$) and the calibrated stereo baseline ($B_{\text{new}}$):
    $$P_{3D} = \text{cv2.reprojectImageTo3D}(D, Q)$$

### 2. Scale Calibration

The calibration runs once when the node starts and **`self.calibrated`** is `False`:

* It finds correspondences ($Z_s$ for stereo depth, $Z_l$ for LiDAR depth) by matching LiDAR points to the nearest stereo points.
* The scale factor $a$ is estimated using robust optimization:
    $$a = \text{estimate\_scale}(Z_s, Z_l)$$
* The calibrated scale $a$ is then applied to the stereo baseline $B$ in the Q-matrix calculation:
    $$B_{\text{new}} = a \cdot B_{\text{old}}$$
    The Q-matrix is recomputed with $B_{\text{new}}$.

### 3. Point Cloud Evaluation (in `utils.py`)

The `utils.py` file provides a rich set of functions for evaluating the quality of the predicted point cloud against the ground truth.

| Metric Group | Metric | Description |
| :--- | :--- | :--- |
| **Distance-Based** | **RMSE / Median** | Summary statistics of nearest-neighbor distances. |
| | **Chamfer Distance** | The average symmetric nearest neighbor distance. |
| | **Precision/Recall/F1** | Calculated for specified distance thresholds. |
| **Normal-Based** | **GraphSIM** | Measures the alignment of local surface normals between point clouds. |
| **Structure-Based** | **PointSSIM** | Compares local neighborhood statistics (mean, eigenvalues, structure index). |
| | **Curvature Similarity** | Compares the local surface curvature (from PCA eigenvalues) of matched points. |

---

## ⚙️ Configuration (`config/config.yaml`)

The behavior of the node is governed by parameters loaded from `config.yaml`.

# Relevant Parameters from config.yaml:

    # --- StereoSGBM Parameters ---
    sgbm:
      num_disparities: 128
      block_size: 5
      p1_multiplier: 8
      p2_multiplier: 32
    
    # --- WLS Filter Parameters ---
    wls:
      enable: true
      lambda_val: 8000.0
      sigma_color: 1.0

    # Post processing
    postprocessing:
      temporal_filter_enable: true
      cropping_min_z: 4.0      # Z-axis cropping (meters)
      cropping_max_z: 50.0
      downsample_enable: true
      downsample_voxel_size: 0.2 # (meters) for evaluation
    
    # --- Evaluation ---
    evaluation:
      eval_enable: true
      advance_eval_enable: false # Toggle for advanced structural metrics
      thresholds : [0.2, 0.5, 0.9] # Distance thresholds for Precision/Recall

🚀Usage

Launching the Node

The package is launched using the provided launch file, which automatically loads the parameters from config/config.yaml.
Bash

ros2 launch depth_estimation depth_estimation.launch.py

Subscribed Topics

The node uses an ApproximateTimeSynchronizer to subscribe to 5 topics:
Topic Name	Type	Description
topics.left_image	sensor_msgs/CompressedImage	Left camera image.
topics.right_image	sensor_msgs/CompressedImage	Right camera image.
topics.left_info	sensor_msgs/CameraInfo	Left camera intrinsics.
topics.right_info	sensor_msgs/CameraInfo	Right camera intrinsics.
topics.velo_points	sensor_msgs/PointCloud2	Ground truth LiDAR point cloud.

Published Topics

Topic Name	Type	Description
topics.disparity_image	sensor_msgs/Image	Visualized, normalized disparity map.
topics.stereo_pointcloud	sensor_msgs/PointCloud2	The final, scaled 3D point cloud from stereo estimation.
topics.gt_pointcloud	sensor_msgs/PointCloud2	The filtered and cropped ground truth LiDAR point cloud (in camera frame).
