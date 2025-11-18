🚀 Depth Estimation with StereoSGBM and LiDAR Calibration (ROS 2)

This ROS 2 package, depth_estimation, implements real-time depth estimation from a rectified stereo camera pair using the StereoSGBM algorithm, enhanced with Weighted Least Squares (WLS) filtering and Contrast Limited Adaptive Histogram Equalization (CLAHE).

A key feature of this package is the automatic scale calibration of the stereo baseline using ground truth depth from an accompanying LiDAR sensor (Velodyne), ensuring metric accuracy of the estimated point clouds.

📦 Package Structure

depth_estimation/
├── config/
│   └── config.yaml             # Main configuration for the depth_estimation_node
├── depth_estimation/
│   ├── depth_estimation.py     # ROS 2 Node implementing the core logic
│   └── utils.py                # Utility functions for point cloud processing and evaluation
├── launch/
│   └── depth_estimation.launch.py # Launch file to start the node with parameters
└── ... (standard ROS 2 files)

✨ Features

    Stereo Matching: Uses OpenCV's StereoSGBM (Semi-Global Block Matching) for efficient disparity computation.

    Quality Filtering: Includes WLS (Weighted Least Squares) post-filtering for disparity refinement and Left-Right Consistency (LRC) checks.

    Preprocessing: Implements CLAHE for robust feature matching, especially in low-contrast areas.

    Temporal Filtering: Applies an adaptive temporal filter to the disparity map, using a quality score derived from LRC and valid pixel fraction to stabilize depth estimates over time.

    LiDAR-Stereo Calibration: Automatically estimates and applies a scale factor to the stereo baseline (B) using matched LiDAR (ground truth) and stereo point correspondences, refined via RANSAC and Huber regression to achieve metric accuracy.

    Comprehensive Evaluation: Computes standard point cloud metrics (RMSE, Median, Chamfer Distance, Precision/Recall/F-Score) and advanced structural similarity metrics (GraphSIM, PointSSIM, Curvature Similarity) against ground truth LiDAR data.

    ROS 2 Integration: Synchronizes five inputs (left_image, right_image, left_info, right_info, velo_points) using ApproximateTimeSynchronizer.

🛠️ Implementation Details

1. Disparity and Reprojection

The core depth estimation uses the standard stereo pipeline:

    Image Preprocessing: Grayscale conversion and optional CLAHE application.

    Disparity Calculation:
    Dleft​=StereoSGBM(Left,Right)
    Dright​=RightMatcher(Right,Left)

    WLS Filtering: Refines the left disparity map using the right one:
    D=WLSFilter(Dleft​,Left Image,Dright​)

    Temporal Filtering: An adaptive filter stabilizes the disparity based on frame quality:
    α=αmax​−(αmax​−αmin​)×Quality
    Dt​=αDt−1​+(1−α)Dt​

    3D Reprojection: The final disparity map (D) is reprojected to a 3D point cloud using the Q-matrix derived from camera intrinsics (K) and stereo baseline (B):
    P3D​=cv2.reprojectImageTo3D(D,Q)

2. Scale Calibration

The DepthEstimation node is designed to run a one-time calibration step:

    It matches points from the initial stereo point cloud (pred_pts) to the ground truth LiDAR cloud (gt_pts) using nearest neighbors (NN).

    The correspondences (Zs​ for stereo depth, Zl​ for LiDAR depth) are used to estimate the scale factor a in the model Zl​=a⋅Zs​.

    This estimation is done robustly using RANSAC for outlier rejection and Huber refinement (via scipy.optimize.least_squares in utils.py).

    The calibrated scale a is then applied to the stereo baseline B in the Q-matrix calculation:
    Qnew​∝Bnew​1​,Bnew​=a⋅Bold​

    This ensures the stereo depth estimates are correctly scaled to match the metric ground truth.

3. Point Cloud Evaluation (in utils.py)

The utils.py file provides a rich set of functions for evaluating the quality of the predicted point cloud against the ground truth:
Metric Group	Function	Description
Distance	compute_nn_metrics	Calculates per-point NN distances (GT$\toPredandPred\to$GT) for precision/recall.
	chamfer_distance	Computes the average symmetric NN distance.
	precision_recall_fscore	Calculates Precision, Recall, and F-Score for various distance thresholds.
Normal-Based	graphsim	Measures the alignment of local surface normals between point clouds, mapping to a similarity score in [0,1].
Structure-Based	pointssim	An analog of SSIM for point clouds, comparing local mean, eigenvalue magnitude (contrast), and structure index.
	curvature_similarity	Compares the local surface curvature (derived from PCA eigenvalues) between matched points, providing correlation and mean absolute difference.
	pca_geometric_features	Computes per-point features like linearity, planarity, and sphericity from local PCA.

⚙️ Configuration (config/config.yaml)

The behavior of the node is governed by parameters loaded from config.yaml.
YAML

# depth_estimation_node:
#   ros__parameters:

    # --- Node Behavior ---
    sync_time: 0.05 # (s) Max time difference for synchronized messages

    # --- StereoSGBM Parameters ---
    sgbm:
      num_disparities: 128       # Must be divisible by 16
      block_size: 5             # Must be odd
      p1_multiplier: 8          # Controls smoothness
      p2_multiplier: 32         # Controls discontinuity penalty
      uniqueness_ratio: 50
    
    # --- WLS Filter Parameters ---
    wls:
      enable: true
      lambda_val: 8000.0        # Smoothness weight
      sigma_color: 1.0          # Color weight (controls edge preservation)

    # Preprocessing 
    preprocessing:
      clahe_enable: true
      clahe_clip_limit: 2.0
      clahe_tile_grid_size: [8, 8]

    # Post processing
    postprocessing:
      temporal_filter_enable: true
      temporal_alpha_min: 0.2
      temporal_alpha_max: 0.9  # Max alpha is used for low quality frames (more memory of past)
      temporal_quality_lrc_weight: 0.7
      cropping_min_z: 4.0      # Z-axis cropping (meters)
      cropping_max_z: 50.0
      downsample_enable: true
      downsample_voxel_size: 0.2 # (meters) for evaluation
    
    # --- Evaluation ---
    evaluation:
      eval_enable: true
      advance_eval_enable: false # Toggle for advanced structural metrics
      thresholds : [0.2, 0.5, 0.9] # Distance thresholds for Precision/Recall

    # --- Debugging & Logging ---
    debug:
      enable: false
      root_dir: "/depth_estimation_debug/"
      logging_period: 2.0         # (s)

🚀 Usage

Launching the Node

The package is launched using the provided launch file, which automatically loads the parameters from config/config.yaml.
Bash

ros2 launch depth_estimation depth_estimation.launch.py

Subscribed Topics

The node uses an ApproximateTimeSynchronizer to subscribe to the following topics:
Topic Name (from config.yaml)	Type	Description
topics.left_image	sensor_msgs/CompressedImage	Left camera image.
topics.right_image	sensor_msgs/CompressedImage	Right camera image.
topics.left_info	sensor_msgs/CameraInfo	Left camera intrinsics (contains K and P matrices).
topics.right_info	sensor_msgs/CameraInfo	Right camera intrinsics (contains P matrix).
topics.velo_points	sensor_msgs/PointCloud2	Ground truth LiDAR point cloud.

Published Topics

Topic Name (from config.yaml)	Type	Description
topics.disparity_image	sensor_msgs/Image	Visualized, normalized disparity map.
topics.stereo_pointcloud	sensor_msgs/PointCloud2	The final, scaled 3D point cloud from stereo estimation.
topics.gt_pointcloud	sensor_msgs/PointCloud2	The filtered and cropped ground truth LiDAR point cloud (in camera frame).

🎯 Debugging and Logging

When debug.enable is set to true in config.yaml:

    Processing time and FPS are logged every debug.logging_period seconds.

    The resulting stereo point cloud is saved as a .pcd file to a timestamped directory under debug.root_dir (e.g., /depth_estimation_debug/1678886400/).

    Standard evaluation metrics (RMSE, Chamfer, F-scores) are logged to the console and saved to eval_log.csv.

    If evaluation.advance_eval_enable is also true, advanced structural metrics (GraphSIM, PointSSIM, Curvature Similarity) are logged as well.
