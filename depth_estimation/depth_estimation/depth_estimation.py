#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import message_filters
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import cv2
import csv
from std_msgs.msg import Header
import numpy as np
import time
import open3d as o3d
import tf_transformations
import tf2_ros
import os
from depth_estimation.utils import (
    downsample_voxel,
    crop_to_camera_fov,
    evaluate_once,
    evaluate_advanced,
    get_correspondences,
    estimate_scale,
)


class DepthEstimation(Node):

    def __init__(self):
        super().__init__("depth_estimation_node")

        # --- 1. Get Default Parameter Structure ---
        default_params = self._get_default_params()

        # --- 2. Declare & Load All Parameters ---
        self.params = {}
        self._load_parameters(default_params, self.params)
        self.get_logger().info("All parameters loaded successfully.")
        self.add_on_set_parameters_callback(self.param_update_callback)

        # --- 3. Use Loaded Parameters for Initialization ---

        # Debug dir (must be set up first)
        self.debug_dir = (
            f"{os.getcwd()}{self.params['debug']['root_dir']}{int(time.time())}/"
        )
        os.makedirs(self.debug_dir, exist_ok=True)
        self.get_logger().info(f"Debug logging saving to: {self.debug_dir}")

        # Stereo matcher
        sgbm_params = self.params["sgbm"]  # Local alias for convenience
        P1 = sgbm_params["p1_multiplier"] * 3 * sgbm_params["block_size"] ** 2
        P2 = sgbm_params["p2_multiplier"] * 3 * sgbm_params["block_size"] ** 2

        self.stereo = cv2.StereoSGBM_create(
            minDisparity=sgbm_params["min_disparity"],
            numDisparities=sgbm_params["num_disparities"],
            blockSize=sgbm_params["block_size"],
            P1=P1,
            P2=P2,
            disp12MaxDiff=sgbm_params["disp12_max_diff"],
            uniquenessRatio=sgbm_params["uniqueness_ratio"],
            speckleWindowSize=sgbm_params["speckle_window_size"],
            speckleRange=sgbm_params["speckle_range"],
        )

        # Right matcher
        self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)

        # WLS filter
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(
            matcher_left=self.stereo
        )
        self.wls_filter.setLambda(self.params["wls"]["lambda_val"])
        self.wls_filter.setSigmaColor(self.params["wls"]["sigma_color"])

        # CLAHE
        if self.params["preprocessing"]["clahe_enable"]:
            pp = self.params["preprocessing"]
            self.clahe = cv2.createCLAHE(
                clipLimit=pp["clahe_clip_limit"],
                tileGridSize=tuple(pp["clahe_tile_grid_size"]),
            )

        # --- 4. Setup ROS Infrastructure ---
        topics = self.params["topics"]

        # --- Create Message Filter Subscribers ---
        # We are syncing 5 topics!
        self.left_info_sub = message_filters.Subscriber(
            self, CameraInfo, topics["left_info"]
        )
        self.right_info_sub = message_filters.Subscriber(
            self, CameraInfo, topics["right_info"]
        )
        self.left_img_sub = message_filters.Subscriber(
            self, CompressedImage, topics["left_image"]
        )
        self.right_img_sub = message_filters.Subscriber(
            self, CompressedImage, topics["right_image"]
        )
        self.velo_sub = message_filters.Subscriber(
            self, PointCloud2, topics["velo_points"]
        )

        # --- Create Synchronizer ---
        # This synchronizer waits for all 5 messages to arrive with similar timestamps
        # `slop=0.1` means messages can be up to 0.1s apart to be considered a match.
        self.time_sync = message_filters.ApproximateTimeSynchronizer(
            [
                self.left_info_sub,
                self.right_info_sub,
                self.left_img_sub,
                self.right_img_sub,
                self.velo_sub,
            ],
            queue_size=10,  # Number of message sets to buffer
            slop=self.params["sync_time"],  # Allow messages to be up to 0.1s apart
        )

        # --- Register the single, synchronized callback ---
        self.time_sync.registerCallback(self.synchronized_callback)

        # --- Publishers (remain the same) ---
        self.disp_pub = self.create_publisher(Image, topics["disparity_image"], 10)
        self.pcd_pub = self.create_publisher(
            PointCloud2, topics["stereo_pointcloud"], 1
        )
        self.gt_pcd_pub = self.create_publisher(PointCloud2, topics["gt_pointcloud"], 1)

        # TF Buffer and Listener (remains the same)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- 5. Initialize Buffers & State Variables ---
        self.bridge = CvBridge()
        self.last_time = time.time()
        self.frame_count = 0
        self.Q = None
        self.disparity_last = None
        self.T_velo_to_cam = NotImplementedError
        self.left_info = None
        self.right_info = None
        self.base_line = 0

        self.calibrated = False
        self.scale_a = 1.0
        self.scale_b = 0.0
        self.baseline_new = None

        self.get_logger().info(
            f"Depth Estimation Node initialized and waiting for synchronized messages."
        )

    def param_update_callback(self, params):
        updated = {}

        for p in params:
            # Example: p.name = "debug.enable"  ->  group="debug", key="enable"
            parts = p.name.split(".")
            ref = self.params
            for key in parts[:-1]:
                ref = ref[key]
            ref[parts[-1]] = p.value
            updated[p.name] = p.value

        # Log updates
        self.get_logger().info(f"Updated parameters: {updated}")

        return SetParametersResult(successful=True)

    def _load_parameters(self, default_config, target_config, prefix=""):
        """
        Recursively declares and loads parameters.
        """
        for key, value in default_config.items():
            full_ros_name = f"{prefix}{key}"

            if isinstance(value, dict):
                # It's a nested group, recurse
                sub_dict = {}
                target_config[key] = sub_dict
                self._load_parameters(value, sub_dict, f"{full_ros_name}.")
            else:
                # It's a final value, declare and get it
                self.declare_parameter(full_ros_name, value)
                loaded_value = self.get_parameter(full_ros_name).value
                target_config[key] = loaded_value

    def _get_default_params(self):
        """
        Returns a nested dictionary of all parameters with their default values.
        This structure mirrors the YAML file.
        """
        return {
            "sync_time": 0.05,
            "topics": {
                "left_image": "default",
                "right_image": "default",
                "left_info": "/kitti/camera_info",
                "right_info": "/kitti/camera_info",
                "velo_points": "/kitti/velo/pointcloud",
                "disparity_image": "/camera/depth/disparity",
                "stereo_pointcloud": "/stereo/depth_points",
                "gt_pointcloud": "/point_cloud/ground_truth",
            },
            "tf_frames": {"camera": "camera_color_left", "velo": "velo_link"},
            "sgbm": {
                "num_disparities": 64,
                "block_size": 7,
                "min_disparity": 0,
                "p1_multiplier": 8,
                "p2_multiplier": 32,
                "disp12_max_diff": 1,
                "uniqueness_ratio": 10,
                "speckle_window_size": 100,
                "speckle_range": 32,
            },
            "wls": {
                "enable": True,
                "lambda_val": 8000.0,  # 'lambda' is a reserved keyword in Python
                "sigma_color": 1.5,
            },
            "preprocessing": {
                "clahe_enable": True,
                "clahe_clip_limit": 2.0,
                "clahe_tile_grid_size": [8, 8],
            },
            "postprocessing": {
                "temporal_filter_enable": True,
                "temporal_alpha_min": 0.2,
                "temporal_alpha_max": 0.9,
                "temporal_quality_lrc_weight": 0.7,
                "temporal_lrc_threshold": 1.0,
                "reprojection_min_disparity": 1.0,
                "cropping_min_z": 5.0,
                "cropping_max_z": 50.0,
                "cropping_min_y": -0.3,
                "downsample_enable": True,
                "downsample_voxel_size": 0.2,
            },
            "evaluation": {
                "eval_enable": True,
                "advance_eval_enable": True,
                "gt_max_z": 50.0,
                "thresholds": [0.2],
            },
            "debug": {
                "enable": False,
                "root_dir": "/depth_estimation_debug/",
                "pcd_write_ascii": True,
                "logging_period": 2.0,
            },
        }

    # --- TF AND CALLBACKS (Unchanged) ---
    def lookup_tf_velo_to_cam(self):
        try:
            frames = self.params["tf_frames"]
            trans = self.tf_buffer.lookup_transform(
                frames["camera"], frames["velo"], rclpy.time.Time()
            )
            t = trans.transform.translation
            q = trans.transform.rotation
            T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            T[0:3, 3] = [t.x, t.y, t.z]
            self.T_velo_to_cam = T
            return True
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return False

    def publish_pointcloud(self, publisher, output_points, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.params["tf_frames"]["camera"]
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc2_msg = point_cloud2.create_cloud(header, fields, output_points)
        publisher.publish(pc2_msg)
        
    # --- compute_Q ---
    def compute_Q(self):
        if self.Q is not None:
            return True

        P_left = np.array(self.left_info.p).reshape(3, 4)
        P_right = np.array(self.right_info.p).reshape(3, 4)

        fx = P_left[0, 0]
        cx_left, cy_left = P_left[0, 2], P_left[1, 2]
        cx_right = P_right[0, 2]

        baseline = -(P_right[0, 3] - P_left[0, 3]) / fx

        self.fx = fx
        self.cx_left = cx_left
        self.cy_left = cy_left
        self.cx_right = cx_right
        self.base_line = baseline
        
        # Build original Q matrix
        self.Q = np.array(
            [
                [1, 0, 0, -cx_left],
                [0, 1, 0, -cy_left],
                [0, 0, 0, fx],
                [0, 0, -1.0/baseline, (cx_left - cx_right)/baseline]
            ],
            dtype=np.float64,
        )
        
        return True

    def apply_scale_to_Q(self, scale):
        self.scale_a = scale
        self.baseline_new = self.base_line * scale

        self.get_logger().info(
            f"Baseline updated: old={self.base_line:.4f}, new={self.baseline_new:.4f}"
        )

        # Recompute Q cleanly
        self.Q = np.array(
            [
                [1, 0, 0, -self.cx_left],
                [0, 1, 0, -self.cy_left],  # cy_left is 5th param of P
                [0, 0, 0, self.fx],
                [
                    0,
                    0,
                    -1.0 / self.baseline_new,
                    (self.cx_left - self.cx_right) / self.baseline_new,
                ],
            ],
            dtype=np.float64,
        )

    def synchronized_callback(
        self, left_info_msg, right_info_msg, left_img_msg, right_img_msg, velo_msg
    ):

        # Store the info messages (we need them to compute Q once)
        self.left_info = left_info_msg
        self.right_info = right_info_msg

        # current time stamp
        stamp = left_img_msg.header.stamp

        # Get parameter aliases for cleaner code
        pp = self.params["preprocessing"]  # Pre-processing
        postp = self.params["postprocessing"]  # Post-processing

        # compute_Q still checks if self.left_info is set
        if not self.compute_Q():
            self.get_logger().warn("Waiting for CameraInfo to compute Q matrix...")
            return

        start = time.time()

        # --- DECODE/CONVERT MESSAGES ---
        # Convert images from CompressedImage to cv2 format
        self.left_img = self.bridge.compressed_imgmsg_to_cv2(
            left_img_msg, desired_encoding="bgr8"
        )
        self.right_img = self.bridge.compressed_imgmsg_to_cv2(
            right_img_msg, desired_encoding="bgr8"
        )

        # Convert PointCloud2 to NumPy array
        self.velo_points = point_cloud2.read_points_numpy(
            velo_msg, field_names=("x", "y", "z"), skip_nans=True
        )

        # --- Preprocessing ---
        left_gray = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(self.right_img, cv2.COLOR_BGR2GRAY)
        gt_points = self.velo_points

        if pp["clahe_enable"]:
            left = self.clahe.apply(left_gray)
            right = self.clahe.apply(right_gray)
        else:
            left = left_gray
            right = right_gray

        # --- Disparity Computation ---
        disp_left = self.stereo.compute(left, right).astype(np.float32) / 16.0
        disp_right = self.right_matcher.compute(right, left).astype(np.float32) / 16.0

        if self.params["wls"]["enable"]:
            disparity = self.wls_filter.filter(
                disp_left, self.left_img, None, disp_right
            )
        else:
            disparity = disp_left

        # --- Temporal Filtering & Quality ---
        h, w = disp_left.shape
        coords_x = np.arange(w)
        coords_x = np.tile(coords_x, (h, 1))
        xr = coords_x - disp_left.astype(np.int32)
        xr = np.clip(xr, 0, w - 1)

        disp_right_shifted = np.take_along_axis(disp_right, xr, axis=1)
        lrc_mask = (
            np.abs(disp_left + disp_right_shifted) < postp["temporal_lrc_threshold"]
        ).astype(np.uint8)
        lrc_score = np.mean(lrc_mask)

        valid_mask = disparity > 0
        valid_fraction = np.mean(valid_mask)

        quality = (
            postp["temporal_quality_lrc_weight"] * lrc_score
            + (1.0 - postp["temporal_quality_lrc_weight"]) * valid_fraction
        )
        quality = np.clip(quality, 0.0, 1.0)

        if postp["temporal_filter_enable"]:
            if self.disparity_last is None:
                self.disparity_last = disparity
            else:
                alpha = (
                    postp["temporal_alpha_max"]
                    - (postp["temporal_alpha_max"] - postp["temporal_alpha_min"])
                    * quality
                )
                disparity = alpha * self.disparity_last + (1 - alpha) * disparity
            self.disparity_last = disparity

        disparity = np.nan_to_num(disparity, nan=0.0, posinf=0.0, neginf=0.0)
        # disparity *= 0.5

        # --- Visualization Publisher ---
        mask_valid = disparity > 0
        disp_vis = np.zeros_like(disparity, dtype=np.uint8)
        if np.any(mask_valid):
            valid_disp = disparity[mask_valid]
            norm_disp = (
                (valid_disp - valid_disp.min()) / (valid_disp.ptp() + 1e-6) * 255
            ).astype(np.uint8)
            disp_vis[mask_valid] = norm_disp
        self.disp_pub.publish(self.bridge.cv2_to_imgmsg(disp_vis, encoding="mono8"))

        # --- Reprojection & Cropping ---
        points_3D = cv2.reprojectImageTo3D(disparity, self.Q)

        mask = disparity > postp["reprojection_min_disparity"]
        output_points = points_3D[mask]
        output_points[..., 0:3] *= -1  # Flip all axes

        cropped_output_points = output_points[
            (output_points[:, 2] > postp["cropping_min_z"])
            & (output_points[:, 2] < postp["cropping_max_z"])
            & (output_points[:, 1] > postp["cropping_min_y"])
        ]

        # --- Evaluation & PointCloud Publishing ---
        if not self.lookup_tf_velo_to_cam():
            self.get_logger().warn("Skipping evaluation — TF not available yet.")
        else:
            try:
                # Get the 3x4 RECTIFIED projection matrix
                P_left = np.array(self.left_info.p).reshape(3, 4)

                # Extract the 3x3 RECTIFIED intrinsic matrix (K_rect) from P
                K = P_left[0:3, 0:3]
                image_shape = (self.left_img.shape[0], self.left_img.shape[1])

                ones = np.ones((gt_points.shape[0], 1))
                pts_h = np.hstack([gt_points, ones])
                pts_cam = (self.T_velo_to_cam @ pts_h.T).T[:, :3]

                gt_cam_inview = crop_to_camera_fov(pts_cam, K, image_shape)
                gt_cam_inview = gt_cam_inview[
                    (gt_cam_inview[:, 2] > postp["cropping_min_z"])
                    & (gt_cam_inview[:, 2] < postp["cropping_max_z"])
                    & (gt_cam_inview[:, 1] > postp["cropping_min_y"])
                ]

                if postp["downsample_enable"]:
                    pred_ds = downsample_voxel(
                        cropped_output_points, postp["downsample_voxel_size"]
                    )
                    gt_cam_inview_ds = downsample_voxel(
                        gt_cam_inview, postp["downsample_voxel_size"]
                    )
                else:
                    pred_ds = cropped_output_points
                    gt_cam_inview_ds = gt_cam_inview

                if not self.calibrated:
                    self.get_logger().info(
                        "Running LiDAR-based stereo scale calibration..."
                    )

                    Zs, Zl = get_correspondences(pred_ds, gt_cam_inview_ds)

                    if len(Zs) < 20:
                        self.get_logger().warn(
                            "Not enough correspondences for calibration."
                        )
                        return

                    a = estimate_scale(Zs, Zl)
                    self.get_logger().info(f"Estimated scale factor: {a:.4f}")

                    self.apply_scale_to_Q(a)
                    self.calibrated = True
                    return

                self.publish_pointcloud(self.pcd_pub, pred_ds, stamp)
                self.publish_pointcloud(self.gt_pcd_pub, gt_cam_inview_ds, stamp)

                if self.params["evaluation"][
                    "eval_enable"
                ]:  # Only run eval if debug is enabled
                    results = evaluate_once(
                        pred_ds,
                        gt_cam_inview_ds,
                        thresholds=self.params["evaluation"]["thresholds"],
                    )

                    if "error" not in results:
                        gt_stats = results["nn_stats_gt2pred"]
                        f1_scores = results["precision_recall"]  # your dict
                        f1_only = {
                            thr: vals["fscore"] for thr, vals in f1_scores.items()
                        }
                        chamfer = results["chamfer"]

                        self.get_logger().info(
                            f"Eval: RMSE={gt_stats['rmse']:.3f}m, "
                            f"Median={gt_stats['median']:.3f}m, "
                            f"F1_scores={f1_only}, "
                            f"Chamfer={chamfer:.3f}m, "
                        )

                        log_path = os.path.join(self.debug_dir, "eval_log.csv")
                        with open(log_path, "a", newline="") as f:
                            writer = csv.writer(f)
                            writer.writerow(
                                [
                                    time.time(),
                                    f"Eval: RMSE={gt_stats['rmse']:.3f}m, "
                                    f"Median={gt_stats['median']:.3f}m, "
                                    f"F1_scores={f1_only}, "
                                    f"Chamfer={chamfer:.3f}m, ",
                                ]
                            )
                if self.params["evaluation"][
                    "advance_eval_enable"
                ]:  # Only run eval if debug is enabled
                    res = evaluate_advanced(pred_ds, gt_cam_inview, k=32)

                    self.get_logger().info(
                        f"graphsim={res['graphsim']}, "
                        f"pointssim={res['pointssim']}, "
                        f"pca_feats_gt={res['pca_feats_gt']}, "
                        f"pca_feats_pred={res['curvature_similarity']}, "
                    )

                    log_path = os.path.join(self.debug_dir, "eval_log.csv")
                    with open(log_path, "a", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerow(
                            [
                                time.time(),
                                f"graphsim={res['graphsim']}, "
                                f"pointssim={res['pointssim']}, "
                                f"pca_feats_gt={res['pca_feats_gt']}, "
                                f"pca_feats_pred={res['curvature_similarity']}, ",
                            ]
                        )
            except Exception as e:
                self.get_logger().warn(f"Evaluation error: {e}")

        # --- Logging and Debug Saving ---
        proc_time = (time.time() - start) * 1000
        self.frame_count += 1
        now = time.time()

        if now - self.last_time >= self.params["debug"]["logging_period"]:
            fps = self.frame_count / (now - self.last_time)
            mean_disp = (
                np.mean(disparity[disparity > 0]) if np.any(disparity > 0) else 0.0
            )
            self.get_logger().info(
                f"FPS: {fps:.1f}, Mean Disp: {mean_disp:.2f}, Proc Time: {proc_time:.1f} ms"
            )

            if self.params["debug"]["enable"]:
                colors = cv2.cvtColor(self.left_img, cv2.COLOR_BGR2RGB)
                output_colors = colors[mask]
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(output_points)
                pcd.colors = o3d.utility.Vector3dVector(output_colors / 255.0)

                pcd_filename = os.path.join(
                    self.debug_dir, f"stereo_pointcloud_{int(time.time())}.pcd"
                )
                o3d.io.write_point_cloud(
                    pcd_filename,
                    pcd,
                    write_ascii=self.params["debug"]["pcd_write_ascii"],
                )
                self.get_logger().info(f"Saved point cloud → {pcd_filename}")

            self.last_time = now
            self.frame_count = 0

        self.left_img = None
        self.right_img = None


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("StereoSGBM node shutting down.")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
