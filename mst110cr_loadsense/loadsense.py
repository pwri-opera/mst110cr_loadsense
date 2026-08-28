#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ROS convention: fixed-axis roll(X), pitch(Y), yaw(Z)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array(
        [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
        dtype=np.float64,
    )
    ry = np.array(
        [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
        dtype=np.float64,
    )
    rz = np.array(
        [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return rz @ ry @ rx


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return quaternion as [x, y, z, w]."""
    m = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(m))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 0.0)) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(max(1.0 + m[1, 1] - m[0, 0] - m[2, 2], 0.0)) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(max(1.0 + m[2, 2] - m[0, 0] - m[1, 1], 0.0)) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s

    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("Failed to convert rotation matrix to a quaternion.")
    return quaternion / norm


class GicpTfNode(Node):
    """
    Register a source PointCloud2 to a target template and broadcast TF.

    TF semantics:
      header.frame_id = target_frame_id  (parent)
      child_frame_id  = source_frame_id  (child)

      p_target = T_target_source @ p_source
    """

    def __init__(self) -> None:
        super().__init__("dump_bed_calibrator")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self._processing_lock = threading.Lock()
        self._success_count = 0
        self._latest_point_cloud: Optional[PointCloud2] = None
        self._warned_robust_kernel_fallback = False

        self._initial_transform = self._make_initial_transform()
        self._last_transform = self._initial_transform.copy()

        self._template_raw = self._load_template(self.template_path)
        self._template_coarse = self._preprocess(
            self._template_raw,
            voxel_size=self.coarse_voxel_size,
            remove_outliers=False,
        )
        self._template_fine = self._preprocess(
            self._template_raw,
            voxel_size=self.fine_voxel_size,
            remove_outliers=False,
        )
        self._template_xyz = np.asarray(
            self._template_raw.points, dtype=np.float32
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.qos_depth,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if self.qos_reliability == "reliable"
                else ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self._template_publisher = self.create_publisher(
            PointCloud2,
            self.template_points_topic,
            qos,
        )
        self._template_timer = self.create_timer(
            1.0 / self.template_publish_hz,
            self._publish_template_points,
        )
        self._transform_timer = self.create_timer(
            1.0 / self.transform_publish_hz,
            self._broadcast_last_transform,
        )
        self._calibrate_service = self.create_service(
            Trigger,
            "calibrate",
            self._calibrate_callback,
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._cache_point_cloud,
            qos,
        )

        self.get_logger().info(
            "GICP TF node started: "
            f"topic={self.input_topic}, "
            f"template={self.template_path}, "
            f"template_topic={self.template_points_topic} "
            f"({self.template_publish_hz:g} Hz), "
            f"TF={self.target_frame_id} -> {self.source_frame_id}, "
            f"TF_publish_rate={self.transform_publish_hz:g} Hz, "
            f"template_points(raw/coarse/fine)="
            f"{len(self._template_raw.points)}/"
            f"{len(self._template_coarse.points)}/"
            f"{len(self._template_fine.points)}"
        )

    def _declare_parameters(self) -> None:
        # Input, template, and TF
        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("template_path", "")
        self.declare_parameter("template_points_topic", "template_points")
        self.declare_parameter("template_publish_hz", 1.0)
        self.declare_parameter("source_frame_id", "rslidar")
        self.declare_parameter("target_frame_id", "loadsense")
        self.declare_parameter("transform_publish_hz", 10.0)
        self.declare_parameter("require_matching_input_frame", True)
        self.declare_parameter("use_input_stamp", True)

        # Subscription QoS
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("qos_reliability", "best_effort")

        # Input validity
        self.declare_parameter("min_input_points", 100)
        self.declare_parameter("max_input_points", 0)

        # Crop box in source/input coordinates
        self.declare_parameter("crop_enabled", False)
        self.declare_parameter("crop_min", [-20.0, -20.0, -20.0])
        self.declare_parameter("crop_max", [20.0, 20.0, 20.0])

        # Input outlier removal
        self.declare_parameter("remove_outliers", False)
        self.declare_parameter("outlier_nb_neighbors", 20)
        self.declare_parameter("outlier_std_ratio", 2.0)

        # Coarse GICP
        self.declare_parameter("coarse_voxel_size", 0.05)
        self.declare_parameter("coarse_max_correspondence_distance", 0.25)
        self.declare_parameter("coarse_max_iterations", 80)

        # Fine GICP
        self.declare_parameter("fine_voxel_size", 0.02)
        self.declare_parameter("fine_max_correspondence_distance", 0.08)
        self.declare_parameter("fine_max_iterations", 60)

        # GICP optimizer
        self.declare_parameter("gicp_epsilon", 0.001)
        self.declare_parameter("relative_fitness", 1.0e-7)
        self.declare_parameter("relative_rmse", 1.0e-7)
        self.declare_parameter("use_robust_kernel", True)
        self.declare_parameter("robust_kernel_scale", 0.05)

        # Initial source -> target transformation
        self.declare_parameter("initial_translation", [0.0, 0.0, 0.0])
        self.declare_parameter("initial_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("use_previous_result_as_initial_guess", True)

        # Reject poor results instead of broadcasting them
        self.declare_parameter("min_fitness", 0.10)
        self.declare_parameter("max_inlier_rmse", 0.10)
        self.declare_parameter("log_every_n", 1)

    def _read_parameters(self) -> None:
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.template_path = str(self.get_parameter("template_path").value)
        self.template_points_topic = str(
            self.get_parameter("template_points_topic").value
        )
        self.template_publish_hz = float(
            self.get_parameter("template_publish_hz").value
        )
        self.source_frame_id = str(self.get_parameter("source_frame_id").value)
        self.target_frame_id = str(self.get_parameter("target_frame_id").value)
        self.transform_publish_hz = float(
            self.get_parameter("transform_publish_hz").value
        )
        self.require_matching_input_frame = bool(
            self.get_parameter("require_matching_input_frame").value
        )
        self.use_input_stamp = bool(self.get_parameter("use_input_stamp").value)

        self.qos_depth = int(self.get_parameter("qos_depth").value)
        self.qos_reliability = str(
            self.get_parameter("qos_reliability").value
        ).lower()

        self.min_input_points = int(self.get_parameter("min_input_points").value)
        self.max_input_points = int(self.get_parameter("max_input_points").value)

        self.crop_enabled = bool(self.get_parameter("crop_enabled").value)
        self.crop_min = np.asarray(
            self.get_parameter("crop_min").value, dtype=np.float64
        )
        self.crop_max = np.asarray(
            self.get_parameter("crop_max").value, dtype=np.float64
        )

        self.remove_outliers = bool(self.get_parameter("remove_outliers").value)
        self.outlier_nb_neighbors = int(
            self.get_parameter("outlier_nb_neighbors").value
        )
        self.outlier_std_ratio = float(
            self.get_parameter("outlier_std_ratio").value
        )

        self.coarse_voxel_size = float(
            self.get_parameter("coarse_voxel_size").value
        )
        self.coarse_max_distance = float(
            self.get_parameter("coarse_max_correspondence_distance").value
        )
        self.coarse_max_iterations = int(
            self.get_parameter("coarse_max_iterations").value
        )

        self.fine_voxel_size = float(self.get_parameter("fine_voxel_size").value)
        self.fine_max_distance = float(
            self.get_parameter("fine_max_correspondence_distance").value
        )
        self.fine_max_iterations = int(
            self.get_parameter("fine_max_iterations").value
        )

        self.gicp_epsilon = float(self.get_parameter("gicp_epsilon").value)
        self.relative_fitness = float(self.get_parameter("relative_fitness").value)
        self.relative_rmse = float(self.get_parameter("relative_rmse").value)
        self.use_robust_kernel = bool(
            self.get_parameter("use_robust_kernel").value
        )
        self.robust_kernel_scale = float(
            self.get_parameter("robust_kernel_scale").value
        )

        self.initial_translation = np.asarray(
            self.get_parameter("initial_translation").value, dtype=np.float64
        )
        self.initial_rpy = np.asarray(
            self.get_parameter("initial_rpy").value, dtype=np.float64
        )
        self.use_previous_result = bool(
            self.get_parameter("use_previous_result_as_initial_guess").value
        )

        self.min_fitness = float(self.get_parameter("min_fitness").value)
        self.max_inlier_rmse = float(
            self.get_parameter("max_inlier_rmse").value
        )
        self.log_every_n = int(self.get_parameter("log_every_n").value)

    def _validate_parameters(self) -> None:
        if not self.template_path:
            raise ValueError("ROS parameter 'template_path' must not be empty.")
        if not self.template_points_topic:
            raise ValueError("template_points_topic must not be empty.")
        if (
            not math.isfinite(self.template_publish_hz)
            or self.template_publish_hz <= 0.0
        ):
            raise ValueError("template_publish_hz must be a positive finite value.")
        if not self.source_frame_id or not self.target_frame_id:
            raise ValueError("source_frame_id and target_frame_id must not be empty.")
        if self.source_frame_id == self.target_frame_id:
            raise ValueError("source_frame_id and target_frame_id must differ.")
        if (
            not math.isfinite(self.transform_publish_hz)
            or self.transform_publish_hz <= 0.0
        ):
            raise ValueError("transform_publish_hz must be a positive finite value.")
        if self.qos_depth < 1:
            raise ValueError("qos_depth must be >= 1.")
        if self.qos_reliability not in {"best_effort", "reliable"}:
            raise ValueError(
                "qos_reliability must be 'best_effort' or 'reliable'."
            )
        if self.log_every_n < 1:
            raise ValueError("log_every_n must be >= 1.")
        if self.min_input_points < 3:
            raise ValueError("min_input_points must be >= 3.")
        if self.max_input_points < 0:
            raise ValueError("max_input_points must be >= 0.")
        if self.crop_min.shape != (3,) or self.crop_max.shape != (3,):
            raise ValueError("crop_min and crop_max must each contain 3 values.")
        if np.any(self.crop_min >= self.crop_max):
            raise ValueError("Every crop_min value must be smaller than crop_max.")
        if self.initial_translation.shape != (3,) or self.initial_rpy.shape != (3,):
            raise ValueError(
                "initial_translation and initial_rpy must each contain 3 values."
            )
        if self.coarse_voxel_size <= 0.0 or self.fine_voxel_size <= 0.0:
            raise ValueError("Voxel sizes must be positive.")
        if self.coarse_max_distance <= 0.0 or self.fine_max_distance <= 0.0:
            raise ValueError("Correspondence distances must be positive.")
        if self.coarse_max_iterations < 1 or self.fine_max_iterations < 1:
            raise ValueError("GICP iteration counts must be >= 1.")
        if self.gicp_epsilon <= 0.0:
            raise ValueError("gicp_epsilon must be positive.")
        if self.use_robust_kernel and self.robust_kernel_scale <= 0.0:
            raise ValueError("robust_kernel_scale must be positive.")

    def _make_initial_transform(self) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rpy_to_rotation_matrix(*self.initial_rpy.tolist())
        transform[:3, 3] = self.initial_translation
        return transform

    def _publish_template_points(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame_id
        message = point_cloud2.create_cloud_xyz32(header, self._template_xyz)
        self._template_publisher.publish(message)

    def _load_template(self, path_string: str) -> o3d.geometry.PointCloud:
        path = Path(path_string).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f"Template point cloud not found: {path}")

        cloud = o3d.io.read_point_cloud(str(path))
        if len(cloud.points) == 0:
            raise ValueError(f"Template contains no readable points: {path}")

        points = np.asarray(cloud.points)
        valid = np.isfinite(points).all(axis=1)
        cloud = cloud.select_by_index(np.flatnonzero(valid).tolist())

        if len(cloud.points) < 3:
            raise ValueError(f"Template has too few valid points: {path}")

        return cloud

    def _point_cloud2_to_xyz(self, message: PointCloud2) -> np.ndarray:
        field_names = {field.name for field in message.fields}
        missing = {"x", "y", "z"} - field_names
        if missing:
            raise ValueError(f"PointCloud2 is missing fields: {sorted(missing)}")

        points = point_cloud2.read_points(
            message,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )

        # Current sensor_msgs_py returns a structured NumPy array.
        if isinstance(points, np.ndarray) and points.dtype.names is not None:
            xyz = np.column_stack(
                (points["x"], points["y"], points["z"])
            ).astype(np.float64, copy=False)
        else:
            # Compatibility path for older sensor_msgs_py implementations.
            xyz = np.asarray(list(points), dtype=np.float64)

        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError(f"Unexpected XYZ array shape: {xyz.shape}")

        valid = np.isfinite(xyz).all(axis=1)
        return xyz[valid]

    def _crop_xyz(self, xyz: np.ndarray) -> np.ndarray:
        if not self.crop_enabled:
            return xyz
        mask = np.logical_and(
            np.all(xyz >= self.crop_min, axis=1),
            np.all(xyz <= self.crop_max, axis=1),
        )
        return xyz[mask]

    def _limit_point_count(self, xyz: np.ndarray) -> np.ndarray:
        if self.max_input_points <= 0 or len(xyz) <= self.max_input_points:
            return xyz

        # Deterministic, approximately uniform index subsampling.
        indices = np.linspace(
            0,
            len(xyz) - 1,
            num=self.max_input_points,
            dtype=np.int64,
        )
        return xyz[indices]

    @staticmethod
    def _xyz_to_open3d(xyz: np.ndarray) -> o3d.geometry.PointCloud:
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            np.ascontiguousarray(xyz, dtype=np.float64)
        )
        return cloud

    def _preprocess(
        self,
        cloud: o3d.geometry.PointCloud,
        voxel_size: float,
        remove_outliers: bool,
    ) -> o3d.geometry.PointCloud:
        processed = cloud.voxel_down_sample(voxel_size)

        if remove_outliers and len(processed.points) >= self.outlier_nb_neighbors:
            processed, _ = processed.remove_statistical_outlier(
                nb_neighbors=self.outlier_nb_neighbors,
                std_ratio=self.outlier_std_ratio,
            )

        return processed

    def _make_estimation(
        self,
    ) -> o3d.pipelines.registration.TransformationEstimationForGeneralizedICP:
        if not self.use_robust_kernel:
            return (
                o3d.pipelines.registration
                .TransformationEstimationForGeneralizedICP(
                    epsilon=self.gicp_epsilon
                )
            )

        kernel = o3d.pipelines.registration.CauchyLoss(
            k=self.robust_kernel_scale
        )
        try:
            return (
                o3d.pipelines.registration
                .TransformationEstimationForGeneralizedICP(
                    epsilon=self.gicp_epsilon,
                    kernel=kernel,
                )
            )
        except TypeError:
            # Some older Open3D wheels do not expose the kernel overload.
            if not self._warned_robust_kernel_fallback:
                self.get_logger().warning(
                    "This Open3D build does not support a robust kernel for "
                    "GICP. Continuing without the kernel."
                )
                self._warned_robust_kernel_fallback = True
            return (
                o3d.pipelines.registration
                .TransformationEstimationForGeneralizedICP(
                    epsilon=self.gicp_epsilon
                )
            )

    def _run_gicp(
        self,
        source: o3d.geometry.PointCloud,
        target: o3d.geometry.PointCloud,
        initial_transform: np.ndarray,
        max_distance: float,
        max_iterations: int,
    ) -> o3d.pipelines.registration.RegistrationResult:
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=self.relative_fitness,
            relative_rmse=self.relative_rmse,
            max_iteration=max_iterations,
        )
        return o3d.pipelines.registration.registration_generalized_icp(
            source=source,
            target=target,
            max_correspondence_distance=max_distance,
            init=initial_transform,
            estimation_method=self._make_estimation(),
            criteria=criteria,
        )

    def _is_result_acceptable(
        self,
        result: o3d.pipelines.registration.RegistrationResult,
    ) -> bool:
        transform = np.asarray(result.transformation)
        return (
            transform.shape == (4, 4)
            and np.isfinite(transform).all()
            and math.isfinite(float(result.fitness))
            and math.isfinite(float(result.inlier_rmse))
            and float(result.fitness) >= self.min_fitness
            and float(result.inlier_rmse) <= self.max_inlier_rmse
        )

    def _broadcast_transform(
        self,
        transform_matrix: np.ndarray,
        stamp,
    ) -> None:
        quaternion = rotation_matrix_to_quaternion(transform_matrix[:3, :3])

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.target_frame_id
        transform.child_frame_id = self.source_frame_id

        transform.transform.translation.x = float(transform_matrix[0, 3])
        transform.transform.translation.y = float(transform_matrix[1, 3])
        transform.transform.translation.z = float(transform_matrix[2, 3])

        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])

        self._tf_broadcaster.sendTransform(transform)

    def _broadcast_last_transform(self) -> None:
        self._broadcast_transform(
            self._last_transform,
            self.get_clock().now().to_msg(),
        )

    def _cache_point_cloud(self, message: PointCloud2) -> None:
        self._latest_point_cloud = message

    def _calibrate_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        message = self._latest_point_cloud
        if message is None:
            response.success = False
            response.message = "No input point cloud has been received."
            return response

        previous_success_count = self._success_count
        self._point_cloud_callback(message)
        response.success = self._success_count > previous_success_count
        response.message = (
            "Calibration succeeded."
            if response.success
            else "Calibration failed; see node logs for details."
        )
        return response

    def _point_cloud_callback(self, message: PointCloud2) -> None:
        if (
            self.require_matching_input_frame
            and message.header.frame_id != self.source_frame_id
        ):
            self.get_logger().warning(
                "Skipping cloud because header.frame_id differs from "
                f"source_frame_id: '{message.header.frame_id}' != "
                f"'{self.source_frame_id}'"
            )
            return

        # Drop the callback rather than queue expensive registrations when a
        # MultiThreadedExecutor is used.
        if not self._processing_lock.acquire(blocking=False):
            self.get_logger().warning("GICP is busy; dropping this cloud.")
            return

        start_time = time.perf_counter()
        try:
            xyz = self._point_cloud2_to_xyz(message)
            xyz = self._crop_xyz(xyz)
            xyz = self._limit_point_count(xyz)

            if len(xyz) < self.min_input_points:
                self.get_logger().warning(
                    f"Too few input points after filtering: {len(xyz)}"
                )
                return

            source_raw = self._xyz_to_open3d(xyz)
            source_coarse = self._preprocess(
                source_raw,
                voxel_size=self.coarse_voxel_size,
                remove_outliers=self.remove_outliers,
            )
            source_fine = self._preprocess(
                source_raw,
                voxel_size=self.fine_voxel_size,
                remove_outliers=self.remove_outliers,
            )

            if (
                len(source_coarse.points) < 3
                or len(source_fine.points) < 3
            ):
                self.get_logger().warning(
                    "Too few points remain after voxel downsampling."
                )
                return

            initial_guess = (
                self._last_transform
                if self.use_previous_result
                else self._initial_transform
            )

            coarse = self._run_gicp(
                source=source_coarse,
                target=self._template_coarse,
                initial_transform=initial_guess,
                max_distance=self.coarse_max_distance,
                max_iterations=self.coarse_max_iterations,
            )

            fine = self._run_gicp(
                source=source_fine,
                target=self._template_fine,
                initial_transform=coarse.transformation,
                max_distance=self.fine_max_distance,
                max_iterations=self.fine_max_iterations,
            )

            elapsed_ms = (time.perf_counter() - start_time) * 1000.0

            if not self._is_result_acceptable(fine):
                self.get_logger().warning(
                    "GICP result rejected: "
                    f"fitness={fine.fitness:.4f}, "
                    f"rmse={fine.inlier_rmse:.4f} m, "
                    f"time={elapsed_ms:.1f} ms"
                )
                return

            self._last_transform = np.asarray(
                fine.transformation, dtype=np.float64
            ).copy()
            stamp = (
                message.header.stamp
                if self.use_input_stamp
                else self.get_clock().now().to_msg()
            )
            self._broadcast_transform(self._last_transform, stamp)
            self._success_count += 1

            if self._success_count % self.log_every_n == 0:
                translation = self._last_transform[:3, 3]
                self.get_logger().info(
                    "GICP accepted: "
                    f"fitness={fine.fitness:.4f}, "
                    f"rmse={fine.inlier_rmse:.4f} m, "
                    f"xyz=[{translation[0]:.3f}, "
                    f"{translation[1]:.3f}, "
                    f"{translation[2]:.3f}] m, "
                    f"points(raw/coarse/fine)="
                    f"{len(xyz)}/"
                    f"{len(source_coarse.points)}/"
                    f"{len(source_fine.points)}, "
                    f"time={elapsed_ms:.1f} ms"
                )

        except Exception as error:  # Keep the ROS node alive on malformed data.
            self.get_logger().error(
                f"Failed to process PointCloud2: {type(error).__name__}: {error}"
            )
        finally:
            self._processing_lock.release()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[GicpTfNode] = None
    try:
        node = GicpTfNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
