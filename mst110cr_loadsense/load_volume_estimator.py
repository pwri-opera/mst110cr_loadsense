#!/usr/bin/env python3
from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d
from skimage.restoration import inpaint_biharmonic

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from mst110cr_loadsense.msg import FloatStamped


class LoadVolumeEstimator(Node):
    """Estimate load volume from a point cloud transformed into the vessel frame."""

    def __init__(self) -> None:
        super().__init__("load_volume_estimator")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()
        (
            self._bed_heightmap,
            self._heightmap_x_cells,
            self._heightmap_y_cells,
        ) = self._create_bed_heightmap()
        x_coordinates = (
            self.clip_min[0]
            + (np.arange(self._heightmap_x_cells, dtype=np.float64) + 0.5)
            * self.heightmap_resolution
        )
        fence_x = self.bed_fence_xz_points[::2]
        fence_z = self.bed_fence_xz_points[1::2]
        self._bed_fence_heights = np.interp(x_coordinates, fence_x, fence_z)
        left = x_coordinates < fence_x[0]
        right = x_coordinates > fence_x[-1]
        self._bed_fence_heights[left] = fence_z[0] + (
            (x_coordinates[left] - fence_x[0])
            * (fence_z[1] - fence_z[0])
            / (fence_x[1] - fence_x[0])
        )
        self._bed_fence_heights[right] = fence_z[-1] + (
            (x_coordinates[right] - fence_x[-1])
            * (fence_z[-1] - fence_z[-2])
            / (fence_x[-1] - fence_x[-2])
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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._volume_publisher = self.create_publisher(
            FloatStamped,
            self.load_volume_topic,
            10,
        )
        self._clipped_points_publisher = self.create_publisher(
            PointCloud2,
            self.clipped_points_topic,
            qos,
        )
        self._heightmap_marker_publisher = self.create_publisher(
            MarkerArray,
            self.heightmap_marker_topic,
            10,
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._point_cloud_callback,
            qos,
        )

        self.get_logger().info(
            "Load volume estimator started: "
            f"input={self.input_topic}, "
            f"target_frame={self.target_frame_id}, "
            f"bed_mesh={self.bed_mesh_path}, "
            f"resolution={self.heightmap_resolution:g} m, "
            f"volume_topic={self.load_volume_topic}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("target_frame_id", "dump_bed")
        self.declare_parameter("load_volume_topic", "load_volume")
        self.declare_parameter("clipped_points_topic", "clipped_points")
        self.declare_parameter("heightmap_marker_topic", "heightmap_markers")
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("use_input_stamp", True)

        # Axis-aligned vessel bounds in target_frame_id coordinates.
        self.declare_parameter("clip_min", [-3.0, -1.5, 0.0])
        self.declare_parameter("clip_max", [3.0, 1.5, 3.0])

        # The mesh and sensor points use target_frame_id coordinates.
        self.declare_parameter("bed_mesh_path", "")
        self.declare_parameter("heightmap_resolution", 0.10)
        self.declare_parameter(
            "bed_fence_xz_points",
            [-3.0, 1.5, 3.0, 1.5],
        )
        self.declare_parameter("min_clipped_points", 10)
        self.declare_parameter("marker_alpha", 0.75)

    def _read_parameters(self) -> None:
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.target_frame_id = str(
            self.get_parameter("target_frame_id").value
        )
        self.load_volume_topic = str(
            self.get_parameter("load_volume_topic").value
        )
        self.clipped_points_topic = str(
            self.get_parameter("clipped_points_topic").value
        )
        self.heightmap_marker_topic = str(
            self.get_parameter("heightmap_marker_topic").value
        )
        self.qos_depth = int(self.get_parameter("qos_depth").value)
        self.qos_reliability = str(
            self.get_parameter("qos_reliability").value
        ).lower()
        self.tf_timeout_sec = float(
            self.get_parameter("tf_timeout_sec").value
        )
        self.use_input_stamp = bool(
            self.get_parameter("use_input_stamp").value
        )
        self.clip_min = np.asarray(
            self.get_parameter("clip_min").value,
            dtype=np.float64,
        )
        self.clip_max = np.asarray(
            self.get_parameter("clip_max").value,
            dtype=np.float64,
        )
        self.heightmap_resolution = float(
            self.get_parameter("heightmap_resolution").value
        )
        self.bed_fence_xz_points = np.asarray(
            self.get_parameter("bed_fence_xz_points").value,
            dtype=np.float64,
        )
        self.bed_mesh_path = str(
            self.get_parameter("bed_mesh_path").value
        )
        self.min_clipped_points = int(
            self.get_parameter("min_clipped_points").value
        )
        self.marker_alpha = float(
            self.get_parameter("marker_alpha").value
        )

    def _validate_parameters(self) -> None:
        if not self.input_topic or not self.target_frame_id:
            raise ValueError("input_topic and target_frame_id must not be empty.")
        if (
            not self.load_volume_topic
            or not self.clipped_points_topic
            or not self.heightmap_marker_topic
        ):
            raise ValueError("Output topic names must not be empty.")
        if self.qos_depth < 1:
            raise ValueError("qos_depth must be >= 1.")
        if self.qos_reliability not in {"best_effort", "reliable"}:
            raise ValueError(
                "qos_reliability must be 'best_effort' or 'reliable'."
            )
        if self.clip_min.shape != (3,) or self.clip_max.shape != (3,):
            raise ValueError("clip_min and clip_max must each contain 3 values.")
        if not np.isfinite(self.clip_min).all() or not np.isfinite(
            self.clip_max
        ).all():
            raise ValueError("Clipping bounds must be finite.")
        if np.any(self.clip_min >= self.clip_max):
            raise ValueError("Every clip_min value must be smaller than clip_max.")
        if (
            not math.isfinite(self.heightmap_resolution)
            or self.heightmap_resolution <= 0.0
        ):
            raise ValueError("heightmap_resolution must be positive and finite.")
        if (
            self.bed_fence_xz_points.ndim != 1
            or self.bed_fence_xz_points.size < 4
            or self.bed_fence_xz_points.size % 2 != 0
            or not np.isfinite(self.bed_fence_xz_points).all()
        ):
            raise ValueError(
                "bed_fence_xz_points must contain at least two finite [x, z] pairs."
            )
        if np.any(np.diff(self.bed_fence_xz_points[::2]) <= 0.0):
            raise ValueError(
                "bed_fence_xz_points x values must be strictly increasing."
            )
        if not self.bed_mesh_path:
            raise ValueError("bed_mesh_path must not be empty.")
        bed_mesh_path = Path(self.bed_mesh_path).expanduser()
        if not bed_mesh_path.is_absolute():
            raise ValueError("bed_mesh_path must be an absolute path.")
        if not bed_mesh_path.is_file():
            raise FileNotFoundError(f"Bed mesh not found: {bed_mesh_path}")
        if not math.isfinite(self.tf_timeout_sec) or self.tf_timeout_sec < 0.0:
            raise ValueError("tf_timeout_sec must be finite and >= 0.")
        if self.min_clipped_points < 1:
            raise ValueError("min_clipped_points must be >= 1.")
        if (
            not math.isfinite(self.marker_alpha)
            or not 0.0 < self.marker_alpha <= 1.0
        ):
            raise ValueError("marker_alpha must be in the range (0, 1].")

    def _heightmap_shape(self) -> tuple[int, int]:
        x_cells = max(
            1,
            math.ceil(
                (self.clip_max[0] - self.clip_min[0])
                / self.heightmap_resolution
            ),
        )
        y_cells = max(
            1,
            math.ceil(
                (self.clip_max[1] - self.clip_min[1])
                / self.heightmap_resolution
            ),
        )
        return x_cells, y_cells

    def _create_bed_heightmap(self) -> tuple[np.ndarray, int, int]:
        mesh_path = Path(self.bed_mesh_path).expanduser()
        legacy_mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        if len(legacy_mesh.vertices) < 3 or len(legacy_mesh.triangles) < 1:
            raise ValueError(
                f"Bed mesh contains no usable triangles: {mesh_path}"
            )

        vertices = np.asarray(legacy_mesh.vertices, dtype=np.float64)
        if not np.isfinite(vertices).all():
            raise ValueError(f"Bed mesh contains non-finite vertices: {mesh_path}")

        scene = o3d.t.geometry.RaycastingScene()
        tensor_mesh = o3d.t.geometry.TriangleMesh.from_legacy(legacy_mesh)
        scene.add_triangles(tensor_mesh)

        x_cells, y_cells = self._heightmap_shape()
        x_coordinates = (
            self.clip_min[0]
            + (np.arange(x_cells, dtype=np.float32) + 0.5)
            * self.heightmap_resolution
        )
        y_coordinates = (
            self.clip_min[1]
            + (np.arange(y_cells, dtype=np.float32) + 0.5)
            * self.heightmap_resolution
        )
        grid_x, grid_y = np.meshgrid(x_coordinates, y_coordinates)

        ray_origin_z = float(
            max(self.clip_max[2], np.max(vertices[:, 2]))
            + max(self.clip_max[2] - self.clip_min[2], 1.0)
        )
        rays = np.column_stack(
            (
                grid_x.ravel(),
                grid_y.ravel(),
                np.full(grid_x.size, ray_origin_z, dtype=np.float32),
                np.zeros(grid_x.size, dtype=np.float32),
                np.zeros(grid_x.size, dtype=np.float32),
                -np.ones(grid_x.size, dtype=np.float32),
            )
        ).astype(np.float32, copy=False)
        cast_result = scene.cast_rays(o3d.core.Tensor(rays))
        distances = cast_result["t_hit"].numpy().astype(
            np.float64,
            copy=False,
        )

        bed_heightmap = np.full(grid_x.size, np.nan, dtype=np.float64)
        hits = np.isfinite(distances)
        bed_heightmap[hits] = ray_origin_z - distances[hits]
        hit_count = int(np.count_nonzero(hits))
        if hit_count == 0:
            raise ValueError(
                "No downward heightmap rays intersected the bed mesh inside "
                "the configured XY clipping bounds."
            )

        self.get_logger().info(
            "Created polygon bed heightmap: "
            f"cells={x_cells}x{y_cells}, "
            f"mesh_cells={hit_count}/{bed_heightmap.size}"
        )
        return bed_heightmap, x_cells, y_cells

    @staticmethod
    def _point_cloud2_to_xyz(message: PointCloud2) -> np.ndarray:
        field_names = {field.name for field in message.fields}
        missing = {"x", "y", "z"} - field_names
        if missing:
            raise ValueError(f"PointCloud2 is missing fields: {sorted(missing)}")

        points = point_cloud2.read_points(
            message,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        if isinstance(points, np.ndarray) and points.dtype.names is not None:
            xyz = np.column_stack(
                (points["x"], points["y"], points["z"])
            ).astype(np.float64, copy=False)
        else:
            xyz = np.asarray(list(points), dtype=np.float64)

        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError(f"Unexpected XYZ array shape: {xyz.shape}")
        return xyz[np.isfinite(xyz).all(axis=1)]

    @staticmethod
    def _quaternion_to_rotation_matrix(
        x: float,
        y: float,
        z: float,
        w: float,
    ) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if not math.isfinite(norm) or norm < 1.0e-12:
            raise ValueError("TF contains an invalid quaternion.")
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

    def _transform_points(
        self,
        xyz: np.ndarray,
        message: PointCloud2,
    ) -> np.ndarray:
        if message.header.frame_id == self.target_frame_id:
            return xyz
        if not message.header.frame_id:
            raise ValueError("Input PointCloud2 has an empty frame_id.")

        transform = self._tf_buffer.lookup_transform(
            self.target_frame_id,
            message.header.frame_id,
            (
                Time.from_msg(message.header.stamp)
                if self.use_input_stamp
                else Time()
            ),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rotation_matrix = self._quaternion_to_rotation_matrix(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        offset = np.array(
            [translation.x, translation.y, translation.z],
            dtype=np.float64,
        )
        return xyz @ rotation_matrix.T + offset

    def _clip_points(self, xyz: np.ndarray) -> np.ndarray:
        mask = np.logical_and(
            np.all(xyz >= self.clip_min, axis=1),
            np.all(xyz <= self.clip_max, axis=1),
        )
        return xyz[mask]

    def _make_heightmap(
        self,
        xyz: np.ndarray,
    ) -> tuple[np.ndarray, int, int]:
        x_cells = self._heightmap_x_cells
        y_cells = self._heightmap_y_cells
        x_indices = np.floor(
            (xyz[:, 0] - self.clip_min[0]) / self.heightmap_resolution
        ).astype(np.int64)
        y_indices = np.floor(
            (xyz[:, 1] - self.clip_min[1]) / self.heightmap_resolution
        ).astype(np.int64)
        np.clip(x_indices, 0, x_cells - 1, out=x_indices)
        np.clip(y_indices, 0, y_cells - 1, out=y_indices)

        heightmap = np.full(x_cells * y_cells, -np.inf, dtype=np.float64)
        flat_indices = y_indices * x_cells + x_indices
        np.maximum.at(heightmap, flat_indices, xyz[:, 2])

        surface = heightmap.reshape((y_cells, x_cells))
        bed_surface = self._bed_heightmap.reshape((y_cells, x_cells))
        valid_bed = np.isfinite(bed_surface)
        surface[~valid_bed] = np.nan
        if not np.any(np.isfinite(surface)):
            return np.zeros_like(heightmap), x_cells, y_cells

        # TODO: Height Mapの端 (xの最小値)がいくつか欠損している場合，その欠損している値は底面の高さとする
        # Fill missing values at the left edge (x minimum) with bed surface height
        left_edge_missing = ~np.isfinite(surface[:, 0])
        surface[left_edge_missing, 0] = bed_surface[left_edge_missing, 0]

        missing = ~np.isfinite(surface)
        if np.any(missing):
            surface = surface.copy()
            surface[missing] = 0.0
            try:
                surface = inpaint_biharmonic(
                    surface,
                    missing,
                    channel_axis=None,
                )
            except TypeError:
                # Compatibility with scikit-image versions before channel_axis.
                surface = inpaint_biharmonic(
                    surface,
                    missing,
                    multichannel=False,
                )

            edge_missing = np.zeros_like(missing)
            edge_missing[[0, -1], :] = missing[[0, -1], :]

            excessive_edges = edge_missing & (
                surface > self._bed_fence_heights[np.newaxis, :]
            )
            if np.any(excessive_edges):
                fence_heights = np.broadcast_to(
                    self._bed_fence_heights,
                    surface.shape,
                )
                surface[excessive_edges] = fence_heights[excessive_edges]
                # self.get_logger().info(f"Excessive edges: {excessive_edges}")

                remaining_missing = missing & ~excessive_edges
                try:
                    surface = inpaint_biharmonic(
                        surface,
                        remaining_missing,
                        channel_axis=None,
                    )
                except TypeError:
                    surface = inpaint_biharmonic(
                        surface,
                        remaining_missing,
                        multichannel=False,
                    )

        heights = np.zeros_like(surface)
        heights[valid_bed] = np.maximum(
            surface[valid_bed] - bed_surface[valid_bed],
            0.0,
        )
        return heights.ravel(), x_cells, y_cells

    def _calculate_volume(self, xyz: np.ndarray) -> float:
        heights, _, _ = self._make_heightmap(xyz)
        return float(
            np.sum(heights)
            * self.heightmap_resolution
            * self.heightmap_resolution
        )

    def _publish_heightmap_markers(
        self,
        heights: np.ndarray,
        x_cells: int,
        stamp,
    ) -> None:
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.stamp = stamp
        clear_marker.header.frame_id = self.target_frame_id
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        valid_bed_heights = self._bed_heightmap[
            np.isfinite(self._bed_heightmap)
        ]
        maximum_height = max(
            self.clip_max[2] - float(np.min(valid_bed_heights)),
            1.0e-6,
        )
        for flat_index in np.flatnonzero(heights > 0.0):
            height = float(heights[flat_index])
            x_index = int(flat_index % x_cells)
            y_index = int(flat_index // x_cells)

            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.target_frame_id
            marker.ns = "load_heightmap"
            marker.id = int(flat_index)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(
                self.clip_min[0]
                + (x_index + 0.5) * self.heightmap_resolution
            )
            marker.pose.position.y = float(
                self.clip_min[1]
                + (y_index + 0.5) * self.heightmap_resolution
            )
            marker.pose.position.z = (
                float(self._bed_heightmap[flat_index]) + height * 0.5
            )
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.heightmap_resolution
            marker.scale.y = self.heightmap_resolution
            marker.scale.z = height

            normalized_height = min(height / maximum_height, 1.0)
            marker.color.r = normalized_height
            marker.color.g = 0.2
            marker.color.b = 1.0 - normalized_height
            marker.color.a = self.marker_alpha
            marker_array.markers.append(marker)

        self._heightmap_marker_publisher.publish(marker_array)

    def _publish_clipped_points(
        self,
        xyz: np.ndarray,
        stamp,
    ) -> None:
        header = Header()
        header.stamp = stamp
        header.frame_id = self.target_frame_id
        message = point_cloud2.create_cloud_xyz32(
            header,
            np.asarray(xyz, dtype=np.float32),
        )
        self._clipped_points_publisher.publish(message)

    def _point_cloud_callback(self, message: PointCloud2) -> None:
        try:
            output_stamp = (
                message.header.stamp
                if self.use_input_stamp
                else self.get_clock().now().to_msg()
            )
            xyz = self._point_cloud2_to_xyz(message)
            xyz = self._transform_points(xyz, message)
            clipped_xyz = self._clip_points(xyz)
            self._publish_clipped_points(clipped_xyz, output_stamp)

            if len(clipped_xyz) < self.min_clipped_points:
                self.get_logger().warning(
                    "Not publishing volume: only "
                    f"{len(clipped_xyz)} clipped points received."
                )
                return

            heights, x_cells, _ = self._make_heightmap(clipped_xyz)
            volume = float(
                np.sum(heights)
                * self.heightmap_resolution
                * self.heightmap_resolution
            )
            self._publish_heightmap_markers(
                heights,
                x_cells,
                output_stamp,
            )
            output = FloatStamped()
            output.header.stamp = output_stamp
            output.header.frame_id = self.target_frame_id
            output.data = volume
            self._volume_publisher.publish(output)
        except TransformException as error:
            self.get_logger().warning(f"Point-cloud TF lookup failed: {error}")
        except Exception as error:
            self.get_logger().error(
                "Failed to estimate load volume: "
                f"{type(error).__name__}: {error}"
            )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[LoadVolumeEstimator] = None
    try:
        node = LoadVolumeEstimator()
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
