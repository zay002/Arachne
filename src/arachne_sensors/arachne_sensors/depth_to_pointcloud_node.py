from __future__ import annotations

import argparse
from functools import lru_cache

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
import tf2_ros
from tf2_ros import TransformException
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


FIELDS_XYZ = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


@lru_cache(maxsize=8)
def _pixel_grid(width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    return np.meshgrid(u, v)


def _matrix_from_transform(msg) -> np.ndarray:
    t = msg.transform.translation
    q = msg.transform.rotation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 1e-9:
        x = y = z = 0.0
        w = 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    matrix[:3, 3] = (float(t.x), float(t.y), float(t.z))
    return matrix


class DepthToPointCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("arachne_depth_to_pointcloud")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("pointcloud_topic", "/arachne/debug/depth_points")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("stride", 1)
        self.declare_parameter("projection_flip_x", False)
        self.declare_parameter("projection_flip_y", False)
        self.declare_parameter("frames", 3)
        self.declare_parameter("continuous", False)
        self.declare_parameter("exit_after_publish", True)
        self.declare_parameter("republish_period_sec", 1.0)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("min_target_z_m", -10.0)
        self.declare_parameter("max_target_z_m", 0.0)
        self.declare_parameter("min_publish_points", 1000)

        self.camera_info: CameraInfo | None = None
        self.points_by_frame: list[np.ndarray] = []
        self.snapshot_points: np.ndarray | None = None
        self.snapshot_frame_id = "camera_depth_optical_frame"
        self.republish_timer = None
        self.published = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._depth_cb,
            qos_profile_sensor_data,
        )
        cloud_qos = QoSProfile(depth=1)
        cloud_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cloud_qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            cloud_qos,
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _depth_cb(self, msg: Image) -> None:
        continuous = bool(self.get_parameter("continuous").value)
        if self.published and not continuous:
            return
        info = self.camera_info
        if info is None:
            return
        depth = self._depth_array(msg)
        if depth is None:
            return
        stride = max(int(self.get_parameter("stride").value), 1)
        if stride > 1:
            depth = depth[::stride, ::stride]
        height, width = depth.shape
        uu, vv = _pixel_grid(width, height)
        if stride > 1:
            uu = uu * stride
            vv = vv * stride

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx == 0.0 or fy == 0.0:
            return

        min_depth = float(self.get_parameter("min_depth_m").value)
        max_depth = float(self.get_parameter("max_depth_m").value)
        mask = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
        output_frame = msg.header.frame_id or "camera_depth_optical_frame"
        target_frame = str(self.get_parameter("target_frame").value).strip()
        if target_frame:
            output_frame = target_frame

        if np.any(mask):
            z = depth[mask].astype(np.float32, copy=False)
            px = cx - uu[mask] if bool(self.get_parameter("projection_flip_x").value) else uu[mask] - cx
            py = cy - vv[mask] if bool(self.get_parameter("projection_flip_y").value) else vv[mask] - cy
            x = (px * z / fx).astype(np.float32, copy=False)
            y = (py * z / fy).astype(np.float32, copy=False)
            points = np.column_stack((x, y, z)).astype(np.float32, copy=False)
            source_frame = msg.header.frame_id or "camera_depth_optical_frame"
            if target_frame:
                if target_frame != source_frame:
                    matrix = self._target_from_camera_matrix(target_frame, source_frame, msg)
                    if matrix is None:
                        self.points_by_frame.clear()
                        return
                    points = self._transform_points(matrix, points)
                z = points[:, 2]
                keep = (
                    np.isfinite(z)
                    & (z >= float(self.get_parameter("min_target_z_m").value))
                    & (z <= float(self.get_parameter("max_target_z_m").value))
                )
                points = points[keep]
            if points.size:
                self.points_by_frame.append(points)

        target_frames = max(int(self.get_parameter("frames").value), 1)
        if len(self.points_by_frame) < target_frames or self.published:
            return
        frame_count = len(self.points_by_frame)
        points = (
            np.concatenate(self.points_by_frame, axis=0)
            if self.points_by_frame
            else np.empty((0, 3), dtype=np.float32)
        )
        min_publish_points = max(int(self.get_parameter("min_publish_points").value), 0)
        if points.shape[0] < min_publish_points:
            self.points_by_frame.clear()
            self.get_logger().warning(
                f"skip sparse depth cloud: points={points.shape[0]} min={min_publish_points}",
                throttle_duration_sec=2.0,
            )
            return
        self.snapshot_points = points
        self.snapshot_frame_id = output_frame
        self._publish_snapshot()
        self.points_by_frame.clear()
        self.published = not continuous
        self.get_logger().info(
            f"published depth snapshot cloud: frame={self.snapshot_frame_id} "
            f"frames={frame_count} points={points.shape[0]}",
            throttle_duration_sec=2.0 if continuous else 0.0,
        )
        if continuous:
            return
        if bool(self.get_parameter("exit_after_publish").value):
            self.create_timer(0.5, lambda: rclpy.shutdown())
        else:
            period = max(float(self.get_parameter("republish_period_sec").value), 0.1)
            self.republish_timer = self.create_timer(period, self._publish_snapshot)

    def _depth_array(self, msg: Image) -> np.ndarray | None:
        encoding = msg.encoding.upper()
        if encoding in ("16UC1", "MONO16"):
            raw = self._image_plane(msg, np.uint16, 2)
            if raw is None:
                return None
            return raw.astype(np.float32) * float(self.get_parameter("depth_scale").value)
        if encoding == "32FC1":
            raw = self._image_plane(msg, np.float32, 4)
            return raw.astype(np.float32, copy=False) if raw is not None else None
        self.get_logger().warning(f"unsupported depth encoding: {msg.encoding}", throttle_duration_sec=5.0)
        return None

    def _image_plane(self, msg: Image, dtype: np.dtype, bytes_per_pixel: int) -> np.ndarray | None:
        row_values = int(msg.step) // bytes_per_pixel
        expected = row_values * int(msg.height)
        data = np.frombuffer(msg.data, dtype=dtype, count=expected)
        if data.size < expected or row_values < int(msg.width):
            return None
        return data.reshape((int(msg.height), row_values))[:, : int(msg.width)]

    def _publish_snapshot(self) -> None:
        if self.snapshot_points is None:
            return
        self.pub.publish(self._cloud_msg(self.snapshot_frame_id, self.snapshot_points))

    def _target_from_camera_matrix(
        self, target_frame: str, camera_frame: str, msg: Image
    ) -> np.ndarray | None:
        """Return target<-camera for this depth frame from live TF."""
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        for time_point in (stamp, rclpy.time.Time()):
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    camera_frame,
                    time_point,
                    timeout=Duration(seconds=0.2),
                )
                return _matrix_from_transform(transform)
            except TransformException as exc:
                last_error = exc
        self.get_logger().warning(
            f"waiting for TF {target_frame} <- {camera_frame}: {last_error}",
            throttle_duration_sec=2.0,
        )
        return None

    def _transform_points(self, matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return points
        return (points @ matrix[:3, :3].T + matrix[:3, 3]).astype(np.float32, copy=False)

    def _cloud_msg(self, frame_id: str, points: np.ndarray) -> PointCloud2:
        cloud = PointCloud2()
        cloud.header.frame_id = frame_id
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = FIELDS_XYZ
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.astype(np.float32, copy=False).tobytes()
        return cloud


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Convert a few depth images to one XYZ PointCloud2 snapshot")
    parser.add_argument("--dry-run-check", action="store_true")
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.dry_run_check:
        print("depth_to_pointcloud dry-run check ok")
        return
    rclpy.init(args=ros_args)
    node = DepthToPointCloudNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
