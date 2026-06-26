from __future__ import annotations

import argparse
from functools import lru_cache

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
        self.declare_parameter("frames", 3)
        self.declare_parameter("exit_after_publish", True)
        self.declare_parameter("republish_period_sec", 1.0)

        self.camera_info: CameraInfo | None = None
        self.points_by_frame: list[np.ndarray] = []
        self.snapshot_points: np.ndarray | None = None
        self.snapshot_frame_id = "camera_depth_optical_frame"
        self.republish_timer = None
        self.published = False
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
        if self.published:
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
            uu *= stride
            vv *= stride

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx == 0.0 or fy == 0.0:
            return

        min_depth = float(self.get_parameter("min_depth_m").value)
        max_depth = float(self.get_parameter("max_depth_m").value)
        mask = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
        if np.any(mask):
            z = depth[mask].astype(np.float32, copy=False)
            x = ((uu[mask] - cx) * z / fx).astype(np.float32, copy=False)
            y = ((vv[mask] - cy) * z / fy).astype(np.float32, copy=False)
            points = np.column_stack((x, y, z)).astype(np.float32, copy=False)
            self.points_by_frame.append(points)

        target_frames = max(int(self.get_parameter("frames").value), 1)
        if len(self.points_by_frame) < target_frames or self.published:
            return
        points = np.concatenate(self.points_by_frame, axis=0) if self.points_by_frame else np.empty((0, 3), dtype=np.float32)
        self.snapshot_points = points
        self.snapshot_frame_id = msg.header.frame_id or "camera_depth_optical_frame"
        self._publish_snapshot()
        self.published = True
        self.get_logger().info(f"published depth snapshot cloud: frames={len(self.points_by_frame)} points={points.shape[0]}")
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
