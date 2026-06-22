#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from arachne_operator.grasp_geometry import pointcloud_grasp_geometry
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src" / "arachne_operator"))
    from arachne_operator.grasp_geometry import pointcloud_grasp_geometry


def _rotation_matrix_xyzw(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm == 0.0:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class GraspPoseDryTiming(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("grasp_pose_dry_timing")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.samples: list[float] = []
        self.create_subscription(PointCloud2, args.cloud_topic, self._cloud_cb, 10)
        self.deadline = time.monotonic() + args.timeout_sec
        self.get_logger().info(
            f"dry timing: {args.cloud_topic} -> {args.base_frame}; no motion, no action goals"
        )

    def done(self) -> bool:
        return len(self.samples) >= self.args.samples or time.monotonic() > self.deadline

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if len(self.samples) >= self.args.samples:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.15),
            )
        except TransformException as exc:
            self.get_logger().warning(f"tf unavailable {msg.header.frame_id}->{self.args.base_frame}: {exc}")
            return

        raw = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not raw:
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        rotation = _rotation_matrix_xyzw(q.x, q.y, q.z, q.w)
        offset = np.asarray((t.x, t.y, t.z), dtype=np.float64)
        points = (np.asarray(raw, dtype=np.float64).reshape((-1, 3)) @ rotation.T) + offset

        start = time.perf_counter()
        result = pointcloud_grasp_geometry(points)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        self.samples.append(elapsed_ms)
        payload = {
            "sample": len(self.samples),
            "points": int(points.shape[0]),
            "ms": round(elapsed_ms, 3),
            "reachable": bool(result and result.reachable),
            "grasp_base": result.grasp if result else None,
            "extent": result.extent if result else None,
        }
        self.get_logger().info(json.dumps(payload, ensure_ascii=False, sort_keys=True))


def main() -> None:
    parser = argparse.ArgumentParser(description="Dry timing for real-equivalent ROI cloud -> grasp pose.")
    parser.add_argument("--cloud-topic", default="/arachne/grasp_preview/roi_cloud")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = GraspPoseDryTiming(args)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.samples:
            avg = sum(node.samples) / len(node.samples)
            node.get_logger().info(
                f"dry timing summary: samples={len(node.samples)} avg={avg:.3f}ms max={max(node.samples):.3f}ms"
            )
        else:
            node.get_logger().warning("dry timing summary: no ROI cloud samples")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
