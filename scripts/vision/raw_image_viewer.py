#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class RawImageViewer(Node):
    def __init__(self, topic: str, window: str) -> None:
        super().__init__("arachne_raw_image_viewer")
        self.topic = topic
        self.window = window
        self.latest: Image | None = None
        self.last_frame_time = 0.0
        self.create_subscription(Image, topic, self._image_cb, 5)
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"raw image viewer ready: {topic}")

    def _image_cb(self, msg: Image) -> None:
        self.latest = msg

    def spin_ui_once(self) -> bool:
        if self.latest is not None:
            try:
                image = self._decode(self.latest)
                cv2.imshow(self.window, image)
                self.last_frame_time = time.monotonic()
            except Exception as exc:
                self.get_logger().warn(f"failed to display image: {exc}", throttle_duration_sec=2.0)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            return False
        try:
            if cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                return False
        except cv2.error:
            return False
        return True

    def _decode(self, msg: Image) -> np.ndarray:
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if encoding in ("bgr8", "rgb8"):
            image = data.reshape(height, step)[:, : width * 3].reshape(height, width, 3)
            if encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image
        if encoding in ("bgra8", "rgba8"):
            image = data.reshape(height, step)[:, : width * 4].reshape(height, width, 4)
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(image, code)
        if encoding in ("mono8", "8uc1"):
            image = data.reshape(height, step)[:, :width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if encoding in ("mono16", "16uc1"):
            array = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, step // 2)[:, :width]
            scaled = cv2.convertScaleAbs(array, alpha=255.0 / max(float(array.max()), 1.0))
            return cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"unsupported encoding: {msg.encoding}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--window", default="Arachne Raw Camera")
    args = parser.parse_args()

    rclpy.init()
    node = RawImageViewer(args.topic, args.window)
    try:
        while rclpy.ok() and node.spin_ui_once():
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
