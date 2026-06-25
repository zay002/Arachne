#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class RawImageViewer(Node):
    def __init__(self, topic: str, window: str, max_fps: float) -> None:
        super().__init__("arachne_raw_image_viewer")
        self.topic = topic
        self.window = window
        self.max_period = 1.0 / max(float(max_fps), 1.0)
        self.latest: Image | None = None
        self.latest_serial = 0
        self.displayed_serial = -1
        self.window_created = False
        self.frame_count = 0
        self.last_frame_time = 0.0
        self.create_subscription(Image, topic, self._image_cb, 5)
        self.get_logger().info(f"raw image viewer ready: {topic}")

    def _image_cb(self, msg: Image) -> None:
        self.latest = msg
        self.latest_serial += 1

    def spin_ui_once(self) -> bool:
        now = time.monotonic()
        if not self.window_created:
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            self.window_created = True
            self._show_waiting_image("waiting for image...")
        if (
            self.latest is not None
            and self.latest_serial != self.displayed_serial
            and now - self.last_frame_time >= self.max_period
        ):
            try:
                image = self._decode(self.latest)
                cv2.imshow(self.window, image)
                self.displayed_serial = self.latest_serial
                self.frame_count += 1
                if self.frame_count == 1:
                    self.get_logger().info(
                        f"first frame displayed: {image.shape[1]}x{image.shape[0]}"
                    )
                self.last_frame_time = now
            except Exception as exc:
                self.get_logger().warn(f"failed to display image: {exc}", throttle_duration_sec=2.0)
        elif self.frame_count == 0 and now - self.last_frame_time >= 1.0:
            self._show_waiting_image(f"waiting for {self.topic}")
            self.last_frame_time = now
        key = cv2.waitKey(10) & 0xFF
        if key in (ord("q"), 27):
            return False
        return True

    def _show_waiting_image(self, text: str) -> None:
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(
            image,
            text,
            (32, 185),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (80, 220, 120),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(self.window, image)

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
    parser.add_argument("--max-fps", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = RawImageViewer(args.topic, args.window, args.max_fps)
    try:
        while rclpy.ok() and node.spin_ui_once():
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
