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
from std_srvs.srv import Trigger


class AprilTagHandEyeInteractive(Node):
    def __init__(self, image_topic: str, calibrator: str, window: str, max_fps: float) -> None:
        super().__init__("arachne_apriltag_hand_eye_interactive")
        calibrator = calibrator.rstrip("/")
        self.image_topic = image_topic
        self.window = window
        self.max_period = 1.0 / max(float(max_fps), 1.0)
        self.latest: Image | None = None
        self.latest_serial = 0
        self.displayed_serial = -1
        self.frame_count = 0
        self.sample_count = 0
        self.window_created = False
        self.last_frame_time = 0.0
        self.status = "SPACE capture | s solve | r reset | q quit"
        self.status_until = 0.0

        self.capture_client = self.create_client(Trigger, f"{calibrator}/capture")
        self.solve_client = self.create_client(Trigger, f"{calibrator}/solve")
        self.reset_client = self.create_client(Trigger, f"{calibrator}/reset")
        self.create_subscription(Image, image_topic, self._image_cb, 5)
        self.get_logger().info(
            f"interactive hand-eye viewer ready: image={image_topic} calibrator={calibrator}"
        )

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
                self._draw_overlay(image)
                cv2.imshow(self.window, image)
                self.displayed_serial = self.latest_serial
                self.frame_count += 1
                self.last_frame_time = now
            except Exception as exc:
                self._set_status(f"display failed: {exc}", error=True)
        elif self.frame_count == 0 and now - self.last_frame_time >= 1.0:
            self._show_waiting_image(f"waiting for {self.image_topic}")
            self.last_frame_time = now

        key = cv2.waitKey(10) & 0xFF
        if key in (ord("q"), 27):
            return False
        if key == ord(" "):
            self._call_trigger(self.capture_client, "capture")
        elif key == ord("s"):
            self._call_trigger(self.solve_client, "solve")
        elif key == ord("r"):
            self._call_trigger(self.reset_client, "reset")
            self.sample_count = 0
        return True

    def _call_trigger(self, client: rclpy.client.Client, action: str) -> None:
        if not client.wait_for_service(timeout_sec=0.1):
            self._set_status(f"{action} service unavailable", error=True)
            return
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            cv2.waitKey(1)
        if not future.done():
            self._set_status(f"{action} timed out", error=True)
            return
        result = future.result()
        if result is None:
            self._set_status(f"{action} failed: no response", error=True)
            return
        if action == "capture" and result.success:
            self.sample_count += 1
        self._set_status(f"{action}: {result.message}", error=not result.success)
        if result.success:
            self.get_logger().info(f"{action}: {result.message}")
        else:
            self.get_logger().warn(f"{action}: {result.message}")

    def _set_status(self, text: str, *, error: bool = False) -> None:
        prefix = "ERROR " if error else ""
        self.status = f"{prefix}{text}"
        self.status_until = time.monotonic() + 6.0

    def _draw_overlay(self, image: np.ndarray) -> None:
        now = time.monotonic()
        if now > self.status_until:
            self.status = "SPACE capture | s solve | r reset | q quit"
        lines = [
            self.status,
            f"samples in this viewer: {self.sample_count}",
        ]
        pad = 10
        line_h = 24
        height = pad * 2 + line_h * len(lines)
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (image.shape[1], height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.48, image, 0.52, 0, image)
        color = (80, 230, 120) if not self.status.startswith("ERROR") else (80, 80, 255)
        for index, line in enumerate(lines):
            cv2.putText(
                image,
                line,
                (pad, pad + 17 + index * line_h),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                color if index == 0 else (220, 220, 220),
                2,
                cv2.LINE_AA,
            )

    def _show_waiting_image(self, text: str) -> None:
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        self._draw_overlay(image)
        cv2.putText(
            image,
            text,
            (32, 200),
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
            return image.copy()
        if encoding in ("bgra8", "rgba8"):
            image = data.reshape(height, step)[:, : width * 4].reshape(height, width, 4)
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(image, code)
        if encoding in ("mono8", "8uc1"):
            image = data.reshape(height, step)[:, :width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if encoding in ("mono16", "16uc1"):
            array = np.frombuffer(msg.data, dtype=np.uint16).reshape(height, step // 2)[:, :width]
            scale = 255.0 / max(float(array.max()), 1.0)
            return cv2.cvtColor(cv2.convertScaleAbs(array, alpha=scale), cv2.COLOR_GRAY2BGR)
        raise ValueError(f"unsupported encoding: {msg.encoding}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--calibrator", default="/arachne_apriltag_hand_eye_calibrator")
    parser.add_argument("--window", default="Arachne AprilTag Hand-Eye")
    parser.add_argument("--max-fps", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    node = AprilTagHandEyeInteractive(
        image_topic=args.image_topic,
        calibrator=args.calibrator,
        window=args.window,
        max_fps=args.max_fps,
    )
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
