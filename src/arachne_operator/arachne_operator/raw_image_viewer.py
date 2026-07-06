#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import site
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class RawImageViewer(Node):
    def __init__(
        self,
        topic: str,
        window: str,
        max_fps: float,
        yolo_model: str = "",
        yolo_task: str = "detect",
        yolo_imgsz: int = 640,
        yolo_conf: float = 0.25,
        yolo_every: int = 5,
        yolo_device: str = "cpu",
        yolo_venv: str = "yolo_workspace/.venv",
    ) -> None:
        super().__init__("arachne_raw_image_viewer")
        self.topic = topic
        self.window = window
        self.max_period = 1.0 / max(float(max_fps), 1.0)
        self.yolo_model_path = Path(yolo_model).expanduser() if yolo_model else None
        self.yolo_task = yolo_task
        self.yolo_imgsz = int(yolo_imgsz)
        self.yolo_conf = float(yolo_conf)
        self.yolo_every = max(int(yolo_every), 1)
        self.yolo_device = yolo_device
        self.yolo_venv = Path(yolo_venv).expanduser()
        self.yolo = None
        self.yolo_error = ""
        self.yolo_frame_count = 0
        self.yolo_last_annotated: np.ndarray | None = None
        self.yolo_busy = False
        self.yolo_lock = threading.Lock()
        self.latest: Image | None = None
        self.latest_serial = 0
        self.displayed_serial = -1
        self.window_created = False
        self.frame_count = 0
        self.last_frame_time = 0.0
        self._init_yolo()
        self.create_subscription(Image, topic, self._image_cb, 5)
        self.get_logger().info(f"raw image viewer ready: {topic}")

    def _image_cb(self, msg: Image) -> None:
        self.latest = msg
        self.latest_serial += 1

    def _init_yolo(self) -> None:
        if self.yolo_model_path is None:
            return
        os.environ.setdefault("YOLO_AUTOINSTALL", "false")
        if not self.yolo_model_path.exists():
            self.yolo_error = f"YOLO model not found: {self.yolo_model_path}"
            self.get_logger().warn(self.yolo_error)
            return
        try:
            _add_venv_site(self.yolo_venv)
            from ultralytics import YOLO

            self.yolo = YOLO(str(self.yolo_model_path), task=self.yolo_task)
            self.get_logger().info(f"YOLO ready: {self.yolo_model_path}")
        except Exception as exc:
            self.yolo_error = f"YOLO unavailable: {exc}"
            self.get_logger().warn(self.yolo_error)

    def _annotate_yolo(self, image: np.ndarray) -> np.ndarray:
        if self.yolo is None:
            if self.yolo_error:
                self._put_status(image, self.yolo_error[:90])
            return image
        self.yolo_frame_count += 1
        if self.yolo_frame_count % self.yolo_every == 0:
            self._start_yolo_worker(image.copy())
        with self.yolo_lock:
            annotated = self.yolo_last_annotated
            busy = self.yolo_busy
            error = self.yolo_error
        if annotated is not None:
            return annotated
        if error:
            self._put_status(image, error[:90])
        elif busy:
            self._put_status(image, "YOLO running...")
        return image

    def _start_yolo_worker(self, image: np.ndarray) -> None:
        with self.yolo_lock:
            if self.yolo_busy:
                return
            self.yolo_busy = True
        threading.Thread(target=self._run_yolo, args=(image,), daemon=True).start()

    def _run_yolo(self, image: np.ndarray) -> None:
        try:
            annotated = self._predict_yolo(image)[0].plot()
            with self.yolo_lock:
                self.yolo_last_annotated = annotated
                self.yolo_error = ""
        except Exception as exc:
            message = f"YOLO failed: {exc}"
            with self.yolo_lock:
                self.yolo_error = message
            self.get_logger().warn(message, throttle_duration_sec=2.0)
        finally:
            with self.yolo_lock:
                self.yolo_busy = False

    def _predict_yolo(self, image: np.ndarray):
        try:
            return self.yolo.predict(
                image,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_conf,
                device=self.yolo_device,
                verbose=False,
            )
        except ValueError as exc:
            if "Invalid CUDA" not in str(exc) or self.yolo_device == "cpu":
                raise
            self.get_logger().warn("YOLO CUDA unavailable; falling back to CPU")
            self.yolo_device = "cpu"
            return self.yolo.predict(
                image,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_conf,
                device="cpu",
                verbose=False,
            )

    def _put_status(self, image: np.ndarray, text: str) -> None:
        cv2.putText(
            image,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

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
                image = self._annotate_yolo(image)
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


def _add_venv_site(venv: Path) -> None:
    if not venv.exists():
        return
    version = f"python{sys.version_info.major}.{sys.version_info.minor}"
    package_dirs = [
        Path.home() / ".local" / "lib" / version / "site-packages",
        Path("/usr/local/lib") / version / "dist-packages",
        Path("/usr/lib") / version / "dist-packages",
    ]
    for package_dir in package_dirs:
        if package_dir.exists() and str(package_dir) not in sys.path:
            site.addsitedir(str(package_dir))
    package_dir = venv / "lib" / version / "site-packages"
    if package_dir.exists() and str(package_dir) not in sys.path:
        sys.path.insert(0, str(package_dir))
        site.addsitedir(str(package_dir))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--window", default="Arachne Raw Camera")
    parser.add_argument("--max-fps", type=float, default=20.0)
    parser.add_argument("--yolo-model", default="")
    parser.add_argument("--yolo-task", default="detect")
    parser.add_argument("--yolo-imgsz", type=int, default=640)
    parser.add_argument("--yolo-conf", type=float, default=0.25)
    parser.add_argument("--yolo-every", type=int, default=5)
    parser.add_argument("--yolo-device", default="cpu")
    parser.add_argument("--yolo-venv", default="yolo_workspace/.venv")
    args = parser.parse_args()

    rclpy.init()
    node = RawImageViewer(
        args.topic,
        args.window,
        args.max_fps,
        args.yolo_model,
        args.yolo_task,
        args.yolo_imgsz,
        args.yolo_conf,
        args.yolo_every,
        args.yolo_device,
        args.yolo_venv,
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
