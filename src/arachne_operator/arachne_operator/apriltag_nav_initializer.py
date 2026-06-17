from __future__ import annotations

import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
except Exception:  # pragma: no cover - optional runtime dependency
    PupilAprilTagDetector = None


def _make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return transform


def _invert_transform(transform: np.ndarray) -> np.ndarray:
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def _rpy_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def _rotation_to_yaw(rotation: np.ndarray) -> float:
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _ros_transform_to_matrix(msg: Any) -> np.ndarray:
    t = msg.transform.translation
    q = msg.transform.rotation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        rotation = np.eye(3, dtype=np.float64)
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        rotation = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
    return _make_transform(rotation, np.array([t.x, t.y, t.z], dtype=np.float64))


def _parse_vector(value: str, count: int, label: str) -> list[float]:
    parts = [part.strip() for part in str(value).replace(";", ",").split(",") if part.strip()]
    if len(parts) != count:
        raise ValueError(f"{label} must contain {count} comma-separated numbers")
    return [float(part) for part in parts]


class AprilTagNavInitializer(Node):
    def __init__(self) -> None:
        super().__init__("arachne_apriltag_nav_initializer")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("tag_family", "tagStandard41h12")
        self.declare_parameter("tag_id", -1)
        self.declare_parameter("tag_size_m", 0.070)
        self.declare_parameter("board_image_path", "/home/jetson/zhaoyang/arachne_floor_apriltag_board_a3.png")
        self.declare_parameter("board_width_m", 0.420)
        self.declare_parameter("board_height_m", 0.297)
        self.declare_parameter("enable_template_fallback", True)
        self.declare_parameter("min_template_matches", 24)
        self.declare_parameter("tag_map_xyz", "0.0,0.0,1.2")
        self.declare_parameter("tag_map_rpy", "0.0,-1.57079632679,0.0")
        self.declare_parameter("base_params_file", "src/arachne_nav/config/nav2_params.yaml")
        self.declare_parameter("output_params_file", "/tmp/arachne_nav_apriltag_params.yaml")
        self.declare_parameter("output_pose_file", "/tmp/arachne_nav_apriltag_pose.json")
        self.declare_parameter("timeout_sec", 20.0)
        self.declare_parameter("max_reprojection_error_px", 6.0)
        self.declare_parameter("min_target_distance_m", 0.2)
        self.declare_parameter("max_target_distance_m", 6.0)
        self.declare_parameter("max_detection_age_sec", 0.5)
        self.declare_parameter("once", False)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.camera_frame_override = str(self.get_parameter("camera_frame").value).strip()
        self.tag_family = str(self.get_parameter("tag_family").value).strip()
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.board_image_path = str(self.get_parameter("board_image_path").value).strip()
        self.board_width_m = float(self.get_parameter("board_width_m").value)
        self.board_height_m = float(self.get_parameter("board_height_m").value)
        self.enable_template_fallback = bool(self.get_parameter("enable_template_fallback").value)
        self.min_template_matches = int(self.get_parameter("min_template_matches").value)
        self.tag_map_xyz = np.array(
            _parse_vector(str(self.get_parameter("tag_map_xyz").value), 3, "tag_map_xyz"),
            dtype=np.float64,
        )
        tag_map_rpy = _parse_vector(str(self.get_parameter("tag_map_rpy").value), 3, "tag_map_rpy")
        self.map_to_tag = _make_transform(_rpy_to_rotation(*tag_map_rpy), self.tag_map_xyz)
        self.base_params_file = self._resolve_path(str(self.get_parameter("base_params_file").value))
        self.output_params_file = Path(str(self.get_parameter("output_params_file").value)).expanduser()
        self.output_pose_file = Path(str(self.get_parameter("output_pose_file").value)).expanduser()
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").value)
        self.min_target_distance_m = float(self.get_parameter("min_target_distance_m").value)
        self.max_target_distance_m = float(self.get_parameter("max_target_distance_m").value)
        self.max_detection_age_sec = float(self.get_parameter("max_detection_age_sec").value)
        self.once = bool(self.get_parameter("once").value)

        self.detector: Any | None = None
        if PupilAprilTagDetector is None:
            self.get_logger().warn(
                "pupil-apriltags is not available; AprilTag decode disabled, using template fallback"
            )
        else:
            try:
                self.detector = PupilAprilTagDetector(
                    families=self.tag_family,
                    nthreads=4,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=True,
                    decode_sharpening=0.25,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"AprilTag backend does not support family={self.tag_family!r}: {exc}; "
                    "using template fallback"
                )

        self.board_gray: np.ndarray | None = None
        self.board_keypoints: Any = None
        self.board_descriptors: Any = None
        self.orb = cv2.ORB_create(nfeatures=2500)
        if self.enable_template_fallback:
            self._load_board_template()
        if self.detector is None and self.board_gray is None:
            raise RuntimeError(
                f"No usable AprilTag/template detector. family={self.tag_family!r}; "
                f"board_image_path={self.board_image_path!r}. "
                "For tagStandard41h12 on this machine, provide the printed board image "
                "or install a detector backend that supports the Standard41h12 family."
            )

        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.latest_result: dict[str, Any] | None = None
        self.latest_result_time = 0.0
        self.done = False
        self.exit_code = 1
        self.started_at = time.monotonic()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.create_service(Trigger, "~/initialize", self._on_initialize)
        if self.once:
            self.create_timer(0.2, self._once_tick)

        self.get_logger().info(
            "AprilTag nav initializer ready: "
            f"family={self.tag_family} id={self.tag_id} tag_size={self.tag_size_m:.3f}m "
            f"image={self.image_topic} camera_info={self.camera_info_topic} "
            f"tag_map_xyz={self.tag_map_xyz.tolist()} tag_map_rpy={tag_map_rpy} "
            f"template_fallback={self.board_gray is not None}"
        )

    def _resolve_path(self, value: str) -> Path:
        path = Path(value).expanduser()
        if path.is_absolute():
            return path
        return Path.cwd() / path

    def _load_board_template(self) -> None:
        path = Path(self.board_image_path).expanduser()
        if not path.exists():
            self.get_logger().warn(f"board image not found, template fallback disabled: {path}")
            return
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().warn(f"failed to read board image, template fallback disabled: {path}")
            return
        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        if descriptors is None or keypoints is None or len(keypoints) < self.min_template_matches:
            count = 0 if keypoints is None else len(keypoints)
            self.get_logger().warn(
                f"board template has too few features, template fallback disabled: {path} features={count}"
            )
            return
        self.board_gray = image
        self.board_keypoints = keypoints
        self.board_descriptors = descriptors
        self.get_logger().info(
            f"loaded board template {path} size={image.shape[1]}x{image.shape[0]} "
            f"features={len(keypoints)} physical={self.board_width_m:.3f}x{self.board_height_m:.3f}m"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)

    def _on_image(self, msg: Image) -> None:
        if self.camera_matrix is None:
            return
        try:
            image = self._image_to_bgr(msg)
            result = self._detect(image, msg.header.frame_id)
            if result is not None:
                self.latest_result = result
                self.latest_result_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f"AprilTag nav detection skipped: {exc}", throttle_duration_sec=2.0)

    def _image_to_bgr(self, msg: Image) -> np.ndarray:
        height = int(msg.height)
        width = int(msg.width)
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        step = int(msg.step)
        if encoding in ("bgr8", "rgb8"):
            image = data.reshape(height, step)[:, : width * 3].reshape(height, width, 3)
            if encoding == "rgb8":
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image
        if encoding in ("bgra8", "rgba8"):
            image = data.reshape(height, step)[:, : width * 4].reshape(height, width, 4)
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(image, code)
        if encoding in ("mono8", "8uc1"):
            image = data.reshape(height, step)[:, :width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    def _detect(self, image: np.ndarray, image_frame: str) -> dict[str, Any] | None:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        fx = float(self.camera_matrix[0, 0])
        fy = float(self.camera_matrix[1, 1])
        cx = float(self.camera_matrix[0, 2])
        cy = float(self.camera_matrix[1, 2])
        detection = None
        if self.detector is not None:
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(fx, fy, cx, cy),
                tag_size=self.tag_size_m,
            )
            candidates = [det for det in detections if self.tag_id < 0 or int(det.tag_id) == self.tag_id]
            if candidates:
                det = min(candidates, key=lambda item: float(getattr(item, "pose_err", 0.0)))
                error = float(getattr(det, "pose_err", 0.0))
                if error <= self.max_reprojection_error_px:
                    detection = (
                        int(det.tag_id),
                        _make_transform(
                            np.asarray(det.pose_R, dtype=np.float64),
                            np.asarray(det.pose_t, dtype=np.float64).reshape(3),
                        ),
                        error,
                        "apriltag",
                    )
        if detection is None and self.enable_template_fallback:
            detection = self._detect_board_quad(image)
        if detection is None and self.enable_template_fallback:
            detection = self._detect_board_template(gray)
        if detection is None:
            return None
        detected_tag_id, camera_to_tag, error, method = detection
        camera_frame = self.camera_frame_override or image_frame
        if not camera_frame:
            raise RuntimeError("image header frame_id is empty; set camera_frame parameter")
        base_to_camera = self._lookup(self.base_frame, camera_frame)
        odom_to_base = self._lookup(self.odom_frame, self.base_frame)
        base_to_tag = base_to_camera @ camera_to_tag
        map_to_base = self.map_to_tag @ _invert_transform(base_to_tag)
        map_to_odom = map_to_base @ _invert_transform(odom_to_base)
        yaw = _rotation_to_yaw(map_to_base[:3, :3])
        return {
            "stamp": datetime.now().isoformat(timespec="milliseconds"),
            "tag_id": detected_tag_id,
            "method": method,
            "pose_error": error,
            "camera_frame": camera_frame,
            "map_start_pose": [
                float(map_to_base[0, 3]),
                float(map_to_base[1, 3]),
                float(yaw),
            ],
            "map_to_odom_xy_yaw": [
                float(map_to_odom[0, 3]),
                float(map_to_odom[1, 3]),
                float(_rotation_to_yaw(map_to_odom[:3, :3])),
            ],
            "base_to_tag_xyz": [float(value) for value in base_to_tag[:3, 3]],
            "tag_map_xyz": [float(value) for value in self.tag_map_xyz],
        }

    def _detect_board_quad(self, image: np.ndarray) -> tuple[int, np.ndarray, float, str] | None:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        saturation = hsv[:, :, 1]
        purple = cv2.inRange(hue, 115, 175)
        saturated = cv2.inRange(saturation, 45, 255)
        mask = cv2.bitwise_and(purple, saturated)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image_area = float(image.shape[0] * image.shape[1])
        expected_aspect = self.board_width_m / self.board_height_m
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 1500.0 or area > 0.5 * image_area:
                continue
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.025 * perimeter, True)
            if len(approx) != 4:
                continue
            points = approx.reshape(4, 2).astype(np.float64)
            _, _, width, height = cv2.boundingRect(points.astype(np.float32))
            if height <= 0 or width <= 0:
                continue
            aspect = float(width) / float(height)
            if aspect < 0.8 * expected_aspect or aspect > 1.25 * expected_aspect:
                continue
            score = area / (1.0 + abs(aspect - expected_aspect))
            candidates.append((score, points))
        if not candidates:
            return None
        _, image_points = max(candidates, key=lambda item: item[0])
        image_points = self._order_quad_points(image_points)
        half_w = self.board_width_m * 0.5
        half_h = self.board_height_m * 0.5
        object_points = np.asarray(
            [
                [-half_w, -half_h, 0.0],
                [half_w, -half_h, 0.0],
                [half_w, half_h, 0.0],
                [-half_w, half_h, 0.0],
            ],
            dtype=np.float64,
        )
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - image_points, axis=1)))
        if error > self.max_reprojection_error_px:
            return None
        distance = float(np.linalg.norm(tvec.reshape(3)))
        if (
            not np.isfinite(distance)
            or distance < self.min_target_distance_m
            or distance > self.max_target_distance_m
        ):
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        return (-1, _make_transform(rotation, tvec.reshape(3)), error, "board_quad")

    def _order_quad_points(self, points: np.ndarray) -> np.ndarray:
        ordered = np.zeros((4, 2), dtype=np.float64)
        sums = points.sum(axis=1)
        diffs = np.diff(points, axis=1).reshape(4)
        ordered[0] = points[int(np.argmin(sums))]
        ordered[2] = points[int(np.argmax(sums))]
        ordered[1] = points[int(np.argmin(diffs))]
        ordered[3] = points[int(np.argmax(diffs))]
        return ordered

    def _detect_board_template(self, gray: np.ndarray) -> tuple[int, np.ndarray, float, str] | None:
        if self.board_gray is None or self.board_descriptors is None:
            return None
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        if descriptors is None or keypoints is None or len(keypoints) < self.min_template_matches:
            return None
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        pairs = matcher.knnMatch(self.board_descriptors, descriptors, k=2)
        good = []
        for pair in pairs:
            if len(pair) != 2:
                continue
            first, second = pair
            if first.distance < 0.72 * second.distance:
                good.append(first)
        if len(good) < self.min_template_matches:
            return None
        object_points = []
        image_points = []
        board_h, board_w = self.board_gray.shape[:2]
        for match in good:
            u, v = self.board_keypoints[match.queryIdx].pt
            x = (u - board_w * 0.5) * self.board_width_m / board_w
            y = (v - board_h * 0.5) * self.board_height_m / board_h
            object_points.append((x, y, 0.0))
            image_points.append(keypoints[match.trainIdx].pt)
        object_array = np.asarray(object_points, dtype=np.float64)
        image_array = np.asarray(image_points, dtype=np.float64)
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_array,
            image_array,
            self.camera_matrix,
            self.dist_coeffs,
            iterationsCount=100,
            reprojectionError=5.0,
            confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok or inliers is None or len(inliers) < self.min_template_matches:
            return None
        inlier_object = object_array[inliers.reshape(-1)]
        inlier_image = image_array[inliers.reshape(-1)]
        ok, rvec, tvec = cv2.solvePnP(
            inlier_object,
            inlier_image,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None
        projected, _ = cv2.projectPoints(
            inlier_object, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - inlier_image, axis=1)))
        if error > self.max_reprojection_error_px:
            return None
        distance = float(np.linalg.norm(tvec.reshape(3)))
        if (
            not np.isfinite(distance)
            or distance < self.min_target_distance_m
            or distance > self.max_target_distance_m
        ):
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        return (-1, _make_transform(rotation, tvec.reshape(3)), error, "board_template")

    def _lookup(self, target: str, source: str) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            target,
            source,
            Time(),
            timeout=Duration(seconds=0.5),
        )
        return _ros_transform_to_matrix(transform)

    def _on_initialize(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        try:
            result = self._fresh_result()
            self._write_outputs(result)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = json.dumps(result, sort_keys=True)
        return response

    def _fresh_result(self) -> dict[str, Any]:
        if self.latest_result is None:
            raise RuntimeError("no AprilTag pose yet")
        age = time.monotonic() - self.latest_result_time
        if age > self.max_detection_age_sec:
            raise RuntimeError(f"AprilTag pose is stale: age={age:.2f}s")
        return dict(self.latest_result)

    def _write_outputs(self, result: dict[str, Any]) -> None:
        params = yaml.safe_load(self.base_params_file.read_text(encoding="utf-8"))
        if not isinstance(params, dict):
            params = {}
        slam_params = params.setdefault("slam_toolbox", {}).setdefault("ros__parameters", {})
        slam_params["map_start_pose"] = [float(value) for value in result["map_start_pose"]]
        amcl_params = params.setdefault("amcl", {}).setdefault("ros__parameters", {})
        initial_pose = amcl_params.setdefault("initial_pose", {})
        pose = result["map_start_pose"]
        initial_pose["x"] = float(pose[0])
        initial_pose["y"] = float(pose[1])
        initial_pose["z"] = 0.0
        initial_pose["yaw"] = float(pose[2])
        amcl_params["set_initial_pose"] = True
        amcl_params["always_reset_initial_pose"] = False
        self.output_params_file.parent.mkdir(parents=True, exist_ok=True)
        self.output_params_file.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
        self.output_pose_file.parent.mkdir(parents=True, exist_ok=True)
        pose_payload = {
            **result,
            "base_params_file": str(self.base_params_file),
            "output_params_file": str(self.output_params_file),
        }
        self.output_pose_file.write_text(
            json.dumps(pose_payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(
            "AprilTag map_start_pose written: "
            f"x={pose[0]:.3f} y={pose[1]:.3f} yaw={math.degrees(pose[2]):.1f}deg "
            f"params={self.output_params_file}"
        )

    def _once_tick(self) -> None:
        if self.done:
            return
        if time.monotonic() - self.started_at > self.timeout_sec:
            self.get_logger().error("AprilTag nav initialization timed out")
            self.done = True
            self.exit_code = 2
            return
        try:
            result = self._fresh_result()
            self._write_outputs(result)
        except Exception:
            return
        self.done = True
        self.exit_code = 0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AprilTagNavInitializer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.once and node.done:
                raise SystemExit(node.exit_code)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
