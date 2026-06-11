from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
import tf2_ros

try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
except Exception:  # pragma: no cover - optional runtime dependency
    PupilAprilTagDetector = None


def _rotation_matrix_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    m = matrix
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return (qx, qy, qz, qw)


def _quaternion_to_rotation_matrix(
    qx: float, qy: float, qz: float, qw: float
) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )


def _rotation_matrix_to_rpy(matrix: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    if sy > 1e-9:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        pitch = math.atan2(-matrix[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


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


def _transform_to_dict(transform: np.ndarray) -> dict[str, Any]:
    xyz = transform[:3, 3]
    rpy = _rotation_matrix_to_rpy(transform[:3, :3])
    quat = _rotation_matrix_to_quaternion(transform[:3, :3])
    return {
        "xyz": [float(v) for v in xyz],
        "rpy": [float(v) for v in rpy],
        "quaternion_xyzw": [float(v) for v in quat],
        "matrix": transform.tolist(),
    }


def _ros_transform_to_matrix(msg: Any) -> np.ndarray:
    translation = np.array(
        [
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ],
        dtype=np.float64,
    )
    rotation = _quaternion_to_rotation_matrix(
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w,
    )
    return _make_transform(rotation, translation)


@dataclass
class Detection:
    tag_id: int
    target_to_camera: np.ndarray
    reprojection_error_px: float


@dataclass
class Sample:
    stamp: str
    tag_id: int
    base_to_gripper: np.ndarray
    target_to_camera: np.ndarray
    reprojection_error_px: float


class AprilTagHandEyeCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("arachne_apriltag_hand_eye_calibrator")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gripper_frame", "gripper_adapter_link")
        self.declare_parameter(
            "board_image_path", "/home/jetson/zhaoyang/arachne_floor_apriltag_board_a3.png"
        )
        self.declare_parameter("board_width_m", 0.420)
        self.declare_parameter("board_height_m", 0.297)
        self.declare_parameter("min_board_matches", 24)
        self.declare_parameter("tag_family", "tagStandard41h12")
        self.declare_parameter("tag_size_m", 0.070)
        self.declare_parameter("tag_pitch_m", 0.100)
        self.declare_parameter("tag_id", -1)
        self.declare_parameter("dictionary", "DICT_APRILTAG_36h11")
        self.declare_parameter("enable_board_template_fallback", False)
        self.declare_parameter("min_target_distance_m", 0.05)
        self.declare_parameter("max_target_distance_m", 3.0)
        self.declare_parameter("max_reprojection_error_px", 8.0)
        self.declare_parameter("output_dir", "log/calibration/hand_eye")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.gripper_frame = str(self.get_parameter("gripper_frame").value)
        self.board_image_path = str(self.get_parameter("board_image_path").value)
        self.board_width_m = float(self.get_parameter("board_width_m").value)
        self.board_height_m = float(self.get_parameter("board_height_m").value)
        self.min_board_matches = int(self.get_parameter("min_board_matches").value)
        self.tag_family = str(self.get_parameter("tag_family").value).strip()
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.tag_pitch_m = float(self.get_parameter("tag_pitch_m").value)
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.enable_board_template_fallback = bool(
            self.get_parameter("enable_board_template_fallback").value
        )
        self.min_target_distance_m = float(self.get_parameter("min_target_distance_m").value)
        self.max_target_distance_m = float(self.get_parameter("max_target_distance_m").value)
        self.max_reprojection_error_px = float(
            self.get_parameter("max_reprojection_error_px").value
        )
        self.output_dir = self._resolve_output_dir(str(self.get_parameter("output_dir").value))

        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.latest_image: Image | None = None
        self.latest_detection: Detection | None = None
        self.samples: list[Sample] = []

        self.apriltag_detector: Any = None
        if self.tag_family:
            if PupilAprilTagDetector is None:
                raise RuntimeError(
                    "tagStandard41h12 requires pupil-apriltags. "
                    "Install it with: python3 -m pip install --user pupil-apriltags"
                )
            self.apriltag_detector = PupilAprilTagDetector(
                families=self.tag_family,
                nthreads=4,
                quad_decimate=1.0,
                quad_sigma=0.0,
                refine_edges=True,
                decode_sharpening=0.25,
            )
            self.dictionary = None
            self.detector = None
        else:
            self.dictionary = self._make_dictionary(str(self.get_parameter("dictionary").value))
            self.detector = self._make_detector(self.dictionary)
        self.orb = cv2.ORB_create(nfeatures=2500)
        self.board_gray: np.ndarray | None = None
        self.board_keypoints: Any = None
        self.board_descriptors: np.ndarray | None = None
        self.board_tag_object_corners: dict[int, np.ndarray] = {}
        self._load_board_template()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.camera_info_topic, self._on_camera_info, 10)
        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        self.create_service(Trigger, "~/capture", self._on_capture)
        self.create_service(Trigger, "~/solve", self._on_solve)
        self.create_service(Trigger, "~/reset", self._on_reset)

        self.get_logger().info(
            "AprilTag hand-eye calibrator ready: "
            f"image={self.image_topic} camera_info={self.camera_info_topic} "
            f"base={self.base_frame} gripper={self.gripper_frame} "
            f"tag_family={self.tag_family or 'opencv'} "
            f"tag_size={self.tag_size_m:.3f}m tag_pitch={self.tag_pitch_m:.3f}m "
            f"tag_id={self.tag_id} "
            f"dictionary={self.get_parameter('dictionary').value} "
            f"template_fallback={self.enable_board_template_fallback} "
            f"board={self.board_image_path}"
        )

    def _load_board_template(self) -> None:
        path = Path(self.board_image_path).expanduser()
        if not path.exists():
            self.get_logger().warn(f"board image not found, template fallback disabled: {path}")
            return
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            self.get_logger().warn(f"failed to read board image, template fallback disabled: {path}")
            return
        self.board_gray = image
        self._load_apriltag_board_reference(image)
        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        self.board_keypoints = keypoints
        self.board_descriptors = descriptors
        count = 0 if keypoints is None else len(keypoints)
        self.get_logger().info(
            f"loaded board template {path} size={image.shape[1]}x{image.shape[0]} "
            f"features={count} apriltag_ids={sorted(self.board_tag_object_corners)} "
            f"physical={self.board_width_m:.3f}x{self.board_height_m:.3f}m"
        )

    def _load_apriltag_board_reference(self, image: np.ndarray) -> None:
        if self.apriltag_detector is None:
            return
        detections = list(self.apriltag_detector.detect(image, estimate_tag_pose=False))
        if not detections:
            self.get_logger().warn(
                f"no {self.tag_family} tags detected in board reference; board PnP disabled"
            )
            return
        centers = np.asarray([det.center for det in detections], dtype=np.float64)
        y_sorted = sorted(float(value) for value in centers[:, 1])
        row_values: list[list[float]] = []
        for value in y_sorted:
            if not row_values or abs(value - float(np.mean(row_values[-1]))) > 0.35 * self._reference_pitch_px(centers):
                row_values.append([value])
            else:
                row_values[-1].append(value)
        row_centers = [float(np.mean(row)) for row in row_values]
        rows: list[list[Any]] = [[] for _ in row_centers]
        for det in detections:
            row_index = min(
                range(len(row_centers)), key=lambda index: abs(float(det.center[1]) - row_centers[index])
            )
            rows[row_index].append(det)
        half = self.tag_size_m * 0.5
        row_count = len(rows)
        loaded = 0
        for row_index, row in enumerate(rows):
            row.sort(key=lambda det: float(det.center[0]))
            col_count = len(row)
            for col_index, det in enumerate(row):
                center_x = (col_index - (col_count - 1) * 0.5) * self.tag_pitch_m
                center_y = (row_index - (row_count - 1) * 0.5) * self.tag_pitch_m
                self.board_tag_object_corners[int(det.tag_id)] = np.asarray(
                    [
                        [center_x - half, center_y + half, 0.0],
                        [center_x + half, center_y + half, 0.0],
                        [center_x + half, center_y - half, 0.0],
                        [center_x - half, center_y - half, 0.0],
                    ],
                    dtype=np.float64,
                )
                loaded += 1
        self.get_logger().info(
            f"loaded {loaded} {self.tag_family} board tags from reference "
            f"rows={row_count} pitch={self.tag_pitch_m:.3f}m tag={self.tag_size_m:.3f}m"
        )

    def _reference_pitch_px(self, centers: np.ndarray) -> float:
        if len(centers) < 2:
            return 100.0
        distances = []
        for index, point in enumerate(centers):
            delta = centers[index + 1 :] - point
            if len(delta):
                distances.extend(float(np.linalg.norm(value)) for value in delta)
        distances = [value for value in distances if value > 1.0]
        return min(distances) if distances else 100.0

    def _resolve_output_dir(self, value: str) -> Path:
        path = Path(value).expanduser()
        if not path.is_absolute():
            try:
                root = Path(get_package_share_directory("arachne_operator")).parents[3]
            except Exception:
                root = Path.cwd()
            path = root / path
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _make_dictionary(self, name: str) -> Any:
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("OpenCV was built without cv2.aruco; install opencv-contrib-python.")
        if not hasattr(cv2.aruco, name):
            raise RuntimeError(f"Unknown ArUco/AprilTag dictionary: {name}")
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def _make_detector(self, dictionary: Any) -> Any:
        if hasattr(cv2.aruco, "DetectorParameters"):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, "ArucoDetector"):
            return cv2.aruco.ArucoDetector(dictionary, parameters)
        return (dictionary, parameters)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if msg.d else np.zeros(5)

    def _on_image(self, msg: Image) -> None:
        self.latest_image = msg
        if self.camera_matrix is None:
            return
        try:
            image = self._image_to_bgr(msg)
            detection = self._detect_tag(image)
        except Exception as exc:
            self.latest_detection = None
            self.get_logger().warn(f"AprilTag detection skipped: {exc}", throttle_duration_sec=2.0)
            return
        self.latest_detection = detection

    def _image_to_bgr(self, msg: Image) -> np.ndarray:
        height = int(msg.height)
        width = int(msg.width)
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)
        step = int(msg.step)
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
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    def _detect_tag(self, image: np.ndarray) -> Detection | None:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.apriltag_detector is not None:
            detection = self._detect_apriltag_board(gray)
            if detection is not None:
                return detection
            if self.tag_id >= 0 or not self.enable_board_template_fallback:
                return None
        if hasattr(cv2.aruco, "ArucoDetector"):
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            dictionary, parameters = self.detector
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        if ids is None or len(ids) == 0:
            if self.tag_id >= 0 or not self.enable_board_template_fallback:
                return None
            return self._detect_board_template(gray)
        flat_ids = ids.reshape(-1)
        selected = 0
        if self.tag_id >= 0:
            matches = np.where(flat_ids == self.tag_id)[0]
            if len(matches) == 0:
                return None
            selected = int(matches[0])
        tag_id = int(flat_ids[selected])
        image_points = corners[selected].reshape(4, 2).astype(np.float64)
        object_points = self._tag_object_points()
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return None
        rotation, _ = cv2.Rodrigues(rvec)
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - image_points, axis=1)))
        distance = float(np.linalg.norm(tvec.reshape(3)))
        if not self._valid_detection_geometry(distance, error):
            return None
        return Detection(
            tag_id=tag_id,
            target_to_camera=_make_transform(rotation, tvec.reshape(3)),
            reprojection_error_px=error,
        )

    def _detect_apriltag_board(self, gray: np.ndarray) -> Detection | None:
        if self.apriltag_detector is None or not self.board_tag_object_corners:
            return None
        detections = list(self.apriltag_detector.detect(gray, estimate_tag_pose=False))
        object_points: list[np.ndarray] = []
        image_points: list[np.ndarray] = []
        used_ids = []
        for det in detections:
            tag_id = int(det.tag_id)
            if self.tag_id >= 0 and tag_id != self.tag_id:
                continue
            corners = self.board_tag_object_corners.get(tag_id)
            if corners is None:
                continue
            object_points.extend(corners)
            image_points.extend(np.asarray(det.corners, dtype=np.float64))
            used_ids.append(tag_id)
        if len(object_points) < 4:
            return None
        object_array = np.asarray(object_points, dtype=np.float64)
        image_array = np.asarray(image_points, dtype=np.float64)
        if len(object_array) == 4:
            ok, rvec, tvec = cv2.solvePnP(
                object_array,
                image_array,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            inlier_object = object_array
            inlier_image = image_array
        else:
            ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_array,
                image_array,
                self.camera_matrix,
                self.dist_coeffs,
                iterationsCount=100,
                reprojectionError=4.0,
                confidence=0.995,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not ok or inliers is None or len(inliers) < 4:
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
        rotation, _ = cv2.Rodrigues(rvec)
        projected, _ = cv2.projectPoints(
            inlier_object, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - inlier_image, axis=1)))
        distance = float(np.linalg.norm(tvec.reshape(3)))
        if not self._valid_detection_geometry(distance, error):
            return None
        tag_label = used_ids[0] if len(used_ids) == 1 else -1
        return Detection(
            tag_id=tag_label,
            target_to_camera=_make_transform(rotation, tvec.reshape(3)),
            reprojection_error_px=error,
        )

    def _detect_board_template(self, gray: np.ndarray) -> Detection | None:
        if self.board_gray is None or self.board_descriptors is None:
            return None
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        if descriptors is None or keypoints is None or len(keypoints) < self.min_board_matches:
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
        if len(good) < self.min_board_matches:
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
        if not ok or inliers is None or len(inliers) < self.min_board_matches:
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
        rotation, _ = cv2.Rodrigues(rvec)
        projected, _ = cv2.projectPoints(
            inlier_object, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reproj = projected.reshape(-1, 2)
        error = float(np.mean(np.linalg.norm(reproj - inlier_image, axis=1)))
        distance = float(np.linalg.norm(tvec.reshape(3)))
        if not self._valid_detection_geometry(distance, error):
            return None
        return Detection(
            tag_id=-1,
            target_to_camera=_make_transform(rotation, tvec.reshape(3)),
            reprojection_error_px=error,
        )

    def _valid_detection_geometry(self, distance_m: float, reprojection_error_px: float) -> bool:
        if not np.isfinite(distance_m) or not np.isfinite(reprojection_error_px):
            return False
        if distance_m < self.min_target_distance_m or distance_m > self.max_target_distance_m:
            return False
        if reprojection_error_px > self.max_reprojection_error_px:
            return False
        return True

    def _tag_object_points(self) -> np.ndarray:
        half = self.tag_size_m * 0.5
        return np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

    def _lookup_base_to_gripper(self) -> np.ndarray:
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.gripper_frame,
            Time(),
            timeout=Duration(seconds=0.5),
        )
        return _ros_transform_to_matrix(transform)

    def _on_capture(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        detection = self.latest_detection
        if detection is None:
            response.success = False
            response.message = "no AprilTag detection yet"
            return response
        try:
            base_to_gripper = self._lookup_base_to_gripper()
        except Exception as exc:
            response.success = False
            response.message = f"missing TF {self.base_frame}->{self.gripper_frame}: {exc}"
            return response
        stamp = datetime.now().isoformat(timespec="milliseconds")
        self.samples.append(
            Sample(
                stamp=stamp,
                tag_id=detection.tag_id,
                base_to_gripper=base_to_gripper,
                target_to_camera=detection.target_to_camera,
                reprojection_error_px=detection.reprojection_error_px,
            )
        )
        response.success = True
        response.message = (
            f"captured sample {len(self.samples)} tag={detection.tag_id} "
            f"reproj={detection.reprojection_error_px:.2f}px"
        )
        self.get_logger().info(response.message)
        return response

    def _on_reset(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.samples.clear()
        response.success = True
        response.message = "cleared hand-eye samples"
        return response

    def _on_solve(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if len(self.samples) < 5:
            response.success = False
            response.message = f"need at least 5 samples, have {len(self.samples)}"
            return response
        if not hasattr(cv2, "calibrateHandEye"):
            response.success = False
            response.message = "OpenCV calibrateHandEye is unavailable"
            return response

        r_gripper_to_base = []
        t_gripper_to_base = []
        r_target_to_camera = []
        t_target_to_camera = []
        for sample in self.samples:
            r_gripper_to_base.append(sample.base_to_gripper[:3, :3])
            t_gripper_to_base.append(sample.base_to_gripper[:3, 3])
            r_target_to_camera.append(sample.target_to_camera[:3, :3])
            t_target_to_camera.append(sample.target_to_camera[:3, 3])

        candidates = self._solve_hand_eye_candidates(
            r_gripper_to_base,
            t_gripper_to_base,
            r_target_to_camera,
            t_target_to_camera,
        )
        valid_candidates = [candidate for candidate in candidates if candidate.get("valid")]
        if not valid_candidates:
            response.success = False
            response.message = "hand-eye solve failed: no valid candidate"
            return response
        selected = min(valid_candidates, key=lambda item: float(item["consistency_score"]))
        camera_to_gripper = np.asarray(selected["camera_to_gripper_matrix"], dtype=np.float64)
        gripper_to_camera = np.asarray(selected["gripper_to_camera_matrix"], dtype=np.float64)
        result = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "base_frame": self.base_frame,
            "gripper_frame": self.gripper_frame,
            "camera_frame": self.latest_image.header.frame_id if self.latest_image else "",
            "board_image_path": self.board_image_path,
            "board_width_m": self.board_width_m,
            "board_height_m": self.board_height_m,
            "tag_size_m": self.tag_size_m,
            "tag_pitch_m": self.tag_pitch_m,
            "tag_id": self.tag_id,
            "tag_family": self.tag_family,
            "sample_count": len(self.samples),
            "mean_reprojection_error_px": float(
                np.mean([sample.reprojection_error_px for sample in self.samples])
            ),
            "selected_method": selected["method"],
            "hand_eye_candidates": candidates,
            "camera_to_gripper": _transform_to_dict(camera_to_gripper),
            "gripper_to_camera": _transform_to_dict(gripper_to_camera),
            "samples": [
                {
                    "stamp": sample.stamp,
                    "tag_id": sample.tag_id,
                    "reprojection_error_px": sample.reprojection_error_px,
                    "base_to_gripper": _transform_to_dict(sample.base_to_gripper),
                    "target_to_camera": _transform_to_dict(sample.target_to_camera),
                }
                for sample in self.samples
            ],
        }
        output_path = self.output_dir / f"hand_eye_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

        xyz = result["gripper_to_camera"]["xyz"]
        rpy = result["gripper_to_camera"]["rpy"]
        response.success = True
        response.message = (
            f"saved {output_path}; gripper_to_camera xyz="
            f"({xyz[0]:.6f},{xyz[1]:.6f},{xyz[2]:.6f}) rpy="
            f"({rpy[0]:.6f},{rpy[1]:.6f},{rpy[2]:.6f})"
        )
        self.get_logger().info(response.message)
        return response

    def _solve_hand_eye_candidates(
        self,
        r_gripper_to_base: list[np.ndarray],
        t_gripper_to_base: list[np.ndarray],
        r_target_to_camera: list[np.ndarray],
        t_target_to_camera: list[np.ndarray],
    ) -> list[dict[str, Any]]:
        methods = [
            ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
            ("PARK", cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
            ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
            ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]
        candidates: list[dict[str, Any]] = []
        base_to_gripper = [
            _make_transform(rotation, translation)
            for rotation, translation in zip(r_gripper_to_base, t_gripper_to_base)
        ]
        target_to_camera = [
            _make_transform(rotation, translation)
            for rotation, translation in zip(r_target_to_camera, t_target_to_camera)
        ]
        for name, method in methods:
            try:
                r_camera_to_gripper, t_camera_to_gripper = cv2.calibrateHandEye(
                    r_gripper_to_base,
                    t_gripper_to_base,
                    r_target_to_camera,
                    t_target_to_camera,
                    method=method,
                )
                camera_to_gripper = _make_transform(
                    np.asarray(r_camera_to_gripper, dtype=np.float64),
                    np.asarray(t_camera_to_gripper, dtype=np.float64).reshape(3),
                )
                gripper_to_camera = _invert_transform(camera_to_gripper)
                translation_norm = float(np.linalg.norm(gripper_to_camera[:3, 3]))
                det = float(np.linalg.det(camera_to_gripper[:3, :3]))
                consistency = self._hand_eye_consistency(
                    base_to_gripper, target_to_camera, gripper_to_camera
                )
                valid = (
                    np.all(np.isfinite(camera_to_gripper))
                    and 0.8 <= det <= 1.2
                    and 0.02 <= translation_norm <= 2.0
                    and np.isfinite(consistency["score"])
                )
                candidates.append(
                    {
                        "method": name,
                        "valid": bool(valid),
                        "determinant": det,
                        "translation_norm_m": translation_norm,
                        "consistency_score": float(consistency["score"]),
                        "consistency_translation_mean_m": float(
                            consistency["translation_mean_m"]
                        ),
                        "consistency_rotation_mean_rad": float(
                            consistency["rotation_mean_rad"]
                        ),
                        "camera_to_gripper": _transform_to_dict(camera_to_gripper),
                        "gripper_to_camera": _transform_to_dict(gripper_to_camera),
                        "camera_to_gripper_matrix": camera_to_gripper.tolist(),
                        "gripper_to_camera_matrix": gripper_to_camera.tolist(),
                    }
                )
            except Exception as exc:
                candidates.append(
                    {
                        "method": name,
                        "valid": False,
                        "error": str(exc),
                        "consistency_score": float("inf"),
                    }
                )
        return candidates

    def _hand_eye_consistency(
        self,
        base_to_gripper: list[np.ndarray],
        target_to_camera: list[np.ndarray],
        gripper_to_camera: np.ndarray,
    ) -> dict[str, float]:
        base_to_target = [
            base_to_gripper_sample @ gripper_to_camera @ _invert_transform(target_to_camera_sample)
            for base_to_gripper_sample, target_to_camera_sample in zip(
                base_to_gripper, target_to_camera
            )
        ]
        if len(base_to_target) < 2:
            return {
                "score": float("inf"),
                "translation_mean_m": float("inf"),
                "rotation_mean_rad": float("inf"),
            }
        reference = base_to_target[0]
        translation_errors = []
        rotation_errors = []
        for transform in base_to_target[1:]:
            delta = _invert_transform(reference) @ transform
            translation_errors.append(float(np.linalg.norm(delta[:3, 3])))
            rotation_errors.append(self._rotation_angle(delta[:3, :3]))
        translation_mean = float(np.mean(translation_errors))
        rotation_mean = float(np.mean(rotation_errors))
        return {
            "score": translation_mean + 0.25 * rotation_mean,
            "translation_mean_m": translation_mean,
            "rotation_mean_rad": rotation_mean,
        }

    def _rotation_angle(self, rotation: np.ndarray) -> float:
        value = (float(np.trace(rotation)) - 1.0) * 0.5
        return float(math.acos(min(1.0, max(-1.0, value))))


def main() -> None:
    rclpy.init()
    node = AprilTagHandEyeCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
