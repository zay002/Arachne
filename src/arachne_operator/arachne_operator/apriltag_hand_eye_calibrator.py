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
        self.declare_parameter("tag_size_m", 0.08)
        self.declare_parameter("tag_id", -1)
        self.declare_parameter("dictionary", "DICT_APRILTAG_36h11")
        self.declare_parameter("output_dir", "log/calibration/hand_eye")

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.gripper_frame = str(self.get_parameter("gripper_frame").value)
        self.board_image_path = str(self.get_parameter("board_image_path").value)
        self.board_width_m = float(self.get_parameter("board_width_m").value)
        self.board_height_m = float(self.get_parameter("board_height_m").value)
        self.min_board_matches = int(self.get_parameter("min_board_matches").value)
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.output_dir = self._resolve_output_dir(str(self.get_parameter("output_dir").value))

        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.latest_image: Image | None = None
        self.latest_detection: Detection | None = None
        self.samples: list[Sample] = []

        self.dictionary = self._make_dictionary(str(self.get_parameter("dictionary").value))
        self.detector = self._make_detector(self.dictionary)
        self.orb = cv2.ORB_create(nfeatures=2500)
        self.board_gray: np.ndarray | None = None
        self.board_keypoints: Any = None
        self.board_descriptors: np.ndarray | None = None
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
            f"tag_size={self.tag_size_m:.3f}m tag_id={self.tag_id} "
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
        keypoints, descriptors = self.orb.detectAndCompute(image, None)
        self.board_keypoints = keypoints
        self.board_descriptors = descriptors
        count = 0 if keypoints is None else len(keypoints)
        self.get_logger().info(
            f"loaded board template {path} size={image.shape[1]}x{image.shape[0]} "
            f"features={count} physical={self.board_width_m:.3f}x{self.board_height_m:.3f}m"
        )

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
        if hasattr(cv2.aruco, "ArucoDetector"):
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            dictionary, parameters = self.detector
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        if ids is None or len(ids) == 0:
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
        return Detection(
            tag_id=tag_id,
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
        return Detection(
            tag_id=-1,
            target_to_camera=_make_transform(rotation, tvec.reshape(3)),
            reprojection_error_px=error,
        )

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

        r_camera_to_gripper, t_camera_to_gripper = cv2.calibrateHandEye(
            r_gripper_to_base,
            t_gripper_to_base,
            r_target_to_camera,
            t_target_to_camera,
            method=cv2.CALIB_HAND_EYE_TSAI,
        )
        camera_to_gripper = _make_transform(
            np.asarray(r_camera_to_gripper, dtype=np.float64),
            np.asarray(t_camera_to_gripper, dtype=np.float64).reshape(3),
        )
        gripper_to_camera = _invert_transform(camera_to_gripper)
        result = {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "base_frame": self.base_frame,
            "gripper_frame": self.gripper_frame,
            "camera_frame": self.latest_image.header.frame_id if self.latest_image else "",
            "board_image_path": self.board_image_path,
            "board_width_m": self.board_width_m,
            "board_height_m": self.board_height_m,
            "tag_size_m": self.tag_size_m,
            "tag_id": self.tag_id,
            "sample_count": len(self.samples),
            "mean_reprojection_error_px": float(
                np.mean([sample.reprojection_error_px for sample in self.samples])
            ),
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
