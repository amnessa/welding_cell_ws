#!/usr/bin/env python3

import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
import tf2_ros
from tf2_geometry_msgs import do_transform_point


@dataclass
class PlaneResult:
    normal: np.ndarray
    x_axis: np.ndarray
    y_axis: np.ndarray
    rectangle: List[np.ndarray]
    square: List[np.ndarray]
    orientation_xyzw: List[float]


class PlaneSolverNode(Node):
    def __init__(self) -> None:
        super().__init__("plane_solver_node")

        self.input_point_topic = self.declare_parameter("input_point_topic", "/red_ball/ground_truth").value
        self.source_frame = self.declare_parameter("source_frame", "world").value
        self.target_frame = self.declare_parameter("target_frame", "base_link").value
        self.output_file = self.declare_parameter("output_file", "/tmp/sand_drawer_plane.json").value
        self.square_scale = float(self.declare_parameter("square_scale", 0.8).value)
        self.auto_solve_on_fourth = bool(self.declare_parameter("auto_solve_on_fourth", True).value)
        self.min_point_separation = float(self.declare_parameter("min_point_separation", 0.01).value)
        self.tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 0.2).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_base_point: Optional[np.ndarray] = None
        self.latest_source_point: Optional[PointStamped] = None
        self.captured_points: List[np.ndarray] = []
        self.last_result: Optional[PlaneResult] = None

        self.point_sub = self.create_subscription(
            PointStamped,
            self.input_point_topic,
            self.point_callback,
            20,
        )

        self.capture_srv = self.create_service(Trigger, "~/capture_point", self.capture_callback)
        self.solve_srv = self.create_service(Trigger, "~/solve_plane", self.solve_callback)
        self.reset_srv = self.create_service(Trigger, "~/reset_points", self.reset_callback)
        self.save_srv = self.create_service(Trigger, "~/save_last", self.save_callback)

        self.get_logger().info(
            f"Plane solver ready. Listening on {self.input_point_topic}, transforming {self.source_frame} -> {self.target_frame}."
        )

    def point_callback(self, msg: PointStamped) -> None:
        self.latest_source_point = msg

        transformed = self.transform_to_target(msg)
        if transformed is None:
            return

        self.latest_base_point = np.array([transformed.point.x, transformed.point.y, transformed.point.z], dtype=float)

    def transform_to_target(self, msg: PointStamped) -> Optional[PointStamped]:
        source_frame = msg.header.frame_id if msg.header.frame_id else self.source_frame

        if source_frame == self.target_frame:
            return msg

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
            return do_transform_point(msg, transform)
        except Exception as exc:
            self.get_logger().warn(f"TF transform failed ({source_frame} -> {self.target_frame}): {exc}")
            return None

    def capture_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if self.latest_base_point is None:
            response.success = False
            response.message = "No valid point available yet. Check point topic and TF."
            return response

        if self.captured_points:
            dist = float(np.linalg.norm(self.latest_base_point - self.captured_points[-1]))
            if dist < self.min_point_separation:
                response.success = False
                response.message = f"Point too close to previous capture ({dist:.4f}m)."
                return response

        self.captured_points.append(self.latest_base_point.copy())
        count = len(self.captured_points)
        response.success = True
        response.message = f"Captured point {count}/4 in {self.target_frame}: {self.latest_base_point.tolist()}"
        self.get_logger().info(response.message)

        if self.auto_solve_on_fourth and count == 4:
            solved, msg = self.solve_and_save()
            response.message += f" | auto-solve: {msg}"
            response.success = solved

        return response

    def solve_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        solved, msg = self.solve_and_save()
        response.success = solved
        response.message = msg
        return response

    def save_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        if self.last_result is None:
            response.success = False
            response.message = "No solved plane to save yet."
            return response
        ok, msg = self.persist_result(self.last_result)
        response.success = ok
        response.message = msg
        return response

    def reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self.captured_points = []
        self.last_result = None
        response.success = True
        response.message = "Captured points and solved plane cleared."
        return response

    def solve_and_save(self) -> (bool, str):
        if len(self.captured_points) < 4:
            return False, f"Need 4 points, have {len(self.captured_points)}"

        try:
            result = self.solve_plane(self.captured_points[0], self.captured_points[1], self.captured_points[2], self.captured_points[3])
        except ValueError as exc:
            return False, str(exc)

        self.last_result = result
        return self.persist_result(result)

    def solve_plane(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray) -> PlaneResult:
        eps = 1e-8

        x_axis = p2 - p1
        x_norm = np.linalg.norm(x_axis)
        if x_norm < eps:
            raise ValueError("Point 1 and point 2 are too close to define plane X axis.")
        x_axis /= x_norm

        y_seed = p3 - p1
        y_axis = y_seed - np.dot(y_seed, x_axis) * x_axis
        y_norm = np.linalg.norm(y_axis)
        if y_norm < eps:
            raise ValueError("Point 3 is collinear with points 1-2. Cannot define plane.")
        y_axis /= y_norm

        normal = np.cross(x_axis, y_axis)
        n_norm = np.linalg.norm(normal)
        if n_norm < eps:
            raise ValueError("Failed to compute plane normal.")
        normal /= n_norm

        u4 = float(np.dot(p4 - p1, x_axis))
        v4 = float(np.dot(p4 - p1, y_axis))

        if abs(u4) < eps:
            u4 = x_norm
        if abs(v4) < eps:
            v4 = y_norm

        c1 = p1
        c2 = p1 + u4 * x_axis
        c4 = p1 + v4 * y_axis
        c3 = p1 + u4 * x_axis + v4 * y_axis
        rectangle = [c1, c2, c3, c4]

        width = abs(u4)
        height = abs(v4)
        side = min(width, height) * float(np.clip(self.square_scale, 0.05, 1.0))

        sx = 1.0 if u4 >= 0.0 else -1.0
        sy = 1.0 if v4 >= 0.0 else -1.0
        center = p1 + 0.5 * u4 * x_axis + 0.5 * v4 * y_axis
        half = side * 0.5

        s1 = center + (-sx * half) * x_axis + (-sy * half) * y_axis
        s2 = center + (sx * half) * x_axis + (-sy * half) * y_axis
        s3 = center + (sx * half) * x_axis + (sy * half) * y_axis
        s4 = center + (-sx * half) * x_axis + (sy * half) * y_axis
        square = [s1, s2, s3, s4, s1]

        quat = self.rotation_matrix_to_quaternion(np.column_stack([x_axis, y_axis, normal]))
        return PlaneResult(normal=normal, x_axis=x_axis, y_axis=y_axis, rectangle=rectangle, square=square, orientation_xyzw=quat)

    def persist_result(self, result: PlaneResult) -> (bool, str):
        payload = {
            "target_frame": self.target_frame,
            "source_topic": self.input_point_topic,
            "captured_points_base": [p.tolist() for p in self.captured_points],
            "plane": {
                "origin": self.captured_points[0].tolist(),
                "x_axis": result.x_axis.tolist(),
                "y_axis": result.y_axis.tolist(),
                "normal": result.normal.tolist(),
            },
            "rectangle_corners": [p.tolist() for p in result.rectangle],
            "square_trajectory": [
                {
                    "position": p.tolist(),
                    "orientation_xyzw": result.orientation_xyzw,
                }
                for p in result.square
            ],
        }

        try:
            out_dir = os.path.dirname(self.output_file)
            if out_dir:
                os.makedirs(out_dir, exist_ok=True)
            with open(self.output_file, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
            msg = f"Plane solved and saved to {self.output_file}"
            self.get_logger().info(msg)
            return True, msg
        except Exception as exc:
            return False, f"Failed writing {self.output_file}: {exc}"

    @staticmethod
    def rotation_matrix_to_quaternion(rot: np.ndarray) -> List[float]:
        m00, m01, m02 = rot[0, 0], rot[0, 1], rot[0, 2]
        m10, m11, m12 = rot[1, 0], rot[1, 1], rot[1, 2]
        m20, m21, m22 = rot[2, 0], rot[2, 1], rot[2, 2]

        trace = m00 + m11 + m22
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (m21 - m12) / s
            y = (m02 - m20) / s
            z = (m10 - m01) / s
        elif m00 > m11 and m00 > m22:
            s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
        elif m11 > m22:
            s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
        else:
            s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s

        q = np.array([x, y, z, w], dtype=float)
        q /= np.linalg.norm(q)
        return q.tolist()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlaneSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
