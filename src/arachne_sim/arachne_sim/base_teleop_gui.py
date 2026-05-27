from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


class BaseTeleopGui(Node):
    def __init__(self) -> None:
        super().__init__("base_teleop_gui")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.25)
        self.declare_parameter("angular_speed", 0.65)
        self.declare_parameter("publish_rate", 10.0)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.linear = 0.0
        self.angular = 0.0
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._publish)

        self.root = tk.Tk()
        self.root.title("Arachne Base")
        self.root.geometry("260x174")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        controls = ttk.Frame(main)
        controls.pack()
        ttk.Button(controls, text="Forward", command=self._forward).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(controls, text="Left", command=self._left).grid(row=1, column=0, padx=4, pady=4)
        ttk.Button(controls, text="Stop", command=self._stop).grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(controls, text="Right", command=self._right).grid(row=1, column=2, padx=4, pady=4)
        ttk.Button(controls, text="Back", command=self._back).grid(row=2, column=1, padx=4, pady=4)

        self.status_var = tk.StringVar(value="Stopped")
        ttk.Label(main, textvariable=self.status_var, anchor=tk.CENTER).pack(fill=tk.X, pady=(8, 0))

    def run(self) -> None:
        self.root.mainloop()

    def _forward(self) -> None:
        self._set_command(self.linear_speed, 0.0, "Forward")

    def _back(self) -> None:
        self._set_command(-self.linear_speed, 0.0, "Back")

    def _left(self) -> None:
        self._set_command(0.0, self.angular_speed, "Left")

    def _right(self) -> None:
        self._set_command(0.0, -self.angular_speed, "Right")

    def _stop(self) -> None:
        self._set_command(0.0, 0.0, "Stopped")

    def _set_command(self, linear: float, angular: float, label: str) -> None:
        self.linear = linear
        self.angular = angular
        self.status_var.set(label)
        self._publish()

    def _publish(self) -> None:
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher.publish(msg)

    def _close(self) -> None:
        self.linear = 0.0
        self.angular = 0.0
        self._publish()
        self.root.quit()
        self.root.destroy()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BaseTeleopGui()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
