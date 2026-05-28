from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class OperatorPanel(Node):
    def __init__(self) -> None:
        super().__init__("arachne_operator_panel")
        self.state = {
            "Safety": "waiting",
            "Base": "waiting",
            "Aubo": "waiting",
            "Gripper": "waiting",
            "Odom": "waiting",
        }
        self.gripper_pub = self.create_publisher(String, "/arachne/gripper/command", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/arachne/safety/state", self._set("Safety"), 10)
        self.create_subscription(String, "/arachne/hardware/base_status", self._set("Base"), 10)
        self.create_subscription(String, "/arachne/hardware/aubo_status", self._set("Aubo"), 10)
        self.create_subscription(String, "/arachne/hardware/gripper_status", self._set("Gripper"), 10)
        self.create_subscription(Odometry, "/odom", self._odom, 10)

    def _set(self, key: str):
        def callback(msg: String) -> None:
            self.state[key] = msg.data

        return callback

    def _odom(self, msg: Odometry) -> None:
        self.state["Odom"] = (
            f"x={msg.pose.pose.position.x:.2f} y={msg.pose.pose.position.y:.2f} "
            f"vx={msg.twist.twist.linear.x:.2f} wz={msg.twist.twist.angular.z:.2f}"
        )

    def publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)

    def stop_base(self) -> None:
        self.stop_pub.publish(Twist())

    def call_trigger(self, service_name: str) -> None:
        client = self.create_client(Trigger, service_name)
        if client.wait_for_service(timeout_sec=0.2):
            client.call_async(Trigger.Request())
        else:
            self.get_logger().warning(f"service unavailable: {service_name}")


class PanelApp:
    def __init__(self, node: OperatorPanel) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("Arachne Operator")
        self.labels: dict[str, tk.StringVar] = {}
        self._build()
        self._refresh()

    def _build(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(row=0, column=0, sticky="nsew")
        for row, key in enumerate(self.node.state):
            ttk.Label(frame, text=key, width=10).grid(row=row, column=0, sticky="w", pady=3)
            value = tk.StringVar(value="waiting")
            self.labels[key] = value
            ttk.Label(frame, textvariable=value, width=72).grid(row=row, column=1, sticky="w", pady=3)

        controls = ttk.Frame(frame)
        controls.grid(row=len(self.node.state), column=0, columnspan=2, sticky="ew", pady=(12, 0))
        buttons = [
            ("Enable", lambda: self.node.call_trigger("/arachne/safety/enable")),
            ("Manual", lambda: self.node.call_trigger("/arachne/safety/set_manual")),
            ("Auto", lambda: self.node.call_trigger("/arachne/safety/set_autonomous")),
            ("Disable", lambda: self.node.call_trigger("/arachne/safety/disable")),
            ("E-Stop", lambda: self.node.call_trigger("/arachne/safety/estop")),
            ("Recover", lambda: self.node.call_trigger("/arachne/safety/recover")),
            ("Stop Base", self.node.stop_base),
            ("Open", lambda: self.node.publish_gripper("open")),
            ("Close", lambda: self.node.publish_gripper("close")),
        ]
        for index, (label, command) in enumerate(buttons):
            ttk.Button(controls, text=label, command=command).grid(
                row=index // 5, column=index % 5, padx=3, pady=3, sticky="ew"
            )

    def _refresh(self) -> None:
        for key, value in self.node.state.items():
            self.labels[key].set(value)
        self.root.after(100, self._refresh)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = OperatorPanel()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        PanelApp(node).run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
