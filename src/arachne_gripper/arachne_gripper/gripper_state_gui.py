from __future__ import annotations

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class GripperStateGui(Node):
    def __init__(self) -> None:
        super().__init__("gripper_state_gui")
        self.declare_parameter("open_service", "/arachne/gripper/open")
        self.declare_parameter("close_service", "/arachne/gripper/close")

        open_service = self.get_parameter("open_service").get_parameter_value().string_value
        close_service = self.get_parameter("close_service").get_parameter_value().string_value

        self.open_client = self.create_client(Trigger, open_service)
        self.close_client = self.create_client(Trigger, close_service)

        self.root = tk.Tk()
        self.root.title("Arachne Gripper")
        self.root.geometry("240x118")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill=tk.BOTH, expand=True)

        buttons = ttk.Frame(main)
        buttons.pack(fill=tk.X)

        ttk.Button(buttons, text="Open", command=self._open).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(buttons, text="Close", command=self._close_gripper).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 0)
        )

        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(main, textvariable=self.status_var, anchor=tk.CENTER).pack(fill=tk.X, pady=(12, 0))

    def run(self) -> None:
        self.root.mainloop()

    def _open(self) -> None:
        self._call(self.open_client, "Opening")

    def _close_gripper(self) -> None:
        self._call(self.close_client, "Closing")

    def _call(self, client, label: str) -> None:
        if not client.service_is_ready() and not client.wait_for_service(timeout_sec=0.1):
            self.status_var.set("Service unavailable")
            return

        self.status_var.set(label)
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda done: self.root.after(0, self._finish_call, done))

    def _finish_call(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.status_var.set(f"Failed: {exc}")
            return

        if response.success:
            self.status_var.set(response.message)
        else:
            self.status_var.set("Command rejected")

    def _close(self) -> None:
        self.root.quit()
        self.root.destroy()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GripperStateGui()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
