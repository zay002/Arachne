#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from aubo_msgs.srv import JsonRpc
import time
import threading
import json
import argparse


TEST_JSON = {
    "cls": "RobotState",
    "func": "getTcpPose",
    "params": "[]",
}


class JsonRpcTestClient(Node):
    def __init__(self, frequency_hz):
        super().__init__('jsonrpc_tester')

        self.cli = self.create_client(JsonRpc, 'jsonrpc_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for service jsonrpc_service...')

        self.frequency_hz = frequency_hz
        self.interval = 1.0 / frequency_hz

        # Reset counters
        self.total_calls = 0
        self.success_calls = 0
        self.failed_calls = 0
        self.latencies = []
        self.last_result = ""
        self.last_error = ""
        self.lock = threading.Lock()

        # 计时器：发送请求
        self.timer = self.create_timer(self.interval, self.send_request)
        # 每秒打印一次统计信息
        self.create_timer(self.interval, self.print_stats)

        self.get_logger().info(f"[PERF TEST] Starting test at {frequency_hz:.1f} Hz")

    def send_request(self):
        request = JsonRpc.Request()
        request.cls = TEST_JSON["cls"]
        request.func = TEST_JSON["func"]
        request.params = TEST_JSON["params"]

        start_time = time.time()
        future = self.cli.call_async(request)

        def callback(fut):
            end_time = time.time()
            latency = (end_time - start_time) * 1000.0  # ms

            with self.lock:
                self.total_calls += 1
                resp = fut.result() 

                if fut.result() is not None:
                    self.success_calls += 1
                    self.latencies.append(latency)
                    # Save real service response
                    self.last_result = resp.result
                    self.last_error = resp.error
                else:
                    self.failed_calls += 1

        future.add_done_callback(callback)

    def print_stats(self):
        with self.lock:
            if self.latencies:
                avg = sum(self.latencies) / len(self.latencies)
                mn = min(self.latencies)
                mx = max(self.latencies)
            else:
                avg = mn = mx = 0.0

            self.get_logger().info(
                f"Calls: {self.total_calls}, Success: {self.success_calls}, Failed: {self.failed_calls}, "
                f"Latency(ms) [Avg: {avg:.2f}, Min: {mn:.2f}, Max: {mx:.2f}], "
                f"LastResult: {self.last_result}, LastError: {self.last_error}"
            )


def main():
    parser = argparse.ArgumentParser(description="ROS2 JsonRpc Service Load Tester")
    parser.add_argument('--hz', type=float, default=200.0, help='Call frequency in Hz (default: 200)')
    args, unknown = parser.parse_known_args()

    rclpy.init(args=unknown)
    node = JsonRpcTestClient(frequency_hz=args.hz)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
