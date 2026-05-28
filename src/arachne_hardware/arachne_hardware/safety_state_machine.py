from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class SafetyStateMachine(Node):
    STATES = ("disabled", "manual", "autonomous", "estop", "fault")

    def __init__(self) -> None:
        super().__init__("arachne_safety_state_machine")
        self.state = "disabled"
        self.last_reason = "startup"
        self.state_pub = self.create_publisher(String, "/arachne/safety/state", 10)
        self.enabled_pub = self.create_publisher(Bool, "/arachne/safety/enabled", 10)

        self.create_service(Trigger, "/arachne/safety/enable", self._enable)
        self.create_service(Trigger, "/arachne/safety/disable", self._disable)
        self.create_service(Trigger, "/arachne/safety/estop", self._estop)
        self.create_service(Trigger, "/arachne/safety/recover", self._recover)
        self.create_service(Trigger, "/arachne/safety/set_manual", self._set_manual)
        self.create_service(Trigger, "/arachne/safety/set_autonomous", self._set_autonomous)
        self.create_timer(0.2, self._publish)
        self.get_logger().info("Safety state machine ready")

    def _set_state(self, state: str, reason: str) -> tuple[bool, str]:
        if state not in self.STATES:
            return False, f"invalid state: {state}"
        if self.state == "estop" and state not in ("disabled", "estop"):
            return False, "recover from estop to disabled before enabling motion"
        self.state = state
        self.last_reason = reason
        self.get_logger().info(f"safety state -> {state}: {reason}")
        return True, f"{state}: {reason}"

    def _enable(self, _request, response):
        response.success, response.message = self._set_state("manual", "enabled")
        return response

    def _disable(self, _request, response):
        response.success, response.message = self._set_state("disabled", "disabled")
        return response

    def _estop(self, _request, response):
        self.state = "estop"
        self.last_reason = "emergency stop"
        response.success = True
        response.message = "estop"
        self.get_logger().error("safety state -> estop")
        return response

    def _recover(self, _request, response):
        self.state = "disabled"
        self.last_reason = "recovered; enable manually"
        response.success = True
        response.message = self.last_reason
        self.get_logger().warning("safety recovered to disabled")
        return response

    def _set_manual(self, _request, response):
        response.success, response.message = self._set_state("manual", "manual mode")
        return response

    def _set_autonomous(self, _request, response):
        response.success, response.message = self._set_state("autonomous", "autonomous mode")
        return response

    def _publish(self) -> None:
        state_msg = String()
        state_msg.data = f"{self.state}:{self.last_reason}"
        self.state_pub.publish(state_msg)

        enabled_msg = Bool()
        enabled_msg.data = self.state in ("manual", "autonomous")
        self.enabled_pub.publish(enabled_msg)


def main() -> None:
    rclpy.init()
    node = SafetyStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
