from __future__ import annotations

import errno
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Arachne Switch Pro Bridge</title>
  <style>
    :root { color-scheme: light dark; font-family: system-ui, sans-serif; }
    body { margin: 0; min-height: 100vh; display: grid; place-items: center; background: #101418; color: #f4f7f9; }
    main { width: min(680px, calc(100vw - 32px)); }
    h1 { margin: 0 0 10px; font-size: 28px; letter-spacing: 0; }
    p { line-height: 1.55; color: #c8d2dc; }
    code { color: #9bd3ff; }
    .panel { border: 1px solid #2b3540; border-radius: 8px; padding: 20px; background: #171d23; }
    .status { font-size: 18px; margin: 14px 0; color: #9dffbd; }
    .muted { color: #93a0ad; font-size: 14px; }
  </style>
</head>
<body>
<main>
  <h1>Arachne Switch Pro Bridge</h1>
  <div class="panel">
    <div class="status" id="status">Press any Switch Pro button...</div>
    <p>WSL2 cannot always read the Switch Pro Controller as <code>/dev/input/js0</code>. Keep this page open; it forwards the browser Gamepad API to ROS2 <code>/joy</code>.</p>
    <p>Left stick proportionally drives body-frame forward/back speed and turn speed; the Aubo arm side is the front. Right stick orbits the Gazebo camera. B/A open and close the gripper.</p>
    <p class="muted" id="detail">No controller detected yet.</p>
  </div>
</main>
<script>
const statusEl = document.getElementById("status");
const detailEl = document.getElementById("detail");
let lastSent = 0;

function dz(v, deadzone = 0.06) {
  return Math.abs(v) < deadzone ? 0 : v;
}

function normalizePad(gp) {
  const rawAxes = gp.axes || [];
  const rawButtons = gp.buttons || [];
  const buttons = Array.from({length: Math.max(17, rawButtons.length)}, (_, i) => {
    const button = rawButtons[i];
    return button && button.pressed ? 1 : 0;
  });
  const lx = dz(rawAxes[0] || 0);
  const ly = dz(rawAxes[1] || 0);
  const rx = dz(rawAxes[2] || 0);
  const ry = dz(rawAxes[3] || 0);
  return {
    id: gp.id,
    axes: [lx, -ly, rx, ry],
    buttons
  };
}

async function sendJoy(payload) {
  await fetch("/joy", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload)
  });
}

function tick(ts) {
  const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
  if (pads.length === 0) {
    statusEl.textContent = "Press any Switch Pro button...";
    detailEl.textContent = "No controller detected yet.";
    requestAnimationFrame(tick);
    return;
  }

  const pad = pads[0];
  const payload = normalizePad(pad);
  const pressedButtons = payload.buttons
    .map((value, index) => value ? index : null)
    .filter(value => value !== null);
  statusEl.textContent = "Connected: " + pad.id;
  detailEl.textContent = "axes " + payload.axes.map(v => v.toFixed(2)).join(", ")
    + " | buttons " + (pressedButtons.length ? pressedButtons.join(", ") : "none");

  if (ts - lastSent > 33) {
    lastSent = ts;
    sendJoy(payload).catch(() => {
      statusEl.textContent = "Bridge server is not reachable.";
    });
  }
  requestAnimationFrame(tick);
}

window.addEventListener("gamepadconnected", event => {
  statusEl.textContent = "Connected: " + event.gamepad.id;
});
requestAnimationFrame(tick);
</script>
</body>
</html>
"""


class WebGamepadBridge(Node):
    def __init__(self) -> None:
        super().__init__("web_gamepad_bridge")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8787)
        self.declare_parameter("joy_topic", "/joy")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        joy_topic = str(self.get_parameter("joy_topic").value)
        self.pub = self.create_publisher(Joy, joy_topic, 10)
        self.server = self._make_server()
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f"Web gamepad bridge ready: http://{self.host}:{self.port}")

    def _make_server(self) -> ThreadingHTTPServer:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:
                if self.path not in ("/", "/index.html"):
                    self.send_error(404)
                    return
                content = HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def do_POST(self) -> None:
                if self.path != "/joy":
                    self.send_error(404)
                    return
                length = int(self.headers.get("Content-Length", "0"))
                try:
                    payload = json.loads(self.rfile.read(length).decode("utf-8"))
                    node.publish_joy(payload)
                except (json.JSONDecodeError, UnicodeDecodeError, ValueError) as exc:
                    self.send_error(400, str(exc))
                    return
                self.send_response(204)
                self.end_headers()

            def log_message(self, _format: str, *_args) -> None:
                return

        first_port = self.port
        for candidate_port in range(first_port, first_port + 20):
            try:
                self.port = candidate_port
                return ReusableThreadingHTTPServer((self.host, self.port), Handler)
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE:
                    raise
        raise OSError(errno.EADDRINUSE, f"No free port in range {first_port}-{first_port + 19}")

    def publish_joy(self, payload: dict) -> None:
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(payload.get("id", "web_gamepad"))
        msg.axes = [float(value) for value in payload.get("axes", [])]
        msg.buttons = [int(value) for value in payload.get("buttons", [])]
        self.pub.publish(msg)

    def stop(self) -> None:
        self.server.shutdown()
        self.server.server_close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WebGamepadBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
