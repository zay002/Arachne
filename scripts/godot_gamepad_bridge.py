#!/usr/bin/python3
"""Local browser Gamepad API bridge for the Godot showcase.

The browser can see a Switch Pro controller connected to Windows while Godot in
WSL2 often cannot. This tiny bridge serves a localhost page, receives browser
gamepad state over HTTP, and forwards it to Godot over UDP.
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Arachne Godot Gamepad Bridge</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; min-height: 100vh; display: grid; place-items: center; background: #121518; color: #f3f5f2; }
    main { width: min(760px, calc(100vw - 32px)); }
    h1 { margin: 0 0 8px; font-size: 30px; }
    p { margin: 8px 0; color: #b8c2bd; }
    code { color: #ffd27c; }
    .card { border: 1px solid #33403d; border-radius: 10px; padding: 20px; background: #1b211f; box-shadow: 0 18px 50px #0008; }
    .row { display: flex; gap: 10px; flex-wrap: wrap; margin-top: 16px; }
    button { border: 0; border-radius: 8px; padding: 10px 14px; color: #111; background: #ffd27c; font-weight: 700; cursor: pointer; }
    button.secondary { background: #9ecad8; }
    pre { min-height: 120px; overflow: auto; background: #0d100f; color: #dce6df; border-radius: 8px; padding: 12px; }
  </style>
</head>
<body>
<main class="card">
  <h1>Arachne Godot Gamepad Bridge</h1>
  <p>Press any Switch Pro button, then keep this page open while Godot is running.</p>
  <p>Left stick drives the Scout. Right stick orbits the camera. Hold right-stick press to run nearest-object auto pick. A closes, B opens, LB/RB select an arm joint, D-pad up/down nudges it.</p>
  <div class="row">
    <button onclick="sendAction('open')">Open</button>
    <button onclick="sendAction('close')">Close</button>
    <button class="secondary" onclick="sendAction('home')">Home</button>
    <button class="secondary" onclick="sendAction('ready')">Ready</button>
    <button class="secondary" onclick="sendAction('reach')">Reach</button>
    <button class="secondary" onclick="sendAction('grasp')">Grasp</button>
    <button class="secondary" onclick="sendAction('lift')">Lift</button>
    <button onclick="sendAction('auto_pick')">Auto Pick</button>
    <button onclick="sendAction('reset')">Reset</button>
  </div>
  <pre id="status">Waiting for a gamepad...</pre>
</main>
<script>
const statusEl = document.getElementById("status");
let lastPost = 0;

window.addEventListener("gamepadconnected", () => tick());

async function post(payload) {
  try {
    await fetch("/state", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(payload)
    });
  } catch (error) {
    statusEl.textContent = "Bridge POST failed: " + error;
  }
}

function sendAction(action) {
  post({action, timestamp: performance.now() / 1000});
}

function firstGamepad() {
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (const pad of pads) {
    if (pad) return pad;
  }
  return null;
}

function tick(now = performance.now()) {
  const pad = firstGamepad();
  if (pad && now - lastPost > 16) {
    const axes = Array.from(pad.axes);
    const buttons = Array.from(pad.buttons, b => b.value);
    post({id: pad.id, axes, buttons, timestamp: now / 1000});
    statusEl.textContent =
      `${pad.id}\\n` +
      `axes: ${axes.map(v => v.toFixed(3)).join(", ")}\\n` +
      `buttons: ${buttons.map(v => v.toFixed(2)).join(", ")}`;
    lastPost = now;
  } else if (!pad) {
    statusEl.textContent = "Waiting for a gamepad... Press any controller button.";
  }
  requestAnimationFrame(tick);
}

tick();
</script>
</body>
</html>
"""


class BridgeHandler(BaseHTTPRequestHandler):
    udp_target: tuple[str, int]
    udp_socket: socket.socket

    def do_GET(self) -> None:
        if self.path not in ("/", "/index.html"):
            self.send_error(404)
            return
        body = HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:
        if self.path != "/state":
            self.send_error(404)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = self.rfile.read(length)
            data = json.loads(payload.decode("utf-8"))
            data["bridge_time"] = time.time()
            encoded = json.dumps(data, separators=(",", ":")).encode("utf-8")
            self.udp_socket.sendto(encoded, self.udp_target)
            self.send_response(204)
            self.end_headers()
        except Exception as exc:  # noqa: BLE001 - tiny local debug server.
            self.send_error(400, str(exc))

    def log_message(self, format: str, *args: object) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8790)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=8791)
    args = parser.parse_args()

    BridgeHandler.udp_target = (args.udp_host, args.udp_port)
    BridgeHandler.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server = ThreadingHTTPServer((args.host, args.port), BridgeHandler)
    print(f"Godot gamepad bridge: http://{args.host}:{args.port} -> udp://{args.udp_host}:{args.udp_port}", flush=True)
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
