#!/usr/bin/env bash
# Arachne helper only; not a stable runtime entrypoint. Prefer ROS2 package entrypoints in README.md.
set -euo pipefail

VERSION="${GODOT_VERSION:-4.6.3-stable}"
ASSET="Godot_v${VERSION}_linux.x86_64.zip"
URL="https://github.com/godotengine/godot/releases/download/${VERSION}/${ASSET}"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

sudo apt-get update
sudo apt-get install -y curl unzip assimp-utils

curl -L --fail --retry 3 -o "${TMP_DIR}/godot4.zip" "${URL}"
unzip -q "${TMP_DIR}/godot4.zip" -d "${TMP_DIR}"

BIN="${TMP_DIR}/Godot_v${VERSION}_linux.x86_64"
if [[ ! -x "${BIN}" ]]; then
  chmod +x "${BIN}"
fi

sudo mkdir -p /opt/godot4
sudo install -m 0755 "${BIN}" /opt/godot4/godot4
sudo ln -sfn /opt/godot4/godot4 /usr/local/bin/godot4

godot4 --version
