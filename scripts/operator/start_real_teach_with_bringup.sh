#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

cd "${ROOT_DIR}"

if [[ ! -f "${ROOT_DIR}/install/setup.bash" ]]; then
  echo "请先构建工作区：./scripts/build/build_workspace.sh" >&2
  exit 1
fi

# 让桌面一键启动也能拿到同样的运行环境
set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/scripts/env/arachne_env.sh"
source "${ROOT_DIR}/install/setup.bash"
set -u

cat <<'EOF'
开始一键启动（real bringup + teach panel）
请确认：急停可达、场地安全、无人员在机械臂附近。
EOF

read -r -p "确认已满足安全条件并允许启动真机？(YES/NO): " confirm
if [[ "${confirm}" != "YES" ]]; then
  echo "已取消启动。"
  exit 1
fi

# 保持默认：先停止旧会话，再启动 real_bringup，最后启动 teach panel
export ARACHNE_TEACH_STOP_EXISTING="${ARACHNE_TEACH_STOP_EXISTING:-true}"
export ARACHNE_TEACH_START_REAL_BRINGUP="${ARACHNE_TEACH_START_REAL_BRINGUP:-true}"

exec "${ROOT_DIR}/scripts/operator/teach_panel.sh" "$@"

