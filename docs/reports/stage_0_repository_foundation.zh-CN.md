# Stage 0 报告：仓库基础

## 结果

仓库已经成为一个 ROS2 workspace，包含第一个包、可复现环境安装脚本，以及硬件、建模、标定、控制和参考资料的文档占位。

## 核心文件

- `README.md`：当前系统概览和 out-of-box 使用路径。
- `scripts/setup_ubuntu.sh`：安装 ROS2 Humble 或 Jazzy 依赖。
- `scripts/check_model.sh`：生成 URDF，并在可用时运行 `check_urdf`。
- `docs/*.md`：硬件事实、建模策略、标定、控制和参考资料的简明说明。
- `third_party/README.md`：记录 vendor 文件应如何管理。

## 文件关系

根目录 README 是用户入口。脚本保证环境可复现。`docs` 记录硬件模型来自哪些上游仓库，以及未来实测硬件数据应保存在哪里。
