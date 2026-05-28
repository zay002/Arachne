# Arachne Godot 展示前端

这是 Arachne 的 Godot 4.x 高帧率第三人称展示前端。它被调成一个可玩的机器人 demo：平滑跟随相机、比例手柄驾驶、更大的平地办公室式初始地图、可碰撞移动、可推动道具、视觉悬挂、机械臂预设、手动机械臂微调、夹爪动画、按固定随机种子撒布的可拾取水瓶/小球，以及 ROS2 bridge 占位接口。

Gazebo 仍是接触更准确的物理验证后端。这个前端是作品集/游戏化展示层，在保持高性能的同时，为驾驶和障碍交互提供足够的物理手感。

## 运行

从仓库根目录执行：

```bash
./scripts/install_godot4.sh   # 如果已安装 godot4，可跳过
./scripts/fetch_third_party.sh
./scripts/fetch_godot_assets.sh   # 可选：CC0 办公家具道具
./scripts/godot_showcase.sh
```

如果 Godot 安装在其它路径或名称下：

```bash
GODOT_BIN=/path/to/Godot_v4.x ./scripts/godot_showcase.sh
```

启动器会在 `assets/vendor/` 下创建本地链接，并在 `assets/generated/` 下生成 GLB 缓存文件，使 Godot 能导入现有 Scout、Aubo i5、MS42DC、AG95 和道具 mesh，而不需要把大体积资产复制到项目中。机器人视觉 mesh 和安装变换与 URDF/Gazebo 模型共享同一来源值；碰撞体为了展示响应性而简化。

在 WSL2 中，`scripts/godot_showcase.sh` 会自动使用 Mesa D3D12 OpenGL：

```bash
GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA ./scripts/godot_showcase.sh
```

这样可以避免 Godot 的 Vulkan 路径退回 CPU `llvmpipe`。原生 Linux 默认保持 Godot 默认渲染器，除非手动覆盖。WSL2 下启动器还会在 `http://127.0.0.1:8790` 启动浏览器 Gamepad API 桥；当 Switch Pro 手柄连接在 Windows 侧而不是 Linux 侧时，这是推荐路径。

无窗口自测场景：

```bash
./scripts/test_godot_showcase.sh
```

## 控制

- `W/S` 或左摇杆 Y：比例前进/后退。
- `A/D` 或左摇杆 X：比例 skid-steer 转向。
- `Q/E` 或右摇杆：环绕跟随相机。如果手柄上报了非标准轴，设置 `ARACHNE_CAMERA_AXIS=2` 或 HUD 中显示的轴。
- `1` 到 `5`：机械臂预设 `home`、`ready`、`reach`、`grasp`、`lift`。
- `H/K` 或 `LB/RB`：选择要微调的 Aubo 关节。
- `U/J` 或 D-pad 上下：移动当前选中的 Aubo 关节。D-pad 不参与底盘驾驶。
- `Space`：切换夹爪。`C/A` 闭合，`O/B` 打开。
- 长按右摇杆按键、按 `P`，或点击浏览器桥中的 `Auto Pick`：运行最近物体 demo。当前版本会寻找可拾取目标、用轻量避障靠近目标、插值 Aubo 拾取姿态、闭合夹爪、抬起并回到 home。
- `R`：重置底盘、相机、机械臂和夹爪。

Auto-pick 路径是展示算法占位，不是完整 MoveIt2 规划器。它为宣传 demo 提供研究工作流，同时保持 Godot 无依赖和响应快速。

## ROS 2 Bridge 占位

`scripts/ros2_bridge_placeholder.gd` 为以下话题定义占位发布方法：

- `/cmd_vel`
- `/joint_states`
- `/odom`
- `/tf`

当前实现只在 Godot 内部保存和发出这些消息。它故意保持无依赖，使展示前端可以在原生 Linux、WSL2 或没有 source ROS 的工作站上流畅运行。后续 bridge 可以替换为 WebSocket、UDP 或 Godot 原生 ROS 2 绑定，同时保持前端场景契约不变。

如果检测到 `ROS_DISTRO`，占位 bridge 会自动切换到 UDP 模式。可手动覆盖：

```bash
ARACHNE_GODOT_BRIDGE=memory ./scripts/godot_showcase.sh
ARACHNE_GODOT_BRIDGE=udp ARACHNE_GODOT_UDP_PORT=8765 ./scripts/godot_showcase.sh
```
