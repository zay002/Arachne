# Stage 5：Godot 展示前端

## 总结

Arachne 现在有一个独立的 Godot 4.x 高帧率展示前端，用于作品集和宣传 demo。Gazebo 仍作为物理验证后端，而 Godot 专注于平地办公室地图中的流畅第三人称交互：比例 Scout 驾驶、可碰撞运动、可推动刚体道具、按固定随机种子撒布的可拾取水瓶/小球、跟随相机阻尼、视觉悬挂、可选 CC0 家具道具、MS42DC 夹爪动画、Aubo i5 预设姿态插值和手动机械臂微调。

## 核心文件

- `godot/arachne_showcase/project.godot`：Godot 4.x 项目入口。
- `godot/arachne_showcase/scenes/arachne_showcase.tscn`：主场景，只有一个脚本化 root。
- `godot/arachne_showcase/scripts/arachne_showcase.gd`：构建场地、加载机器人 mesh、处理键盘/手柄驾驶、相机跟随、碰撞运动、车体运动效果、夹爪动画、机械臂预设、auto-pick demo 逻辑和 headless 自测。
- `godot/arachne_showcase/scripts/ros2_bridge_placeholder.gd`：无依赖的 `/cmd_vel`、`/joint_states`、`/odom`、`/tf` 内存/UDP 占位 bridge。
- `godot/arachne_showcase/tools/link_assets.sh`：把现有 Scout、Aubo i5、MS42DC、AG95 和道具 mesh 链接进 Godot 项目，并将 DAE 转换为本地 GLB 缓存。
- `scripts/install_godot4.sh`：安装固定版本 Godot 4.x Linux 二进制和 `assimp-utils`。
- `scripts/fetch_godot_assets.sh`：下载可选 Kenney Furniture Kit 办公室道具并导入 Godot 项目。
- `scripts/godot_gamepad_bridge.py`：WSL2/Switch Pro 输入到 Godot 的浏览器 Gamepad API 桥。
- `scripts/godot_showcase.sh`：仓库级启动脚本。
- `scripts/test_godot_showcase.sh`：headless 场景测试，检查资产加载、脚本驾驶、相机距离、mesh 数量和 bridge 消息流。

## 说明

Godot demo 使用可碰撞 character body 和调过手感的 arcade 物理，以获得响应快的第三人称体验。它不替代 Gazebo 接触仿真，但已经有足够的平地驾驶、障碍交互和宣传录制反馈。后续 MuJoCo 或 ROS2 bridge 可以接在占位接口后面，不需要替换场景。

WSL2 当前通过启动脚本使用 Mesa D3D12 OpenGL，因为 Godot 的 Vulkan 路径可能选择 CPU `llvmpipe`。启动脚本还会打开本地浏览器手柄桥；对于连接在 Windows 侧的 Switch Pro 手柄，这条路径更可靠。轮子动画由实测底盘位移和 yaw delta 驱动，因此碰撞和速度限制能保持视觉同步。D-pad 保留给机械臂微调，不参与底盘驾驶。额外 Scout 侧边条几何已经移除，使 Godot 机器人更接近 Gazebo/URDF 模型。视觉 mesh 链共享 URDF/Gazebo 源资产和安装常量，碰撞体为保持响应性而简化。

第一个研究式交互是 auto-pick 序列，可通过右摇杆长按、`P` 或浏览器桥按钮启动。它会寻找最近物体、进行简单避障底盘靠近、插值 Aubo 拾取姿态、闭合/抬起 MS42DC，并回到 home。该逻辑是 Godot 侧展示占位；之后的机器人实现应把同样意图交给 MoveIt2 和 ROS2 控制器。
