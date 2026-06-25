# 第三方资产

Arachne 在 `third_party/` 中保留可直接构建和演示所需的最小第三方集合；大型参考资料和完整素材包不进入 git，由脚本或来源链接下载。

## 随仓库保留

- `aubo_description`：Aubo 官方描述包元信息、完整 URDF/xacro 文本，以及 Aubo i5 系列 DAE/STL 运行网格，来源为 `AuboRobot/aubo_description`；这样桌面端和 Jetson 分支会共用官方 i5 尺寸与关节定义。
- `scout_ros2`：Scout 2.0 ROS2 描述、消息和 base 节点，来源为 `agilexrobotics/scout_ros2`。
- `ugv_sdk`：AgileX UGV SDK 源码和构建文件，不包含大型手册。
- `aubo_ros2_driver`：Aubo ROS2 driver，并保留 Arachne 当前真机安全启动流程所需补丁。
- `dh_ag95_gripper_ros2`：AG95 可选夹爪描述和 driver。
- `ms42dc_step_motor_ros2`：易爪机器人 MS42DC 厂家 ROS2 示例源码。
- `MS42DC.step` 和 `MS42DC_SPLIT/*.stl`：当前 MS42DC 可动模型来源；拆分 STL 由项目作者手动处理。

## 本地下载

- Aubo 非 i5 系列大型网格、UGV 大 PDF 手册、厂家视频/安装包、`kenney` Godot 素材包、`LARA_AUBOi5_AG95` 可选素材和 ROS1 `scout_ros` 不随仓库上传。
- 需要刷新完整上游时运行：

```bash
ARACHNE_REFRESH_THIRD_PARTY=true ./scripts/fetch_third_party.sh
```

- 需要 Godot 办公室素材时运行：

```bash
./scripts/fetch_godot_assets.sh
```

后续添加 CAD、STL、SDK 或说明书时，请记录来源、许可证、版本和 checksum；不能再分发的文件只记录获取方式。
