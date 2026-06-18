# Road Cleanup 秒级抓取规划优化方案

目标：road cleanup demo 中，识别到垃圾后，抓取动作规划进入秒级响应，避免底盘等待过久。

## 当前状态

真机 road cleanup 常用启动参数里，抓取规划关键点为：

```text
grasp -> safe_mid -> basket_over
```

即实时规划同时覆盖“到抓取点”和“抓取后去篮筐投放”。这会把投放路径也放进在线规划，导致规划耗时偏长。

## 优化思路

只把实时规划留给最必要的一段：

```text
current arm pose -> grasp
```

抓到物体后的动作改为固定流程：

```text
close gripper -> fixed lift/safe pose -> fixed basket pose -> open gripper -> return/search pose
```

这样在线规划只解决“从当前搜索姿态抓到目标”，投放不再依赖实时 IK/碰撞搜索。

## 建议参数

第一阶段只改配置，不重构：

```text
--planning-key-waypoints grasp
--local-planning-timeout-sec 0.8
--max-grasp-orientation-candidates 1
--trajectory-max-duration 3
```

保留：

```text
--planner-backend local
--real-sdk-semantic-targets-only
```

## 预期收益

- 在线规划点数从 3 个降到 1 个。
- 规划失败面缩小，只需要验证抓取点可达。
- road cleanup 中“识别到物体 -> 机械臂开始抓取”的延迟有机会降到 1 秒级。

## 风险

- 投放路径不再根据当前障碍物动态避障。
- 固定投放姿态必须用真机实测确认，不适合自动泛化到新篮筐位置。
- 如果搜索姿态变化很大，固定 lift/basket 轨迹需要重新录制。

## 最小实施步骤

1. 录制或确认 3 个固定关节姿态：`lift_safe`、`basket_over`、`search_return`。
2. road cleanup 抓取配置先改为只规划 `grasp`。
3. 抓取成功后按固定关节姿态执行投放。
4. 用日志记录每次耗时：YOLO、depth/ROI、local IK、SDK moveJoint。
5. 若 `grasp` 仍超过 1 秒，再把 local planning timeout 降到 `0.5s`，失败时直接跳过该目标。

## 暂不实现

本文件只记录方案；当前不改 road cleanup/grasp server 执行逻辑。
