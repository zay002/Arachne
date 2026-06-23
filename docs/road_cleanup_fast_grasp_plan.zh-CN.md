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

## 已落地的保守改动

当前先保留 `grasp -> safe_mid -> basket_over` 三个在线关键点，但让 local IK 使用真实启动参数中的宽松容差，并把 road/teach 抓取入口的本地规划超时设为适合 Jetson Orin Nano 的 2 秒：

```text
--local-position-tolerance 0.045
--local-orientation-tolerance 0.50
--local-planning-timeout-sec 2.0
--local-ik-max-iterations 90
```

这样不会改变投放语义，也不会猜测 basket 关节姿态。

## 下一步参数

录制并确认 basket 分区关节姿态后，再切到只在线规划抓取点：

```text
--planning-key-waypoints grasp
--local-planning-timeout-sec 0.8
--max-grasp-orientation-candidates 1
--trajectory-max-duration 3
--real-fixed-post-grasp
--real-fixed-basket-joints <6 joints rad>
```

可选：

```text
--real-fixed-lift-joints <6 joints rad>
--real-fixed-search-joints <6 joints rad>
```

`--real-fixed-post-grasp` 缺少 `--real-fixed-basket-joints` 时会直接退出，不会把 home/search 姿态误当成 basket。

sim 中已经按这个方向对齐：`grasp` 做 IK，`safe_mid`、`basket_over`、`search_return` 走固定关节。当前默认：

- `basket_over`：tool0/法兰盘垂直朝下，位于 basket 中心上方 20cm。
- `safe_mid`：从 `basket_over` 沿 base `+x` 方向 36cm、`+z` 方向 12cm。
- `grasp`：使用固定最终抓取姿态；不要通过每次抓取的 yaw offset 额外旋转目标。

可用下面变量继续调姿态：

```bash
FIXED_SAFE_MID_JOINTS="-1.392228627,-0.587456810,1.402798238,0.420158124,1.570706911,0.178573568" \
FIXED_BASKET_OVER_JOINTS="-1.187131238,-0.087444694,2.606213310,1.122582998,1.570733434,0.383692391" \
FIXED_SEARCH_JOINTS="" \
./scripts/sim/urban_trash_sorting_demo.sh
```

完整流程验证必须使用 `SYNTHETIC_GRASP_BENCHMARK=false`：底盘直线巡航、相机扫描、目标锁定、ROI 点云、grasp IK、固定投放、继续巡航。`SYNTHETIC_GRASP_BENCHMARK=true` 只用于压力测试和轨迹可视化，不代表完整 demo 流程。

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

## 当前边界

当前还没有默认开启 `grasp-only + fixed basket`，因为真实 basket/分区关节姿态还没录制确认。
