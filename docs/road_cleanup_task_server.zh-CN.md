# 道路垃圾巡检分拣真机流程

`road_cleanup_task_server` 是移动巡检分拣的真机任务入口。真机视觉识别不另起一套节点，直接复用现有 `grasp_server` 里的 YOLO-SEG + 点云 + 抓取 pipeline；后续只需要把 YOLO-SEG 权重替换成 TACO 训练权重。

## 示教器入口

在示教器 Home 页：

- `RoadSrv`：启动/停止道路巡检任务服务器。
- `Road Preflight`：检查底层抓取 primitive 是否可用。
- `Road Start`：开始直线往复巡检。
- `Road Stop`：停止巡检、底盘和当前抓取任务。

顶部快捷栏也提供 `Road` 和 `Road Stop`。

## 感知接口

`grasp_preview_pipeline.py` 会把现有 YOLO-SEG 的最佳检测发布到：

```text
/arachne/perception/taco_instances
```

消息类型为 `std_msgs/String` JSON，支持单实例或多实例。当前权重下 `label` 是现有垃圾类别；换成 TACO 权重后，同一个字段会自然变成 TACO 类别名。

```json
{
  "instances": [
    {
      "label": "Clear plastic bottle",
      "taco_class": "Clear plastic bottle",
      "confidence": 0.91,
      "bbox_xyxy": [120, 96, 220, 210],
      "has_mask": true,
      "mask_area_px": 4200
    }
  ]
}
```

`road_cleanup_task_server` 只关心类别和置信度来触发停车；具体 3D 抓取点、ROI 点云、MoveIt/SDK 执行仍由 `grasp_server` 计算。

## 运行逻辑

1. 示教器启动 `camera`、`grasp_server` 和 `cleanup_server`。
   - 示教器启动 `grasp_server` 后会保留轻量 `idle_preview`，供 road cleanup 行进中持续 YOLO 检测。
2. `Road Start` 调用 `/arachne/road_cleanup/start`。
3. 任务服务器调用 `/arachne/grasp_task/preflight` 做安全预检。
4. 任务先把机械臂移动到固定扫描姿态；成功后底盘以小步 `drive_relative` 前进，默认前进 1.0 m。
5. 行进中持续监听 `grasp_server` 发布的 YOLO-SEG 检测结果；发现置信度足够的垃圾后立刻调用 base stop。
6. 调用 `/arachne/grasp_task/start` 执行“视觉定位 -> 点云/ROI -> MoveIt/SDK -> 抓取 -> 投篮”。
7. 如果 YOLO 和点云正常但抓取规划失败或目标不可达，任务服务器会让底盘继续小步移动，清掉旧候选，并持续触发 grasp preview 重新搜索，等待新的 YOLO 检测事件，然后重新计算点云和抓取规划。
8. 抓取完成后恢复巡检；返程阶段直接倒车返回起点，不再扫描/抓取。

## 当前边界

- TACO 不需要单独任务节点；当前默认抓取权重是 `yolo_workspace/weights/yolo26n_seg_taco_best.pt`，也可以用 `ARACHNE_GRASP_YOLO_MODEL` 覆盖。
- Road Start 前会先通过 `/arachne/aubo/move_joint` 移动到扫描预置位，未到位不发车。
- `road_cleanup_task_server` 现在只做任务编排，不替代 `grasp_task_server` 的抓取策略。
- reach recovery 默认开启：规划失败/不可达时沿当前巡检方向小步补偿，默认最多 3 次，每次 0.10 m。

## 不可达兜底

当抓取服务返回规划失败、IK 不可达、轨迹不可用或超时一类失败，且真实机械臂还没有开始执行时，道路清扫任务不会立刻结束，而是：

1. 停止当前抓取任务。
2. 向 `/arachne/grasp_preview/restart_search` 发出重搜信号，避免继续使用旧锁定框/旧点云。
3. 底盘沿当前巡检方向移动 `reach_recovery_step_m`。
4. 丢弃旧候选目标。
5. 进入 `tracking` 状态，并周期性触发 grasp preview 重新搜索。
6. 等待 `reach_recovery_wait_detection_sec` 内的新 YOLO-SEG 检测。
7. 用新检测重新走“点云 ROI -> 抓取规划 -> 执行”。

默认参数：

```bash
reach_recovery_enabled:=true
reach_recovery_max_attempts:=3
reach_recovery_step_m:=0.10
reach_recovery_wait_detection_sec:=3.0
reach_recovery_continue_on_exhausted:=true
restart_search_topic:=/arachne/grasp_preview/restart_search
```

如果超过次数仍不可达，默认会记录 skip 并继续巡检，而不是让整条道路清扫任务失败。

当前版本的“追踪”是保守的视觉重捕获：持续解除 YOLO 的锁定/暂停并等待新检测，底盘只做小步补偿，不自动扭动腕部。后续如果要加入腕部视觉伺服，需要接入单独的安全自动扫描接口，限制腕部速度、角度和碰撞边界。

## 权重替换

默认会加载：

```bash
yolo_workspace/weights/yolo26n_seg_taco_best.pt
```

如果需要临时覆盖，在启动示教器或 `grasp_server` 前设置：

```bash
export ARACHNE_GRASP_YOLO_MODEL=/path/to/taco_yolo_seg_best.pt
export ARACHNE_GRASP_CLASSES=""
```

如果 TACO 训练时做了类别合并，也可以把 `ARACHNE_GRASP_CLASSES` 设置成需要抓取的类别名或类别 id。

抓取链路默认拒绝自动下载官方 YOLO 权重；如果本地权重路径不存在，会直接报错。确认只是在调试通用权重时，才设置：

```bash
export ARACHNE_GRASP_ALLOW_MODEL_DOWNLOAD=true
```
