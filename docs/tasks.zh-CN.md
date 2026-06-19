# 任务服务

## Visual Grasp

入口：

```bash
./scripts/vision/grasp_task_server.sh
```

`grasp_task_server` 负责 YOLO/点云候选、抓取规划、夹具控制和 Aubo action/SDK 执行调度。

## Road Cleanup

入口：

```bash
./scripts/vision/road_cleanup_task_server.sh
```

`road_cleanup_task_server` 负责巡检、停车、调用 `/arachne/grasp_task/start`、等待抓取完成和恢复行进。识别到可达目标时，抓取和行进应该阻塞协调，避免底盘在抓取规划期间继续移动。

## Demo Orchestrator

`demo_orchestrator` 暴露 `/arachne/demo/*` 服务，用于统一状态、preflight、camera、visual grasp、road cleanup 和 stop。

离线 smoke 只调用 status/preflight，不启动真实任务。
