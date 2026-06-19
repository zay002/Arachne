# 开发流程

提交前最小检查：

```bash
python3 -m compileall src/arachne_hardware/arachne_hardware src/arachne_operator/arachne_operator scripts/vision
bash -n scripts/build/check_offline_regression.sh
bash -n scripts/operator/teach_panel.sh
bash -n scripts/hardware/real_bringup.sh
./scripts/build/check_workspace.sh
source scripts/env/arachne_env.sh && colcon build --base-paths src --packages-select arachne_hardware arachne_operator
./scripts/build/check_offline_regression.sh
```

可选离线 smoke：

```bash
./scripts/test/smoke_aubo_move_joint_dry_run.sh
./scripts/test/smoke_demo_orchestrator_offline.sh
```

不要在普通开发检查中发送真实 motion goal、启动 Visual Grasp 真机执行或 Road Cleanup 真机任务。

阶段性记录归档在 `docs/archive/2026-06-refactor/`，长期文档只维护当前事实。
