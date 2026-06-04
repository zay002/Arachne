# Calibration

The first calibration target is the fixed transform from Scout to Aubo:

```text
top_plate_link -> arm_mount_link -> aubo_base_link
```

The current default is approximate:

```text
arm_mount_xyz = 0.22 0.0 0.155
arm_mount_rpy = 0.0 0.0 1.57079632679
```

Update the launch arguments or `config/physical_parameters.yaml` after measuring the real mounting plate.

## MS42DC Finger Stroke

The Yizhua Robot MS42DC two-finger flexible servo gripper model uses the user-created split files from `third_party/MS42DC_SPLIT`. The current simulation uses:

```text
ms42dc_left_finger_joint:  0.0 to 1.0 rad, axis 0 0 -1
ms42dc_right_finger_joint: -1.0 to 0.0 rad, axis 0 0 -1, mimic left with multiplier -1.0
default simulated close target: 0.6 rad
```

After RViz inspection, the hinge axis was confirmed as the CAD Z axis, the visual closing direction was corrected with `0 0 -1`, and `0.6 rad` was selected as the current default close target. If the physical gripper or CAD split changes later, tune the close target with `gripper_closed_position:=...` or `GRIPPER_CLOSED_POSITION=... ./scripts/model/view_model.sh`.
