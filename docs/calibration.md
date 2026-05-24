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
