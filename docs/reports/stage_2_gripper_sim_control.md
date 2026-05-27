# Stage 2 Report: Gripper Simulation Control

## Result

MS42DC and AG95 can now open and close in RViz simulation through the same demo interface; the only model difference is the selected gripper. MS42DC uses user-created split CAD parts with a revolute left finger and a right finger that mimics it in the opposite direction. AG95 drives the vendor model's knuckle/finger joints. A small `Arachne Gripper` GUI provides Open/Close buttons for visual demos.

## Core Files

- `src/arachne_gripper/`: ROS2 Python package for gripper simulation utilities.
- `arachne_gripper/gripper_sim_controller.py`: publishes simulated joint states for `ms42dc` and `ag95`.
- `arachne_gripper/gripper_state_gui.py`: two-button GUI that calls the gripper open/close services.
- `arachne_gripper/joint_state_mux.py`: merges GUI/default joint states with gripper states and publishes the single `/joint_states` stream.
- `urdf/gripper/ms42dc.urdf.xacro`: loads the user-created MS42DC split meshes and defines left/right revolute finger joints.
- `launch/display.launch.py`: can merge gripper simulation joint states into the unified `/joint_states` stream when `with_gripper_sim:=true`.
- `scripts/test_gripper_sim.sh`: smoke test for MS42DC and AG95 simulated open/close behavior.

## Interfaces

- User-facing services: `/arachne/gripper/open`, `/arachne/gripper/close`
- State topics: `/arachne/gripper/joint_states`, `/arachne/default_joint_states`, `/arachne/gui_joint_states`, `/joint_states`

The MS42DC meshes are real, user-created split CAD parts. The hinge direction is now `0 0 -1`, with `0.6 rad` as the default close target and launch-time overrides through `gripper_closed_position` or `GRIPPER_CLOSED_POSITION` for future retuning.
