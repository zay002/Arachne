# Agent Instructions

- Treat simulation as a rehearsal of the real robot workflow. Do not create sim-only shortcuts that bypass the real operation sequence.
- For road cleanup, simulation should mirror the real pipeline: teach/operator entrypoint, camera-first observation, YOLO/segmentation-style target acquisition, point-cloud/grasp planning, base stop/recovery, arm execution, and basket drop-off.
- When behavior diverges between simulation and hardware, prefer changing the simulation to match the real machine unless the user explicitly asks for a visualization-only mock.
