extends Node3D

const ROS2_BRIDGE_SCRIPT = preload("res://scripts/ros2_bridge_placeholder.gd")

const ARM_JOINT_NAMES := [
	"aubo_shoulder_joint",
	"aubo_upperArm_joint",
	"aubo_foreArm_joint",
	"aubo_wrist1_joint",
	"aubo_wrist2_joint",
	"aubo_wrist3_joint",
]

const ARM_PRESETS := {
	"home": [1.664, 0.034, -1.324, 0.034, -1.732, 0.0],
	"ready": [1.45, -0.28, -1.10, 0.18, -1.34, 0.0],
	"reach": [1.18, -0.78, -0.92, 0.24, -1.02, 0.25],
	"grasp": [1.18, -0.92, -1.04, 0.18, -1.12, 0.25],
	"lift": [1.35, -0.46, -1.36, 0.28, -1.28, 0.0],
}

const MS42DC_LEFT_PIVOT := Vector3(0.013334103, 0.057683325, 0.061074070)
const MS42DC_RIGHT_PIVOT := Vector3(0.085334103, 0.057683325, 0.061074070)
const MS42DC_CAD_XYZ := Vector3(-0.049334103, 0.049874070, 0.016816675)
const MS42DC_CAD_RPY := Vector3(PI * 0.5, 0.0, 0.0)
const MS42DC_LEFT_VISUAL_XYZ := -MS42DC_LEFT_PIVOT
const MS42DC_RIGHT_VISUAL_XYZ := -MS42DC_RIGHT_PIVOT
const ROS_TO_GODOT_BASIS := Basis(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, -1.0), Vector3(0.0, 1.0, 0.0))

@export var max_linear_speed := 1.25
@export var max_angular_speed := 1.85
@export var input_deadzone := 0.12
@export var input_curve := 0.35
@export var camera_distance := 2.5
@export var camera_height := 1.35
@export var camera_orbit_speed := 1.8
@export var camera_follow_rate := 9.0
@export var joint_follow_rate := 6.0
@export var gripper_follow_rate := 8.0
@export var gripper_closed_position := 0.6
@export var ground_clearance := 0.015

var robot_root: Node3D
var visual_root: Node3D
var camera: Camera3D
var camera_yaw := 0.0
var robot_ground_y := 0.0
var current_linear := 0.0
var current_angular := 0.0
var current_pose_name := "home"
var current_joints := ARM_PRESETS["home"].duplicate()
var target_joints := ARM_PRESETS["home"].duplicate()
var gripper_position := 0.0
var gripper_target := 0.0
var joint_nodes: Array[Node3D] = []
var joint_rest_bases: Array[Basis] = []
var wheel_nodes: Array[Node3D] = []
var left_finger_joint: Node3D
var right_finger_joint: Node3D
var status_label: Label
var ros_bridge: Node
var materials := {}

func _ready() -> void:
	Engine.max_fps = 144
	_setup_materials()
	_build_world()
	_build_robot()
	_lift_robot_to_ground()
	_build_camera()
	_build_hud()
	ros_bridge = ROS2_BRIDGE_SCRIPT.new()
	ros_bridge.name = "ROS2BridgePlaceholder"
	add_child(ros_bridge)
	_select_pose("home")


func _physics_process(delta: float) -> void:
	_update_base(delta)
	_update_arm(delta)
	_update_gripper(delta)
	_update_camera(delta)
	_update_hud()
	_publish_ros_placeholders()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_1:
				_select_pose("home")
			KEY_2:
				_select_pose("ready")
			KEY_3:
				_select_pose("reach")
			KEY_4:
				_select_pose("grasp")
			KEY_5:
				_select_pose("lift")
			KEY_SPACE:
				_toggle_gripper()
			KEY_O:
				_set_gripper(false)
			KEY_C:
				_set_gripper(true)
			KEY_R:
				_reset_demo()
	if event is InputEventJoypadButton and event.pressed:
		match event.button_index:
			JOY_BUTTON_A:
				_select_pose("grasp")
				_set_gripper(true)
			JOY_BUTTON_B:
				_set_gripper(false)
			JOY_BUTTON_Y:
				_select_pose("ready")
			JOY_BUTTON_X:
				_select_pose("lift")
			JOY_BUTTON_START:
				_reset_demo()


func _setup_materials() -> void:
	materials["body"] = _material(Color(0.18, 0.20, 0.21), 0.65, 0.05)
	materials["dark"] = _material(Color(0.02, 0.025, 0.03), 0.8, 0.0)
	materials["arm"] = _material(Color(0.90, 0.92, 0.90), 0.5, 0.0)
	materials["accent"] = _material(Color(0.96, 0.63, 0.18), 0.4, 0.0)
	materials["ground"] = _material(Color(0.46, 0.51, 0.47), 0.9, 0.0)
	materials["obstacle"] = _material(Color(0.20, 0.27, 0.32), 0.72, 0.0)
	materials["zone"] = _material(Color(0.16, 0.42, 0.58, 0.35), 0.7, 0.0)


func _build_world() -> void:
	var world := WorldEnvironment.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color(0.69, 0.76, 0.82)
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color(0.72, 0.76, 0.80)
	environment.ambient_light_energy = 0.85
	world.environment = environment
	add_child(world)

	var sun := DirectionalLight3D.new()
	sun.name = "KeyLight"
	sun.rotation_degrees = Vector3(-52.0, -34.0, 0.0)
	sun.light_energy = 3.4
	sun.shadow_enabled = true
	add_child(sun)

	var fill := OmniLight3D.new()
	fill.name = "SoftFill"
	fill.position = Vector3(-3.5, 3.0, 4.0)
	fill.light_energy = 0.8
	add_child(fill)

	var ground := MeshInstance3D.new()
	ground.name = "MatteGround"
	var ground_mesh := BoxMesh.new()
	ground_mesh.size = Vector3(18.0, 0.04, 18.0)
	ground.mesh = ground_mesh
	ground.position = Vector3(0.0, -0.02, 0.0)
	ground.material_override = materials["ground"]
	add_child(ground)

	_add_grid_lines()
	_add_obstacles()


func _build_robot() -> void:
	robot_root = Node3D.new()
	robot_root.name = "ArachneRobot"
	add_child(robot_root)

	visual_root = Node3D.new()
	visual_root.name = "RobotVisualRoot"
	robot_root.add_child(visual_root)

	_build_scout_visual()
	_build_aubo_visual()


func _lift_robot_to_ground() -> void:
	var bounds := _node_visual_bounds(robot_root)
	if bounds.size == Vector3.ZERO:
		return
	robot_ground_y = robot_root.global_position.y - bounds.position.y + ground_clearance
	robot_root.global_position.y = robot_ground_y


func _build_scout_visual() -> void:
	var base := _load_visual(
		[
			"res://assets/generated/scout/base_link.glb",
			"res://assets/generated/scout/base_link_full.glb",
			"res://assets/vendor/scout/base_link.dae",
			"res://assets/vendor/scout/base_link_full.dae",
		],
		_box_mesh(Vector3(0.93, 0.38, 0.20)),
		materials["body"],
		Vector3.ONE
	)
	base.name = "ScoutChassis"
	visual_root.add_child(base)

	var wheel_positions := {
		"front_right": _ros_vec(Vector3(0.249, -0.2915, -0.0702)),
		"front_left": _ros_vec(Vector3(0.249, 0.2915, -0.0702)),
		"rear_left": _ros_vec(Vector3(-0.249, 0.2915, -0.0702)),
		"rear_right": _ros_vec(Vector3(-0.249, -0.2915, -0.0702)),
	}
	for wheel_name in wheel_positions.keys():
		var wheel_path := "res://assets/vendor/scout/wheel_type1.dae"
		var wheel_generated_path := "res://assets/generated/scout/wheel_type1.glb"
		if wheel_name in ["front_left", "rear_right"]:
			wheel_path = "res://assets/vendor/scout/wheel_type2.dae"
			wheel_generated_path = "res://assets/generated/scout/wheel_type2.glb"
		var wheel := _load_visual(
			[
				wheel_generated_path,
				"res://assets/generated/scout/wheel.glb",
				wheel_path,
				"res://assets/vendor/scout/wheel.dae",
			],
			_cylinder_mesh(0.165, 0.116),
			materials["dark"],
			Vector3.ONE
		)
		wheel.name = "%s_wheel" % wheel_name
		wheel.position = wheel_positions[wheel_name]
		wheel_nodes.append(wheel)
		visual_root.add_child(wheel)

	var deck := _box_visual("ArmDeck", _ros_size(Vector3(0.42, 0.30, 0.035)), materials["accent"])
	deck.position = _ros_vec(Vector3(0.22, 0.0, 0.135))
	visual_root.add_child(deck)


func _build_aubo_visual() -> void:
	var arm_mount := Node3D.new()
	arm_mount.name = "arm_mount_link"
	arm_mount.position = _ros_vec(Vector3(0.22, 0.0, 0.155))
	arm_mount.basis = _ros_rpy_basis(Vector3(0.0, 0.0, PI * 0.5))
	visual_root.add_child(arm_mount)

	var link0 := _load_visual(["res://assets/generated/aubo_i5/link0.glb", "res://assets/vendor/aubo_i5/link0.DAE"], _cylinder_mesh(0.10, 0.08), materials["arm"], Vector3.ONE)
	link0.name = "aubo_base_link"
	arm_mount.add_child(link0)

	var shoulder := _make_joint("aubo_shoulder_joint", Vector3(0.0, 0.0, 0.122), Vector3(0.0, 0.0, PI), arm_mount)
	var link1 := _load_visual(["res://assets/generated/aubo_i5/link1.glb", "res://assets/vendor/aubo_i5/link1.DAE"], _cylinder_mesh(0.10, 0.13), materials["arm"], Vector3.ONE)
	link1.name = "aubo_shoulder_Link"
	shoulder.add_child(link1)

	var upper := _make_joint("aubo_upperArm_joint", Vector3(0.0, 0.1215, 0.0), Vector3(-PI * 0.5, -PI * 0.5, 0.0), shoulder)
	var link2 := _load_visual(["res://assets/generated/aubo_i5/link2.glb", "res://assets/vendor/aubo_i5/link2.DAE"], _box_mesh(Vector3(0.42, 0.08, 0.08)), materials["arm"], Vector3.ONE)
	link2.name = "aubo_upperArm_Link"
	upper.add_child(link2)

	var fore := _make_joint("aubo_foreArm_joint", Vector3(0.408, 0.0, 0.0), Vector3(-PI, 0.0, 0.0), upper)
	var link3 := _load_visual(["res://assets/generated/aubo_i5/link3.glb", "res://assets/vendor/aubo_i5/link3.DAE"], _box_mesh(Vector3(0.38, 0.07, 0.07)), materials["arm"], Vector3.ONE)
	link3.name = "aubo_foreArm_Link"
	fore.add_child(link3)

	var wrist1 := _make_joint("aubo_wrist1_joint", Vector3(0.376, 0.0, 0.0), Vector3(PI, 0.0, PI * 0.5), fore)
	var link4 := _load_visual(["res://assets/generated/aubo_i5/link4.glb", "res://assets/vendor/aubo_i5/link4.DAE"], _cylinder_mesh(0.065, 0.08), materials["arm"], Vector3.ONE)
	link4.name = "aubo_wrist1_Link"
	wrist1.add_child(link4)

	var wrist2 := _make_joint("aubo_wrist2_joint", Vector3(0.0, 0.1025, 0.0), Vector3(-PI * 0.5, 0.0, 0.0), wrist1)
	var link5 := _load_visual(["res://assets/generated/aubo_i5/link5.glb", "res://assets/vendor/aubo_i5/link5.DAE"], _cylinder_mesh(0.06, 0.08), materials["arm"], Vector3.ONE)
	link5.name = "aubo_wrist2_Link"
	wrist2.add_child(link5)

	var wrist3 := _make_joint("aubo_wrist3_joint", Vector3(0.0, -0.094, 0.0), Vector3(PI * 0.5, 0.0, 0.0), wrist2)
	var link6 := _load_visual(["res://assets/generated/aubo_i5/link6.glb", "res://assets/vendor/aubo_i5/link6.DAE"], _cylinder_mesh(0.055, 0.08), materials["arm"], Vector3.ONE)
	link6.name = "aubo_wrist3_Link"
	wrist3.add_child(link6)

	var tool0 := Node3D.new()
	tool0.name = "tool0"
	tool0.basis = _ros_rpy_basis(Vector3(0.0, 0.0, PI * 0.5))
	wrist3.add_child(tool0)

	var gripper_adapter := Node3D.new()
	gripper_adapter.name = "gripper_adapter_link"
	tool0.add_child(gripper_adapter)
	var adapter_visual := MeshInstance3D.new()
	adapter_visual.name = "gripper_adapter_visual"
	var adapter_mesh := CylinderMesh.new()
	adapter_mesh.top_radius = 0.055
	adapter_mesh.bottom_radius = 0.055
	adapter_mesh.height = 0.025
	adapter_mesh.radial_segments = 48
	adapter_visual.mesh = adapter_mesh
	adapter_visual.position = _ros_vec(Vector3(0.0, 0.0, 0.0125))
	adapter_visual.material_override = materials["dark"]
	gripper_adapter.add_child(adapter_visual)
	_build_ms42dc_gripper(gripper_adapter)


func _build_ms42dc_gripper(parent: Node3D) -> void:
	var gripper := Node3D.new()
	gripper.name = "ms42dc_body_link"
	parent.add_child(gripper)

	var base_frame := Node3D.new()
	base_frame.name = "ms42dc_base_frame"
	base_frame.position = _ros_vec(MS42DC_CAD_XYZ)
	base_frame.basis = _ros_rpy_basis(MS42DC_CAD_RPY)
	gripper.add_child(base_frame)

	var body := _load_ros_visual(
		["res://assets/generated/ms42dc/ms42dc_base.glb", "res://assets/vendor/ms42dc/ms42dc_base.stl"],
		_box_mesh(Vector3(0.10, 0.08, 0.05)),
		materials["dark"],
		Vector3(0.001, 0.001, 0.001)
	)
	body.name = "ms42dc_base_link"
	base_frame.add_child(body)

	var mid := _load_ros_visual(
		["res://assets/generated/ms42dc/ms42dc_mid.glb", "res://assets/vendor/ms42dc/ms42dc_mid.stl"],
		_box_mesh(Vector3(0.04, 0.05, 0.05)),
		materials["accent"],
		Vector3(0.001, 0.001, 0.001)
	)
	mid.name = "ms42dc_mid_link"
	base_frame.add_child(mid)

	left_finger_joint = Node3D.new()
	left_finger_joint.name = "ms42dc_left_finger_joint"
	left_finger_joint.position = _ros_vec(MS42DC_LEFT_PIVOT)
	base_frame.add_child(left_finger_joint)
	var left_finger := _load_ros_visual(
		["res://assets/generated/ms42dc/ms42dc_left_finger.glb", "res://assets/vendor/ms42dc/ms42dc_left_finger.stl"],
		_box_mesh(Vector3(0.02, 0.09, 0.05)),
		materials["accent"],
		Vector3(0.001, 0.001, 0.001)
	)
	left_finger.name = "ms42dc_left_finger_link"
	left_finger.position = _ros_vec(MS42DC_LEFT_VISUAL_XYZ)
	left_finger_joint.add_child(left_finger)

	right_finger_joint = Node3D.new()
	right_finger_joint.name = "ms42dc_right_finger_joint"
	right_finger_joint.position = _ros_vec(MS42DC_RIGHT_PIVOT)
	base_frame.add_child(right_finger_joint)
	var right_finger := _load_ros_visual(
		["res://assets/generated/ms42dc/ms42dc_right_finger.glb", "res://assets/vendor/ms42dc/ms42dc_right_finger.stl"],
		_box_mesh(Vector3(0.02, 0.09, 0.05)),
		materials["accent"],
		Vector3(0.001, 0.001, 0.001)
	)
	right_finger.name = "ms42dc_right_finger_link"
	right_finger.position = _ros_vec(MS42DC_RIGHT_VISUAL_XYZ)
	right_finger_joint.add_child(right_finger)


func _build_camera() -> void:
	camera = Camera3D.new()
	camera.name = "FollowCamera"
	camera.fov = 56.0
	camera.near = 0.03
	camera.far = 120.0
	add_child(camera)
	camera.make_current()


func _build_hud() -> void:
	var layer := CanvasLayer.new()
	layer.name = "HUD"
	add_child(layer)

	var panel := PanelContainer.new()
	panel.name = "StatusPanel"
	panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	panel.offset_left = 16
	panel.offset_top = 16
	panel.offset_right = 410
	panel.offset_bottom = 104
	layer.add_child(panel)

	status_label = Label.new()
	status_label.name = "Status"
	status_label.text = ""
	status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	panel.add_child(status_label)


func _add_grid_lines() -> void:
	var line_material := _material(Color(0.78, 0.82, 0.80, 0.55), 1.0, 0.0)
	for i in range(-9, 10):
		var x_line := _box_visual("grid_x_%d" % i, Vector3(18.0, 0.006, 0.012), line_material)
		x_line.position = Vector3(0.0, 0.003, float(i))
		add_child(x_line)
		var z_line := _box_visual("grid_z_%d" % i, Vector3(0.012, 0.006, 18.0), line_material)
		z_line.position = Vector3(float(i), 0.004, 0.0)
		add_child(z_line)


func _add_obstacles() -> void:
	_add_obstacle_box("PalletA", Vector3(1.2, 0.18, 0.75), Vector3(2.2, 0.09, -1.35))
	_add_obstacle_box("PalletB", Vector3(0.72, 0.25, 1.0), Vector3(-2.1, 0.125, 1.15))
	_add_obstacle_cylinder("MarkerA", 0.18, 0.45, Vector3(1.4, 0.225, 1.55), materials["accent"])
	_add_obstacle_cylinder("MarkerB", 0.15, 0.35, Vector3(-1.1, 0.175, -1.65), materials["obstacle"])

	for i in range(4):
		var cube := _box_visual("Crate_%d" % i, Vector3(0.28, 0.24, 0.28), materials["obstacle"])
		cube.position = Vector3(-0.8 + i * 0.34, 0.12, 2.2)
		add_child(cube)


func _add_obstacle_box(name: String, size: Vector3, position: Vector3) -> void:
	var obstacle := StaticBody3D.new()
	obstacle.name = name
	obstacle.position = position
	add_child(obstacle)

	var mesh_instance := _box_visual("%s_visual" % name, size, materials["obstacle"])
	mesh_instance.position = Vector3.ZERO
	obstacle.add_child(mesh_instance)

	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	shape.shape = box
	obstacle.add_child(shape)


func _add_obstacle_cylinder(name: String, radius: float, height: float, position: Vector3, material: Material) -> void:
	var obstacle := StaticBody3D.new()
	obstacle.name = name
	obstacle.position = position
	add_child(obstacle)

	var mesh_instance := MeshInstance3D.new()
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = height
	mesh_instance.mesh = mesh
	mesh_instance.material_override = material
	obstacle.add_child(mesh_instance)

	var shape := CollisionShape3D.new()
	var cylinder := CylinderShape3D.new()
	cylinder.radius = radius
	cylinder.height = height
	shape.shape = cylinder
	obstacle.add_child(shape)


func _update_base(delta: float) -> void:
	var stick := _read_drive_stick()
	var radius := stick.length()
	if radius < input_deadzone:
		current_linear = lerp(current_linear, 0.0, _follow_alpha(10.0, delta))
		current_angular = lerp(current_angular, 0.0, _follow_alpha(12.0, delta))
	else:
		var unit: Vector2 = stick / max(radius, 0.0001)
		var speed_radius: float = clamp((min(radius, 1.0) - input_deadzone) / max(1.0 - input_deadzone, 0.0001), 0.0, 1.0)
		speed_radius = lerp(speed_radius, speed_radius * speed_radius, input_curve)
		current_linear = unit.y * speed_radius * max_linear_speed
		current_angular = unit.x * speed_radius * max_angular_speed

	robot_root.rotation.y -= current_angular * delta
	robot_root.global_position += robot_root.global_transform.basis.x * current_linear * delta

	var wheel_delta := current_linear * delta * 7.0
	for wheel in wheel_nodes:
		wheel.rotate_object_local(Vector3(0, 1, 0), wheel_delta)


func _update_arm(delta: float) -> void:
	var alpha := _follow_alpha(joint_follow_rate, delta)
	for i in range(current_joints.size()):
		current_joints[i] = lerp(float(current_joints[i]), float(target_joints[i]), alpha)
		var node := joint_nodes[i]
		node.basis = joint_rest_bases[i] * _ros_rpy_basis(Vector3(0.0, 0.0, current_joints[i]))


func _update_gripper(delta: float) -> void:
	gripper_position = lerp(gripper_position, gripper_target, _follow_alpha(gripper_follow_rate, delta))
	if left_finger_joint != null:
		left_finger_joint.basis = _ros_rpy_basis(Vector3(0.0, 0.0, -gripper_position))
	if right_finger_joint != null:
		right_finger_joint.basis = _ros_rpy_basis(Vector3(0.0, 0.0, gripper_position))


func _update_camera(delta: float) -> void:
	var orbit_input := 0.0
	if Input.is_key_pressed(KEY_Q):
		orbit_input -= 1.0
	if Input.is_key_pressed(KEY_E):
		orbit_input += 1.0
	if Input.get_connected_joypads().size() > 0:
		orbit_input += _deadzone_axis(Input.get_joy_axis(0, JOY_AXIS_RIGHT_X), 0.10)
	camera_yaw += orbit_input * camera_orbit_speed * delta

	var yaw := robot_root.rotation.y + camera_yaw
	var offset := Basis(Vector3.UP, yaw) * Vector3(-camera_distance, camera_height, 0.0)
	var focus := robot_root.global_position + Vector3(0.0, 0.72, 0.0)
	var desired_position := focus + offset
	camera.global_position = camera.global_position.lerp(desired_position, _follow_alpha(camera_follow_rate, delta))
	camera.look_at(focus, Vector3.UP)


func _update_hud() -> void:
	status_label.text = "Arachne Godot Showcase\nPose: %s    Gripper: %.2f\nv: %.2f m/s    yaw rate: %.2f rad/s" % [
		current_pose_name,
		gripper_position,
		current_linear,
		-current_angular,
	]


func _publish_ros_placeholders() -> void:
	if ros_bridge == null:
		return
	ros_bridge.publish_cmd_vel(current_linear, -current_angular)
	ros_bridge.publish_joint_states(ARM_JOINT_NAMES, current_joints)
	ros_bridge.publish_odom(robot_root.global_position, robot_root.rotation.y, current_linear, -current_angular)
	ros_bridge.publish_tf([
		{"parent": "odom", "child": "base_link", "position": robot_root.global_position, "yaw": robot_root.rotation.y},
		{"parent": "base_link", "child": "arm_mount_link"},
	])


func _read_drive_stick() -> Vector2:
	var turn := 0.0
	var forward := 0.0
	if Input.is_key_pressed(KEY_W) or Input.is_key_pressed(KEY_UP):
		forward += 1.0
	if Input.is_key_pressed(KEY_S) or Input.is_key_pressed(KEY_DOWN):
		forward -= 1.0
	if Input.is_key_pressed(KEY_D) or Input.is_key_pressed(KEY_RIGHT):
		turn += 1.0
	if Input.is_key_pressed(KEY_A) or Input.is_key_pressed(KEY_LEFT):
		turn -= 1.0

	if Input.get_connected_joypads().size() > 0:
		turn += _deadzone_axis(Input.get_joy_axis(0, JOY_AXIS_LEFT_X), 0.08)
		forward += -_deadzone_axis(Input.get_joy_axis(0, JOY_AXIS_LEFT_Y), 0.08)

	var stick := Vector2(turn, forward)
	if stick.length() > 1.0:
		stick = stick.normalized()
	return stick


func _select_pose(pose_name: String) -> void:
	if not ARM_PRESETS.has(pose_name):
		return
	current_pose_name = pose_name
	target_joints = ARM_PRESETS[pose_name].duplicate()
	if pose_name == "grasp" or pose_name == "lift":
		_set_gripper(true)
	elif pose_name == "home" or pose_name == "ready":
		_set_gripper(false)


func _toggle_gripper() -> void:
	_set_gripper(gripper_target < gripper_closed_position * 0.5)


func _set_gripper(closed: bool) -> void:
	gripper_target = gripper_closed_position if closed else 0.0


func _reset_demo() -> void:
	robot_root.position = Vector3(0.0, robot_ground_y, 0.0)
	robot_root.rotation = Vector3.ZERO
	camera_yaw = 0.0
	current_linear = 0.0
	current_angular = 0.0
	_select_pose("home")
	_set_gripper(false)


func _make_joint(joint_name: String, position: Vector3, rest_rotation: Vector3, parent: Node3D) -> Node3D:
	var joint := Node3D.new()
	joint.name = joint_name
	joint.position = _ros_vec(position)
	joint.basis = _ros_rpy_basis(rest_rotation)
	parent.add_child(joint)
	joint_nodes.append(joint)
	joint_rest_bases.append(joint.basis)
	return joint


func _load_visual(paths: Array, fallback_mesh: Mesh, fallback_material: Material, local_scale: Vector3) -> Node3D:
	for path in paths:
		if not FileAccess.file_exists(path) and not ResourceLoader.exists(path):
			continue
		if path.to_lower().ends_with(".stl"):
			var stl_mesh := _load_stl_mesh(path)
			if stl_mesh != null:
				var stl_instance := MeshInstance3D.new()
				stl_instance.mesh = stl_mesh
				stl_instance.scale = local_scale
				stl_instance.material_override = fallback_material
				return stl_instance
		var resource: Resource = load(path)
		if resource is PackedScene:
			var packed_scene := resource as PackedScene
			var scene: Node = packed_scene.instantiate()
			if scene is Node3D:
				scene.scale = local_scale
				_apply_material_to_meshes(scene, fallback_material)
				return scene
			var wrapper := Node3D.new()
			wrapper.scale = local_scale
			wrapper.add_child(scene)
			_apply_material_to_meshes(wrapper, fallback_material)
			return wrapper
		if resource is Mesh:
			var mesh_instance := MeshInstance3D.new()
			mesh_instance.mesh = resource
			mesh_instance.scale = local_scale
			mesh_instance.material_override = fallback_material
			return mesh_instance
	var fallback := MeshInstance3D.new()
	fallback.name = "FallbackMesh"
	fallback.mesh = fallback_mesh
	fallback.material_override = fallback_material
	return fallback


func _load_ros_visual(paths: Array, fallback_mesh: Mesh, fallback_material: Material, local_scale: Vector3) -> Node3D:
	var wrapper := Node3D.new()
	wrapper.basis = ROS_TO_GODOT_BASIS
	wrapper.add_child(_load_visual(paths, fallback_mesh, fallback_material, local_scale))
	return wrapper


func _apply_material_to_meshes(node: Node, material: Material) -> void:
	if node is MeshInstance3D:
		node.material_override = material
	for child in node.get_children():
		_apply_material_to_meshes(child, material)


func _load_stl_mesh(path: String) -> ArrayMesh:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		return null
	file.big_endian = false
	if file.get_length() < 84:
		file.close()
		return _load_ascii_stl_mesh(path)
	file.seek(80)
	var triangle_count := file.get_32()
	var expected_binary_size := 84 + int(triangle_count) * 50
	file.close()
	if expected_binary_size == FileAccess.get_file_as_bytes(path).size():
		return _load_binary_stl_mesh(path, triangle_count)
	return _load_ascii_stl_mesh(path)


func _load_binary_stl_mesh(path: String, triangle_count: int) -> ArrayMesh:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		return null
	file.big_endian = false
	file.seek(84)
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	for _i in range(triangle_count):
		var normal := Vector3(file.get_float(), file.get_float(), file.get_float())
		var a := Vector3(file.get_float(), file.get_float(), file.get_float())
		var b := Vector3(file.get_float(), file.get_float(), file.get_float())
		var c := Vector3(file.get_float(), file.get_float(), file.get_float())
		file.get_16()
		if normal.length_squared() < 0.000001:
			normal = _triangle_normal(a, b, c)
		_append_triangle(vertices, normals, a, b, c, normal.normalized())
	file.close()
	return _mesh_from_triangles(vertices, normals)


func _load_ascii_stl_mesh(path: String) -> ArrayMesh:
	var text := FileAccess.get_file_as_string(path)
	if text.is_empty():
		return null
	var vertices := PackedVector3Array()
	var normals := PackedVector3Array()
	var triangle := []
	for raw_line in text.split("\n"):
		var line := raw_line.strip_edges()
		if not line.begins_with("vertex"):
			continue
		var parts := line.split(" ", false)
		if parts.size() < 4:
			continue
		triangle.append(Vector3(parts[1].to_float(), parts[2].to_float(), parts[3].to_float()))
		if triangle.size() == 3:
			var a: Vector3 = triangle[0]
			var b: Vector3 = triangle[1]
			var c: Vector3 = triangle[2]
			_append_triangle(vertices, normals, a, b, c, _triangle_normal(a, b, c))
			triangle.clear()
	if vertices.is_empty():
		return null
	return _mesh_from_triangles(vertices, normals)


func _append_triangle(vertices: PackedVector3Array, normals: PackedVector3Array, a: Vector3, b: Vector3, c: Vector3, normal: Vector3) -> void:
	vertices.append(a)
	vertices.append(b)
	vertices.append(c)
	normals.append(normal)
	normals.append(normal)
	normals.append(normal)


func _mesh_from_triangles(vertices: PackedVector3Array, normals: PackedVector3Array) -> ArrayMesh:
	var mesh := ArrayMesh.new()
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = vertices
	arrays[Mesh.ARRAY_NORMAL] = normals
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	return mesh


func _triangle_normal(a: Vector3, b: Vector3, c: Vector3) -> Vector3:
	var normal := (b - a).cross(c - a)
	if normal.length_squared() < 0.000001:
		return Vector3.UP
	return normal.normalized()


func _box_visual(name: String, size: Vector3, material: Material) -> MeshInstance3D:
	var mesh_instance := MeshInstance3D.new()
	mesh_instance.name = name
	mesh_instance.mesh = _box_mesh(size)
	mesh_instance.material_override = material
	return mesh_instance


func _box_mesh(size: Vector3) -> BoxMesh:
	var mesh := BoxMesh.new()
	mesh.size = size
	return mesh


func _cylinder_mesh(radius: float, height: float) -> CylinderMesh:
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = height
	mesh.radial_segments = 40
	return mesh


func _material(color: Color, roughness: float, metallic: float) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = roughness
	material.metallic = metallic
	if color.a < 1.0:
		material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	return material


func _deadzone_axis(value: float, deadzone: float) -> float:
	var magnitude: float = abs(value)
	if magnitude < deadzone:
		return 0.0
	var scaled: float = (magnitude - deadzone) / max(1.0 - deadzone, 0.0001)
	return sign(value) * clamp(scaled, 0.0, 1.0)


func _follow_alpha(rate: float, delta: float) -> float:
	return 1.0 - exp(-rate * delta)


func _ros_vec(value: Vector3) -> Vector3:
	return ROS_TO_GODOT_BASIS * value


func _ros_size(value: Vector3) -> Vector3:
	return Vector3(value.x, value.z, value.y)


func _ros_rpy_basis(rpy: Vector3) -> Basis:
	var cr := cos(rpy.x)
	var sr := sin(rpy.x)
	var cp := cos(rpy.y)
	var sp := sin(rpy.y)
	var cy := cos(rpy.z)
	var sy := sin(rpy.z)
	var ros_basis := Basis(
		Vector3(cy * cp, sy * cp, -sp),
		Vector3(cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
		Vector3(cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr)
	)
	return ROS_TO_GODOT_BASIS * ros_basis * ROS_TO_GODOT_BASIS.inverse()


func _node_visual_bounds(node: Node) -> AABB:
	var bounds := AABB()
	var has_bounds := false
	var stack: Array[Node] = [node]
	while not stack.is_empty():
		var current: Node = stack.pop_back()
		if current is MeshInstance3D and current.mesh != null:
			var mesh_bounds := _mesh_global_bounds(current)
			if not has_bounds:
				bounds = mesh_bounds
				has_bounds = true
			else:
				bounds = bounds.merge(mesh_bounds)
		for child in current.get_children():
			stack.append(child)
	return bounds if has_bounds else AABB()


func _mesh_global_bounds(mesh_instance: MeshInstance3D) -> AABB:
	var local_bounds := mesh_instance.mesh.get_aabb()
	var points := [
		local_bounds.position,
		local_bounds.position + Vector3(local_bounds.size.x, 0.0, 0.0),
		local_bounds.position + Vector3(0.0, local_bounds.size.y, 0.0),
		local_bounds.position + Vector3(0.0, 0.0, local_bounds.size.z),
		local_bounds.position + Vector3(local_bounds.size.x, local_bounds.size.y, 0.0),
		local_bounds.position + Vector3(local_bounds.size.x, 0.0, local_bounds.size.z),
		local_bounds.position + Vector3(0.0, local_bounds.size.y, local_bounds.size.z),
		local_bounds.position + local_bounds.size,
	]
	var first: Vector3 = mesh_instance.global_transform * points[0]
	var bounds := AABB(first, Vector3.ZERO)
	for i in range(1, points.size()):
		var point: Vector3 = mesh_instance.global_transform * points[i]
		bounds = bounds.expand(point)
	return bounds
