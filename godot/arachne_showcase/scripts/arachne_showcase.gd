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
	"home": [-1.5707963267949, 0.201570428261868, 1.65970467002488, 0.485178041391533, 1.67675136677345, 0.76432946885334],
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
const SCOUT_WHEEL_RADIUS := 0.16459
const SCOUT_TRACK_WIDTH := 0.58306
const SCOUT_WHEEL_SPIN_AXIS := Vector3(0.0, 0.0, 1.0)
const ARENA_LIMIT := 15.5
const OFFICE_GROUND_SIZE := Vector3(34.0, 0.04, 28.0)
const BODY_COLLISION_SIZE := Vector3(0.96, 0.34, 0.72)
const BODY_COLLISION_CENTER := Vector3(0.0, 0.24, 0.0)
const DEFAULT_CAMERA_AXIS := -1
const GAMEPAD_BRIDGE_PORT_DEFAULT := 8791
const WEB_GAMEPAD_TIMEOUT_MSEC := 1000
const AUTO_PICK_HOLD_SECONDS := 0.75
const AUTO_PICK_APPROACH_DISTANCE := 0.78
const AUTO_PICK_REACHED_DISTANCE := 0.18
const AUTO_PICK_REACHED_HEADING := 0.34
const ARM_JOINT_LIMITS := [
	[-3.05, 3.05],
	[-2.20, 2.20],
	[-2.65, 2.65],
	[-3.05, 3.05],
	[-2.20, 2.20],
	[-3.05, 3.05],
]

@export var max_linear_speed := 2.05
@export var max_angular_speed := 2.45
@export var linear_acceleration := 4.8
@export var braking_acceleration := 8.5
@export var angular_acceleration := 7.0
@export var input_deadzone := 0.12
@export var input_curve := 0.28
@export var camera_distance := 2.05
@export var camera_height := 1.08
@export var camera_orbit_speed := 2.8
@export var camera_follow_rate := 14.0
@export var joint_follow_rate := 6.0
@export var manual_joint_rate := 0.72
@export var gripper_follow_rate := 8.0
@export var gripper_closed_position := 0.6
@export var ground_clearance := 0.015
@export var suspension_follow_rate := 18.0
@export var cinematic_camera := true
@export var camera_axis_index := DEFAULT_CAMERA_AXIS

var robot_root: CharacterBody3D
var visual_root: Node3D
var camera: Camera3D
var camera_yaw := 0.0
var robot_ground_y := 0.0
var target_linear := 0.0
var target_angular := 0.0
var current_linear := 0.0
var current_angular := 0.0
var previous_linear := 0.0
var current_pose_name := "home"
var current_joints := ARM_PRESETS["home"].duplicate()
var target_joints := ARM_PRESETS["home"].duplicate()
var selected_joint_index := 0
var gripper_position := 0.0
var gripper_target := 0.0
var joint_nodes: Array[Node3D] = []
var joint_rest_bases: Array[Basis] = []
var wheel_nodes: Array[Dictionary] = []
var pickable_objects: Array[RigidBody3D] = []
var navigation_obstacles: Array[Dictionary] = []
var left_finger_joint: Node3D
var right_finger_joint: Node3D
var arm_mount_node: Node3D
var grasp_anchor: Node3D
var status_label: Label
var ros_bridge: Node
var materials := {}
var backend_label := "standalone"
var visual_profile := "cinematic"
var camera_axis_label := "auto"
var gamepad_bridge_label := "native"
var gamepad_udp: PacketPeerUDP
var web_gamepad_connected := false
var web_gamepad_axes: Array = []
var web_gamepad_buttons: Array = []
var web_gamepad_button_previous: Array = []
var web_gamepad_button_edges: Array = []
var web_gamepad_last_msec := 0
var forced_drive_enabled := false
var forced_drive_stick := Vector2.ZERO
var self_test_mode := false
var right_stick_hold_time := 0.0
var right_stick_long_press_fired := false
var auto_pick_state := "idle"
var auto_pick_message := "idle"
var auto_pick_timer := 0.0
var auto_pick_target: RigidBody3D
var auto_pick_nav_goal := Vector3.ZERO
var held_pickable: RigidBody3D

func _ready() -> void:
	Engine.max_fps = 165
	self_test_mode = _has_user_arg("--self-test")
	visual_profile = OS.get_environment("ARACHNE_GODOT_PROFILE")
	if visual_profile.is_empty():
		visual_profile = "cinematic"
	_configure_input_overrides()
	_setup_gamepad_bridge()
	_setup_materials()
	_build_world()
	_build_robot()
	_lift_robot_to_ground()
	_build_camera()
	_build_hud()
	ros_bridge = ROS2_BRIDGE_SCRIPT.new()
	ros_bridge.name = "ROS2BridgePlaceholder"
	add_child(ros_bridge)
	_configure_backend()
	_select_pose("home")
	if self_test_mode:
		call_deferred("_run_self_test")


func _configure_backend() -> void:
	var ros_distro := OS.get_environment("ROS_DISTRO")
	var ros_available := OS.get_environment("ARACHNE_ROS2_AVAILABLE") == "1"
	var bridge_mode := OS.get_environment("ARACHNE_GODOT_BRIDGE")
	if bridge_mode.is_empty() and (not ros_distro.is_empty() or ros_available):
		bridge_mode = "udp"
	if ros_bridge.has_method("configure_from_environment"):
		ros_bridge.configure_from_environment()
	if not bridge_mode.is_empty():
		backend_label = "%s bridge" % bridge_mode
	elif not ros_distro.is_empty():
		backend_label = "ROS2 %s ready" % ros_distro
	elif ros_available:
		backend_label = "ROS2 command ready"
	else:
		backend_label = "standalone physics"


func _configure_input_overrides() -> void:
	var requested_camera_axis := OS.get_environment("ARACHNE_CAMERA_AXIS")
	if requested_camera_axis.is_valid_int():
		camera_axis_index = requested_camera_axis.to_int()
		camera_axis_label = str(camera_axis_index)
	else:
		camera_axis_index = DEFAULT_CAMERA_AXIS
		camera_axis_label = "auto"


func _setup_gamepad_bridge() -> void:
	var mode := OS.get_environment("ARACHNE_GODOT_GAMEPAD")
	var requested_port := OS.get_environment("ARACHNE_GODOT_GAMEPAD_PORT")
	if mode.is_empty() and requested_port.is_valid_int():
		mode = "udp"
	if mode != "udp":
		return

	var port := GAMEPAD_BRIDGE_PORT_DEFAULT
	if requested_port.is_valid_int():
		port = requested_port.to_int()
	gamepad_udp = PacketPeerUDP.new()
	var error := gamepad_udp.bind(port, "127.0.0.1")
	if error != OK:
		error = gamepad_udp.bind(port)
	if error == OK:
		gamepad_bridge_label = "web:%d" % port
	else:
		gamepad_udp = null
		gamepad_bridge_label = "web bind failed"
		push_warning("Could not bind Godot gamepad UDP bridge on port %d" % port)


func _poll_gamepad_bridge() -> void:
	if gamepad_udp == null:
		return
	while gamepad_udp.get_available_packet_count() > 0:
		var packet := gamepad_udp.get_packet()
		var parsed = JSON.parse_string(packet.get_string_from_utf8())
		if typeof(parsed) != TYPE_DICTIONARY:
			continue
		_ingest_web_gamepad_packet(parsed)

	if web_gamepad_connected and Time.get_ticks_msec() - web_gamepad_last_msec > WEB_GAMEPAD_TIMEOUT_MSEC:
		web_gamepad_connected = false
		web_gamepad_axes.clear()
		web_gamepad_buttons.clear()
		web_gamepad_button_previous.clear()
		web_gamepad_button_edges.clear()


func _ingest_web_gamepad_packet(packet: Dictionary) -> void:
	web_gamepad_connected = true
	web_gamepad_last_msec = Time.get_ticks_msec()
	if packet.has("axes") and typeof(packet["axes"]) == TYPE_ARRAY:
		web_gamepad_axes = packet["axes"].duplicate()
	if packet.has("buttons") and typeof(packet["buttons"]) == TYPE_ARRAY:
		_set_web_gamepad_buttons(packet["buttons"])
	var action := str(packet.get("action", ""))
	if not action.is_empty():
		_handle_web_gamepad_action(action)


func _set_web_gamepad_buttons(buttons: Array) -> void:
	web_gamepad_button_previous = web_gamepad_buttons.duplicate()
	web_gamepad_buttons = buttons.duplicate()
	web_gamepad_button_edges.clear()
	for i in range(web_gamepad_buttons.size()):
		var previous := _web_button_value_from_array(web_gamepad_button_previous, i)
		var current := _web_button_value_from_array(web_gamepad_buttons, i)
		web_gamepad_button_edges.append(current > 0.5 and previous <= 0.5)


func _handle_web_gamepad_action(action: String) -> void:
	match action:
		"reset":
			_reset_demo()
		"open":
			_set_gripper(false)
		"close":
			_set_gripper(true)
		"auto_pick":
			_start_auto_pick()
		"home", "ready", "reach", "grasp", "lift":
			_select_pose(action)


func _handle_web_gamepad_edge_actions() -> void:
	if not web_gamepad_connected:
		return
	if _web_button_just_pressed(0):
		_set_gripper(false)
	if _web_button_just_pressed(1):
		_select_pose("grasp")
		_set_gripper(true)
	if _web_button_just_pressed(2):
		_select_pose("lift")
	if _web_button_just_pressed(3):
		_select_pose("ready")
	if _web_button_just_pressed(4):
		_select_previous_joint()
	if _web_button_just_pressed(5):
		_select_next_joint()
	if _web_button_just_pressed(9):
		_reset_demo()
	for i in range(web_gamepad_button_edges.size()):
		web_gamepad_button_edges[i] = false


func _physics_process(delta: float) -> void:
	_poll_gamepad_bridge()
	_handle_web_gamepad_edge_actions()
	_update_auto_pick_trigger(delta)
	_update_auto_pick(delta)
	_update_base(delta)
	if auto_pick_state == "idle":
		_update_manual_arm(delta)
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
			KEY_P:
				_start_auto_pick()
			KEY_H:
				_select_previous_joint()
			KEY_K:
				_select_next_joint()
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
			JOY_BUTTON_RIGHT_STICK:
				pass
			JOY_BUTTON_LEFT_SHOULDER:
				_select_previous_joint()
			JOY_BUTTON_RIGHT_SHOULDER:
				_select_next_joint()
			JOY_BUTTON_DPAD_LEFT:
				_set_gripper(false)
			JOY_BUTTON_DPAD_RIGHT:
				_set_gripper(true)


func _setup_materials() -> void:
	materials["body"] = _material(Color(0.08, 0.09, 0.09), 0.68, 0.05)
	materials["body_clearcoat"] = _material(Color(0.18, 0.19, 0.18), 0.58, 0.04)
	materials["dark"] = _material(Color(0.015, 0.017, 0.020), 0.82, 0.0)
	materials["tire"] = _material(Color(0.012, 0.012, 0.013), 0.94, 0.0)
	materials["arm"] = _material(Color(1.0, 0.48, 0.08), 0.42, 0.0)
	materials["arm_joint"] = _material(Color(0.025, 0.028, 0.032), 0.66, 0.06)
	materials["accent"] = _material(Color(1.0, 0.60, 0.10), 0.38, 0.0)
	materials["accent_blue"] = _material(Color(0.10, 0.48, 0.70), 0.42, 0.0)
	materials["ground"] = _material(Color(0.34, 0.38, 0.35), 0.88, 0.0)
	materials["asphalt_dark"] = _material(Color(0.12, 0.14, 0.15), 0.92, 0.0)
	materials["obstacle"] = _material(Color(0.23, 0.28, 0.30), 0.70, 0.02)
	materials["crate"] = _material(Color(0.52, 0.39, 0.22), 0.76, 0.0)
	materials["desk"] = _material(Color(0.48, 0.42, 0.34), 0.72, 0.0)
	materials["wall"] = _material(Color(0.72, 0.74, 0.70), 0.82, 0.0)
	materials["glass"] = _material(Color(0.48, 0.72, 0.82, 0.34), 0.36, 0.0)
	materials["carpet"] = _material(Color(0.20, 0.25, 0.27), 0.88, 0.0)
	materials["zone"] = _material(Color(0.16, 0.42, 0.58, 0.35), 0.7, 0.0)
	materials["line"] = _material(Color(0.92, 0.86, 0.62), 0.78, 0.0)
	materials["bottle"] = _material(Color(0.24, 0.66, 0.92, 0.82), 0.30, 0.0)
	materials["bottle_cap"] = _material(Color(0.05, 0.11, 0.16), 0.44, 0.0)
	materials["ball_red"] = _material(Color(0.88, 0.12, 0.10), 0.52, 0.0)
	materials["ball_yellow"] = _material(Color(0.95, 0.72, 0.12), 0.50, 0.0)
	materials["ball_blue"] = _material(Color(0.14, 0.35, 0.88), 0.48, 0.0)


func _build_world() -> void:
	var world := WorldEnvironment.new()
	var environment := Environment.new()
	environment.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var sky_material := ProceduralSkyMaterial.new()
	sky_material.sky_top_color = Color(0.37, 0.56, 0.76)
	sky_material.sky_horizon_color = Color(0.72, 0.80, 0.86)
	sky_material.ground_bottom_color = Color(0.18, 0.20, 0.19)
	sky_material.ground_horizon_color = Color(0.52, 0.56, 0.50)
	sky.sky_material = sky_material
	environment.sky = sky
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color(0.60, 0.66, 0.70)
	environment.ambient_light_energy = 0.58
	environment.tonemap_mode = Environment.TONE_MAPPER_ACES
	environment.tonemap_exposure = 1.05
	environment.tonemap_white = 2.4
	var fast_profile := visual_profile == "performance" or visual_profile == "wsl"
	environment.ssao_enabled = not fast_profile
	environment.ssao_radius = 2.2
	environment.ssao_intensity = 1.45
	environment.glow_enabled = not fast_profile
	environment.glow_intensity = 0.18
	environment.fog_enabled = not fast_profile
	environment.fog_density = 0.006
	world.environment = environment
	add_child(world)

	var sun := DirectionalLight3D.new()
	sun.name = "KeyLight"
	sun.rotation_degrees = Vector3(-47.0, -38.0, 0.0)
	sun.light_energy = 3.8
	sun.shadow_enabled = not fast_profile
	add_child(sun)

	var fill := OmniLight3D.new()
	fill.name = "SoftFill"
	fill.position = Vector3(-3.5, 3.0, 4.0)
	fill.light_energy = 0.65
	add_child(fill)

	var rim := SpotLight3D.new()
	rim.name = "WorkshopRimLight"
	rim.position = Vector3(4.0, 4.2, -3.8)
	rim.rotation_degrees = Vector3(-58.0, 38.0, 0.0)
	rim.light_energy = 6.0
	rim.spot_range = 7.0
	rim.spot_angle = 42.0
	rim.shadow_enabled = false
	add_child(rim)

	var ground_body := StaticBody3D.new()
	ground_body.name = "GroundCollider"
	add_child(ground_body)

	var ground := MeshInstance3D.new()
	ground.name = "PaintedConcrete"
	var ground_mesh := BoxMesh.new()
	ground_mesh.size = OFFICE_GROUND_SIZE
	ground.mesh = ground_mesh
	ground.position = Vector3(0.0, -0.02, 0.0)
	ground.material_override = materials["ground"]
	ground_body.add_child(ground)

	var ground_shape := CollisionShape3D.new()
	var ground_box := BoxShape3D.new()
	ground_box.size = ground_mesh.size
	ground_shape.shape = ground_box
	ground_shape.position = ground.position
	ground_body.add_child(ground_shape)

	_add_office_map()


func _build_robot() -> void:
	robot_root = CharacterBody3D.new()
	robot_root.name = "ArachneRobot"
	robot_root.motion_mode = CharacterBody3D.MOTION_MODE_FLOATING
	robot_root.safe_margin = 0.02
	add_child(robot_root)

	var body_shape := CollisionShape3D.new()
	body_shape.name = "ScoutPhysicsProxy"
	var body_box := BoxShape3D.new()
	body_box.size = BODY_COLLISION_SIZE
	body_shape.shape = body_box
	body_shape.position = BODY_COLLISION_CENTER
	robot_root.add_child(body_shape)

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
		materials["body_clearcoat"],
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
			materials["tire"],
			Vector3.ONE
		)
		wheel.name = "%s_wheel" % wheel_name
		wheel.position = wheel_positions[wheel_name]
		wheel_nodes.append({
			"node": wheel,
			"side": -1.0 if wheel_name.ends_with("_left") else 1.0,
			"rest_basis": wheel.basis,
			"spin": 0.0,
		})
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
	arm_mount_node = arm_mount

	var link0 := _load_visual(["res://assets/generated/aubo_i5/link0.glb", "res://assets/vendor/aubo_i5/link0.DAE"], _cylinder_mesh(0.10, 0.08), materials["arm_joint"], Vector3.ONE)
	link0.name = "aubo_base_link"
	arm_mount.add_child(link0)

	var shoulder := _make_joint("aubo_shoulder_joint", Vector3(0.0, 0.0, 0.122), Vector3(0.0, 0.0, PI), arm_mount)
	var link1 := _load_visual(["res://assets/generated/aubo_i5/link1.glb", "res://assets/vendor/aubo_i5/link1.DAE"], _cylinder_mesh(0.10, 0.13), materials["arm_joint"], Vector3.ONE)
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
	var link4 := _load_visual(["res://assets/generated/aubo_i5/link4.glb", "res://assets/vendor/aubo_i5/link4.DAE"], _cylinder_mesh(0.065, 0.08), materials["arm_joint"], Vector3.ONE)
	link4.name = "aubo_wrist1_Link"
	wrist1.add_child(link4)

	var wrist2 := _make_joint("aubo_wrist2_joint", Vector3(0.0, 0.1025, 0.0), Vector3(-PI * 0.5, 0.0, 0.0), wrist1)
	var link5 := _load_visual(["res://assets/generated/aubo_i5/link5.glb", "res://assets/vendor/aubo_i5/link5.DAE"], _cylinder_mesh(0.06, 0.08), materials["arm_joint"], Vector3.ONE)
	link5.name = "aubo_wrist2_Link"
	wrist2.add_child(link5)

	var wrist3 := _make_joint("aubo_wrist3_joint", Vector3(0.0, -0.094, 0.0), Vector3(PI * 0.5, 0.0, 0.0), wrist2)
	var link6 := _load_visual(["res://assets/generated/aubo_i5/link6.glb", "res://assets/vendor/aubo_i5/link6.DAE"], _cylinder_mesh(0.055, 0.08), materials["arm_joint"], Vector3.ONE)
	link6.name = "aubo_wrist3_Link"
	wrist3.add_child(link6)

	var tool0 := Node3D.new()
	tool0.name = "tool0"
	tool0.basis = _ros_rpy_basis(Vector3(0.0, 0.0, PI * 0.5))
	wrist3.add_child(tool0)

	var gripper_adapter := Node3D.new()
	gripper_adapter.name = "gripper_adapter_link"
	tool0.add_child(gripper_adapter)
	grasp_anchor = Node3D.new()
	grasp_anchor.name = "grasp_anchor"
	grasp_anchor.position = _ros_vec(Vector3(0.0, 0.0, 0.165))
	gripper_adapter.add_child(grasp_anchor)
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
	panel.offset_right = 560
	panel.offset_bottom = 136
	layer.add_child(panel)

	status_label = Label.new()
	status_label.name = "Status"
	status_label.text = ""
	status_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	panel.add_child(status_label)


func _add_grid_lines() -> void:
	var line_material := _material(Color(0.78, 0.82, 0.80, 0.30), 1.0, 0.0)
	for i in range(-16, 17):
		var x_line := _box_visual("grid_x_%d" % i, Vector3(32.0, 0.004, 0.010), line_material)
		x_line.position = Vector3(0.0, 0.003, float(i))
		add_child(x_line)
		var z_line := _box_visual("grid_z_%d" % i, Vector3(0.010, 0.004, 26.0), line_material)
		z_line.position = Vector3(float(i), 0.004, 0.0)
		add_child(z_line)


func _add_office_map() -> void:
	_add_office_floor_plan()
	_add_office_furniture()
	_add_office_training_props()
	_spawn_pickable_objects()


func _add_office_floor_plan() -> void:
	var reception_carpet := _box_visual("OfficeReceptionCarpet", Vector3(5.6, 0.006, 3.5), materials["carpet"])
	reception_carpet.position = Vector3(-5.4, 0.006, 4.7)
	add_child(reception_carpet)

	var lab_zone := _box_visual("RobotLabZonePaint", Vector3(5.8, 0.006, 3.6), materials["zone"])
	lab_zone.position = Vector3(5.5, 0.008, -4.9)
	add_child(lab_zone)

	var center_lane := _box_visual("OfficeNavigationLane", Vector3(18.5, 0.007, 0.14), materials["line"])
	center_lane.position = Vector3(0.0, 0.010, -0.2)
	add_child(center_lane)

	_add_obstacle_box("OfficeWallNorthA", Vector3(8.0, 1.55, 0.14), Vector3(-6.5, 0.775, -7.4), materials["wall"])
	_add_obstacle_box("OfficeWallNorthB", Vector3(7.0, 1.55, 0.14), Vector3(6.8, 0.775, -7.4), materials["wall"])
	_add_obstacle_box("OfficeWallSouthA", Vector3(8.6, 1.55, 0.14), Vector3(-6.2, 0.775, 7.4), materials["wall"])
	_add_obstacle_box("OfficeWallSouthB", Vector3(6.8, 1.55, 0.14), Vector3(7.1, 0.775, 7.4), materials["wall"])
	_add_obstacle_box("OfficeWallWest", Vector3(0.14, 1.55, 14.8), Vector3(-11.0, 0.775, 0.0), materials["wall"])
	_add_obstacle_box("OfficeWallEastA", Vector3(0.14, 1.55, 5.3), Vector3(11.0, 0.775, -4.25), materials["wall"])
	_add_obstacle_box("OfficeWallEastB", Vector3(0.14, 1.55, 5.1), Vector3(11.0, 0.775, 4.35), materials["wall"])

	_add_obstacle_box("GlassMeetingRoomWall", Vector3(0.08, 1.25, 4.3), Vector3(-2.1, 0.625, 4.8), materials["glass"])
	_add_obstacle_box("GlassMeetingRoomFront", Vector3(4.6, 1.25, 0.08), Vector3(-4.4, 0.625, 2.65), materials["glass"])
	_add_obstacle_box("LabDividerWall", Vector3(0.10, 1.20, 4.8), Vector3(2.4, 0.60, -4.9), materials["glass"])
	_add_obstacle_box("OpenOfficeDivider", Vector3(4.2, 1.20, 0.08), Vector3(5.7, 0.60, 1.65), materials["glass"])


func _add_office_furniture() -> void:
	_add_desk("DeskA", Vector3(-8.2, 0.75, -5.4), 0.0)
	_add_desk("DeskB", Vector3(-8.2, 0.75, -3.6), 0.0)
	_add_desk("DeskC", Vector3(-8.2, 0.75, -1.8), 0.0)
	_add_desk("DeskD", Vector3(6.2, 0.75, 3.3), PI)
	_add_desk("DeskE", Vector3(8.0, 0.75, 3.3), PI)
	_add_desk("DeskF", Vector3(8.0, 0.75, 5.0), PI)

	if not _add_asset_prop("MeetingTableAsset", "table", Vector3(-6.1, 0.0, 5.0), PI * 0.5, Vector3(2.25, 2.25, 2.25), Vector3(2.25, 0.76, 1.15), Vector3(0.0, 0.38, 0.0)):
		_add_obstacle_box("MeetingTable", Vector3(2.25, 0.16, 1.15), Vector3(-6.1, 0.52, 5.0), materials["desk"])
		_add_obstacle_box("MeetingTableBase", Vector3(0.34, 0.70, 0.26), Vector3(-6.1, 0.35, 5.0), materials["dark"])
	for i in range(4):
		_add_asset_prop("MeetingChair_%d" % i, "chair", Vector3(-7.0 + i * 0.58, 0.0, 3.95), 0.0, Vector3(1.6, 1.6, 1.6), Vector3(0.45, 0.65, 0.45), Vector3(0.0, 0.325, 0.0))

	_add_obstacle_box("LabWorkbench", Vector3(2.25, 0.20, 0.84), Vector3(6.05, 0.58, -6.05), materials["desk"])
	_add_asset_prop("LabCabinetA", "bookcaseClosedWide", Vector3(9.15, 0.0, -6.05), PI * 0.5, Vector3(1.75, 1.75, 1.75), Vector3(0.70, 1.45, 1.45), Vector3(0.0, 0.725, 0.0))
	_add_asset_prop("ShelfA", "bookcaseOpen", Vector3(-10.35, 0.0, 4.15), PI * 0.5, Vector3(1.7, 1.7, 1.7), Vector3(0.58, 1.40, 1.30), Vector3(0.0, 0.70, 0.0))
	_add_asset_prop("PlantA", "pottedPlant", Vector3(-9.9, 0.0, 6.55), 0.0, Vector3(1.35, 1.35, 1.35), Vector3(0.46, 0.90, 0.46), Vector3(0.0, 0.45, 0.0))
	_add_asset_prop("LoungeSofa", "loungeSofa", Vector3(-3.6, 0.0, 6.25), PI, Vector3(1.85, 1.85, 1.85), Vector3(1.45, 0.82, 0.72), Vector3(0.0, 0.41, 0.0))
	_add_obstacle_box("ChargingDock", Vector3(0.78, 0.12, 0.58), Vector3(8.75, 0.06, -1.25), materials["dark"])


func _add_desk(name: String, position: Vector3, yaw: float) -> void:
	if _add_asset_prop("%s_asset_desk" % name, "desk", Vector3(position.x, 0.0, position.z), yaw, Vector3(1.65, 1.65, 1.65), Vector3(1.25, 0.75, 0.70), Vector3(0.0, 0.375, 0.0)):
		var chair_offset := Basis(Vector3.UP, yaw) * Vector3(0.0, 0.0, 0.92)
		_add_asset_prop("%s_asset_chair" % name, "chairDesk", Vector3(position.x, 0.0, position.z) + chair_offset, yaw + PI, Vector3(1.55, 1.55, 1.55), Vector3(0.50, 0.72, 0.50), Vector3(0.0, 0.36, 0.0))
		return

	var desk := StaticBody3D.new()
	desk.name = name
	desk.position = position
	desk.rotation.y = yaw
	add_child(desk)

	var top := _box_visual("%s_top" % name, Vector3(1.24, 0.12, 0.68), materials["desk"])
	desk.add_child(top)
	var top_shape := CollisionShape3D.new()
	var top_box := BoxShape3D.new()
	top_box.size = Vector3(1.24, 0.75, 0.68)
	top_shape.shape = top_box
	top_shape.position = Vector3(0.0, -0.375, 0.0)
	desk.add_child(top_shape)

	for x in [-0.48, 0.48]:
		for z in [-0.23, 0.23]:
			var leg := _box_visual("%s_leg" % name, Vector3(0.08, 0.70, 0.08), materials["dark"])
			leg.position = Vector3(x, -0.35, z)
			desk.add_child(leg)

	var chair := _box_visual("%s_chair_visual" % name, Vector3(0.42, 0.48, 0.42), materials["obstacle"])
	chair.position = Vector3(0.0, -0.51, 0.78)
	desk.add_child(chair)
	var chair_shape := CollisionShape3D.new()
	var chair_box := BoxShape3D.new()
	chair_box.size = Vector3(0.42, 0.48, 0.42)
	chair_shape.shape = chair_box
	chair_shape.position = chair.position
	desk.add_child(chair_shape)


func _add_office_training_props() -> void:
	_add_obstacle_cylinder("OfficeMarkerA", 0.16, 0.40, Vector3(1.75, 0.20, 1.15), materials["accent"])
	_add_obstacle_cylinder("OfficeMarkerB", 0.14, 0.36, Vector3(5.55, 0.18, -2.35), materials["obstacle"])

	for i in range(8):
		_add_pushable_box("OfficePushCrate_%d" % i, Vector3(0.28, 0.24, 0.28), Vector3(-1.2 + i * 0.32, 0.16, -3.25), materials["crate"])


func _spawn_pickable_objects() -> void:
	var rng := RandomNumberGenerator.new()
	rng.seed = 424242
	var anchor_positions: Array[Vector3] = [
		Vector3(2.15, 0.0, 1.35),
		Vector3(3.70, 0.0, -1.25),
		Vector3(-1.85, 0.0, 2.10),
		Vector3(5.20, 0.0, 0.85),
		Vector3(-3.40, 0.0, -1.25),
		Vector3(1.20, 0.0, -3.90),
		Vector3(7.10, 0.0, -2.05),
	]
	for i in range(anchor_positions.size()):
		var jitter := Vector3(rng.randf_range(-0.35, 0.35), 0.0, rng.randf_range(-0.30, 0.30))
		var position: Vector3 = anchor_positions[i] + jitter
		if i % 3 == 0:
			_add_pickable_bottle("pickable_bottle_%d" % i, position)
		else:
			var material_name: String = "ball_red"
			if i % 3 == 1:
				material_name = "ball_yellow"
			elif i % 3 == 2:
				material_name = "ball_blue"
			_add_pickable_ball("pickable_ball_%d" % i, position, material_name)


func _add_pickable_ball(name: String, ground_position: Vector3, material_name: String) -> RigidBody3D:
	var radius := 0.105
	var body := RigidBody3D.new()
	body.name = name
	body.position = ground_position + Vector3(0.0, radius + 0.02, 0.0)
	body.mass = 0.12
	body.linear_damp = 2.2
	body.angular_damp = 2.8
	body.set_meta("pickable", true)
	add_child(body)

	var visual := MeshInstance3D.new()
	var mesh := SphereMesh.new()
	mesh.radius = radius
	mesh.height = radius * 2.0
	visual.mesh = mesh
	visual.material_override = materials[material_name]
	body.add_child(visual)

	var shape := CollisionShape3D.new()
	var sphere := SphereShape3D.new()
	sphere.radius = radius
	shape.shape = sphere
	body.add_child(shape)
	pickable_objects.append(body)
	return body


func _add_pickable_bottle(name: String, ground_position: Vector3) -> RigidBody3D:
	var body := RigidBody3D.new()
	body.name = name
	body.position = ground_position + Vector3(0.0, 0.15, 0.0)
	body.mass = 0.16
	body.linear_damp = 2.6
	body.angular_damp = 3.0
	body.set_meta("pickable", true)
	add_child(body)

	var bottle := MeshInstance3D.new()
	var bottle_mesh := CylinderMesh.new()
	bottle_mesh.top_radius = 0.055
	bottle_mesh.bottom_radius = 0.065
	bottle_mesh.height = 0.24
	bottle_mesh.radial_segments = 28
	bottle.mesh = bottle_mesh
	bottle.material_override = materials["bottle"]
	body.add_child(bottle)

	var neck := MeshInstance3D.new()
	var neck_mesh := CylinderMesh.new()
	neck_mesh.top_radius = 0.030
	neck_mesh.bottom_radius = 0.038
	neck_mesh.height = 0.075
	neck_mesh.radial_segments = 24
	neck.mesh = neck_mesh
	neck.position.y = 0.155
	neck.material_override = materials["bottle"]
	body.add_child(neck)

	var cap := MeshInstance3D.new()
	var cap_mesh := CylinderMesh.new()
	cap_mesh.top_radius = 0.032
	cap_mesh.bottom_radius = 0.032
	cap_mesh.height = 0.030
	cap_mesh.radial_segments = 24
	cap.mesh = cap_mesh
	cap.position.y = 0.210
	cap.material_override = materials["bottle_cap"]
	body.add_child(cap)

	var shape := CollisionShape3D.new()
	var capsule := CapsuleShape3D.new()
	capsule.radius = 0.065
	capsule.height = 0.30
	shape.shape = capsule
	body.add_child(shape)
	pickable_objects.append(body)
	return body


func _add_course_markings() -> void:
	var start_zone := _box_visual("StartZonePaint", Vector3(1.8, 0.006, 1.15), materials["zone"])
	start_zone.position = Vector3(0.0, 0.006, 0.0)
	add_child(start_zone)

	var runway := _box_visual("PracticeRunway", Vector3(8.5, 0.007, 0.16), materials["line"])
	runway.position = Vector3(0.0, 0.009, -2.7)
	add_child(runway)

	for i in range(7):
		var dash := _box_visual("RunwayDash_%d" % i, Vector3(0.42, 0.010, 0.10), materials["line"])
		dash.position = Vector3(-3.2 + i * 1.05, 0.014, -2.25)
		add_child(dash)

	var title := Label3D.new()
	title.name = "ArenaTitle"
	title.text = "ARACHNE"
	title.font_size = 68
	title.modulate = Color(0.08, 0.12, 0.13, 0.72)
	title.position = Vector3(-4.8, 0.022, 4.2)
	title.rotation_degrees = Vector3(-90.0, 0.0, 0.0)
	add_child(title)


func _add_obstacles() -> void:
	_add_obstacle_box("PalletA", Vector3(1.2, 0.18, 0.75), Vector3(2.2, 0.09, -1.35))
	_add_obstacle_box("PalletB", Vector3(0.72, 0.25, 1.0), Vector3(-2.1, 0.125, 1.15))
	_add_obstacle_cylinder("MarkerA", 0.18, 0.45, Vector3(1.4, 0.225, 1.55), materials["accent"])
	_add_obstacle_cylinder("MarkerB", 0.15, 0.35, Vector3(-1.1, 0.175, -1.65), materials["obstacle"])

	for i in range(4):
		_add_pushable_box("PushCrate_%d" % i, Vector3(0.28, 0.24, 0.28), Vector3(-0.8 + i * 0.34, 0.14, 2.2), materials["crate"])


func _add_obstacle_box(name: String, size: Vector3, position: Vector3, material: Material = null) -> void:
	var obstacle := StaticBody3D.new()
	obstacle.name = name
	obstacle.position = position
	add_child(obstacle)

	var mesh_instance := _box_visual("%s_visual" % name, size, material if material != null else materials["obstacle"])
	mesh_instance.position = Vector3.ZERO
	obstacle.add_child(mesh_instance)

	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	shape.shape = box
	obstacle.add_child(shape)
	_register_navigation_obstacle(name, position, max(size.x, size.z) * 0.5 + 0.22)


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
	_register_navigation_obstacle(name, position, radius + 0.34)


func _add_asset_prop(name: String, asset_name: String, position: Vector3, yaw: float, local_scale: Vector3, collision_size: Vector3, collision_center: Vector3) -> bool:
	var path := "res://assets/generated/kenney/%s.glb" % asset_name
	if not ResourceLoader.exists(path):
		return false

	var prop := StaticBody3D.new()
	prop.name = name
	prop.position = position
	prop.rotation.y = yaw
	add_child(prop)

	var visual := _load_visual([path], _box_mesh(collision_size), materials["obstacle"], local_scale, false)
	visual.name = "%s_visual" % name
	prop.add_child(visual)

	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = collision_size
	shape.shape = box
	shape.position = collision_center
	prop.add_child(shape)
	_register_navigation_obstacle(name, position + collision_center, max(collision_size.x, collision_size.z) * 0.5 + 0.28)
	return true


func _add_pushable_box(name: String, size: Vector3, position: Vector3, material: Material) -> void:
	var body := RigidBody3D.new()
	body.name = name
	body.position = position
	body.mass = 2.4
	body.linear_damp = 2.8
	body.angular_damp = 3.5
	add_child(body)

	var mesh_instance := _box_visual("%s_visual" % name, size, material)
	body.add_child(mesh_instance)

	var shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = size
	shape.shape = box
	body.add_child(shape)
	_register_navigation_obstacle(name, position, max(size.x, size.z) * 0.5 + 0.30)


func _register_navigation_obstacle(name: String, position: Vector3, radius: float) -> void:
	if radius > 2.4:
		return
	navigation_obstacles.append({
		"name": name,
		"position": position,
		"radius": radius,
	})


func _update_auto_pick_trigger(delta: float) -> void:
	var pressed := _right_stick_button_pressed()
	if pressed:
		right_stick_hold_time += delta
		if right_stick_hold_time >= AUTO_PICK_HOLD_SECONDS and not right_stick_long_press_fired:
			right_stick_long_press_fired = true
			_start_auto_pick()
	else:
		right_stick_hold_time = 0.0
		right_stick_long_press_fired = false


func _right_stick_button_pressed() -> bool:
	var joypads := Input.get_connected_joypads()
	if not joypads.is_empty() and Input.is_joy_button_pressed(int(joypads[0]), JOY_BUTTON_RIGHT_STICK):
		return true
	return web_gamepad_connected and _web_button_pressed(11)


func _start_auto_pick() -> void:
	if auto_pick_state != "idle":
		return
	auto_pick_target = _find_nearest_pickable()
	if auto_pick_target == null:
		auto_pick_message = "no pickable target"
		return
	auto_pick_nav_goal = _approach_goal_for_pickable(auto_pick_target)
	auto_pick_state = "navigate"
	auto_pick_timer = 0.0
	auto_pick_message = "target: %s" % auto_pick_target.name
	_set_gripper(false)


func _find_nearest_pickable() -> RigidBody3D:
	var best: RigidBody3D = null
	var best_distance := INF
	for pickable in pickable_objects:
		if pickable == null or not is_instance_valid(pickable):
			continue
		if bool(pickable.get_meta("held", false)):
			continue
		var distance := robot_root.global_position.distance_to(pickable.global_position)
		if distance < best_distance:
			best_distance = distance
			best = pickable
	return best


func _approach_goal_for_pickable(pickable: RigidBody3D) -> Vector3:
	var target := pickable.global_position
	var from_robot := target - robot_root.global_position
	from_robot.y = 0.0
	if from_robot.length_squared() < 0.01:
		from_robot = robot_root.global_transform.basis.x
	var approach := from_robot.normalized()
	var goal := target - approach * AUTO_PICK_APPROACH_DISTANCE
	goal.y = robot_ground_y
	goal.x = clamp(goal.x, -ARENA_LIMIT, ARENA_LIMIT)
	goal.z = clamp(goal.z, -ARENA_LIMIT, ARENA_LIMIT)
	return goal


func _update_auto_pick(delta: float) -> void:
	if auto_pick_state == "idle":
		return
	auto_pick_timer += delta
	if auto_pick_target == null or not is_instance_valid(auto_pick_target):
		_finish_auto_pick("target lost")
		return

	match auto_pick_state:
		"navigate":
			auto_pick_message = "navigating to %s" % auto_pick_target.name
			if _auto_pick_navigation_reached() or auto_pick_timer > 8.0:
				target_linear = 0.0
				target_angular = 0.0
				current_linear = 0.0
				current_angular = 0.0
				target_joints = _solve_aubo_pick_pose(auto_pick_target.global_position)
				current_pose_name = "auto_reach"
				auto_pick_state = "reach"
				auto_pick_timer = 0.0
		"reach":
			auto_pick_message = "solving IK + reaching"
			if _joint_error_to_target() < 0.09 or auto_pick_timer > 1.45:
				_set_gripper(true)
				auto_pick_state = "grasp"
				auto_pick_timer = 0.0
		"grasp":
			auto_pick_message = "closing gripper"
			if auto_pick_timer > 0.45:
				_attach_pickable(auto_pick_target)
				target_joints = _auto_lift_pose()
				current_pose_name = "auto_lift"
				auto_pick_state = "lift"
				auto_pick_timer = 0.0
		"lift":
			auto_pick_message = "lifting"
			if _joint_error_to_target() < 0.10 or auto_pick_timer > 1.10:
				target_joints = ARM_PRESETS["home"].duplicate()
				current_pose_name = "auto_return"
				auto_pick_state = "return"
				auto_pick_timer = 0.0
		"return":
			auto_pick_message = "returning home"
			if _joint_error_to_target() < 0.08 or auto_pick_timer > 1.60:
				_finish_auto_pick("complete")


func _auto_pick_drive_targets() -> Vector2:
	if auto_pick_state != "navigate" or auto_pick_target == null:
		return Vector2.ZERO
	var to_goal := auto_pick_nav_goal - robot_root.global_position
	to_goal.y = 0.0
	var distance := to_goal.length()
	if distance < 0.02:
		return Vector2.ZERO

	var desired := to_goal.normalized()
	var avoidance := _navigation_avoidance_vector()
	var steering := desired + avoidance
	if steering.length_squared() < 0.001:
		steering = desired
	steering = steering.normalized()

	var local_forward := robot_root.global_transform.basis.x.dot(steering)
	var local_right := robot_root.global_transform.basis.z.dot(steering)
	var face_target := auto_pick_target.global_position - robot_root.global_position
	face_target.y = 0.0
	if distance < 0.35 and face_target.length_squared() > 0.001:
		local_right = robot_root.global_transform.basis.z.dot(face_target.normalized())
	var linear: float = clamp(local_forward, -0.25, 1.0) * min(max_linear_speed * 0.42, distance * 1.15)
	if abs(local_right) > 0.45:
		linear *= 0.35
	var angular: float = clamp(local_right * 2.4, -1.0, 1.0) * max_angular_speed * 0.62
	return Vector2(linear, angular)


func _navigation_avoidance_vector() -> Vector3:
	var repulse := Vector3.ZERO
	var position := robot_root.global_position
	for obstacle in navigation_obstacles:
		var obstacle_position: Vector3 = obstacle.get("position", Vector3.ZERO) as Vector3
		var radius: float = float(obstacle.get("radius", 0.0))
		var delta := position - obstacle_position
		delta.y = 0.0
		var distance: float = delta.length()
		var influence: float = radius + 0.95
		if distance < 0.001 or distance > influence:
			continue
		var strength: float = (influence - distance) / influence
		repulse += delta.normalized() * strength * strength * 1.25
	return repulse


func _auto_pick_navigation_reached() -> bool:
	var goal_delta := auto_pick_nav_goal - robot_root.global_position
	goal_delta.y = 0.0
	var target_delta := auto_pick_target.global_position - robot_root.global_position
	target_delta.y = 0.0
	if target_delta.length_squared() < 0.001:
		return goal_delta.length() < AUTO_PICK_REACHED_DISTANCE
	var heading_error: float = abs(robot_root.global_transform.basis.z.dot(target_delta.normalized()))
	return goal_delta.length() < AUTO_PICK_REACHED_DISTANCE and heading_error < AUTO_PICK_REACHED_HEADING


func _solve_aubo_pick_pose(world_target: Vector3) -> Array:
	var local: Vector3 = robot_root.global_transform.affine_inverse() * world_target
	var lateral: float = clamp(local.z, -0.65, 0.65)
	var forward: float = clamp(local.x, 0.30, 1.05)
	var reach: float = clamp((forward - 0.42) / 0.63, 0.0, 1.0)
	var shoulder: float = clamp(1.18 - lateral * 0.58, -0.45, 2.65)
	var upper: float = lerp(-0.54, -0.98, reach)
	var fore: float = lerp(-1.18, -0.88, reach)
	var wrist1: float = lerp(0.28, 0.12, reach)
	var wrist2: float = lerp(-1.22, -1.02, reach)
	var wrist3: float = clamp(lateral * 0.28, -0.35, 0.35)
	return [shoulder, upper, fore, wrist1, wrist2, wrist3]


func _auto_lift_pose() -> Array:
	var pose := _solve_aubo_pick_pose(auto_pick_target.global_position)
	pose[1] = float(pose[1]) + 0.36
	pose[2] = float(pose[2]) - 0.28
	pose[3] = float(pose[3]) + 0.12
	return pose


func _joint_error_to_target() -> float:
	var error := 0.0
	for i in range(current_joints.size()):
		error = max(error, abs(float(current_joints[i]) - float(target_joints[i])))
	return error


func _attach_pickable(pickable: RigidBody3D) -> void:
	if pickable == null or grasp_anchor == null:
		return
	held_pickable = pickable
	pickable.set_meta("held", true)
	pickable.freeze = true
	pickable.collision_layer = 0
	pickable.collision_mask = 0
	var old_parent := pickable.get_parent()
	if old_parent != null:
		old_parent.remove_child(pickable)
	grasp_anchor.add_child(pickable)
	pickable.position = Vector3(0.0, -0.02, 0.055)
	pickable.rotation = Vector3.ZERO


func _finish_auto_pick(message: String) -> void:
	auto_pick_message = message
	auto_pick_state = "idle"
	auto_pick_timer = 0.0
	auto_pick_target = null
	target_linear = 0.0
	target_angular = 0.0


func _update_base(delta: float) -> void:
	if auto_pick_state == "navigate":
		var auto_targets := _auto_pick_drive_targets()
		target_linear = auto_targets.x
		target_angular = auto_targets.y
	else:
		var stick := _read_drive_stick()
		var radius := stick.length()
		if radius < input_deadzone:
			target_linear = 0.0
			target_angular = 0.0
		else:
			var unit: Vector2 = stick / max(radius, 0.0001)
			var speed_radius: float = clamp((min(radius, 1.0) - input_deadzone) / max(1.0 - input_deadzone, 0.0001), 0.0, 1.0)
			speed_radius = lerp(speed_radius, speed_radius * speed_radius, input_curve)
			target_linear = unit.y * speed_radius * max_linear_speed
			target_angular = unit.x * speed_radius * max_angular_speed

	var linear_rate := linear_acceleration
	if sign(target_linear) != sign(current_linear) or abs(target_linear) < abs(current_linear):
		linear_rate = braking_acceleration
	current_linear = move_toward(current_linear, target_linear, linear_rate * delta)
	current_angular = move_toward(current_angular, target_angular, angular_acceleration * delta)

	if abs(current_linear) < 0.002:
		current_linear = 0.0
	if abs(current_angular) < 0.002:
		current_angular = 0.0

	var previous_position := robot_root.global_position
	var previous_yaw := robot_root.rotation.y
	robot_root.rotation.y -= current_angular * delta
	var desired_velocity := robot_root.global_transform.basis.x * current_linear
	robot_root.velocity = Vector3(desired_velocity.x, 0.0, desired_velocity.z)
	robot_root.move_and_slide()
	_apply_push_impulses()
	robot_root.global_position.x = clamp(robot_root.global_position.x, -ARENA_LIMIT, ARENA_LIMIT)
	robot_root.global_position.z = clamp(robot_root.global_position.z, -ARENA_LIMIT, ARENA_LIMIT)
	robot_root.global_position.y = lerp(robot_root.global_position.y, robot_ground_y, _follow_alpha(suspension_follow_rate, delta))

	var actual_motion := robot_root.global_position - previous_position
	var actual_forward_distance := robot_root.global_transform.basis.x.dot(actual_motion)
	var actual_yaw_delta := robot_root.rotation.y - previous_yaw
	for wheel_state in wheel_nodes:
		var wheel := wheel_state["node"] as Node3D
		var side := float(wheel_state["side"])
		var turn_distance := -actual_yaw_delta * side * SCOUT_TRACK_WIDTH * 0.5
		wheel_state["spin"] = float(wheel_state["spin"]) - (actual_forward_distance + turn_distance) / SCOUT_WHEEL_RADIUS
		wheel.basis = wheel_state["rest_basis"] * Basis(SCOUT_WHEEL_SPIN_AXIS, float(wheel_state["spin"]))
	_update_vehicle_body_fx(delta)
	previous_linear = current_linear


func _apply_push_impulses() -> void:
	for i in range(robot_root.get_slide_collision_count()):
		var collision := robot_root.get_slide_collision(i)
		var collider := collision.get_collider()
		if collider is RigidBody3D:
			var rigid_body := collider as RigidBody3D
			var impulse_direction: Vector3 = robot_root.global_transform.basis.x * sign(current_linear)
			if impulse_direction.length_squared() < 0.001:
				impulse_direction = -collision.get_normal()
			var impulse_strength: float = clamp(abs(current_linear) * 1.6 + abs(current_angular) * 0.25, 0.0, 3.2)
			var contact_offset: Vector3 = collision.get_position() - rigid_body.global_position
			rigid_body.apply_impulse(impulse_direction.normalized() * impulse_strength, contact_offset)


func _update_arm(delta: float) -> void:
	var alpha := _follow_alpha(joint_follow_rate, delta)
	for i in range(current_joints.size()):
		current_joints[i] = lerp(float(current_joints[i]), float(target_joints[i]), alpha)
		var node := joint_nodes[i]
		node.basis = joint_rest_bases[i] * _ros_rpy_basis(Vector3(0.0, 0.0, current_joints[i]))


func _update_manual_arm(delta: float) -> void:
	var command := 0.0
	if Input.is_key_pressed(KEY_U):
		command += 1.0
	if Input.is_key_pressed(KEY_J):
		command -= 1.0

	var joypads := Input.get_connected_joypads()
	if not joypads.is_empty():
		var joypad_id: int = int(joypads[0])
		if Input.is_joy_button_pressed(joypad_id, JOY_BUTTON_DPAD_UP):
			command += 1.0
		if Input.is_joy_button_pressed(joypad_id, JOY_BUTTON_DPAD_DOWN):
			command -= 1.0

	if web_gamepad_connected:
		if _web_button_pressed(12):
			command += 1.0
		if _web_button_pressed(13):
			command -= 1.0

	if abs(command) < 0.001:
		return
	var limit: Array = ARM_JOINT_LIMITS[selected_joint_index]
	target_joints[selected_joint_index] = clamp(
		float(target_joints[selected_joint_index]) + command * manual_joint_rate * delta,
		float(limit[0]),
		float(limit[1])
	)
	current_pose_name = "manual"


func _update_gripper(delta: float) -> void:
	gripper_position = lerp(gripper_position, gripper_target, _follow_alpha(gripper_follow_rate, delta))
	if left_finger_joint != null:
		left_finger_joint.basis = _ros_rpy_basis(Vector3(0.0, 0.0, -gripper_position))
	if right_finger_joint != null:
		right_finger_joint.basis = _ros_rpy_basis(Vector3(0.0, 0.0, gripper_position))


func _update_camera(delta: float) -> void:
	var orbit_input := _read_camera_orbit_input()
	camera_yaw += orbit_input * camera_orbit_speed * delta

	var yaw: float = robot_root.rotation.y + camera_yaw
	var speed_zoom: float = clamp(abs(current_linear) / max_linear_speed, 0.0, 1.0) * 0.38
	var turn_shoulder: float = clamp(current_angular / max_angular_speed, -1.0, 1.0) * 0.20
	if not cinematic_camera:
		speed_zoom = 0.0
		turn_shoulder = 0.0
	var offset: Vector3 = Basis(Vector3.UP, yaw) * Vector3(-camera_distance - speed_zoom, camera_height, turn_shoulder)
	var focus: Vector3 = robot_root.global_position + Vector3(0.18, 0.64, 0.0)
	var desired_position: Vector3 = focus + offset
	camera.global_position = camera.global_position.lerp(desired_position, _follow_alpha(camera_follow_rate, delta))
	var look_ahead: Vector3 = robot_root.global_transform.basis.x * clamp(current_linear, -0.35, 0.80) * 0.22
	camera.look_at(focus + look_ahead, Vector3.UP)


func _update_vehicle_body_fx(delta: float) -> void:
	var acceleration: float = (current_linear - previous_linear) / max(delta, 0.0001)
	var target_pitch: float = clamp(acceleration / max(linear_acceleration, 0.001), -1.0, 1.0) * 0.040
	var target_roll: float = clamp(current_angular / max_angular_speed, -1.0, 1.0) * 0.055
	var bump_phase: float = Time.get_ticks_msec() * 0.001 * (8.0 + abs(current_linear) * 3.0)
	var bump_amount: float = sin(bump_phase) * clamp(abs(current_linear) / max_linear_speed, 0.0, 1.0) * 0.006
	visual_root.rotation.z = lerp(visual_root.rotation.z, -target_pitch, _follow_alpha(9.0, delta))
	visual_root.rotation.x = lerp(visual_root.rotation.x, target_roll, _follow_alpha(8.0, delta))
	visual_root.position.y = lerp(visual_root.position.y, bump_amount, _follow_alpha(14.0, delta))


func _sample_track_height(_position: Vector3) -> float:
	return 0.0


func _update_hud() -> void:
	var input_label := "web" if web_gamepad_connected else ("native" if Input.get_connected_joypads().size() > 0 else "keyboard")
	status_label.text = "Arachne Playable Showcase\nBackend: %s    Input: %s/%s    Pose: %s    Gripper: %.2f\nJoint: %s    Auto: %s    v: %.2f m/s    yaw: %.2f rad/s    cam axis: %s" % [
		backend_label,
		input_label,
		gamepad_bridge_label,
		current_pose_name,
		gripper_position,
		ARM_JOINT_NAMES[selected_joint_index],
		auto_pick_message,
		current_linear,
		-current_angular,
		camera_axis_label,
	]


func _publish_ros_placeholders() -> void:
	if ros_bridge == null:
		return
	ros_bridge.publish_cmd_vel(current_linear, -current_angular)
	var names := ARM_JOINT_NAMES.duplicate()
	var positions := current_joints.duplicate()
	names.append("ms42dc_left_finger_joint")
	names.append("ms42dc_right_finger_joint")
	positions.append(gripper_position)
	positions.append(-gripper_position)
	ros_bridge.publish_joint_states(names, positions)
	ros_bridge.publish_odom(robot_root.global_position, robot_root.rotation.y, current_linear, -current_angular)
	ros_bridge.publish_tf([
		{"parent": "odom", "child": "base_link", "position": robot_root.global_position, "yaw": robot_root.rotation.y},
		{"parent": "base_link", "child": "arm_mount_link"},
	])


func _read_drive_stick() -> Vector2:
	if forced_drive_enabled:
		return forced_drive_stick

	var turn := 0.0
	var forward := 0.0
	if Input.is_key_pressed(KEY_W):
		forward += 1.0
	if Input.is_key_pressed(KEY_S):
		forward -= 1.0
	if Input.is_key_pressed(KEY_D):
		turn += 1.0
	if Input.is_key_pressed(KEY_A):
		turn -= 1.0

	var joypads := Input.get_connected_joypads()
	if joypads.size() > 0:
		var joypad_id: int = int(joypads[0])
		if not _native_dpad_pressed(joypad_id):
			turn += _deadzone_axis(Input.get_joy_axis(joypad_id, JOY_AXIS_LEFT_X), 0.08)
			forward += -_deadzone_axis(Input.get_joy_axis(joypad_id, JOY_AXIS_LEFT_Y), 0.08)

	if web_gamepad_connected and not _web_dpad_pressed():
		turn += _read_web_axis(0, 0.08)
		forward += -_read_web_axis(1, 0.08)

	var stick := Vector2(turn, forward)
	if stick.length() > 1.0:
		stick = stick.normalized()
	return stick


func _read_camera_orbit_input() -> float:
	var orbit_input := 0.0
	if Input.is_key_pressed(KEY_Q):
		orbit_input -= 1.0
	if Input.is_key_pressed(KEY_E):
		orbit_input += 1.0

	var joypads := Input.get_connected_joypads()
	if not joypads.is_empty():
		var joypad_id: int = int(joypads[0])
		if camera_axis_index >= 0:
			orbit_input += _deadzone_axis(Input.get_joy_axis(joypad_id, camera_axis_index), 0.10)
		else:
			orbit_input += _read_best_camera_axis(joypad_id)

	if web_gamepad_connected:
		var web_orbit := _read_web_axis(2, 0.10)
		orbit_input += web_orbit
		if abs(web_orbit) > 0.0:
			camera_axis_label = "web:2"

	return clamp(orbit_input, -1.0, 1.0)


func _read_best_camera_axis(joypad_id: int) -> float:
	var candidates: Array[int] = [JOY_AXIS_RIGHT_X, JOY_AXIS_RIGHT_Y, 4, 5]
	var best_value := 0.0
	var best_axis := -1
	for axis in candidates:
		var value := _deadzone_axis(Input.get_joy_axis(joypad_id, axis), 0.10)
		if abs(value) > abs(best_value):
			best_value = value
			best_axis = axis
	if best_axis >= 0:
		camera_axis_label = "auto:%d" % best_axis
	return best_value


func _select_pose(pose_name: String) -> void:
	if not ARM_PRESETS.has(pose_name):
		return
	current_pose_name = pose_name
	target_joints = ARM_PRESETS[pose_name].duplicate()
	if pose_name == "grasp" or pose_name == "lift":
		_set_gripper(true)
	elif pose_name == "home" or pose_name == "ready":
		_set_gripper(false)


func _select_previous_joint() -> void:
	selected_joint_index = (selected_joint_index - 1 + ARM_JOINT_NAMES.size()) % ARM_JOINT_NAMES.size()


func _select_next_joint() -> void:
	selected_joint_index = (selected_joint_index + 1) % ARM_JOINT_NAMES.size()


func _toggle_gripper() -> void:
	_set_gripper(gripper_target < gripper_closed_position * 0.5)


func _set_gripper(closed: bool) -> void:
	gripper_target = gripper_closed_position if closed else 0.0


func _reset_demo() -> void:
	robot_root.position = Vector3(0.0, robot_ground_y, 0.0)
	robot_root.rotation = Vector3.ZERO
	robot_root.velocity = Vector3.ZERO
	camera_yaw = 0.0
	target_linear = 0.0
	target_angular = 0.0
	current_linear = 0.0
	current_angular = 0.0
	previous_linear = 0.0
	auto_pick_state = "idle"
	auto_pick_message = "idle"
	auto_pick_timer = 0.0
	auto_pick_target = null
	right_stick_hold_time = 0.0
	right_stick_long_press_fired = false
	visual_root.position = Vector3.ZERO
	visual_root.rotation = Vector3.ZERO
	for wheel_state in wheel_nodes:
		var wheel := wheel_state["node"] as Node3D
		wheel_state["spin"] = 0.0
		wheel.basis = wheel_state["rest_basis"]
	selected_joint_index = 0
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


func _load_visual(paths: Array, fallback_mesh: Mesh, fallback_material: Material, local_scale: Vector3, override_material := true) -> Node3D:
	for path in paths:
		if not FileAccess.file_exists(path) and not ResourceLoader.exists(path):
			continue
		if path.to_lower().ends_with(".stl"):
			var stl_mesh := _load_stl_mesh(path)
			if stl_mesh != null:
				var stl_instance := MeshInstance3D.new()
				stl_instance.mesh = stl_mesh
				stl_instance.scale = local_scale
				if override_material:
					stl_instance.material_override = fallback_material
				return stl_instance
		var resource: Resource = load(path)
		if resource is PackedScene:
			var packed_scene := resource as PackedScene
			var scene: Node = packed_scene.instantiate()
			if scene is Node3D:
				scene.scale = local_scale
				if override_material:
					_apply_material_to_meshes(scene, fallback_material)
				return scene
			var wrapper := Node3D.new()
			wrapper.scale = local_scale
			wrapper.add_child(scene)
			if override_material:
				_apply_material_to_meshes(wrapper, fallback_material)
			return wrapper
		if resource is Mesh:
			var mesh_instance := MeshInstance3D.new()
			mesh_instance.mesh = resource
			mesh_instance.scale = local_scale
			if override_material:
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


func _add_cylinder_visual(name: String, radius: float, height: float, material: Material) -> MeshInstance3D:
	var mesh_instance := MeshInstance3D.new()
	mesh_instance.name = name
	mesh_instance.mesh = _cylinder_mesh(radius, height)
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


func _read_web_axis(index: int, deadzone: float) -> float:
	if index < 0 or index >= web_gamepad_axes.size():
		return 0.0
	return _deadzone_axis(float(web_gamepad_axes[index]), deadzone)


func _web_button_pressed(index: int) -> bool:
	return _web_button_value_from_array(web_gamepad_buttons, index) > 0.5


func _web_button_just_pressed(index: int) -> bool:
	if index < 0 or index >= web_gamepad_button_edges.size():
		return false
	return bool(web_gamepad_button_edges[index])


func _web_dpad_pressed() -> bool:
	return _web_button_pressed(12) or _web_button_pressed(13) or _web_button_pressed(14) or _web_button_pressed(15)


func _native_dpad_pressed(joypad_id: int) -> bool:
	return (
		Input.is_joy_button_pressed(joypad_id, JOY_BUTTON_DPAD_UP)
		or Input.is_joy_button_pressed(joypad_id, JOY_BUTTON_DPAD_DOWN)
		or Input.is_joy_button_pressed(joypad_id, JOY_BUTTON_DPAD_LEFT)
		or Input.is_joy_button_pressed(joypad_id, JOY_BUTTON_DPAD_RIGHT)
	)


func _web_button_value_from_array(buttons: Array, index: int) -> float:
	if index < 0 or index >= buttons.size():
		return 0.0
	var value = buttons[index]
	if typeof(value) == TYPE_BOOL:
		return 1.0 if bool(value) else 0.0
	if typeof(value) == TYPE_DICTIONARY:
		return float(value.get("value", 0.0))
	return float(value)


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


func _has_user_arg(flag: String) -> bool:
	for arg in OS.get_cmdline_user_args():
		if arg == flag:
			return true
	for arg in OS.get_cmdline_args():
		if arg == flag:
			return true
	return false


func _run_self_test() -> void:
	var errors: Array[String] = []
	await get_tree().physics_frame
	var start_position := robot_root.global_position
	forced_drive_enabled = true
	forced_drive_stick = Vector2(0.30, 1.0)
	for _i in range(110):
		await get_tree().physics_frame
	forced_drive_stick = Vector2(-0.65, 0.35)
	for _i in range(70):
		await get_tree().physics_frame
	forced_drive_stick = Vector2.ZERO
	for _i in range(30):
		await get_tree().physics_frame
	forced_drive_enabled = false

	var travel := robot_root.global_position.distance_to(start_position)
	if travel < 0.75:
		errors.append("robot did not travel far enough during scripted drive: %.3f m" % travel)
	if robot_root.global_position.y < -0.05:
		errors.append("robot sank below the ground: y=%.3f" % robot_root.global_position.y)
	if camera.global_position.distance_to(robot_root.global_position) > 5.0:
		errors.append("follow camera drifted too far from robot")
	if wheel_nodes.size() != 4:
		errors.append("expected four wheel visual nodes, got %d" % wheel_nodes.size())
	if _count_mesh_instances(robot_root) < 20:
		errors.append("robot visual mesh count is unexpectedly low")
	if _count_collision_shapes(self) < 24:
		errors.append("office map collision coverage is unexpectedly low")
	if find_child("OfficeWallNorthA", true, false) == null:
		errors.append("office map did not load")
	if abs(_sample_track_height(Vector3(0.0, 0.0, -5.35))) > 0.001:
		errors.append("track height should stay flat")
	if ros_bridge == null or ros_bridge.last_cmd_vel.is_empty():
		errors.append("ROS2 bridge placeholder did not receive cmd_vel")
	if pickable_objects.size() < 7:
		errors.append("expected pickable bottles and balls in the showcase map")
	var nearest_pickable: RigidBody3D = _find_nearest_pickable()
	if nearest_pickable == null:
		errors.append("auto-pick target search did not find a pickable object")
	else:
		var solved_pose: Array = _solve_aubo_pick_pose(nearest_pickable.global_position)
		if solved_pose.size() != ARM_JOINT_NAMES.size():
			errors.append("auto-pick IK returned %d joints" % solved_pose.size())
		auto_pick_state = "navigate"
		auto_pick_target = nearest_pickable
		auto_pick_nav_goal = _approach_goal_for_pickable(nearest_pickable)
		var drive_command: Vector2 = _auto_pick_drive_targets()
		if drive_command.length() < 0.001:
			errors.append("auto-pick navigation generated a zero drive command")
		auto_pick_state = "idle"
		auto_pick_target = null

	if errors.is_empty():
		print("Arachne Godot self-test passed: travel=%.2f m, meshes=%d, backend=%s" % [travel, _count_mesh_instances(robot_root), backend_label])
		get_tree().quit(0)
	else:
		for error in errors:
			push_error(error)
		get_tree().quit(1)


func _count_mesh_instances(node: Node) -> int:
	var count := 0
	var stack: Array[Node] = [node]
	while not stack.is_empty():
		var current: Node = stack.pop_back()
		if current is MeshInstance3D:
			count += 1
		for child in current.get_children():
			stack.append(child)
	return count


func _count_collision_shapes(node: Node) -> int:
	var count := 0
	var stack: Array[Node] = [node]
	while not stack.is_empty():
		var current: Node = stack.pop_back()
		if current is CollisionShape3D:
			count += 1
		for child in current.get_children():
			stack.append(child)
	return count


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
