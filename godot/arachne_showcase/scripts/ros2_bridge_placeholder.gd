extends Node
class_name Ros2BridgePlaceholder

signal cmd_vel_published(linear_x: float, angular_z: float)
signal joint_states_published(names: Array, positions: Array)
signal odom_published(pose: Dictionary)
signal tf_published(transforms: Array)

@export var enabled := false
@export var debug_print := false
@export var transport := "memory"
@export var udp_host := "127.0.0.1"
@export var udp_port := 8765
@export var cmd_vel_topic := "/cmd_vel"
@export var joint_states_topic := "/joint_states"
@export var odom_topic := "/odom"
@export var tf_topic := "/tf"

var udp := PacketPeerUDP.new()
var last_cmd_vel := {"linear_x": 0.0, "angular_z": 0.0}
var last_joint_states := {"name": [], "position": []}
var last_odom := {}
var last_tf := []

func configure_from_environment() -> void:
	var requested_transport := OS.get_environment("ARACHNE_GODOT_BRIDGE")
	var ros_available := OS.get_environment("ARACHNE_ROS2_AVAILABLE") == "1"
	if requested_transport.is_empty() and (not OS.get_environment("ROS_DISTRO").is_empty() or ros_available):
		requested_transport = "udp"
	if not requested_transport.is_empty():
		transport = requested_transport
		enabled = transport != "memory"
	var requested_host := OS.get_environment("ARACHNE_GODOT_UDP_HOST")
	if not requested_host.is_empty():
		udp_host = requested_host
	var requested_port := OS.get_environment("ARACHNE_GODOT_UDP_PORT")
	if requested_port.is_valid_int():
		udp_port = requested_port.to_int()
	if transport == "udp" and enabled:
		var err := udp.connect_to_host(udp_host, udp_port)
		if err != OK:
			push_warning("Failed to configure UDP bridge to %s:%d" % [udp_host, udp_port])


func publish_cmd_vel(linear_x: float, angular_z: float) -> void:
	last_cmd_vel = {"linear_x": linear_x, "angular_z": angular_z}
	cmd_vel_published.emit(linear_x, angular_z)
	_send(cmd_vel_topic, last_cmd_vel)
	if debug_print:
		print("%s linear.x=%.3f angular.z=%.3f" % [cmd_vel_topic, linear_x, angular_z])


func publish_joint_states(names: Array, positions: Array) -> void:
	last_joint_states = {"name": names.duplicate(), "position": positions.duplicate()}
	joint_states_published.emit(names, positions)
	_send(joint_states_topic, last_joint_states)
	if debug_print:
		print("%s %s" % [joint_states_topic, str(last_joint_states)])


func publish_odom(position: Vector3, yaw: float, linear_x: float, angular_z: float) -> void:
	last_odom = {
		"frame_id": "odom",
		"child_frame_id": "base_link",
		"position": position,
		"yaw": yaw,
		"linear_x": linear_x,
		"angular_z": angular_z,
	}
	odom_published.emit(last_odom)
	_send(odom_topic, last_odom)
	if debug_print:
		print("%s x=%.3f z=%.3f yaw=%.3f" % [odom_topic, position.x, position.z, yaw])


func publish_tf(transforms: Array) -> void:
	last_tf = transforms.duplicate(true)
	tf_published.emit(transforms)
	_send(tf_topic, last_tf)
	if debug_print:
		print("%s transforms=%d" % [tf_topic, transforms.size()])


func _send(topic: String, payload: Variant) -> void:
	if not enabled or transport != "udp":
		return
	var packet := {
		"topic": topic,
		"stamp_msec": Time.get_ticks_msec(),
		"payload": _json_safe(payload),
	}
	udp.put_packet(JSON.stringify(packet).to_utf8_buffer())


func _json_safe(value: Variant) -> Variant:
	if value is Vector3:
		return {"x": value.x, "y": value.y, "z": value.z}
	if value is Array:
		var out := []
		for item in value:
			out.append(_json_safe(item))
		return out
	if value is Dictionary:
		var out := {}
		for key in value.keys():
			out[str(key)] = _json_safe(value[key])
		return out
	return value
