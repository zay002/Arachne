extends Node
class_name Ros2BridgePlaceholder

signal cmd_vel_published(linear_x: float, angular_z: float)
signal joint_states_published(names: Array, positions: Array)
signal odom_published(pose: Dictionary)
signal tf_published(transforms: Array)

@export var enabled := false
@export var debug_print := false
@export var cmd_vel_topic := "/cmd_vel"
@export var joint_states_topic := "/joint_states"
@export var odom_topic := "/odom"
@export var tf_topic := "/tf"

var last_cmd_vel := {"linear_x": 0.0, "angular_z": 0.0}
var last_joint_states := {"name": [], "position": []}
var last_odom := {}
var last_tf := []

func publish_cmd_vel(linear_x: float, angular_z: float) -> void:
	last_cmd_vel = {"linear_x": linear_x, "angular_z": angular_z}
	cmd_vel_published.emit(linear_x, angular_z)
	if debug_print:
		print("%s linear.x=%.3f angular.z=%.3f" % [cmd_vel_topic, linear_x, angular_z])


func publish_joint_states(names: Array, positions: Array) -> void:
	last_joint_states = {"name": names.duplicate(), "position": positions.duplicate()}
	joint_states_published.emit(names, positions)
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
	if debug_print:
		print("%s x=%.3f z=%.3f yaw=%.3f" % [odom_topic, position.x, position.z, yaw])


func publish_tf(transforms: Array) -> void:
	last_tf = transforms.duplicate(true)
	tf_published.emit(transforms)
	if debug_print:
		print("%s transforms=%d" % [tf_topic, transforms.size()])
