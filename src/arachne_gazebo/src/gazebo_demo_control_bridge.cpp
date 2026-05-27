#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/joint_trajectory.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class GazeboDemoControlBridge : public rclcpp::Node
{
public:
  GazeboDemoControlBridge()
  : rclcpp::Node("gazebo_demo_control_bridge")
  {
    this->declare_parameter("gripper_command_topic", "/arachne/gripper/command");
    this->declare_parameter("arm_joint_state_topic", "/arachne/gui_joint_states");
    this->declare_parameter("reset_topic", "/arachne/demo/reset");
    this->declare_parameter("gripper_type", "ms42dc");
    this->declare_parameter("world_name", "arachne_showroom");
    this->declare_parameter("target_name", "arachne");
    this->declare_parameter("reset_x", 0.0);
    this->declare_parameter("reset_y", 0.0);
    this->declare_parameter("reset_z", 0.22);
    this->declare_parameter("reset_yaw", 0.0);
    this->declare_parameter("gripper_open_position", 0.0);
    this->declare_parameter("gripper_closed_position", 0.6);
    this->declare_parameter("gripper_max_velocity", 1.2);
    this->declare_parameter("ms42dc_left_topic", "/arachne/gazebo/ms42dc_left_finger/command");
    this->declare_parameter("ms42dc_right_topic", "/arachne/gazebo/ms42dc_right_finger/command");
    this->declare_parameter("arm_trajectory_topic", "/model/arachne/joint_trajectory");
    this->declare_parameter("publish_rate", 60.0);

    const auto gripperCommandTopic = this->get_parameter("gripper_command_topic").as_string();
    const auto armJointStateTopic = this->get_parameter("arm_joint_state_topic").as_string();
    const auto resetTopic = this->get_parameter("reset_topic").as_string();
    this->gripperType = this->get_parameter("gripper_type").as_string();
    this->worldName = this->get_parameter("world_name").as_string();
    this->targetName = this->get_parameter("target_name").as_string();
    this->resetX = this->get_parameter("reset_x").as_double();
    this->resetY = this->get_parameter("reset_y").as_double();
    this->resetZ = this->get_parameter("reset_z").as_double();
    this->resetYaw = this->get_parameter("reset_yaw").as_double();
    this->gripperOpenPosition = this->get_parameter("gripper_open_position").as_double();
    this->gripperClosedPosition = this->get_parameter("gripper_closed_position").as_double();
    this->gripperMaxVelocity = this->get_parameter("gripper_max_velocity").as_double();
    const auto leftTopic = this->get_parameter("ms42dc_left_topic").as_string();
    const auto rightTopic = this->get_parameter("ms42dc_right_topic").as_string();
    const auto armTrajectoryTopic = this->get_parameter("arm_trajectory_topic").as_string();
    const auto publishRate = this->get_parameter("publish_rate").as_double();

    this->leftFingerPublisher = this->gzNode.Advertise<gz::msgs::Double>(leftTopic);
    this->rightFingerPublisher = this->gzNode.Advertise<gz::msgs::Double>(rightTopic);
    this->armTrajectoryPublisher = this->gzNode.Advertise<gz::msgs::JointTrajectory>(armTrajectoryTopic);

    this->gripperPosition = this->gripperOpenPosition;
    this->gripperTarget = this->gripperPosition;
    this->lastUpdate = this->now();

    this->gripperSub = this->create_subscription<std_msgs::msg::String>(
      gripperCommandTopic,
      10,
      std::bind(&GazeboDemoControlBridge::onGripperCommand, this, std::placeholders::_1));
    this->resetSub = this->create_subscription<std_msgs::msg::Empty>(
      resetTopic,
      10,
      std::bind(&GazeboDemoControlBridge::onReset, this, std::placeholders::_1));
    this->armSub = this->create_subscription<sensor_msgs::msg::JointState>(
      armJointStateTopic,
      10,
      std::bind(&GazeboDemoControlBridge::onArmJointState, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(publishRate, 1.0));
    this->timer = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GazeboDemoControlBridge::tick, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Gazebo demo control bridge ready: gripper=%s, arm trajectory=%s, reset=%s",
      this->gripperType.c_str(),
      armTrajectoryTopic.c_str(),
      resetTopic.c_str());
  }

private:
  void onGripperCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto command = msg->data;
    if (command == "open") {
      this->gripperTarget = this->gripperOpenPosition;
    } else if (command == "close") {
      this->gripperTarget = this->gripperClosedPosition;
    } else if (command == "stop") {
      this->gripperTarget = this->gripperPosition;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown gripper command: %s", command.c_str());
    }
  }

  void onArmJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty()) {
      return;
    }

    static const std::vector<std::string> armJoints = {
      "aubo_shoulder_joint",
      "aubo_upperArm_joint",
      "aubo_foreArm_joint",
      "aubo_wrist1_joint",
      "aubo_wrist2_joint",
      "aubo_wrist3_joint",
    };

    std::unordered_map<std::string, double> positions;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      positions[msg->name[i]] = msg->position[i];
    }

    gz::msgs::JointTrajectory trajectory;
    auto * point = trajectory.add_points();
    for (const auto & joint : armJoints) {
      const auto found = positions.find(joint);
      if (found == positions.end()) {
        return;
      }
      trajectory.add_joint_names(joint);
      point->add_positions(found->second);
    }
    point->mutable_time_from_start()->set_nsec(120000000);
    this->armTrajectoryPublisher.Publish(trajectory);
  }

  void onReset(const std_msgs::msg::Empty::SharedPtr)
  {
    this->gripperPosition = this->gripperOpenPosition;
    this->gripperTarget = this->gripperOpenPosition;
    this->publishGripperPosition();
    this->publishArmTrajectory(this->defaultArmPositions);
    this->resetGazeboModel();
    RCLCPP_INFO(this->get_logger(), "Reset Gazebo demo model pose and commands");
  }

  void tick()
  {
    const auto now = this->now();
    const auto dt = std::max((now - this->lastUpdate).seconds(), 0.0);
    this->lastUpdate = now;

    const auto delta = this->gripperTarget - this->gripperPosition;
    const auto maxStep = this->gripperMaxVelocity * dt;
    if (std::abs(delta) <= maxStep) {
      this->gripperPosition = this->gripperTarget;
    } else {
      this->gripperPosition += std::copysign(maxStep, delta);
    }

    this->publishGripperPosition();
  }

  void publishGripperPosition()
  {
    if (this->gripperType == "ms42dc") {
      gz::msgs::Double left;
      left.set_data(this->gripperPosition);
      gz::msgs::Double right;
      right.set_data(-this->gripperPosition);
      this->leftFingerPublisher.Publish(left);
      this->rightFingerPublisher.Publish(right);
    }
  }

  void publishArmTrajectory(const std::vector<double> & positions)
  {
    static const std::vector<std::string> armJoints = {
      "aubo_shoulder_joint",
      "aubo_upperArm_joint",
      "aubo_foreArm_joint",
      "aubo_wrist1_joint",
      "aubo_wrist2_joint",
      "aubo_wrist3_joint",
    };

    if (positions.size() != armJoints.size()) {
      return;
    }

    gz::msgs::JointTrajectory trajectory;
    auto * point = trajectory.add_points();
    for (size_t i = 0; i < armJoints.size(); ++i) {
      trajectory.add_joint_names(armJoints[i]);
      point->add_positions(positions[i]);
    }
    point->mutable_time_from_start()->set_nsec(120000000);
    this->armTrajectoryPublisher.Publish(trajectory);
  }

  void resetGazeboModel()
  {
    const auto worldControlService = "/world/" + this->worldName + "/control";
    gz::msgs::WorldControl control;
    control.mutable_reset()->set_model_only(true);
    gz::msgs::Boolean controlReply;
    bool controlResult = false;
    const bool controlExecuted = this->gzNode.Request(
      worldControlService,
      control,
      1000u,
      controlReply,
      controlResult);

    if (!controlExecuted || !controlResult || !controlReply.data()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Gazebo world model reset service did not confirm success: %s",
        worldControlService.c_str());
    }

    const auto setPoseService = "/world/" + this->worldName + "/set_pose";
    gz::msgs::Pose pose;
    pose.set_name(this->targetName);
    pose.mutable_position()->set_x(this->resetX);
    pose.mutable_position()->set_y(this->resetY);
    pose.mutable_position()->set_z(this->resetZ);
    const auto halfYaw = this->resetYaw * 0.5;
    pose.mutable_orientation()->set_w(std::cos(halfYaw));
    pose.mutable_orientation()->set_z(std::sin(halfYaw));

    gz::msgs::Boolean poseReply;
    bool poseResult = false;
    const bool poseExecuted = this->gzNode.Request(
      setPoseService,
      pose,
      1000u,
      poseReply,
      poseResult);

    if (!poseExecuted || !poseResult || !poseReply.data()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Gazebo model pose reset service did not confirm success: %s",
        setPoseService.c_str());
    }
  }

  gz::transport::Node gzNode;
  gz::transport::Node::Publisher leftFingerPublisher;
  gz::transport::Node::Publisher rightFingerPublisher;
  gz::transport::Node::Publisher armTrajectoryPublisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripperSub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr resetSub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr armSub;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Time lastUpdate;
  std::string gripperType;
  std::string worldName;
  std::string targetName;
  double resetX{0.0};
  double resetY{0.0};
  double resetZ{0.22};
  double resetYaw{0.0};
  double gripperOpenPosition{0.0};
  double gripperClosedPosition{0.6};
  double gripperMaxVelocity{1.2};
  double gripperPosition{0.0};
  double gripperTarget{0.0};
  std::vector<double> defaultArmPositions{1.664, 0.034, -1.324, 0.034, -1.732, 0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboDemoControlBridge>());
  rclcpp::shutdown();
  return 0;
}
