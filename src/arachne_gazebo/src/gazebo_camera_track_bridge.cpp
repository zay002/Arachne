#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/vector3.hpp>
#include <gz/msgs/cameratrack.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class GazeboCameraTrackBridge : public rclcpp::Node
{
public:
  GazeboCameraTrackBridge()
  : rclcpp::Node("gazebo_camera_track_bridge")
  {
    this->declare_parameter("offset_topic", "/arachne/gazebo_camera/follow_offset");
    this->declare_parameter("gz_track_topic", "/gui/track");
    this->declare_parameter("target_name", "arachne");
    this->declare_parameter("follow_pgain", 0.85);
    this->declare_parameter("track_pgain", 1.0);
    this->declare_parameter("publish_rate", 60.0);
    this->declare_parameter("track_height", 0.58);

    const auto offsetTopic = this->get_parameter("offset_topic").as_string();
    const auto gzTrackTopic = this->get_parameter("gz_track_topic").as_string();
    this->targetName = this->get_parameter("target_name").as_string();
    this->followPgain = this->get_parameter("follow_pgain").as_double();
    this->trackPgain = this->get_parameter("track_pgain").as_double();
    this->trackHeight = this->get_parameter("track_height").as_double();
    const auto publishRate = this->get_parameter("publish_rate").as_double();

    this->publisher = this->gzNode.Advertise<gz::msgs::CameraTrack>(gzTrackTopic);
    this->offsetSub = this->create_subscription<geometry_msgs::msg::Vector3>(
      offsetTopic,
      rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->offset = *msg;
        this->haveOffset = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(publishRate, 1.0));
    this->timer = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GazeboCameraTrackBridge::publishTrackMessage, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Gazebo camera track bridge ready: %s -> %s, target=%s",
      offsetTopic.c_str(),
      gzTrackTopic.c_str(),
      this->targetName.c_str());
  }

private:
  void publishTrackMessage()
  {
    geometry_msgs::msg::Vector3 currentOffset;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (!this->haveOffset) {
        return;
      }
      currentOffset = this->offset;
    }

    gz::msgs::CameraTrack msg;
    msg.set_track_mode(gz::msgs::CameraTrack::FOLLOW_LOOK_AT);

    auto * followTarget = msg.mutable_follow_target();
    followTarget->set_name(this->targetName);
    followTarget->set_type(gz::msgs::Entity::MODEL);

    auto * trackTarget = msg.mutable_track_target();
    trackTarget->set_name(this->targetName);
    trackTarget->set_type(gz::msgs::Entity::MODEL);

    auto * followOffset = msg.mutable_follow_offset();
    followOffset->set_x(currentOffset.x);
    followOffset->set_y(currentOffset.y);
    followOffset->set_z(currentOffset.z);

    auto * trackOffset = msg.mutable_track_offset();
    trackOffset->set_z(this->trackHeight);

    msg.set_follow_pgain(this->followPgain);
    msg.set_track_pgain(this->trackPgain);
    this->publisher.Publish(msg);
  }

  gz::transport::Node gzNode;
  gz::transport::Node::Publisher publisher;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr offsetSub;
  rclcpp::TimerBase::SharedPtr timer;
  std::mutex mutex;
  geometry_msgs::msg::Vector3 offset;
  bool haveOffset{false};
  std::string targetName;
  double followPgain{0.85};
  double trackPgain{1.0};
  double trackHeight{0.58};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboCameraTrackBridge>());
  rclcpp::shutdown();
  return 0;
}
