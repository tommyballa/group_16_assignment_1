#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher()
  : Node("initial_pose_publisher")
  { 
      // Create QoS profile, only keep last message if subscribers start later
      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local();
      qos.reliable();    // Resend if needed

      // Create publisher over /initial_pose
      pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", qos);

      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&InitialPosePublisher::publish_once, this));

      // Check only one init pose is published
      published_ = false;
  }

private:
  void publish_once()
  {
    if (published_) return;

    // Init empty message with timestamp
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    // Coord frame set to map for AMCL initial pose
    msg.header.frame_id = "map";

    // Initial pos in (0,0,0)
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    // No orientation, face x axis
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    for (int i = 0; i < 36; i++)
      msg.pose.covariance[i] = 0.0;

    // Publish on /initial_pose and log
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Initial pose pubblicata");

    published_ = true;
    timer_->cancel();  // Stop timer after first publish
  }

  // Variables
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool published_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
