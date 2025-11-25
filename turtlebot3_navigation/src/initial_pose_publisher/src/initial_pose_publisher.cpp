#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher()
  : Node("initial_pose_publisher")
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", qos);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&InitialPosePublisher::publish_once, this));

    published_ = false;
  }

private:
  void publish_once()
  {
    if (published_) return;

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.pose.pose.orientation.w = 1.0;

    for (int i = 0; i < 36; i++)
      msg.pose.covariance[i] = 0.0;

    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Initial pose pubblicata");

    published_ = true;
    timer_->cancel();  // fermo il timer dopo la prima pubblicazione
  }

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

