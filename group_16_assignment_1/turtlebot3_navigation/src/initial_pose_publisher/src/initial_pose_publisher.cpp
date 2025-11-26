#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class InitialPosePublisher : public rclcpp::Node
{
public:
  InitialPosePublisher()
  : Node("initial_pose_publisher")
  {
    // QoS compatible with RViz / Nav2
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();
    qos.reliable();

    pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", qos);

    // Publish every second
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&InitialPosePublisher::publish_pose, this));

    // Sub to AMCL -> AMCL publishes something only after RViz accepted the initialpose
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose",
      10,
      std::bind(&InitialPosePublisher::amcl_callback, this, std::placeholders::_1));

    received_amcl_pose_ = false;
  }

private:
  void publish_pose()
  {
    if (received_amcl_pose_) {
      return;
    }

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
  }

  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr)
  {
    if (!received_amcl_pose_) {
      RCLCPP_INFO(this->get_logger(), "AMCL ha ricevuto la initial pose. Stop pubblicazione.");
      received_amcl_pose_ = true;

      // Stop publishing
      timer_->cancel();
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

  bool received_amcl_pose_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}

