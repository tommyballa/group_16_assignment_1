#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <cmath>

class NavigationNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigationNode()
  : Node("navigation_node")
  {
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal",
      10,
      std::bind(&NavigationNode::goal_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Navigation node ready. Listening on /goal...");
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // ---------------------- SUBSCRIBER: NEW GOAL RECEIVED ----------------------
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    double qw = msg->pose.orientation.w;
    double qz = msg->pose.orientation.z;

    double yaw = 2.0 * std::atan2(qz, qw);

    RCLCPP_INFO(this->get_logger(),
        "Received goal on /goal: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);

    send_goal(*msg);
  }

  // ---------------------- SEND GOAL TO NAV2 ----------------------------------
  void send_goal(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server NOT available");
      return;
    }

    auto goal = NavigateToPose::Goal();
    goal.pose = pose_msg;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result)
      {
        RCLCPP_INFO(this->get_logger(), "Navigation finished with code: %d", result.code);
      };

    nav_client_->async_send_goal(goal, send_goal_options);

    RCLCPP_INFO(this->get_logger(), "Goal sent to Nav2.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}

