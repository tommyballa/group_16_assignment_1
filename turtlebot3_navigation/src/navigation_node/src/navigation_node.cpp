#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class ExtraNavigationNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ExtraNavigationNode()
    : Node("extra_navigation_node"),
      goal_received_(false),
      in_corridor_(false),
      corridor_done_(false)
    {
        client_nav_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", 10,
            std::bind(&ExtraNavigationNode::goal_callback, this, std::placeholders::_1));

        sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 20,
            std::bind(&ExtraNavigationNode::lidar_callback, this, std::placeholders::_1));

        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&ExtraNavigationNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "Manual navigation node ready");
    }

private:
    // ------------------------------------------------------
    // ROS interfaces
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_nav_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------------------------------------------------
    // State variables
    geometry_msgs::msg::PoseStamped goal_;
    bool goal_received_;

    bool in_corridor_;
    bool corridor_done_;

    float front_dist_;
    float left_dist_;
    float right_dist_;

    // ------------------------------------------------------
    // Handle GOAL reception (only once)
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (goal_received_)
            return;       // non ascolta altri goal

        goal_received_ = true;
        goal_ = *msg;

        RCLCPP_INFO(get_logger(), "Goal received. Sending initial Nav2 goal.");

        send_nav2_goal();
    }

    // ------------------------------------------------------
    // Send goal to Nav2
    void send_nav2_goal()
    {
        if (!client_nav_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(get_logger(), "Nav2 not available");
            return;
        }

        NavigateToPose::Goal g;
        g.pose = goal_;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.result_callback = [this](const GoalHandleNav::WrappedResult & res)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Nav2 completed (result code %d)", res.code);
        };

        client_nav_->async_send_goal(g, opts);
    }

    // ------------------------------------------------------
    // Lidar processing: detect corridor
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int mid = msg->ranges.size() / 2;
        int left = msg->ranges.size() * 3 / 4;
        int right = msg->ranges.size() / 4;

        front_dist_ = msg->ranges[mid];
        left_dist_  = msg->ranges[left];
        right_dist_ = msg->ranges[right];
    }

    // ------------------------------------------------------
    // MAIN CONTROL LOOP
    void control_loop()
    {
        if (!goal_received_)
            return;

        // --------------------------
        // Detect corridor entrance
        // --------------------------
        if (!in_corridor_) {
            if (left_dist_ < 0.7 && right_dist_ < 0.7) {
                RCLCPP_INFO(get_logger(), "Corridor detected: stopping Nav2");
                stop_robot();

                client_nav_->async_cancel_all_goals(); 
                in_corridor_ = true;
                return;
            }
            return; // Nav2 sta navigando
        }

        // --------------------------
        // Corridor navigation
        // --------------------------
        if (in_corridor_ && !corridor_done_) {
            corridor_following();
            return;
        }

        // --------------------------
        // End corridor → resume Nav2
        // --------------------------
        if (corridor_done_ && in_corridor_) {
            RCLCPP_INFO(get_logger(), "Corridor finished, resuming Nav2");
            in_corridor_ = false;
            send_nav2_goal();
            corridor_done_ = false; 
            return;
        }
    }

    // ------------------------------------------------------
    // Corridor following logic
    void corridor_following()
    {
        geometry_msgs::msg::Twist cmd;

        // exit condition: corridor widens
        if (left_dist_ > 1.0 && right_dist_ > 1.0) {
            RCLCPP_INFO(get_logger(), "Corridor exit detected");
            corridor_done_ = true;
            stop_robot();
            return;
        }

        // basic corridor behaviour
        cmd.linear.x = 0.15;

        // wall following correction
        float error = left_dist_ - right_dist_;
        cmd.angular.z = -error * 1.2;  

        pub_vel_->publish(cmd);
    }

    // ------------------------------------------------------
    void stop_robot()
    {
        geometry_msgs::msg::Twist z;
        pub_vel_->publish(z);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExtraNavigationNode>());
    rclcpp::shutdown();
    return 0;
}

