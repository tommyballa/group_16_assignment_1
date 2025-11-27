#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp" 

#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class ExtraNavigationNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    ExtraNavigationNode()
    : Node("extra_navigation_node"),
      // State booleans  
      goal_received_(false),
      in_corridor_(false),
      corridor_done_(false)
    {
        // Client that talks to Nav2 action
        client_nav_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        // Listen to topic /goal, call goal_callback on publish
        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", 10,
            std::bind(&ExtraNavigationNode::goal_callback, this, std::placeholders::_1));

        // Read laser scan queue
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 20,
            std::bind(&ExtraNavigationNode::lidar_callback, this, std::placeholders::_1));

        // Publish velocity commands to robot
        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Publisher for the navigation state
        pub_goal_status_ = this->create_publisher<std_msgs::msg::Bool>("navigation_completed", 10);

        // Run control loop every 50 ms
        timer_ = this->create_wall_timer(
            50ms,
            std::bind(&ExtraNavigationNode::control_loop, this));

        // Log startup
        RCLCPP_INFO(get_logger(), "Manual navigation node ready");
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_nav_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_goal_status_; // NUOVO
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped goal_;
    bool goal_received_;

    bool in_corridor_;
    bool corridor_done_;

    float front_dist_;
    float left_dist_;
    float right_dist_;


    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (goal_received_)
            return;       // Do not listen to other goals

        // Store received goal
        goal_received_ = true;
        goal_ = *msg;

        RCLCPP_INFO(get_logger(), "Goal received. Sending initial Nav2 goal.");

        send_nav2_goal();       // Send to nav2
    }

    void send_nav2_goal()
    {
        // Wait 2s for Nav2
        if (!client_nav_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(get_logger(), "Nav2 not available");
            return;
        }

        // Create goal message
        NavigateToPose::Goal g;
        g.pose = goal_;

        // Log result and send goal asynchronously
        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        opts.result_callback = [this](const GoalHandleNav::WrappedResult & res)
        {
            RCLCPP_INFO(this->get_logger(), "Nav2 completed");
            
            std_msgs::msg::Bool status;
            // If Nav2 succeed, send True
            status.data = (res.code == rclcpp_action::ResultCode::SUCCEEDED); 
            this->pub_goal_status_->publish(status);

        };

        client_nav_->async_send_goal(g, opts);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Laser scan indexing
        int mid = msg->ranges.size() / 2;
        int left = msg->ranges.size() * 3 / 4;
        int right = msg->ranges.size() / 4;

        // Distances for corridor
        front_dist_ = msg->ranges[mid];
        left_dist_  = msg->ranges[left];
        right_dist_ = msg->ranges[right];
    }

    void control_loop()
    {
        // Only act if there is a goal
        if (!goal_received_)
            return;

        if (!in_corridor_) {
            // If two walls are detected see it as a corridor
            if (left_dist_ < 0.7 && right_dist_ < 0.7) {
                RCLCPP_INFO(get_logger(), "Corridor detected: stopping Nav2");
                stop_robot();
                
                // Cancel goal, switch to corridor mode
                client_nav_->async_cancel_all_goals(); 
                in_corridor_ = true;
                return;
            }
            return; 
        }

        // Corridor following
        if (in_corridor_ && !corridor_done_) {
            corridor_following();
            return;
        }

        // Corridor finished and nav2 navigation is resumed
        if (corridor_done_ && in_corridor_) {
            RCLCPP_INFO(get_logger(), "Corridor finished, resuming Nav2");
            in_corridor_ = false;
            send_nav2_goal();
            corridor_done_ = false; 
            return;
        }
    }

    void corridor_following()
    {
        geometry_msgs::msg::Twist cmd;

        // Exiting condition: edge distance greater than 1 meter
        if (left_dist_ > 1.0 && right_dist_ > 1.0) {
            RCLCPP_INFO(get_logger(), "Corridor exit detected");
            corridor_done_ = true;
            stop_robot();
            return;
        }

        cmd.linear.x = 0.15;        // Slow drive forward

        // Proportional controller to center the robot (P-controller)
        float error = left_dist_ - right_dist_;
        cmd.angular.z = -error * 0.8;  

        // Send velocity command
        pub_vel_->publish(cmd);
    }

    // Set velocity command to zero
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
