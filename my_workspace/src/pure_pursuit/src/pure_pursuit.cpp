#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp> // #include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp> // #include <tf2_ros/transform_listener.h>

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
        linear_velocity_ = this->declare_parameter<double>("linear_velocity", 0.5);
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("PathPlanner/path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<geometry_msgs::msg::Odometry>("bicycle_steering_controller/odometry", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
        
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_publisher_;

    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Pose current_pose_;

    double lookahead_distance_;
    double linear_velocity_;

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        current_path_ = *msg;
    }

    void odomCallback(const geometry_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        computeControlCommand();
    }

    void computeControlCommand() {
        if (current_path_.poses.empty()) {
            return;
        }

        // Implement the pure pursuit algorithm here to compute the control command
        // based on the current pose and the path.
        // Publish the computed command to cmd_vel_publisher_.
    }
};