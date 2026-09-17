#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp> 
#include <tf2_ros/transform_listener.hpp> 

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 1.0);
        linear_velocity_ = this->declare_parameter<double>("linear_velocity", 0.5);
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("PathPlanner/path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("bicycle_steering_controller/odometry", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/bicycle_steering_controller/reference", 10);        buffer_memory_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        listener_memory_ = std::make_shared<tf2_ros::TransformListener>(*buffer_memory_);
    }
private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr      cmd_vel_publisher_;
    std::unique_ptr<tf2_ros::Buffer>                                    buffer_memory_;
    std::shared_ptr<tf2_ros::TransformListener>                         listener_memory_;

    nav_msgs::msg::Path current_path_;
    geometry_msgs::msg::Pose current_pose_;

    double lookahead_distance_;
    double linear_velocity_;
    size_t current_target_index_ = 0;  
    double L_d = 0.5;                  
    double v = 0.2;
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) return;

        current_path_ = *msg; 
        double delta_x = current_pose_.position.x - current_path_.poses[0].pose.position.x;
        double delta_y = current_pose_.position.y - current_path_.poses[0].pose.position.y;

        for (size_t i = 0; i < current_path_.poses.size(); i++) {
            current_path_.poses[i].pose.position.x += delta_x; 
            current_path_.poses[i].pose.position.y += delta_y;
        }
        
        current_target_index_ = 0; 
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        computeControlCommand();
    }

    void computeControlCommand() {
        if (current_path_.poses.empty()) {
            return;
        }

        double robot_x = current_pose_.position.x;
        double robot_y = current_pose_.position.y;
        
        double target_x = 0.0;
        double target_y = 0.0;
        bool target_found = false;

        for (size_t i = current_target_index_; i < current_path_.poses.size(); i++) {
            target_x = current_path_.poses[i].pose.position.x;
            target_y = current_path_.poses[i].pose.position.y;

            double distance = std::hypot(target_x - robot_x, target_y - robot_y);

            if (distance >= L_d) {
                current_target_index_ = i;
                target_found = true;
                break;
            }
        }

        if (!target_found) {
            size_t last_index = current_path_.poses.size() - 1;
            target_x = current_path_.poses[last_index].pose.position.x;
            target_y = current_path_.poses[last_index].pose.position.y;
            current_target_index_ = last_index;

            double final_distance = std::hypot(target_x - robot_x, target_y - robot_y);
            
            if (final_distance < 0.05) {
                geometry_msgs::msg::TwistStamped stop_msg;
                stop_msg.header.stamp = this->now();
                stop_msg.header.frame_id = "base_link";
                stop_msg.twist.linear.x = 0.0;
                stop_msg.twist.angular.z = 0.0;
                cmd_vel_publisher_->publish(stop_msg);
                
                current_path_.poses.clear(); 
                return; 
            }
        }

        double angle_to_target = std::atan2(target_y - robot_y, target_x - robot_x);

        tf2::Quaternion q(
            current_pose_.orientation.x,
            current_pose_.orientation.y,
            current_pose_.orientation.z,
            current_pose_.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double alpha = angle_to_target - yaw;
        alpha = std::atan2(std::sin(alpha), std::cos(alpha));

        double distance_to_target = std::hypot(target_x - robot_x, target_y - robot_y);
        double omega = 0.0;
        
        if (distance_to_target > 0.001) {
            omega = (2.0 * v * std::sin(alpha)) / distance_to_target;
        }

        geometry_msgs::msg::TwistStamped cmd_msg;
        cmd_msg.header.stamp = this->now();
        cmd_msg.header.frame_id = "base_link";
        cmd_msg.twist.linear.x = v;
        cmd_msg.twist.angular.z = omega;
        
        cmd_vel_publisher_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}