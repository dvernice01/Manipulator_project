#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
    
    sensor_subscriber_= this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera/depth/points", 10, std::bind(&PerceptionNode::pointcloud_callback, this, std::placeholders::_1)
    );
    trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 
        10
    );
    
    }

private:
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        (void)msg;
        //RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        joint_trajectory.joint_names = {"joint1",  "joint2",  "joint3",  "joint4",  "joint5",  "joint6"};
        trajectory_msgs::msg::JointTrajectoryPoint punto_obiettivo;
        punto_obiettivo.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        punto_obiettivo.time_from_start.sec = 2;
        punto_obiettivo.time_from_start.nanosec = 0;
        joint_trajectory.points.push_back(punto_obiettivo);
        trajectory_publisher_->publish(joint_trajectory);
    };
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;   
}; 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}