#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>


class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
    }

private:
}