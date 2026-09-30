#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>


class PerceptionNode : public rclcpp::Node {
public:
    PerceptionNode() : Node("perception_node") {
    
    sensor_subscriber_= this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera/depth/points", 10, std::bind(&PerceptionNode::pointcloud_callback, this, std::placeholders::_1)
    );
    trajectory_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 
        10
    );
    
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr trajectory_publisher_;  

    void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. Creiamo i puntatori per la nuvola originale e per quella che filtreremo
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr red_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        Eigen::Vector4f centroid_vector;
        geometry_msgs::msg::PoseStamped target_pose;

        // 2. Convertiamo il messaggio ROS (*msg) nel formato PCL (*pcl_cloud)
        pcl::fromROSMsg(*msg, *pcl_cloud);        

        vox.setLeafSize (0.01f,0.01f,0.01f);
        vox.setInputCloud(pcl_cloud);
        vox.filter(*cloud_filtered);

        for (const auto& punto : cloud_filtered->points) {
            if (punto.r > 150 && punto.g < 50 && punto.b < 50) {
                red_cloud->points.push_back(punto);
            }
        }
        
        if (red_cloud->points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "Nessun oggetto rosso rilevato in questo frame.");
            return; 
        }

        pcl::compute3DCentroid(*red_cloud, centroid_vector);
        target_pose.pose.position.x = centroid_vector[0];
        target_pose.pose.position.y = centroid_vector[1];
        target_pose.pose.position.z = centroid_vector[2];
        target_pose.header.frame_id = msg->header.frame_id;
        target_pose.header.stamp = this->now();
        target_pose.pose.orientation.w = 1.0;
        trajectory_publisher_->publish(target_pose);

    }
    // void pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    //     (void)msg;
    //     //RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", msg->width * msg->height);
    //     trajectory_msgs::msg::JointTrajectory joint_trajectory;
    //     joint_trajectory.joint_names = {"joint1",  "joint2",  "joint3",  "joint4",  "joint5",  "joint6"};
    //     trajectory_msgs::msg::JointTrajectoryPoint punto_obiettivo;
    //     punto_obiettivo.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //     punto_obiettivo.time_from_start.sec = 2;
    //     punto_obiettivo.time_from_start.nanosec = 0;
    //     joint_trajectory.points.push_back(punto_obiettivo);
    //     trajectory_publisher_->publish(joint_trajectory);
    // };
 
}; 

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}