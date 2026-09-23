#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>

#include <control_msgs/srv/set_odometry.hpp> 
// #include <gazebo_msgs/srv/set_entity_state.hpp> // Header for Gazebo

#include <chrono> 
#include <string>
#include <thread>
#include <csignal>

using namespace std::chrono_literals;

enum RobotState : int8_t {
    NORMAL = 0,
    STOP = 1,
    FORWARD = 2
};

class TrajectoryCreationNode : public rclcpp::Node {
public:
    TrajectoryCreationNode() : Node("trajectory_creation_node") {
        std::cout << "  -> [Costruttore] Inizio creazione nodo ROS" << std::endl;
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("PathPlanner/path", 10);
        state_publisher_ = this->create_publisher<std_msgs::msg::Int8>("PathPlanner/command", 10);
        odom_reset_client_ = this->create_client<control_msgs::srv::SetOdometry>("/bicycle_steering_controller/set_odometry");
        
        canvas_ = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    
        cv::namedWindow("Mouse Trajectory");
        cv::imshow("Mouse Trajectory", canvas_); 
        cv::waitKey(1);
        cv::setMouseCallback("Mouse Trajectory", onMouseCallback, this);
    
        current_path_.header.frame_id = "odom";
    }
    
    void ClearFunction() {
        current_path_.poses.clear();
        canvas_.setTo(cv::Scalar(255, 255, 255));
        cv::imshow("Mouse Trajectory", canvas_);
    }
    void SpawnRobot() {
        // RViz (Odometry Reset)
        if (!odom_reset_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Odometry reset service not available!");
            return;
        }
        auto odom_request = std::make_shared<control_msgs::srv::SetOdometry::Request>();
        odom_request->x = 0.0;
        odom_request->y = 0.0;
        odom_request->yaw = 0.0;
        odom_reset_client_->async_send_request(odom_request);
        RCLCPP_INFO(this->get_logger(), "Odometry reset request sent.");

        /* 
        // 3b.Gazebo (Teletransport)
        if (gazebo_client_->wait_for_service(std::chrono::seconds(1))) {
            auto gz_request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
            gz_request->state.name = "robot_name"; // Replace with your robot's name in Gazebo
            gz_request->state.pose.position.x = 0.0;
            gz_request->state.pose.position.y = 0.0;
            gz_request->state.pose.position.z = 0.0;
            gazebo_client_->async_send_request(gz_request);
        }
        */
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr state_publisher_;

    rclcpp::Client<control_msgs::srv::SetOdometry>::SharedPtr odom_reset_client_;
    // rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr gazebo_client_;

    nav_msgs::msg::Path current_path_;

    double conversion_factor_ = 100.0; 
    double center_x_ = 400.0;
    double center_y_ = 300.0;
    double last_x_ = 0.0; 
    double last_y_ = 0.0; 
    
    int last_x_pixel_ = -1;
    int last_y_pixel_ = -1;
    int current_track_id_ = 1;
    cv::Mat canvas_;

    static void onMouseCallback(int event, int x, int y, int flags, void* userdata) {
        TrajectoryCreationNode* node = static_cast<TrajectoryCreationNode*>(userdata);
        if (node != nullptr) {
            node->processMouse(event, x, y, flags);
        }
    }

    void processMouse(int event, int x, int y, int flags) {
        RCLCPP_INFO(this->get_logger(), "Mouse event: %d at (%d, %d)", event, x, y);
        std::cout << "Mouse event: " << event << " at (" << x << ", " << y << ")" << std::endl;
            if (event == cv::EVENT_LBUTTONDOWN) {
                current_path_.poses.clear(); 
                last_x_pixel_ = x;
                last_y_pixel_ = y;
                last_x_ = (x - center_x_) / conversion_factor_;
                last_y_ = -(y - center_y_) / conversion_factor_;
            }
            else if (event == cv::EVENT_MBUTTONDOWN) {
                std_msgs::msg::Int8 state_msg;
                state_msg.data = STOP;
                state_publisher_->publish(state_msg);
            }

            else if (event == cv::EVENT_LBUTTONDBLCLK) {
                std_msgs::msg::Int8 state_msg;
                state_msg.data = FORWARD;
                state_publisher_->publish(state_msg);
            }

            // else if (event == cv::EVENT_RBUTTONDOWN) {
            //     if (!current_path_.poses.empty()) {
            //         path_publisher_->publish(current_path_);
            //     }
            // }
            else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) { 
                std_msgs::msg::Int8 state_msg;
                state_msg.data = NORMAL;
                state_publisher_->publish(state_msg);
                
                if (last_x_pixel_ == -1 && last_y_pixel_ == -1) {
                    last_x_pixel_ = x;
                    last_y_pixel_ = y;
                    last_x_ = (x - center_x_) / conversion_factor_;
                    last_y_ = -(y - center_y_) / conversion_factor_;
                }

                double x_meters =  (x - center_x_) / conversion_factor_;
                double y_meters = -(y - center_y_) / conversion_factor_;

                geometry_msgs::msg::PoseStamped new_point;
                
                new_point.header.frame_id = std::to_string(current_track_id_);
                
                new_point.pose.position.x = x_meters;
                new_point.pose.position.y = y_meters;

                double dx = x_meters - last_x_;
                double dy = y_meters - last_y_;
                double yaw = std::atan2(dy, dx);

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, yaw); 
                new_point.pose.orientation.x = q.x();
                new_point.pose.orientation.y = q.y();
                new_point.pose.orientation.z = q.z();
                new_point.pose.orientation.w = q.w();
                
                last_x_ = x_meters;
                last_y_ = y_meters;

                current_path_.poses.push_back(new_point);
                
                cv::line(canvas_, cv::Point(last_x_pixel_, last_y_pixel_), cv::Point(x, y), cv::Scalar(255, 0, 0), 2);
                last_x_pixel_ = x;
                last_y_pixel_ = y;
                
                cv::imshow("Mouse Trajectory", canvas_);
                cv::waitKey(1);
                
            } 

            else if (event == cv::EVENT_LBUTTONUP) {
                
                if (!current_path_.poses.empty()) {
                    path_publisher_->publish(current_path_);
                }

                current_track_id_ += 1;
                last_x_pixel_ = -1;
                last_y_pixel_ = -1;
            }
        }
    };

// Automatically called when it's clicked Ctrl+C
void signalHandler(int signum) {
    (void)signum; 
    rclcpp::shutdown(); // Stopping rclcpp::ok()
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<TrajectoryCreationNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor]() {
        executor.spin(); 
    });

    while (rclcpp::ok()) {
        int key = cv::waitKey(10); 
        
        if (key == 27) {
            rclcpp::shutdown();
        }
        else if (key == -1) { 
            node->ClearFunction();
        }
        else if (key == 32) { 
            node->SpawnRobot();
        }
    }

    cv::destroyAllWindows();
    
    if (ros_thread.joinable()) {
        ros_thread.join(); 
    }

    return 0;
}
    
