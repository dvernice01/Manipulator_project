#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono> 
#include <string>
#include <thread>
#include <csignal>

using namespace std::chrono_literals;

class TrajectoryCreationNode : public rclcpp::Node {
public:
    TrajectoryCreationNode() : Node("trajectory_creation_node") {
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("PathPlanner/path", 10);
        canvas_ = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::namedWindow("Mouse Trajectory");
        cv::imshow("Mouse Trajectory", canvas_); 
        cv::waitKey(1);
        cv::setMouseCallback("Mouse Trajectory", onMouseCallback, this);
    
        current_path_.header.frame_id = "odom";
    }
    
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
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
            if (event == cv::EVENT_LBUTTONDOWN) {
                current_path_.poses.clear(); 
                last_x_pixel_ = x;
                last_y_pixel_ = y;
                last_x_ = (x - center_x_) / conversion_factor_;
                last_y_ = -(y - center_y_) / conversion_factor_;
            }
            else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) { 
                
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
    
    //  Ctrl+C (SIGINT)
    std::signal(SIGINT, signalHandler);

    auto node = std::make_shared<TrajectoryCreationNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
        executor.spin_some();      
        
        int key = cv::waitKey(10); 
        if (key == 27) {           // ESC key pressed
            rclcpp::shutdown();
        }
    }
    cv::destroyAllWindows();
    return 0;
}