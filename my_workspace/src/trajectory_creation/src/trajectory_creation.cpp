#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono> 
#include <string>
#include <thread>
#include <csignal>

using namespace std::chrono_literals;

class TrajectoryCreationNode : public rclcpp::Node {
public:
    TrajectoryCreationNode() : Node("trajectory_creation_node") {
        std::cout << "  -> [Costruttore] Inizio creazione nodo ROS" << std::endl;
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("PathPlanner/path", 10);
        stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("PathPlanner/stop", 10);
        
        std::cout << "  -> [Costruttore] Creazione della matrice immagine (canvas)" << std::endl;
        canvas_ = cv::Mat(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));
        
        std::cout << "  -> [Costruttore] Tento di aprire namedWindow..." << std::endl;
        cv::namedWindow("Mouse Trajectory");
        
        std::cout << "  -> [Costruttore] Tento di eseguire imshow..." << std::endl;
        cv::imshow("Mouse Trajectory", canvas_); 
        
        std::cout << "  -> [Costruttore] Tento di eseguire waitKey(1)..." << std::endl;
        cv::waitKey(1);
        
        std::cout << "  -> [Costruttore] Imposto la callback del mouse..." << std::endl;
        cv::setMouseCallback("Mouse Trajectory", onMouseCallback, this);
    
        current_path_.header.frame_id = "odom";
        std::cout << "  -> [Costruttore] Costruttore completato con successo!" << std::endl;
    }
    
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_publisher_;
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
                std_msgs::msg::Bool stop_msg;
                stop_msg.data = true;
                stop_publisher_->publish(stop_msg);
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
    std::cout << "📍 Step 1: Inizio del main e rclcpp::init" << std::endl;
    rclcpp::init(argc, argv);
    std::signal(SIGINT, signalHandler);

    std::cout << "📍 Step 2: PROVA MODIFICA" << std::endl;
    auto node = std::make_shared<TrajectoryCreationNode>();

    std::cout << "📍 Step 3: Avvio del thread ROS 2" << std::endl;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread ros_thread([&executor]() {
        executor.spin(); 
    });

    std::cout << "📍 Step 4: Entrata nel ciclo while principale" << std::endl;
    while (rclcpp::ok()) {
        int key = cv::waitKey(10); 
        if (key == 27) {
            rclcpp::shutdown();
        }
    }
    // 3. Pulizia finale
    cv::destroyAllWindows();
    
    if (ros_thread.joinable()) {
        ros_thread.join(); 
    }

    return 0;
}
    
    // ... resto del codice per la chiusura ...