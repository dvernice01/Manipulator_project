cmake_minimum_required(VERSION 3.20)
project(perception_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(perception_node src/perception_node.cpp)

target_link_libraries(perception_node PUBLIC
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  pcl_conversions::pcl_conversions
)

target_include_directories(perception_node PUBLIC
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(perception_node PUBLIC
  ${PCL_LIBRARIES}
)

install(TARGETS
perception_node
DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
