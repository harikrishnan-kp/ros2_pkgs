cmake_minimum_required(VERSION 3.8)
project(stereo_image_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)

add_executable(stereo_image_pub src/stereo_image_pub.cpp)
ament_target_dependencies(stereo_image_pub rclcpp sensor_msgs cv_bridge OpenCV camera_info_manager)

install(
  TARGETS stereo_image_pub
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
