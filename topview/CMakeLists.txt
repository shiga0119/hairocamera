cmake_minimum_required(VERSION 3.8)
project(topview)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui videoio)
find_package(rclcpp_components REQUIRED)

set(NODES camera_node;twocam_node)
foreach(target IN LISTS NODES)
   add_executable(${target} src/${target}.cpp)
   ament_target_dependencies(${target} rclcpp OpenCV cv_bridge sensor_msgs)
   install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
