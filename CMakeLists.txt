cmake_minimum_required(VERSION 3.8)
project(fast_lio)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast -fopenmp -fPIC")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_transport REQUIRED)

set(dependencies
  cv_bridge
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  pcl_ros
  pcl_conversions
  livox_ros_driver2
  image_transport
)

# Thirdparty libraries
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

message(Eigen: ${EIGEN3_INCLUDE_DIR})
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")

rosidl_generate_interfaces(fastlio_mapping_msgs
  "msg/Pose6D.msg"
  LIBRARY_NAME fastlio_mapping
)
rosidl_get_typesupport_target(cpp_typesupport_target
fastlio_mapping_msgs "rosidl_typesupport_cpp")

# ament_export_dependencies(rosidl_default_runtime)

ament_auto_add_library(fastlio_mapping SHARED src/laser_mapping.cpp src/ikd_tree.cpp src/preprocess.cpp src/imu_processing.cpp src/cam_processing.cpp src/project_ellipse.cpp)
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(fastlio_mapping "${cpp_typesupport_target}")

target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})

ament_target_dependencies(fastlio_mapping ${dependencies})

rclcpp_components_register_node(fastlio_mapping
PLUGIN "fastlio::LaserMappingNode"
EXECUTABLE fastlio_mapping_node
)

ament_auto_package(INSTALL_TO_SHARE
config
launch
rviz
)