cmake_minimum_required(VERSION 3.8)
project(gtsam_points_2d_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  "srv/LoadMap.srv"
  "srv/AddLoopClosure.srv"
  "srv/OptimizeMap.srv"
)

# Find gtsam_points
find_package(gtsam_points REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)

# Add SLAM node executable
add_executable(slam_node
  src/slam_node.cpp
)

target_include_directories(slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_node
  gtsam_points::gtsam_points
  gtsam
)

# Add LiDAR Odometry node executable
add_executable(lidar_odometry_node
  src/lidar_odometry_node.cpp
)

target_include_directories(lidar_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(lidar_odometry_node
  gtsam_points::gtsam_points
  gtsam
)

# Add SLAM with Loop Closure node executable
add_executable(slam_with_loop_closure_node
  src/slam_with_loop_closure_node.cpp
)

target_include_directories(slam_with_loop_closure_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_with_loop_closure_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_loop_closure_node
  gtsam_points::gtsam_points
  gtsam
)

# Add SLAM with Wheel Odometry node executable
add_executable(slam_with_wheel_odom_node
  src/slam_with_wheel_odom_node.cpp
)

target_include_directories(slam_with_wheel_odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_with_wheel_odom_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_wheel_odom_node
  gtsam_points::gtsam_points
  gtsam
)

# Add SLAM with IMU node executable
add_executable(slam_with_imu_node
  src/slam_with_imu_node.cpp
)

target_include_directories(slam_with_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_with_imu_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_imu_node
  gtsam_points::gtsam_points
  gtsam
)

# Add SLAM with Fixed-lag Smoothing node executable
add_executable(slam_with_fixed_lag_node
  src/slam_with_fixed_lag_node.cpp
)

target_include_directories(slam_with_fixed_lag_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

 ament_target_dependencies(slam_with_fixed_lag_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_fixed_lag_node
  gtsam_points::gtsam_points
  gtsam
)

# Add SLAM with CT-ICP node executable
add_executable(slam_with_ct_icp_node
  src/slam_with_ct_icp_node.cpp
)

target_include_directories(slam_with_ct_icp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

 ament_target_dependencies(slam_with_ct_icp_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_ct_icp_node
  gtsam_points::gtsam_points
  gtsam
)

# Add SLAM with Map Save node executable
add_executable(slam_with_map_save_node
  src/slam_with_map_save_node.cpp
)

target_include_directories(slam_with_map_save_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_with_map_save_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_map_save_node
  gtsam_points::gtsam_points
  gtsam
)

# Link service interfaces to the node
rosidl_target_interfaces(slam_with_map_save_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add SLAM with CT-GICP node executable
add_executable(slam_with_ct_gicp_node
  src/slam_with_ct_gicp_node.cpp
)

target_include_directories(slam_with_ct_gicp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_with_ct_gicp_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_ct_gicp_node
  gtsam_points::gtsam_points
  gtsam
)

# Add offline map optimizer node executable
add_executable(offline_map_optimizer
  src/offline_map_optimizer.cpp
)

target_include_directories(offline_map_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(offline_map_optimizer
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(offline_map_optimizer
  gtsam_points::gtsam_points
  gtsam
)

# Link service interfaces to offline_map_optimizer
rosidl_target_interfaces(offline_map_optimizer
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add RANSAC SLAM node executable
add_executable(slam_with_ransac_node
  src/slam_with_ransac_node.cpp
)

target_include_directories(slam_with_ransac_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_with_ransac_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_ransac_node
  gtsam_points::gtsam_points
  gtsam
)

# Add GNC SLAM node executable
add_executable(slam_with_gnc_node
  src/slam_with_gnc_node.cpp
)

target_include_directories(slam_with_gnc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(slam_with_gnc_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_gnc_node
  gtsam_points::gtsam_points
  gtsam
)

# Add Segmentation SLAM node executable
add_executable(slam_with_segmentation_node
  src/slam_with_segmentation_node.cpp
)

target_include_directories(slam_with_segmentation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

 ament_target_dependencies(slam_with_segmentation_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_with_segmentation_node
  gtsam_points::gtsam_points
  gtsam
)

# Install targets
install(TARGETS
  slam_node
  lidar_odometry_node
  slam_with_loop_closure_node
  slam_with_wheel_odom_node
  slam_with_imu_node
  slam_with_fixed_lag_node
  slam_with_ct_icp_node
  slam_with_map_save_node
  slam_with_ct_gicp_node
  offline_map_optimizer
  slam_with_ransac_node
  slam_with_gnc_node
  slam_with_segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
