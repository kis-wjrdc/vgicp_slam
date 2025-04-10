cmake_minimum_required(VERSION 3.10)
project(vgicp_slam)

# コンパイラオプション
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# OpenMP
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_definitions(-DUSE_OPENMP)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# ROS2関連
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)  # Trigger.srv 用

# PCLとfast_gicp
find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(fast_gicp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# 実行ファイル定義
add_executable(vgicp_slam_node src/vgicp_slam_node.cpp)

# 依存ライブラリ設定（ここで std_srvs はインターフェースとして使う）
ament_target_dependencies(vgicp_slam_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_msgs
  std_msgs
  tf2
  tf2_ros
  fast_gicp
  std_srvs
)

# 明示的なリンクが必要なライブラリのみ記載
target_link_libraries(vgicp_slam_node
  ${PCL_LIBRARIES}
  fast_gicp
  OpenMP::OpenMP_CXX
)

# インストールルール
install(TARGETS
  vgicp_slam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
