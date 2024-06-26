cmake_minimum_required(VERSION 3.8)
project(stereo_visual_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(library_name sv_lib)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${G2O_INCLUDE_DIRS}
)
add_library(${library_name}
  src/stereo_visual_odometry.cpp
  src/extractor/ORBExtractor.cpp
  src/tracker/ORBTracker.cpp
  src/tracker/opticalTracker.cpp
  src/optimizer/G2O_Optimizer.cpp
  src/optimizer/CV_Optimizer.cpp
)
ament_target_dependencies(${library_name}
  rclcpp
  sensor_msgs
  nav_msgs
  OpenCV
  tf2_eigen
  tf2_ros
  cv_bridge
)
target_link_libraries(${library_name}
  g2o_core
  g2o_stuff
  ceres
  Sophus::Sophus
)


add_executable(stereo_visual_odometry_node
  src/stereo_visual_odometry_node.cpp
)
ament_target_dependencies(stereo_visual_odometry_node
  rclcpp
)
target_link_libraries(stereo_visual_odometry_node
  ${library_name}
)

target_include_directories(stereo_visual_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  stereo_visual_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY
  launch
  params
  rviz
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
