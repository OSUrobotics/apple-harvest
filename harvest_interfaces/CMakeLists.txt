cmake_minimum_required(VERSION 3.8)
project(harvest_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/EventDetection.action"
  "action/SendTrajectory.action"

  "srv/ApplePrediction.srv"
  "srv/SendTrajectory.srv"
  "srv/CoordinateToTrajectory.srv"
  "srv/VoxelMask.srv"
  "srv/MoveToPose.srv"
  "srv/TrajectoryBetweenPoints.srv"
  "srv/SetValue.srv"
  "srv/RecordTopics.srv"
  "srv/GetGripperPose.srv"
  "srv/VoxelGrid.srv"
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)


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