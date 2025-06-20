cmake_minimum_required(VERSION 3.8)
project(harvest_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(harvest_interfaces)
find_package(geometric_shapes REQUIRED)
include_directories(${geometric_shapes_INCLUDE_DIRS})


### C++ ###
add_executable(move_arm src/move_arm.cpp)

target_include_directories(move_arm PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_compile_features(move_arm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  move_arm
  "moveit_ros_planning_interface"
  "harvest_interfaces"
  "rclcpp"
  "trajectory_msgs"
  "std_srvs"
  "shape_msgs"
  )
  
  # add_executable(move_arm_action_based src/move_arm_action_based.cpp)
  
  # target_include_directories(move_arm_action_based PUBLIC
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<INSTALL_INTERFACE:include>)

  # target_compile_features(move_arm_action_based PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  # ament_target_dependencies(
    #   move_arm_action_based
    #   "moveit_ros_planning_interface"
    #   "harvest_interfaces"
    #   "rclcpp"
    #   "trajectory_msgs"
    #   "std_srvs"
    # )

### PYTHON ###
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/coordinate_to_trajectory.py
  scripts/event_detector.py
  scripts/force_filter.py
  scripts/get_manual_apple_locations.py
  scripts/heuristic_controller.py
  scripts/linear_controller.py
  scripts/pose_listener.py
  scripts/pressure_averager.py
  scripts/pull_twist_controller.py
  scripts/visual_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS move_arm
  DESTINATION lib/${PROJECT_NAME})

# install(TARGETS move_arm_action_based
  # DESTINATION lib/${PROJECT_NAME})

### LAUNCH ###
# Install launch files.
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install resources files.
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
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
