cmake_minimum_required(VERSION 3.8)
project(harvest_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
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
find_package(jsoncpp REQUIRED)


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
)

# Link jsoncpp manually
find_library(JSONCPP_LIBRARIES NAMES jsoncpp PATHS /usr/lib /usr/local/lib)
if (JSONCPP_LIBRARIES)
  target_link_libraries(move_arm ${JSONCPP_LIBRARIES})
else()
  message(FATAL_ERROR "Could not find jsoncpp library")
endif()

### PYTHON ###
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/coordinate_to_trajectory.py
  scripts/coordinate_to_trajectory_client.py
  scripts/visual_servo.py
  scripts/pyb_scripts/load_objects.py
  scripts/pyb_scripts/load_robot.py
  scripts/pyb_scripts/pyb_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS move_arm
  DESTINATION lib/${PROJECT_NAME})

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
