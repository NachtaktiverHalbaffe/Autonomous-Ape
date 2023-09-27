cmake_minimum_required(VERSION 3.8)
project(sopias4_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################################
# ----------- Find dependencies --------------
#######################################
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# ------------------- C++ ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

######################################
# ----------- ROS specific stuff ---------------
######################################
# Service definitions
rosidl_generate_interfaces(${PROJECT_NAME}
      # Framework
      "msg/Robot.msg"
      "msg/RobotStates.msg"
      "srv/ShowDialog.srv"
      "srv/RegistryService.srv"
      "srv/EmptyWithStatuscode.srv"
      "srv/GetRobots.srv"
      "srv/GetRobotIdentity.srv"
      "srv/GetNamespaces.srv"
      "srv/DriveToPos.srv"
      "srv/LaunchNav2Stack.srv"
      "srv/Drive.srv"
      "srv/StopMapping.srv"
      # Plugin Bridges
      "srv/CreatePlan.srv"
      "srv/UpdateCosts.srv"
      "srv/ComputeVelocityCommands.srv"
      DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs nav2_msgs
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
