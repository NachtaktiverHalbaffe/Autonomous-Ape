cmake_minimum_required(VERSION 3.8)
project(sopias4_framework)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################################
# ----------- Find dependencies --------------
#######################################
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# ------------------- C++ ------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#------------------ Python ---------------------
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


######################################
# --- Configure executabels/Nodes ---------
######################################
# ------------------- C++ ------------------------
include_directories(include)
# Create exectuables. For this purpose, add 
# following line for each node:
# add_executable(cpp_exe src/<node file>)
# ament_target_dependencies(cpp_exe rclcpp)
# Install C++ executables. For this purpose, add 
# following line for each node:
# install(TARGETS
#     cpp_exe
#     DESTINATION lib/${PROJECT_NAME}
# )

#------------------ Python ---------------------
# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# Install executables
# install(PROGRAMS
#   sopias4_framework/nodes/robot_manager.py
#   DESTINATION lib/${PROJECT_NAME}
# )


######################################
# ----------- ROS specific stuff ---------------
######################################
# Service definitions
rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Robot.msg"
      "srv/ShowDialog.srv"
      "srv/Register.srv"
      "srv/EmptyWithStatuscode.srv"
      "srv/GetRobots.srv"
      "srv/GetRobotIdentity.srv"
      "srv/GetNamespaces.srv"
      DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs
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
