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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sopias4_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(jsoncpp REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
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
include_directories(
  include
  ${JSONCPP_INCLUDE_DIRS}
)

set(library_name plugin_bridges)
# Declare dependecies here
set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    visualization_msgs
    nav2_util
    nav2_msgs
    nav_msgs
    geometry_msgs
    builtin_interfaces
    tf2_ros
    nav2_costmap_2d
    nav2_core
    pluginlib
    sopias4_msgs
    yaml_cpp_vendor
    ${JSONCPP_LIBRARIES}
)
# Add the plugin bridges source files here
add_library(${library_name} SHARED
    src/plugin_bridge_planner.cpp
    src/plugin_bridge_layer.cpp
    src/msgs_utils.cpp
)
# Include dependecies in compiled plugins
ament_target_dependencies(${library_name}
    ${dependencies}
)
target_include_directories(${library_name} SYSTEM
  PUBLIC
    ${JSONCPP_INCLUDE_DIRS}
    ${YamlCpp_INCLUDE_DIRS})
target_link_libraries(${library_name}
  ${JSONCPP_LIBRARIES}
  ${YamlCpp_LIBRARIES}
)
# Define plugins here
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(nav2_core plugin_bridge_planner.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin_bridge_layer.xml)

# Install C++ executables
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(FILES plugin_bridge_planner.xml
  DESTINATION share/${PROJECT_NAME}
)
#------------------ Python ---------------------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install executables
install(PROGRAMS
  sopias4_framework/nodes/robot_manager.py
  sopias4_framework/nodes/global_planner_testserver.py
  DESTINATION lib/${PROJECT_NAME}
)


######################################
# ----------- ROS specific stuff ---------------
######################################
# Install launch files, configs, plugin descriptions
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY assets DESTINATION share/${PROJECT_NAME})


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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
