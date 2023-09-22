cmake_minimum_required(VERSION 3.8)
project(sopias4_fleetbroker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################################
# ----------- Find dependencies --------------
#######################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sopias4_msgs REQUIRED)
find_package(domain_bridge REQUIRED)
find_package(nav2_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


######################################
# --- Configure executabels/Nodes ---------
######################################
include_directories(include)
# Create exectuables. For this purpose, add 
# following line for each node:
add_executable(multi_robot_coordinator src/multi_robot_coordinator.cpp)
ament_target_dependencies(multi_robot_coordinator
  sopias4_msgs
  geometry_msgs
  domain_bridge
  nav2_msgs
  rclcpp)

add_executable(sopias4_domain_bridge src/sopias4_domain_bridge.cpp)
ament_target_dependencies(sopias4_domain_bridge
  sopias4_msgs
  geometry_msgs
  domain_bridge
  nav2_msgs
  rclcpp)
  
# Install C++ executables. For this purpose, add 
# following line for each node:
install(TARGETS
    multi_robot_coordinator
    sopias4_domain_bridge
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install executables
install(PROGRAMS
  sopias4_fleetbroker/gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configs
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
  
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
ament_package()
