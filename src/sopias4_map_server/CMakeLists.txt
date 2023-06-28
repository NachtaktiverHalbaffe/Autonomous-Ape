cmake_minimum_required(VERSION 3.8)
project(sopias4_map_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################################
# ----------- Find dependencies --------------
#######################################
find_package(sopias4_framework REQUIRED)

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


######################################
# --- Configure executabels/Nodes ---------
######################################
# ------------------- C++ ------------------------
include_directories(include)
# Create exectuables. For this purpose, add 
# following line for each node:
add_executable(multi_robot_coordinator src/multi_robot_coordinator.cpp)
ament_target_dependencies(multi_robot_coordinator
  sopias4_framework
  rclcpp)
# Install C++ executables. For this purpose, add 
# following line for each node:
install(TARGETS
    multi_robot_coordinator
    DESTINATION lib/${PROJECT_NAME}
)

#------------------ Python ---------------------
# Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# Install executables
# install(PROGRAMS
#   #scripts/py_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files, configs
install(
  DIRECTORY launch config maps
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
