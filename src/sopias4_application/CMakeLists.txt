cmake_minimum_required(VERSION 3.8)
project(sopias4_application)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################################
# ----------- Find dependencies --------------
#######################################
# Declare dependecies here
find_package(sopias4_msgs REQUIRED)
find_package(sopias4_framework REQUIRED)

# Include dependecies in compiled plugins
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

#------------------ Python ---------------------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install executables
install(PROGRAMS
  sopias4_application/gui.py
  sopias4_application/astar.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configs, plugin descriptions
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
