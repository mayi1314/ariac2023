cmake_minimum_required(VERSION 3.8)
project(ariac_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Install the config directory to the package share directory
install(DIRECTORY
config
DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
launch/monitor.py
scripts/test.py
scripts/test3.py
DESTINATION lib/${PROJECT_NAME}
)

# Install the config directory to the package share directory
install(DIRECTORY
config
launch
meshes
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

# Install C++ executables
add_executable(robot_commander
src/robot_commander.cpp include/robot_commander.hpp)
target_include_directories(robot_commander PRIVATE include)

ament_target_dependencies(robot_commander
rclcpp
moveit_ros_planning_interface
ariac_msgs
shape_msgs)

install(TARGETS
robot_commander
DESTINATION lib/${PROJECT_NAME})


ament_package()