cmake_minimum_required(VERSION 3.5)
project(go2_legged_real)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
include_directories(include 
                    include/common 
                    include/nlohmann
                    )


set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
  sensor_msgs
  cv_bridge
  image_transport
  techshare_ros_pkg2
  nav_msgs
  unitree_interfaces
  std_srvs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(techshare_ros_pkg2 REQUIRED)
find_package(unitree_sdk2 REQUIRED)
add_executable(show_rosgaitcmd src/show_rosgaitcmd.cpp)
add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)
add_executable(record_bag src/record_bag.cpp)
add_executable(sport_mode_ctrl src/sport_mode_ctrl.cpp src/common/ros2_sport_client.cpp)
add_executable(go2_dds src/go2_dds.cpp src/common/ros2_sport_client.cpp)
add_executable(go2w_dds src/go2w_dds.cpp src/common/ros2_sport_client.cpp)
add_executable(b2_dds src/b2_dds.cpp src/common/ros2_sport_client.cpp)
add_executable(h1_dds src/h1_dds.cpp src/common/ros2_sport_client.cpp)
add_executable(thermo_monitor src/thermo_monitor.cpp)
add_executable(go2_front_camera src/go2_front_camera.cpp)
add_executable(motion_switcher src/motion_switcher.cpp)
target_link_libraries(go2_front_camera ${OpenCV_LIBS})
target_link_libraries(motion_switcher unitree_sdk2)
ament_target_dependencies(show_rosgaitcmd ${DEPENDENCY_LIST})
ament_target_dependencies(low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
ament_target_dependencies(record_bag ${DEPENDENCY_LIST})
ament_target_dependencies(sport_mode_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(go2_dds ${DEPENDENCY_LIST})
ament_target_dependencies(go2w_dds ${DEPENDENCY_LIST})
ament_target_dependencies(b2_dds ${DEPENDENCY_LIST})
ament_target_dependencies(h1_dds ${DEPENDENCY_LIST})
ament_target_dependencies(thermo_monitor ${DEPENDENCY_LIST})
ament_target_dependencies(go2_front_camera ${DEPENDENCY_LIST} )
ament_target_dependencies(motion_switcher ${DEPENDENCY_LIST} )


install(TARGETS 
        low_level_ctrl
        read_low_state
        read_motion_state
        show_rosgaitcmd
        read_wireless_controller
        record_bag
        sport_mode_ctrl
        go2_dds
        go2w_dds
        b2_dds
        h1_dds
        thermo_monitor
        go2_front_camera
        motion_switcher
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
