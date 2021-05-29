cmake_minimum_required(VERSION 3.5)
project(uni_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# for standard msg
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# for customazed msg
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

#for customazed msg
set(msg_files
"msg/Duplex.msg"
"msg/OdometryCovariances.msg"
"msg/Point.msg"
"msg/Pose.msg"
"msg/Quaternion.msg"
"msg/RobotError.msg"
"msg/SetEncoder.msg"
"msg/Twist.msg"
"msg/Vector3.msg"
)

#for customazed msg
set(srv_files
"srv/RequestMotorBehavior.srv"
"srv/RequestOdometryCovariances.srv"
"srv/RequestParam.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

#define the node name ex.} project_name + "node"
set(EXECUTABLE_NAME "uni_interface_node")


################################################################################
# Build
################################################################################
include_directories(
  include
)

add_executable(${EXECUTABLE_NAME}
  src/uni_interface.cpp
  src/ros2_uni.cpp
  src/modbusMap.cpp
  src/modbusTCPs.cpp
  src/modbusTCPm.cpp
  src/can_bat_adl24linmc_39ah_50a_cb.cpp
  src/can_mot_sbl2360.cpp
  src/systemIO.cpp
)

ament_target_dependencies(${EXECUTABLE_NAME}
"rclcpp"
"std_msgs"
"geometry_msgs"
"nav_msgs"
"sensor_msgs"
"tf2_ros"
)

################################################################################
# Install
################################################################################
install(TARGETS
  ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
rosidl_target_interfaces(${EXECUTABLE_NAME}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()
