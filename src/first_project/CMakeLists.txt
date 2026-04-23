cmake_minimum_required(VERSION 3.8)
project(first_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------------
# Dependencies
# ----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(bunker_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ----------------------------
# Messages / Services
# ----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Reset.srv"
  "msg/TfErrorMsg.msg"
  DEPENDENCIES std_msgs
)

# ----------------------------
# Executables
# ----------------------------
add_executable(odometer src/odometer.cpp)
add_executable(tf_error src/tf_error.cpp)

ament_target_dependencies(odometer
  rclcpp
  nav_msgs
  bunker_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(tf_error
  rclcpp
  tf2_ros
  tf2_geometry_msgs
)


# ----------------------------
# Link generated message interfaces (MODERN WAY)
# ----------------------------
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(odometer "${cpp_typesupport_target}")
target_link_libraries(tf_error "${cpp_typesupport_target}")

# ----------------------------
# Install targets
# ----------------------------
install(TARGETS
  odometer
  tf_error
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()