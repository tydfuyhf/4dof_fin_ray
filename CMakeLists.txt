cmake_minimum_required(VERSION 3.8)
project(my_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# =======================
# Executables (추가)
# =======================

add_executable(my_arm_ik_node src/my_arm_ik_node.cpp)
ament_target_dependencies(my_arm_ik_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(my_arm_ik_node Eigen3::Eigen)

add_executable(my_arm_seq_controller_node src/my_arm_seq_controller_node.cpp)
ament_target_dependencies(my_arm_seq_controller_node
  rclcpp
  sensor_msgs
)

# Install executables
install(TARGETS
  my_arm_ik_node
  my_arm_seq_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# =======================

# Install non-code resources
install(DIRECTORY
  launch
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()