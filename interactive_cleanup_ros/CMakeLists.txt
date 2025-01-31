cmake_minimum_required(VERSION 3.8)
project(interactive_cleanup_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interactive_cleanup REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
  /usr/local/include
)

#-- for Launch
install(DIRECTORY launch/
DESTINATION share/${PROJECT_NAME}/launch)
#--- end Launch

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

include_directories(include)
link_directories(/usr/local/lib)
add_executable(interactive_cleanup_sample src/interactive_cleanup_sample.cpp)
add_executable(teleop_key_interactive_cleanup src/teleop_key_interactive_cleanup.cpp)

ament_target_dependencies(interactive_cleanup_sample
  rclcpp
  geometry_msgs
  std_msgs
  trajectory_msgs
  tf2
  tf2_ros
  interactive_cleanup
)

ament_target_dependencies(teleop_key_interactive_cleanup
  rclcpp
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interactive_cleanup
)

install(TARGETS
  interactive_cleanup_sample
  teleop_key_interactive_cleanup
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

target_link_libraries(interactive_cleanup_sample)
target_link_libraries(teleop_key_interactive_cleanup)
ament_package()
