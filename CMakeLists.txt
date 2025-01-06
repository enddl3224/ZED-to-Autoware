cmake_minimum_required(VERSION 3.8)
project(zed_convert)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(zed_msgs REQUIRED)

# include directories
include_directories(
  include
)

# create library
add_library(zedconvert_lib SHARED src/zed_convert.cpp)
ament_target_dependencies(
  zedconvert_lib 
  rclcpp
  rclcpp_components
  autoware_auto_perception_msgs
  zed_msgs
)

# create executable
add_executable(zed_convert_node src/zed_convert.cpp)
ament_target_dependencies(
  zed_convert_node
  rclcpp
  rclcpp_components
  autoware_auto_perception_msgs
  zed_msgs
)
target_link_libraries(zed_convert_node zedconvert_lib)

# install targets
install(TARGETS
  zedconvert_lib
  zed_convert_node
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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

