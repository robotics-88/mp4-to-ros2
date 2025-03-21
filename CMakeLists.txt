cmake_minimum_required(VERSION 3.8)
project(mp4_to_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(EXIV2 REQUIRED exiv2)
include_directories(${EXIV2_INCLUDE_DIRS})
link_directories(${EXIV2_LIBRARY_DIRS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(messages_88 REQUIRED)

find_package(OpenCV REQUIRED)

add_executable(mp4_node
  src/mp4.cpp
)

target_link_libraries(mp4_node 
  ${OpenCV_LIBRARIES}
  ${EXIV2_LIBRARIES}
)

ament_target_dependencies(mp4_node
  rclcpp
  cv_bridge
  sensor_msgs
  camera_calibration_parsers
  tf2
  tf2_ros
  geometry_msgs
  messages_88
)

install(TARGETS
  mp4_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()