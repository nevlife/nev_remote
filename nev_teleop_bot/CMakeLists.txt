cmake_minimum_required(VERSION 3.8)
project(nev_teleop_bot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

set(BRIDGE_INCS ${CMAKE_CURRENT_SOURCE_DIR}/include ${GST_INCLUDE_DIRS})
set(BRIDGE_LIBS ${GST_LIBRARIES} zenohc)

add_executable(video_bridge_h265 src/video_bridge_h265.cpp)
target_include_directories(video_bridge_h265 PRIVATE ${BRIDGE_INCS})
target_link_libraries(video_bridge_h265 ${BRIDGE_LIBS})
ament_target_dependencies(video_bridge_h265 rclcpp sensor_msgs)

add_executable(video_bridge_h264 src/video_bridge_h264.cpp)
target_include_directories(video_bridge_h264 PRIVATE ${BRIDGE_INCS})
target_link_libraries(video_bridge_h264 ${BRIDGE_LIBS})
ament_target_dependencies(video_bridge_h264 rclcpp sensor_msgs)

install(TARGETS video_bridge_h265 video_bridge_h264 DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/net_bridge.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/net_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
