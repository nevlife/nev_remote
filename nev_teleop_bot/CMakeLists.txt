cmake_minimum_required(VERSION 3.8)
project(nev_teleop_bot)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CUDAToolkit REQUIRED)
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

add_executable(multi_cam_raw_pub_node src/multi_cam_raw_pub_node.cpp)
target_include_directories(multi_cam_raw_pub_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${GST_INCLUDE_DIRS})
target_link_libraries(multi_cam_raw_pub_node ${GST_LIBRARIES})
ament_target_dependencies(multi_cam_raw_pub_node rclcpp sensor_msgs)

add_executable(multi_cam_pub_node src/multi_cam_pub_node.cpp)
target_include_directories(multi_cam_pub_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${GST_INCLUDE_DIRS})
target_link_libraries(multi_cam_pub_node
  ${GST_LIBRARIES} opencv_core opencv_imgproc opencv_calib3d
  nlohmann_json::nlohmann_json
  CUDA::cudart CUDA::nppig CUDA::nppc)
ament_target_dependencies(multi_cam_pub_node rclcpp sensor_msgs ament_index_cpp)

add_executable(wide_cam_pub_node src/wide_cam_pub_node.cpp)
target_include_directories(wide_cam_pub_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(wide_cam_pub_node rclcpp sensor_msgs message_filters)

add_library(multi_cam_pub_component SHARED src/multi_cam_pub_component.cpp)
target_include_directories(multi_cam_pub_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${GST_INCLUDE_DIRS})
target_link_libraries(multi_cam_pub_component
  ${GST_LIBRARIES} opencv_core opencv_imgproc opencv_calib3d
  nlohmann_json::nlohmann_json
  CUDA::cudart CUDA::nppig CUDA::nppc)
ament_target_dependencies(multi_cam_pub_component
  rclcpp rclcpp_components sensor_msgs ament_index_cpp)
rclcpp_components_register_nodes(multi_cam_pub_component "MultiCamPub")

add_library(wide_cam_pub_component SHARED src/wide_cam_pub_component.cpp)
target_include_directories(wide_cam_pub_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(wide_cam_pub_component
  rclcpp rclcpp_components sensor_msgs message_filters)
rclcpp_components_register_nodes(wide_cam_pub_component "WideCamPub")

install(TARGETS video_bridge_h265 video_bridge_h264 multi_cam_raw_pub_node multi_cam_pub_node wide_cam_pub_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS multi_cam_pub_component wide_cam_pub_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(PROGRAMS scripts/net_bridge.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/net_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config param DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
