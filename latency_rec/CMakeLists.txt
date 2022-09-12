cmake_minimum_required(VERSION 3.5)
project(latency_rec)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_lib SHARED src/latency_log.cpp src/latency_rec.cpp)
target_link_libraries(${PROJECT_NAME}_lib
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "latency_rec::LatencyRec"
  EXECUTABLE latency_rec
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
