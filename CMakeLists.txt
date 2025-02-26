cmake_minimum_required(VERSION 3.8)
project(turtlebot3_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

install(PROGRAMS
  src/turtle3_prop_controller.py
  src/turtle3_reactive_nav.py
  src/turtle3_map_generator.py
  src/turtle3_presenter.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
