cmake_minimum_required(VERSION 3.0.2)
project(random_coordinates_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(random_coordinates_publisher_node src/random_coordinates_publisher.cpp)
target_link_libraries(random_coordinates_publisher_node
  ${catkin_LIBRARIES}
)
