cmake_minimum_required(VERSION 3.0.2)
project(random_coordinates_receiver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  pcl_ros
)

find_package(PCL REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
random_coordinates_receiver_node src/random_coordinates_receiver.cpp
)
add_executable(
pointcloud_creator src/pointcloud_creator.cpp
)
add_executable(
D_star_planner src/D_star_planner.cpp
)

target_link_libraries(random_coordinates_receiver_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pointcloud_creator
  ${catkin_LIBRARIES}
)

target_link_libraries(D_star_planner
  ${catkin_LIBRARIES}
)

target_link_libraries(pointcloud_creator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)
