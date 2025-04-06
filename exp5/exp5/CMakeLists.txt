cmake_minimum_required(VERSION 3.0.2)
project(exp5)
 find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  actionlib
  move_base_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(multi_point_nav src/multi_point_nav.cpp)

target_link_libraries(multi_point_nav
${catkin_LIBRARIES}
)
