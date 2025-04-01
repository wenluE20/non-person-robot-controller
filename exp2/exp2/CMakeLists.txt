cmake_minimum_required(VERSION 3.0.2)
project(exp2)
 find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  cv_bridge
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(exp2_1 src/exp2_1.cpp)
add_executable(exp2_2 src/exp2_2.cpp)
target_link_libraries(exp2_1
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
target_link_libraries(exp2_2
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
