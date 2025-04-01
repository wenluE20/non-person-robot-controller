cmake_minimum_required(VERSION 3.0.2)
project(exp4)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)
catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/eaibot/vicon_ws/devel/include
)

add_executable(motionCapture src/motionCapture.cpp)
target_link_libraries(motionCapture
${catkin_LIBRARIES}
)
