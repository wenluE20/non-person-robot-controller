cmake_minimum_required(VERSION 3.0.2)
project(detect_lidar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)



## 配置 msg 源文件
add_message_files(
  FILES
  LidarDetect.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#执行时依赖
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)



# sub_data
add_executable(detect_lidar src/detect_lidar.cpp)
target_link_libraries(detect_lidar ${catkin_LIBRARIES} ${OpenCV_LIBS} )
add_dependencies(detect_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
