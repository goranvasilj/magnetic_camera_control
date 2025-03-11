cmake_minimum_required(VERSION 2.8.3)
project(magnetic_camera_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)
catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(magnetic_camera_control src/magnetic_camera_control.cpp)
target_link_libraries(magnetic_camera_control ${catkin_LIBRARIES})

