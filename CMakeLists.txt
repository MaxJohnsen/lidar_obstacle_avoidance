cmake_minimum_required(VERSION 2.8.3)
project(lidar_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)


catkin_package()


include_directories(${catkin_INCLUDE_DIRS})


add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES})
add_dependencies(main lidar_obstacle_avoidance_generate_messages_cpp)
