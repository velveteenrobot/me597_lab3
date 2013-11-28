cmake_minimum_required(VERSION 2.8.3)
project(me597_lab3)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  rospy
  sensor_msgs
  geometry_msgs
  roscpp
  tf
  message_generation
)

add_message_files(
  FILES
  ips_msg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

#catkin_package(
#  CATKIN_DEPENDS 
#  message_runtime 
#  geometry_msgs 
#  std_msgs 
#  sensor_msgs
#)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(turtlebot_localizer src/localizer.cpp src/PoseParticle.cpp src/marker.cpp)
add_executable(make_a_map src/make_a_map.cpp)
add_executable(sim_pose_publisher src/sim_pose_publisher.cpp)
add_executable(vis_pose_publisher src/vis_pose_publisher.cpp)

target_link_libraries(make_a_map
  ${catkin_LIBRARIES}
)

target_link_libraries(sim_pose_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(vis_pose_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(turtlebot_localizer
  ${catkin_LIBRARIES}
)
