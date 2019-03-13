cmake_minimum_required(VERSION 2.8.3)
project(dual_arm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  robot.srv
)
add_message_files(
  FILES
  robot_msg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  CATKIN_DEPENDS message_runtime
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/robot_pub.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker dual_arm_control_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
