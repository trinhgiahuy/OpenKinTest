cmake_minimum_required(VERSION 2.8.3)
project(imu_sequenced)
find_package(catkin REQUIRED sensor_msgs message_generation)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(
  FILES
	ImuSequenced.msg
)

generate_messages(
  DEPENDENCIES
	sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)


#target_link_libraries(imu_sequenced
#        ${catkin_LIBRARIES}
#)
