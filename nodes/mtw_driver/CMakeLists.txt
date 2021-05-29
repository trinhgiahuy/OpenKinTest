cmake_minimum_required(VERSION 2.8.3)
project(mtw_node)
find_package(catkin REQUIRED roscpp imu_sequenced std_msgs sensor_msgs message_generation)


#add_message_files(
#  FILES
#	StringStamped.msg
#)

#generate_messages(
#  DEPENDENCIES
#	std_msgs
#)

catkin_package(
#	INCLUDE_DIRS /usr/local/xsens/include
#	CATKIN_DEPENDS message_runtime
)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} /usr/local/xsens/include)

link_directories(/usr/local/xsens/lib)

add_executable(mtw_node src/mtw_node.cpp src/console.cpp src/myxda.cpp src/conio.c)
#add_library(pozyx-lib include/Pozyx-custom-library/Pozyx_core.cpp include/Pozyx-custom-library/Pozyx_lib.cpp include/Pozyx-custom-library/helpers.cpp)

target_link_libraries(mtw_node m xsensdeviceapi xstypes pthread )

target_link_libraries(mtw_node
        ${catkin_LIBRARIES}
)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
