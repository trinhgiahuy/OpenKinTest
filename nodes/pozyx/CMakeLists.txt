cmake_minimum_required(VERSION 2.8.3)
project(pozyx)
find_package(catkin REQUIRED roscpp std_msgs)
catkin_package(
	INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(pozyx src/pozyx.cpp)

target_link_libraries(pozyx
        ${catkin_LIBRARIES}
)
