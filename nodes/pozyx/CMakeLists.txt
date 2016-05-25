cmake_minimum_required(VERSION 2.8.3)
project(pozyx)
find_package(catkin REQUIRED roscpp std_msgs geometry_msgs sensor_msgs)
catkin_package(
	INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(pozyx src/pozyx.cpp)
add_library(pozyx-lib include/Pozyx-custom-library/Pozyx_core.cpp include/Pozyx-custom-library/Pozyx_lib.cpp include/Pozyx-custom-library/helpers.cpp)

target_link_libraries(pozyx pozyx-lib)

target_link_libraries(pozyx
        ${catkin_LIBRARIES}
)
