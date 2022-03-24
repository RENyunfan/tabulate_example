cmake_minimum_required(VERSION 2.8.3)
project(device_monitor)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        livox_ros_driver
        )

catkin_package(

        INCLUDE_DIRS include
        LIBRARIES
        CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        ../../devel/include
        include
        ${catkin_INCLUDE_DIRS}

)


set(THIRD_PARTY
        ${catkin_LIBRARIES}
        )

add_executable(device_monitor
        Apps/device_monitor_node.cpp
        )
target_link_libraries(device_monitor
        ${THIRD_PARTY}
        )

add_executable(test_node
        Apps/test_node.cpp
        )
target_link_libraries(test_node
        ${THIRD_PARTY}
        )
