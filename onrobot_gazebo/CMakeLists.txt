cmake_minimum_required(VERSION 2.8.3)
project(onrobot_gazebo)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    actionlib
    control_msgs
    sensor_msgs
)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    LIBRARIES
        onrobot_gripper_plugin
    CATKIN_DEPENDS
        roscpp
        gazebo_ros
        actionlib
        control_msgs
        sensor_msgs
)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(onrobot_gripper_plugin src/gripper_plugin.cpp)
target_link_libraries(onrobot_gripper_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(onrobot_gripper_plugin ${catkin_EXPORTED_TARGETS})

install(
    TARGETS onrobot_gripper_plugin
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir include)
    install(DIRECTORY ${dir}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
