cmake_minimum_required(VERSION 2.8.3)
project(shadowlibs)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs moveit_msgs std_msgs control_msgs trajectory_msgs visualization_msgs
        tf roscpp rospy interactive_markers
        moveit_ros_planning moveit_ros_planning_interface moveit_core moveit_visual_tools
        sr_robot_msgs
        )

find_package(Eigen3 REQUIRED)
if (Eigen3_FOUND)
    message(STATUS "Eigen3 found")
else ()
    message(FATAL_ERROR "Eigen3 not found")
endif ()

# ROS MoveIt needs 1.58
set(BOOST_VERSION_REQUIRED "1.58")
# Checks custom Boost folder for installation
# Used only if there are multiple installs
set(Boost_NO_SYSTEM_PATHS TRUE)
# Find Boost in custom folder
if (Boost_NO_SYSTEM_PATHS)
    set(BOOST_ROOT "/opt/boost")
    set(BOOST_INCLUDE_DIRS "${BOOST_ROOT}/include")
    set(BOOST_LIBRARY_DIRS "${BOOST_ROOT}/lib")
endif (Boost_NO_SYSTEM_PATHS)
# Check for default Boost installation
find_package(Boost ${BOOST_VERSION_REQUIRED} REQUIRED)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        geometry_msgs moveit_msgs std_msgs trajectory_msgs visualization_msgs control_msgs sensor_msgs
        tf roscpp rospy actionlib interactive_markers
        moveit_ros_planning moveit_ros_planning_interface moveit_core moveit_visual_tools
        sr_robot_msgs
        DEPENDS Boost
        LIBRARIES ${PROJECT_NAME}
)

file(GLOB lib_src ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} ${lib_src})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
        )
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )