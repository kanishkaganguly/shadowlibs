cmake_minimum_required(VERSION 2.8.3)
project(shadowlibs)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)
add_compile_options(-pthread)

# ========================== Find all packages ================================================== #
find_package(catkin REQUIRED COMPONENTS
		geometry_msgs moveit_msgs std_msgs control_msgs trajectory_msgs visualization_msgs
		tf roscpp rospy interactive_markers
		moveit_ros_planning moveit_ros_planning_interface moveit_core moveit_visual_tools
		kdl_parser
		sr_robot_msgs
		sr_mechanism_controllers pluginlib ros_ethercat_model realtime_tools controller_interface
		)

# ========================== Find Eigen ========================================================= #
find_package(Eigen3 REQUIRED)
if (Eigen3_FOUND)
	message(STATUS "Eigen3 found")
else ()
	message(FATAL_ERROR "Eigen3 not found")
endif ()

# ========================== Find Boost ========================================================= #
# ROS MoveIt needs 1.58
set(BOOST_ROS_VERSION "1.58")
# boost::PyBind needs 1.67
set(BOOST_CUSTOM_VERSION "1.67")
option(BoostROS "Use ROS Boost 1.58" ON)
if (BoostROS)
	message(STATUS "Using ROS Boost")
	# Check for default Boost installation
	find_package(Boost ${BOOST_ROS_VERSION} REQUIRED)
else ()
	# Don't use system paths for searching
	set(Boost_NO_SYSTEM_PATHS TRUE)
	if (Boost_NO_SYSTEM_PATHS)
		set(BOOST_ROOT "/opt/boost")
		set(BOOST_INCLUDE_DIRS "${BOOST_ROOT}/include")
		set(BOOST_LIBRARY_DIRS "${BOOST_ROOT}/lib")
	endif (Boost_NO_SYSTEM_PATHS)
	# Check for custom Boost installation
	find_package(Boost ${BOOST_CUSTOM_VERSION} REQUIRED)
endif ()

if (Boost_FOUND)
	message(STATUS "Boost found")
else ()
	message(FATAL_ERROR "Boost not found")
endif ()

# ========================== Doxygen setup ====================================================== #
option(BUILD_DOCS "Build documentation" OFF)
if (BUILD_DOCS)
	if (DOXYGEN_FOUND)
		# set input and output files
		message(STATUS ${PROJECT_SOURCE_DIR})
		set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/docs/Doxyfile.in)
		set(DOXYGEN_OUT ${PROJECT_SOURCE_DIR}/docs/Doxyfile)
		# request to configure the file
		configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
		message(STATUS "Doxygen build started")
		add_custom_target(doc_doxygen ALL
				COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
				WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/docs
				COMMENT "Generating API documentation with Doxygen"
				VERBATIM)
	else (DOXYGEN_FOUND)
		message("Doxygen need to be installed to generate the doxygen documentation")
	endif (DOXYGEN_FOUND)
endif ()

# ========================== Setup catkin packages and libraries ================================ #
catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS
		geometry_msgs moveit_msgs std_msgs trajectory_msgs visualization_msgs control_msgs sensor_msgs
		tf roscpp rospy actionlib interactive_markers
		moveit_ros_planning moveit_ros_planning_interface moveit_core moveit_visual_tools
		sr_robot_msgs
		sr_mechanism_controllers pluginlib ros_ethercat_model realtime_tools controller_interface
		DEPENDS Boost
		LIBRARIES ${PROJECT_NAME} shadow_realtime_controllers
)

# ========================== Include headers ==================================================== #
include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${Boost_INCLUDE_DIR}
		${EIGEN3_INCLUDE_DIRS}
)

# ========================== Setup catkin packages and libraries ================================ #
file(GLOB lib_src ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} ${lib_src})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_library(shadow_realtime_controllers src/shadow_realtime_controller.cpp)

# ========================== Setup installation locations ======================================= #
install(DIRECTORY include/${PROJECT_NAME}/
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.hpp"
		)
install(TARGETS ${PROJECT_NAME}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
		)