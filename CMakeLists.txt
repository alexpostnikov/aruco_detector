cmake_minimum_required(VERSION 3.0.3)
project(aruco_detector)


set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS_DEBUG}")	
add_compile_options(-std=c++11 -Wall -Werror -pthread)


find_package(PkgConfig REQUIRED)
find_package(	
	catkin REQUIRED COMPONENTS
	cv_bridge
	roscpp
	roslib
	sensor_msgs
	std_msgs
	geometry_msgs
	visualization_msgs
	message_generation
	tf
)

pkg_search_module(EIGEN REQUIRED eigen3)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(${Eigen_INCLUDE_DIRS})


add_message_files(
  FILES
  exposure.msg
)


add_service_files(
  FILES
  ChangeExposure.srv
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(DEPENDS Eigen CATKIN_DEPENDS message_runtime std_msgs)

include_directories( 
	include 
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
)



# src/aruco_calibration.cpp src/aruco_initial_calibration.cpp

add_executable(aruco_detector_node src/aruco_main.cpp src/aruco_marker_classes.cpp src/aruco_utils.cpp src/aruco_scene.cpp src/serialunix.cpp)


target_link_libraries (
	aruco_detector_node 
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
	${Eigen_LIBRARIES}
	serial

)

add_dependencies(aruco_detector_node aruco_detector_generate_messages_cpp)

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   FILES_MATCHING PATTERN "*.launch"
# )

