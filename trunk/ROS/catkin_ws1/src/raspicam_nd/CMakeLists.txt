cmake_minimum_required(VERSION 2.8.3)
project(raspicam_nd)

find_package(catkin REQUIRED COMPONENTS
	image_transport
	roscpp
	cv_bridge
	camera_info_manager
	nodelet
)
find_package(raspicam REQUIRED)
find_package(OpenCV)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

add_library(raspicam_nd src/raspicam_nd.cpp)
target_link_libraries(raspicam_nd ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(raspicam_nd ${catkin_EXPORTED_LIBRARIES})
endif()
