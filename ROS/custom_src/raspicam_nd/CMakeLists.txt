set(CMAKE_VERBOSE_MAKEFILE ON)
cmake_minimum_required(VERSION 2.8.3)
project(raspicam_nd)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)

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

catkin_package(
	LIBRARIES raspicam_nd
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)

add_library(raspicam_nd src/raspicam_nd.cpp)
target_link_libraries(
	raspicam_nd
	${raspicam_LIBS}
	${raspicam_CV_LIBS}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)
if(catkin_EXPORTED_LIBRARIES)
	add_dependencies(raspicam_nd ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES raspicam_nd.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
