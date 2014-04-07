cmake_minimum_required(VERSION 2.8.3)
project(oculus2wd)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  geometry_msgs
  sensor_msgs
  pcl_ros
  opencv_candidate
  cv_bridge
  image_transport
  dynamic_reconfigure
  nodelet
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   drive.msg
   drive_status.msg
   battery.msg
   arduino_power.msg
   mpu6050.msg
   distance_sensor.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/extract_line.cfg
  #...
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oculus2wd
  LIBRARIES image_signal_nd
  CATKIN_DEPENDS roscpp rospy std_msgs nodelet
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(oculus2wd
#   src/${PROJECT_NAME}/oculus2wd.cpp
# )

add_library(image_signal_nd src/image_signal_nd.cpp)
target_link_libraries(image_signal_nd ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(image_signal_nd ${catkin_EXPORTED_LIBRARIES})
endif()

## Declare a cpp executable
# add_executable(oculus2wd_node src/oculus2wd_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(oculus2wd_node oculus2wd_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(oculus2wd_node
#   ${catkin_LIBRARIES}
# )

add_executable(img2www src/img2www.cpp)
target_link_libraries(img2www ${catkin_LIBRARIES})

add_executable(points2_processor src/points2.cpp)
target_link_libraries(points2_processor ${catkin_LIBRARIES})

add_executable(extract_line src/extract_line.cpp)
target_link_libraries(extract_line ${catkin_LIBRARIES})

add_executable(image_signal src/image_signal.cpp)
target_link_libraries(image_signal ${catkin_LIBRARIES})

# make sure configure headers are built before any node using them
add_dependencies(extract_line ${PROJECT_NAME}_gencfg)


#add_executable(nurbs_surface src/nurbs_surface.cpp)
#target_link_libraries(nurbs_surface ${catkin_LIBRARIES})

add_executable(poisson_surface src/poisson_surface.cpp)
target_link_libraries(poisson_surface ${catkin_LIBRARIES})

add_executable(distance_ranger src/distance_ranger.cpp)
target_link_libraries(distance_ranger ${catkin_LIBRARIES})
add_dependencies(distance_ranger oculus2wd_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS oculus2wd oculus2wd_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_oculus2wd.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
