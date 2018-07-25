cmake_minimum_required(VERSION 2.8.3)
project(platform_vision)

## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure

  pcl_conversions
  pcl_ros
)

set(OpenCV_INCLUDE_DIRS /usr/local/)
set(OpenCV_LIBS /usr/local/lib/libopencv_core.so)

find_package(OpenCV 3 REQUIRED
NO_MODULE # should be optional, tells CMake to use config mode
PATHS /usr/local/ # look here/usr/local/include/opencv
NO_DEFAULT_PATH) # and don't look anywhere else


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
 catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
   cfg/pid.cfg
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
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES platform_vision
#  CATKIN_DEPENDS camera_info_manager cv_bridge geometry_msgs image_transport roscpp rospy std_msgs tf
#  DEPENDS system_lib
INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
${catkin_INCLUDE_DIRS}
${CUDA_INCLUDE_DIRS}
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/platform_vision.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/platform_vision_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_platform_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#add_executable(cuda_test src/cuda_test.cpp)
#target_link_libraries(cuda_test
#                      ${OpenCV_LIBS}
#                      ${catkin_LIBRARIES})



add_executable(rectify_image src/rectify_image.cpp)
target_link_libraries(rectify_image
                      ${OpenCV_LIBS}
                      ${catkin_LIBRARIES})



#Find the CUDA packages
find_package(CUDA REQUIRED)

add_executable(match_stereo src/match_stereo.cpp)
target_link_libraries(match_stereo ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(rectify_img_bouguet src/rectify_img_bouguet.cpp)
target_link_libraries(rectify_img_bouguet ${OpenCV_LIBS} ${catkin_LIBRARIES})


add_executable(track_marker src/track_marker.cpp)
target_link_libraries(track_marker ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(cross_corrolation src/cross_corrolation.cpp)
target_link_libraries(cross_corrolation ${OpenCV_LIBS} ${catkin_LIBRARIES})
include_directories(/home/abdulla/dev/catkin_ws/src/platform_vision/src/include)

add_executable(collect_data src/collect_data.cpp)
target_link_libraries(collect_data ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(disparity_node src/disparity_node.cpp)
target_link_libraries(disparity_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(platform_rectify_node src/platform_rectify_node.cpp)
target_link_libraries(platform_rectify_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(pointcloud_segmentation src/pointcloud_segmentation.cpp)
target_link_libraries(pointcloud_segmentation ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(save_pointCloud src/save_pointCloud.cpp)
target_link_libraries(save_pointCloud ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(structureFromMotion src/structureFromMotion.cpp)
target_link_libraries(structureFromMotion ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(featureBasedMatchingCuda src/featureBasedMatchingCuda.cpp)
target_link_libraries(featureBasedMatchingCuda ${OpenCV_LIBS} ${catkin_LIBRARIES})
