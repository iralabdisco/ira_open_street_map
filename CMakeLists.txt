cmake_minimum_required(VERSION 2.8.3)
project(ira_open_street_map)

SET (CMAKE_BUILD_TYPE Debug)
SET (ROS_BUILD_TYPE Debug)
#SET (CMAKE_BUILD_TYPE Release)
#SET (ROS_BUILD_TYPE Release)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED   
  dynamic_reconfigure
  geodesy
  geographic_msgs
  geometry_msgs
  rospy
  roslib
  tf
  std_msgs
  visualization_msgs
  message_generation
  roscpp
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


## Generate services in the 'srv' folder
# add OSM Query node service
add_service_files(
  FILES
  ecef_2_lla.srv
  lla_2_ecef.srv
  get_distance_from_xy.srv
  latlon_2_xy.srv
  xy_2_latlon.srv
  way_direction.srv
  oneway.srv
  snap_particle_xy.srv
  get_closest_way_distance_utm.srv
  get_node_coordinates.srv
  getHighwayInfo.srv
  getDistanceFromLaneCenter.srv
)

# Generate service messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
#file(STRINGS ${CMAKE_SOURCE_DIR}/.gitignore gitignore)
#foreach(line IN LISTS gitignore)
#  message("line = ${line}")
#endforeach()
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
  IF(NOT ${child} MATCHES "dataset")
      IF(IS_DIRECTORY ${child})
      file(GLOB_RECURSE dir_files "${child}/*")
      LIST(APPEND extra_files ${dir_files})
    ENDIF()
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})


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
#  LIBRARIES ira_open_street_map
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(ira_open_street_map
#   src/${PROJECT_NAME}/ira_open_street_map.cpp
# )

## Declare cpp executable
add_executable(osm_downloader_node
	src/osm_downloader_node.cpp 
)

add_executable(osm_query_node
        src/osm_query_node.cpp
)

add_executable(osm_map_test_node
        src/osm_map_test_node.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ira_open_street_map_node ira_open_street_map_generate_messages_cpp)

add_dependencies(osm_query_node ira_open_street_map_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(osm_downloader_node
  ${catkin_LIBRARIES} curl
)

target_link_libraries(osm_query_node
  ${catkin_LIBRARIES} osmpbf z pthread protobuf-lite expat
)

target_link_libraries(osm_map_test_node
  ${catkin_LIBRARIES}
)

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
# install(TARGETS ira_open_street_map ira_open_street_map_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ira_open_street_map.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
