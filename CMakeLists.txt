cmake_minimum_required(VERSION 2.8.3)
project(map)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-std=c++14)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)



include_directories(${CMAKE_CURRENT_BINARY_DIR} ${libpointmatcher_INCLUDE_DIRS} ${opencv_include_dirs} )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

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
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  nav_msgs
  pcl_conversions
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_planner
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


#sample 3d to 2d
#add_executable(sample_3d_to_2d
#        src/simple_3d_to_2d.cpp
#        include/simple_3d_to_2d.h
#        sample/sample_3d_to_2d.cpp)
#add_dependencies(sample_3d_to_2d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(sample_3d_to_2d ${libpointmatcher_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${catkin_LIBRARIES})


#测试common_area的
add_executable(test_common_area
        sample/test_common_area.cpp)
     #   include/common.h
     #   include/common_area.h
     #   include/cloud2map.h)
add_dependencies(test_common_area ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_common_area ${libpointmatcher_LIBRARIES}
        ${OpenCV_LIBS}
       ${catkin_LIBRARIES})


#测试cvmat和gridmap的互相转换
add_executable(test_cv2grid
        sample/test_cv2grid.cpp
        include/cvMat2GridMap.h
        include/common.h)
        #src/cloud2map.cpp)
add_dependencies(test_cv2grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_cv2grid ${libpointmatcher_LIBRARIES}
        ${OpenCV_LIBS}
       ${catkin_LIBRARIES})



#测试cloud到2D的程序
add_executable(test_cloud2map
        sample/test_cloud2map.cpp
        include/cloud2map.h
        include/common.h)
        #src/cloud2map.cpp)
add_dependencies(test_cloud2map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_cloud2map ${libpointmatcher_LIBRARIES}
        ${OpenCV_LIBS}
       ${catkin_LIBRARIES})




#主程序
add_executable(map
  src/map.cpp
        include/simple_3d_to_2d.h
        src/simple_3d_to_2d.cpp
        include/common.h
        include/cvMat2GridMap.h
        #include/cloud2map.h
        #src/cloud2map.cpp
        include/common_area.h include/showPointCloud.h)
add_dependencies(map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(map ${libpointmatcher_LIBRARIES}
${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

#add_library(publisher
#  src/publisher.cpp
#)
#add_dependencies(publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(publisher
#  ${catkin_LIBRARIES}
#)
