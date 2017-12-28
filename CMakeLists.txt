cmake_minimum_required(VERSION 2.8.3)
project(map_core)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_definitions(-std=c++14)

find_package(catkin REQUIRED)
find_package(libpointmatcher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${libpointmatcher_INCLUDE_DIRS} ${opencv_include_dirs} )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  nav_msgs
  pcl_conversions
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  rospy
  std_msgs
        message_generation)

add_service_files(
        FILES
        LocalMapRetrieve.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
        sensor_msgs
)

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

#HYH的程序地方的功能测试
add_executable(test_map_db_node
        sample/test_map_db_node.cpp
        src/point_cloud.cpp
        src/laserutils.cpp)
add_dependencies(test_map_db_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} map_core_generate_messages_cpp)
target_link_libraries(test_map_db_node ${libpointmatcher_LIBRARIES}
        ${OpenCV_LIBS}
      ${catkin_LIBRARIES})



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
 #       include/simple_3d_to_2d.h
        #src/simple_3d_to_2d.cpp
        include/common.h
  #      include/cvMat2GridMap.h
        #include/cloud2map.h
        #src/cloud2map.cpp
        include/common_area.h
        include/showPointCloud.h
        include/point_cloud.h
        src/point_cloud.cpp
        include/map_db_node.h)
add_dependencies(map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(map ${libpointmatcher_LIBRARIES}
${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

