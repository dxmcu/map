# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "map_core: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(map_core_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv" NAME_WE)
add_custom_target(_map_core_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "map_core" "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv" "geometry_msgs/PoseStamped:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:sensor_msgs/PointField:geometry_msgs/Pose:sensor_msgs/PointCloud2"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(map_core
  "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_core
)

### Generating Module File
_generate_module_cpp(map_core
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_core
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(map_core_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(map_core_generate_messages map_core_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv" NAME_WE)
add_dependencies(map_core_generate_messages_cpp _map_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_core_gencpp)
add_dependencies(map_core_gencpp map_core_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_core_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(map_core
  "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_core
)

### Generating Module File
_generate_module_eus(map_core
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_core
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(map_core_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(map_core_generate_messages map_core_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv" NAME_WE)
add_dependencies(map_core_generate_messages_eus _map_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_core_geneus)
add_dependencies(map_core_geneus map_core_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_core_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(map_core
  "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_core
)

### Generating Module File
_generate_module_lisp(map_core
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_core
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(map_core_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(map_core_generate_messages map_core_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv" NAME_WE)
add_dependencies(map_core_generate_messages_lisp _map_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_core_genlisp)
add_dependencies(map_core_genlisp map_core_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_core_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(map_core
  "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_core
)

### Generating Module File
_generate_module_nodejs(map_core
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_core
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(map_core_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(map_core_generate_messages map_core_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv" NAME_WE)
add_dependencies(map_core_generate_messages_nodejs _map_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_core_gennodejs)
add_dependencies(map_core_gennodejs map_core_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_core_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(map_core
  "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_core
)

### Generating Module File
_generate_module_py(map_core
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_core
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(map_core_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(map_core_generate_messages map_core_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bohuan/catkin_test/src/map/srv/LocalMapRetrieve.srv" NAME_WE)
add_dependencies(map_core_generate_messages_py _map_core_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(map_core_genpy)
add_dependencies(map_core_genpy map_core_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS map_core_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/map_core
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(map_core_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(map_core_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/map_core
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(map_core_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(map_core_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/map_core
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(map_core_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(map_core_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_core)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/map_core
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(map_core_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(map_core_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_core)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_core\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/map_core
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(map_core_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(map_core_generate_messages_py sensor_msgs_generate_messages_py)
endif()
