# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "actor_plugin: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(actor_plugin_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv" NAME_WE)
add_custom_target(_actor_plugin_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "actor_plugin" "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv" ""
)

get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv" NAME_WE)
add_custom_target(_actor_plugin_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "actor_plugin" "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/actor_plugin
)
_generate_srv_cpp(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/actor_plugin
)

### Generating Module File
_generate_module_cpp(actor_plugin
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/actor_plugin
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(actor_plugin_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(actor_plugin_generate_messages actor_plugin_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_cpp _actor_plugin_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_cpp _actor_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(actor_plugin_gencpp)
add_dependencies(actor_plugin_gencpp actor_plugin_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS actor_plugin_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/actor_plugin
)
_generate_srv_eus(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/actor_plugin
)

### Generating Module File
_generate_module_eus(actor_plugin
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/actor_plugin
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(actor_plugin_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(actor_plugin_generate_messages actor_plugin_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_eus _actor_plugin_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_eus _actor_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(actor_plugin_geneus)
add_dependencies(actor_plugin_geneus actor_plugin_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS actor_plugin_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/actor_plugin
)
_generate_srv_lisp(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/actor_plugin
)

### Generating Module File
_generate_module_lisp(actor_plugin
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/actor_plugin
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(actor_plugin_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(actor_plugin_generate_messages actor_plugin_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_lisp _actor_plugin_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_lisp _actor_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(actor_plugin_genlisp)
add_dependencies(actor_plugin_genlisp actor_plugin_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS actor_plugin_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/actor_plugin
)
_generate_srv_nodejs(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/actor_plugin
)

### Generating Module File
_generate_module_nodejs(actor_plugin
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/actor_plugin
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(actor_plugin_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(actor_plugin_generate_messages actor_plugin_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_nodejs _actor_plugin_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_nodejs _actor_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(actor_plugin_gennodejs)
add_dependencies(actor_plugin_gennodejs actor_plugin_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS actor_plugin_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/actor_plugin
)
_generate_srv_py(actor_plugin
  "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/actor_plugin
)

### Generating Module File
_generate_module_py(actor_plugin
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/actor_plugin
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(actor_plugin_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(actor_plugin_generate_messages actor_plugin_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/SetPose.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_py _actor_plugin_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jieming/car_ws/src/actor_plugin/srv/GetVel.srv" NAME_WE)
add_dependencies(actor_plugin_generate_messages_py _actor_plugin_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(actor_plugin_genpy)
add_dependencies(actor_plugin_genpy actor_plugin_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS actor_plugin_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/actor_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/actor_plugin
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(actor_plugin_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/actor_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/actor_plugin
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(actor_plugin_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/actor_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/actor_plugin
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(actor_plugin_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/actor_plugin)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/actor_plugin
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(actor_plugin_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/actor_plugin)
  install(CODE "execute_process(COMMAND \"/home/jieming/miniconda3/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/actor_plugin\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/actor_plugin
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(actor_plugin_generate_messages_py std_msgs_generate_messages_py)
endif()
