# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "vacuum_cmd_msg: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(vacuum_cmd_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv" NAME_WE)
add_custom_target(_vacuum_cmd_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "vacuum_cmd_msg" "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(vacuum_cmd_msg
  "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vacuum_cmd_msg
)

### Generating Module File
_generate_module_cpp(vacuum_cmd_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vacuum_cmd_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(vacuum_cmd_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(vacuum_cmd_msg_generate_messages vacuum_cmd_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv" NAME_WE)
add_dependencies(vacuum_cmd_msg_generate_messages_cpp _vacuum_cmd_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vacuum_cmd_msg_gencpp)
add_dependencies(vacuum_cmd_msg_gencpp vacuum_cmd_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vacuum_cmd_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(vacuum_cmd_msg
  "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vacuum_cmd_msg
)

### Generating Module File
_generate_module_eus(vacuum_cmd_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vacuum_cmd_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(vacuum_cmd_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(vacuum_cmd_msg_generate_messages vacuum_cmd_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv" NAME_WE)
add_dependencies(vacuum_cmd_msg_generate_messages_eus _vacuum_cmd_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vacuum_cmd_msg_geneus)
add_dependencies(vacuum_cmd_msg_geneus vacuum_cmd_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vacuum_cmd_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(vacuum_cmd_msg
  "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vacuum_cmd_msg
)

### Generating Module File
_generate_module_lisp(vacuum_cmd_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vacuum_cmd_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(vacuum_cmd_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(vacuum_cmd_msg_generate_messages vacuum_cmd_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv" NAME_WE)
add_dependencies(vacuum_cmd_msg_generate_messages_lisp _vacuum_cmd_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vacuum_cmd_msg_genlisp)
add_dependencies(vacuum_cmd_msg_genlisp vacuum_cmd_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vacuum_cmd_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(vacuum_cmd_msg
  "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vacuum_cmd_msg
)

### Generating Module File
_generate_module_nodejs(vacuum_cmd_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vacuum_cmd_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(vacuum_cmd_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(vacuum_cmd_msg_generate_messages vacuum_cmd_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv" NAME_WE)
add_dependencies(vacuum_cmd_msg_generate_messages_nodejs _vacuum_cmd_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vacuum_cmd_msg_gennodejs)
add_dependencies(vacuum_cmd_msg_gennodejs vacuum_cmd_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vacuum_cmd_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(vacuum_cmd_msg
  "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vacuum_cmd_msg
)

### Generating Module File
_generate_module_py(vacuum_cmd_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vacuum_cmd_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(vacuum_cmd_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(vacuum_cmd_msg_generate_messages vacuum_cmd_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/iclab/arc_ws/src/manipulator_7a/vacuum/vacuum_cmd_msg/srv/VacuumCmd.srv" NAME_WE)
add_dependencies(vacuum_cmd_msg_generate_messages_py _vacuum_cmd_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(vacuum_cmd_msg_genpy)
add_dependencies(vacuum_cmd_msg_genpy vacuum_cmd_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS vacuum_cmd_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vacuum_cmd_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/vacuum_cmd_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(vacuum_cmd_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vacuum_cmd_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/vacuum_cmd_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(vacuum_cmd_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vacuum_cmd_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/vacuum_cmd_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(vacuum_cmd_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vacuum_cmd_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/vacuum_cmd_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(vacuum_cmd_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vacuum_cmd_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vacuum_cmd_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/vacuum_cmd_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(vacuum_cmd_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
