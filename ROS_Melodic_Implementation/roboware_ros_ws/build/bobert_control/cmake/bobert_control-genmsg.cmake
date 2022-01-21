# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bobert_control: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ibobert_control:/home/steffen/bobert_ws/src/bobert_control/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bobert_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg" NAME_WE)
add_custom_target(_bobert_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bobert_control" "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg" ""
)

get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg" NAME_WE)
add_custom_target(_bobert_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bobert_control" "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bobert_control
)
_generate_msg_cpp(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bobert_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(bobert_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bobert_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bobert_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bobert_control_generate_messages bobert_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_cpp _bobert_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_cpp _bobert_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bobert_control_gencpp)
add_dependencies(bobert_control_gencpp bobert_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bobert_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bobert_control
)
_generate_msg_eus(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bobert_control
)

### Generating Services

### Generating Module File
_generate_module_eus(bobert_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bobert_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bobert_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bobert_control_generate_messages bobert_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_eus _bobert_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_eus _bobert_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bobert_control_geneus)
add_dependencies(bobert_control_geneus bobert_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bobert_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bobert_control
)
_generate_msg_lisp(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bobert_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(bobert_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bobert_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bobert_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bobert_control_generate_messages bobert_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_lisp _bobert_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_lisp _bobert_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bobert_control_genlisp)
add_dependencies(bobert_control_genlisp bobert_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bobert_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bobert_control
)
_generate_msg_nodejs(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bobert_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(bobert_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bobert_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bobert_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bobert_control_generate_messages bobert_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_nodejs _bobert_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_nodejs _bobert_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bobert_control_gennodejs)
add_dependencies(bobert_control_gennodejs bobert_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bobert_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bobert_control
)
_generate_msg_py(bobert_control
  "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bobert_control
)

### Generating Services

### Generating Module File
_generate_module_py(bobert_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bobert_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bobert_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bobert_control_generate_messages bobert_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/armCmd.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_py _bobert_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/steffen/bobert_ws/src/bobert_control/msg/bobertTelemetry.msg" NAME_WE)
add_dependencies(bobert_control_generate_messages_py _bobert_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bobert_control_genpy)
add_dependencies(bobert_control_genpy bobert_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bobert_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bobert_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bobert_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bobert_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bobert_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bobert_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(bobert_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bobert_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bobert_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(bobert_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bobert_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bobert_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(bobert_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bobert_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bobert_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bobert_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bobert_control_generate_messages_py std_msgs_generate_messages_py)
endif()
