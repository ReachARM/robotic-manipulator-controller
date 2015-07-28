# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ArmController: 0 messages, 4 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ArmController_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv" NAME_WE)
add_custom_target(_ArmController_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ArmController" "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv" ""
)

get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv" NAME_WE)
add_custom_target(_ArmController_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ArmController" "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv" ""
)

get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv" NAME_WE)
add_custom_target(_ArmController_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ArmController" "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv" ""
)

get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv" NAME_WE)
add_custom_target(_ArmController_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ArmController" "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ArmController
)
_generate_srv_cpp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ArmController
)
_generate_srv_cpp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ArmController
)
_generate_srv_cpp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ArmController
)

### Generating Module File
_generate_module_cpp(ArmController
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ArmController
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ArmController_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ArmController_generate_messages ArmController_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_cpp _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_cpp _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_cpp _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_cpp _ArmController_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ArmController_gencpp)
add_dependencies(ArmController_gencpp ArmController_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ArmController_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ArmController
)
_generate_srv_lisp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ArmController
)
_generate_srv_lisp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ArmController
)
_generate_srv_lisp(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ArmController
)

### Generating Module File
_generate_module_lisp(ArmController
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ArmController
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ArmController_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ArmController_generate_messages ArmController_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_lisp _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_lisp _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_lisp _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_lisp _ArmController_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ArmController_genlisp)
add_dependencies(ArmController_genlisp ArmController_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ArmController_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController
)
_generate_srv_py(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController
)
_generate_srv_py(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController
)
_generate_srv_py(ArmController
  "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController
)

### Generating Module File
_generate_module_py(ArmController
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ArmController_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ArmController_generate_messages ArmController_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetArmStatus.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_py _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveRelativeTool.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_py _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/GetMotorAngle.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_py _ArmController_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mojo/arm_control_workspace/src/ArmController/srv/MoveAbsoluteMotor.srv" NAME_WE)
add_dependencies(ArmController_generate_messages_py _ArmController_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ArmController_genpy)
add_dependencies(ArmController_genpy ArmController_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ArmController_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ArmController)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ArmController
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ArmController_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ArmController)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ArmController
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ArmController_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ArmController
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ArmController_generate_messages_py std_msgs_generate_messages_py)
