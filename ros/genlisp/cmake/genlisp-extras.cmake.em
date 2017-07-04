@[if DEVELSPACE]@
# bin and template dir variables in develspace
set(GENLISP_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/gen_lisp.py")
set(GENLISP_TEMPLATE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# bin and template dir variables in installspace
set(GENLISP_BIN "${genlisp_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/gen_lisp.py")
set(GENLISP_TEMPLATE_DIR "${genlisp_DIR}/..")
@[end if]@

# Generate .msg or .srv -> .lisp
# The generated .lisp files should be added ALL_GEN_OUTPUT_FILES_lisp
macro(_generate_lisp ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  file(MAKE_DIRECTORY ${ARG_GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(MSG_NAME ${ARG_MSG} NAME)
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}.lisp)
  set(GEN_OUTPUT_FILE ${ARG_GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  assert(CATKIN_ENV)
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENLISP_BIN} ${ARG_MSG} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENLISP_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${ARG_GEN_OUTPUT_DIR}
    COMMENT "Generating Lisp code from ${ARG_PKG}/${MSG_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_lisp ${GEN_OUTPUT_FILE})

endmacro()

#genlisp uses the same program to generate srv and msg files, so call the same macro
macro(_generate_msg_lisp ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  _generate_lisp(${ARG_PKG} ${ARG_MSG} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}/msg")
endmacro()

#genlisp uses the same program to generate srv and msg files, so call the same macro
macro(_generate_srv_lisp ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)
  _generate_lisp(${ARG_PKG} ${ARG_SRV} "${ARG_IFLAGS}" "${ARG_MSG_DEPS}" "${ARG_GEN_OUTPUT_DIR}/srv")
endmacro()

macro(_generate_module_lisp ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)
endmacro()

set(common_lisp_INSTALL_DIR share/common-lisp)
set(genlisp_INSTALL_DIR ${common_lisp_INSTALL_DIR}/ros)
