#
#  Generated from roslisp/cmake/roslisp-extras.cmake.in
#

@[if DEVELSPACE]@
# location of script in develspace
set(ROSLISP_MAKE_NODE_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/make_node_exec")
set(ROSLISP_COMPILE_MANIFEST_BIN "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/compile_load_manifest")
set(ROSLISP_EXE_SCRIPT "@(CMAKE_CURRENT_SOURCE_DIR)/scripts/make_exe_script")
@[else]@
# location of script in installspace
set(ROSLISP_MAKE_NODE_BIN "${roslisp_DIR}/../scripts/make_node_exec")
set(ROSLISP_COMPILE_MANIFEST_BIN "${roslisp_DIR}/../scripts/compile_load_manifest")
set(ROSLISP_EXE_SCRIPT "${roslisp_DIR}/../scripts/make_exe_script")
@[end if]@

# Build up a list of executables, in order to make them depend on each
# other, to avoid building them in parallel, because it's not safe to do
# that.
# The first entry in this list will be a target to compile ros-load-manifest
# as all the executables depend on it.
if(NOT TARGET _roslisp_load_manifest)
  add_custom_target(_roslisp_load_manifest ALL COMMAND ${ROSLISP_COMPILE_MANIFEST_BIN})
endif()
set(ROSLISP_EXECUTABLES _roslisp_load_manifest)


# example usage:
# add_compiled_lisp_executable(my_script my-system my-system:my-func [my_targetname])
function(add_compiled_lisp_executable output system_name entry_point)
  if(${ARGC} LESS 3 OR ${ARGC} GREATER 4)
    message(SEND_ERROR "[roslisp] add_compiled_lisp_executable can only have 3 or 4 arguments")
  elseif(${ARGC} LESS 4)
    set(targetname _roslisp_${output})
  else()
    set(extra_macro_args ${ARGN})
    list(GET extra_macro_args 0 targetname)
  endif()
  string(REPLACE "/" "_" targetname ${targetname})
  set(targetdir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

  # Add dummy custom command to get make clean behavior right.
  add_custom_command(OUTPUT ${targetdir}/${output} ${targetdir}/${output}.lisp
    COMMAND echo -n)
  add_custom_target(${targetname} ALL
    DEPENDS ${targetdir}/${output} ${targetdir}/${output}.lisp
    COMMAND ${ROSLISP_MAKE_NODE_BIN} ${PROJECT_NAME} ${system_name} ${entry_point} ${targetdir}/${output})

  # Make this executable depend on all previously declared executables, to serialize them.
  if(ROSLISP_EXECUTABLES)
    add_dependencies(${targetname} ${ROSLISP_EXECUTABLES})
  endif()
  # Add this executable to the list of executables on which all future
  # executables will depend.
  list(APPEND ROSLISP_EXECUTABLES ${targetname})
  set(ROSLISP_EXECUTABLES "${ROSLISP_EXECUTABLES}" PARENT_SCOPE)

  # mark the generated executables for installation
  install(PROGRAMS ${targetdir}/${output}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(FILES ${targetdir}/${output}.lisp
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction(add_compiled_lisp_executable)


# example usage:
# add_lisp_executable(my_script my-system my-package:my-func)
function(add_lisp_executable output system_name entry_point)
  if(NOT ${ARGC} EQUAL 3)
    message(SEND_ERROR "[roslisp] add_lisp_executable can only have 3 arguments")
  endif()

  if(output MATCHES "/")
    message(WARNING "First argument of ADD_LISP_EXECUTABLE (${output}) cannot contain slashes! Ignoring.")
    string(REPLACE "/" "_" output ${output})
  endif()

  set(targetdir ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
  set(targetname _roslisp_${output})

  # create directory if it does not exist
  file(MAKE_DIRECTORY ${targetdir})

  # generate script
  add_custom_command(OUTPUT ${targetdir}/${output}
    COMMAND ${ROSLISP_EXE_SCRIPT} ${PROJECT_NAME} ${system_name} ${entry_point}
    ${targetdir}/${output}
    DEPENDS ${ROSLISP_EXE_SCRIPT}
    COMMENT "Generating lisp exe script ${targetdir}/${output}"
    VERBATIM)
  add_custom_target(${targetname} ALL DEPENDS ${targetdir}/${output})

  # mark the generated executables for installation
  install(PROGRAMS ${targetdir}/${output}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endfunction(add_lisp_executable)
