macro(generate_dynamic_reconfigure_options)
  if(${PROJECT_NAME}_CATKIN_PACKAGE)
    message(FATAL_ERROR "generate_dynamic_reconfigure_options() must be called before catkin_package() in project '${PROJECT_NAME}'")
  endif()

  # ensure that package destination variables are defined
  catkin_destinations()

  set(_autogen "")
  foreach(_cfg ${ARGN})
    # Construct the path to the .cfg file
    set(_input ${_cfg})
    if(NOT IS_ABSOLUTE ${_input})
      set(_input ${PROJECT_SOURCE_DIR}/${_input})
    endif()

    # The .cfg file is its own generator.
    set(gencfg_build_files
      ${dynamic_reconfigure_BASE_DIR}/templates/ConfigType.py.template
      ${dynamic_reconfigure_BASE_DIR}/templates/ConfigType.h.template
    )

    get_filename_component(_cfgonly ${_cfg} NAME_WE)
    set(_output_cpp ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfgonly}Config.h)
    set(_output_py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/${_cfgonly}Config.py)
    set(_output_dox ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfgonly}Config.dox)
    set(_output_wikidoc ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfgonly}Config.wikidoc)
    set(_output_usage ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/docs/${_cfgonly}Config-usage.dox)

    # we need to explicitly add the devel space to the PYTHONPATH
    # since it might contain dynamic_reconfigure or Python code of the current package
    set("_CUSTOM_PYTHONPATH_ENV")
    if(EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}")
      configure_file(
        "${dynamic_reconfigure_BASE_DIR}/cmake/setup_custom_pythonpath.sh.in"
        "setup_custom_pythonpath.sh"
        @ONLY
      )
      set("_CUSTOM_PYTHONPATH_ENV" "${CMAKE_CURRENT_BINARY_DIR}/setup_custom_pythonpath.sh")
    endif()

    assert(CATKIN_ENV)
    set(_cmd
      ${CATKIN_ENV}
      ${_CUSTOM_PYTHONPATH_ENV}
      ${_input}
      ${dynamic_reconfigure_BASE_DIR}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )

    #file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/__init__.py)

    add_custom_command(OUTPUT
      ${_output_cpp} ${_output_dox} ${_output_usage} ${_output_py} ${_output_wikidoc}
      COMMAND ${_cmd}
      DEPENDS ${_input} ${gencfg_build_files}
      COMMENT "Generating dynamic reconfigure files from ${_cfg}: ${_output_cpp} ${_output_py}"
    )

    list(APPEND ${PROJECT_NAME}_generated
      ${_output_cpp} ${_output_py}
    )

    install(FILES ${_output_cpp}
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  endforeach(_cfg)

  # gencfg target for hard dependency on dynamic_reconfigure generation
  add_custom_target(${PROJECT_NAME}_gencfg ALL DEPENDS ${${PROJECT_NAME}_generated})

  # register target for catkin_package(EXPORTED_TARGETS)
  list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_gencfg)

  dynreconf_called()
endmacro()

macro(dynreconf_called)
  if(NOT dynamic_reconfigure_CALLED)
    set(dynamic_reconfigure_CALLED TRUE)

    # mark that generate_dynamic_reconfigure_options() was called in order to detect wrong order of calling with catkin_python_setup()
    set(${PROJECT_NAME}_GENERATE_DYNAMIC_RECONFIGURE TRUE)
    # check if catkin_python_setup() installs an __init__.py file for a package with the current project name
    # in order to skip the installation of a generated __init__.py file
    set(package_has_static_sources ${${PROJECT_NAME}_CATKIN_PYTHON_SETUP_HAS_PACKAGE_INIT})

    # generate empty __init__ to make parent folder of msg/srv a python module
    if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py)
      file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py "")
    endif()
    if(NOT package_has_static_sources)
      # install package __init__.py
      install(
        FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
      )
    endif()

    # make sure we can find generated messages and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # ensure that the folder exists
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

    # generate cfg module __init__.py
    if(NOT EXISTS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/__init__.py)
      file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg/__init__.py "")
    endif()

    # compile python code before installing
    find_package(PythonInterp REQUIRED)
    install(CODE "execute_process(COMMAND \"${PYTHON_EXECUTABLE}\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg\")")
    install(
      DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/cfg
      DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
    )
  endif()
endmacro()
