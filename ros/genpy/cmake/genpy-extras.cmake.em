@[if DEVELSPACE]@
# location of scripts in develspace
set(GENPY_BIN_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/scripts")
@[else]@
# location of scripts in installspace
set(GENPY_BIN_DIR "${genpy_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)")
@[end if]@

set(GENMSG_PY_BIN ${GENPY_BIN_DIR}/genmsg_py.py)
set(GENSRV_PY_BIN ${GENPY_BIN_DIR}/gensrv_py.py)
find_package(Protobuf REQUIRED)
set(GENPROTO_PY_BIN ${PROTOBUF_PROTOC_EXECUTABLE})

# Generate .msg->.h for py
# The generated .h files should be added ALL_GEN_OUTPUT_FILES_py
macro(_generate_msg_py ARG_PKG ARG_MSG ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)

  #Append msg to output dir
  set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}/msg")
  file(MAKE_DIRECTORY ${GEN_OUTPUT_DIR})
  #Create input and output filenames
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME _${MSG_SHORT_NAME}.py)
  set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENMSG_PY_BIN} ${ARG_MSG} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_PY_BIN} ${ARG_MSG}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${GEN_OUTPUT_DIR}
    COMMENT "Generating Python from MSG ${ARG_PKG}/${MSG_SHORT_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_py ${GEN_OUTPUT_FILE})

endmacro()

#todo, these macros are practically equal. Check for input file extension instead
macro(_generate_srv_py ARG_PKG ARG_SRV ARG_IFLAGS ARG_MSG_DEPS ARG_GEN_OUTPUT_DIR)

  #Append msg to output dir
  set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}/srv")
  file(MAKE_DIRECTORY ${GEN_OUTPUT_DIR})

  #Create input and output filenames
  get_filename_component(SRV_SHORT_NAME ${ARG_SRV} NAME_WE)

  set(SRV_GENERATED_NAME _${SRV_SHORT_NAME}.py)
  set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/${SRV_GENERATED_NAME})

  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    DEPENDS ${GENSRV_PY_BIN} ${ARG_SRV} ${ARG_MSG_DEPS}
    COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENSRV_PY_BIN} ${ARG_SRV}
    ${ARG_IFLAGS}
    -p ${ARG_PKG}
    -o ${GEN_OUTPUT_DIR}
    COMMENT "Generating Python code from SRV ${ARG_PKG}/${SRV_SHORT_NAME}"
    )

  list(APPEND ALL_GEN_OUTPUT_FILES_py ${GEN_OUTPUT_FILE})

endmacro()

macro(_generate_proto_py ARG_PKG ARG_MSG ARG_IFLAGS ARG_GEN_OUTPUT_DIR)
  #Append msg to output dir
  set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}/msg")
  file(MAKE_DIRECTORY ${GEN_OUTPUT_DIR})
  #Create input and output filenames
  get_filename_component(MSG_SHORT_NAME ${ARG_MSG} NAME_WE)

  set(MSG_GENERATED_NAME ${MSG_SHORT_NAME}_pb2.py)
  set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/${MSG_GENERATED_NAME})

  message("${GENPROTO_PY_BIN} ${ARG_IFLAGS} --python_out=${GEN_OUTPUT_DIR} ${ARG_MSG}")
  add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
    COMMAND ${GENPROTO_PY_BIN} ${ARG_IFLAGS}
    --python_out=${GEN_OUTPUT_DIR}
    ${ARG_MSG}
    COMMENT "Generating Python from proto ${ARG_PKG}/${MSG_SHORT_NAME}.proto"
  )

  set(PB_MSGS_INIT_FILE "${ARG_GEN_OUTPUT_DIR}/msg/__init__.py")
  set(IMPORT_INFO "from .${MSG_SHORT_NAME}_pb2 import *\n")
  if(NOT EXISTS ${PB_MSGS_INIT_FILE})
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../pb_msgs/msg/__init__.py PB_MSGS_INIT_FILE_CONTENT)
    file(WRITE ${PB_MSGS_INIT_FILE} ${PB_MSGS_INIT_FILE_CONTENT})
    file(APPEND ${PB_MSGS_INIT_FILE} ${IMPORT_INFO})
  else()
    file(READ ${PB_MSGS_INIT_FILE} PB_MSGS_INIT_FILE_CONTENT)
    # message(STATUS "PB_MSGS_INIT_FILE_CONTENT:")
    # message("${PB_MSGS_INIT_FILE_CONTENT}")
    string(FIND ${PB_MSGS_INIT_FILE_CONTENT} ${IMPORT_INFO} IMPORT_INFO_POSITION)
    if(${IMPORT_INFO_POSITION} EQUAL -1)
      message(STATUS "Append python import info: ${IMPORT_INFO}")
      file(APPEND ${PB_MSGS_INIT_FILE} ${IMPORT_INFO})
    endif()
  endif()
  
  list(APPEND ALL_GEN_OUTPUT_FILES_py ${GEN_OUTPUT_FILE})

endmacro()

macro(_generate_module_py ARG_PKG ARG_GEN_OUTPUT_DIR ARG_GENERATED_FILES)

  # generate empty __init__ to make parent folder of msg/srv a python module
  if(NOT EXISTS ${ARG_GEN_OUTPUT_DIR}/__init__.py)
    file(WRITE ${ARG_GEN_OUTPUT_DIR}/__init__.py "")
  endif()

  #Append msg to output dir
  foreach(type "msg" "srv")
    set(GEN_OUTPUT_DIR "${ARG_GEN_OUTPUT_DIR}/${type}")
    set(GEN_OUTPUT_FILE ${GEN_OUTPUT_DIR}/__init__.py)

    if(IS_DIRECTORY ${GEN_OUTPUT_DIR})
      add_custom_command(OUTPUT ${GEN_OUTPUT_FILE}
        DEPENDS ${GENMSG_PY_BIN} ${ARG_GENERATED_FILES}
        COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_PY_BIN}
        -o ${GEN_OUTPUT_DIR}
        --initpy
        COMMENT "Generating Python ${type} __init__.py for ${ARG_PKG}")
      list(APPEND ALL_GEN_OUTPUT_FILES_py ${GEN_OUTPUT_FILE})
    endif()

  endforeach()
endmacro()

if(NOT EXISTS @(PROJECT_NAME)_SOURCE_DIR)
  set(genpy_INSTALL_DIR ${PYTHON_INSTALL_DIR})
endif()
