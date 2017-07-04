#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fastrtps" for configuration "Release"
set_property(TARGET fastrtps APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(fastrtps PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfastrtps.so"
  IMPORTED_SONAME_RELEASE "libfastrtps.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS fastrtps )
list(APPEND _IMPORT_CHECK_FILES_FOR_fastrtps "${_IMPORT_PREFIX}/lib/libfastrtps.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
