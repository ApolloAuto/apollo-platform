#
# Download a file containing data from a URL.
#
# It is commonly used to download larger data files which should not be
# stored in the repository.
#
# .. note:: It is not recommended to rely on downloaded data during
#   a configure / make cycle since this prevents building the package
#   when no network connectivity is available.
#
# .. note:: The target will be registered as a dependency
#   of the "download_extra_data" target.
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string
#
# :param DESTINATION: the directory where the file is downloaded to
#   (default: ${PROJECT_BINARY_DIR})
# :type DESTINATION: string
# :param FILENAME: the filename of the downloaded file
#   (default: the basename of the url)
# :type FILENAME: string
# :param MD5: the expected md5 hash to compare against
#   (default: empty, skipping the check)
# :type MD5: string
#
# Additionally, options EXCLUDE_FROM_ALL and REQUIRED can be specified.
#
# @public
function(catkin_download target url)
  cmake_parse_arguments(ARG
    "EXCLUDE_FROM_ALL;REQUIRED" "DESTINATION;FILENAME;MD5" "" ${ARGN})
  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "catkin_download() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_DESTINATION)
    set(ARG_DESTINATION ${PROJECT_BINARY_DIR})
  endif()

  if(NOT ARG_FILENAME)
    get_filename_component(ARG_FILENAME ${url} NAME)
  endif()

  set(required "")
  if(NOT ARG_REQUIRED)
    set(required "--ignore-error")
  endif()

  set(output "${ARG_DESTINATION}/${ARG_FILENAME}")

  # With this, the command is always called, even when the output is up to date.
  # this is because we want to check the md5 sum if it's given, and redownload
  # the target if the md5 sum does not match.
  add_custom_target(${target}
    COMMAND ${PYTHON} ${catkin_EXTRAS_DIR}/test/download_checkmd5.py ${url} ${output} ${ARG_MD5} ${required}
    VERBATIM)

  if(ARG_EXCLUDE_FROM_ALL)
    set_target_properties(${target} PROPERTIES EXCLUDE_FROM_ALL TRUE)
  endif()

  if(TARGET download_extra_data)
    add_dependencies(download_extra_data ${target})
  endif()
endfunction()

if(NOT TARGET download_extra_data)
  add_custom_target(download_extra_data)
endif()
