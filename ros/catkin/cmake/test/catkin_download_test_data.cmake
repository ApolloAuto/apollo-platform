_generate_function_if_testing_is_disabled("catkin_download_test_data")

#
# Download a file containing test data from a URL.
#
# It is commonly used to download larger data files for unit tests
# which should not be stored in the repository.
#
# .. note:: It is not recommended to rely on downloaded data during
#   a configure / make cycle since this prevents building the package
#   when no network connectivity is available.
#
# .. note:: The target will be registered as a dependency
#   of the "tests" and "download_extra_data" targets, but not of "all" target.
#
# .. note:: If the tests should be run on the ROS buildfarm the URL
#   must be publically and reliably accessible.
#
# :param target: the target name
# :type target: string
# :param url: the url to download
# :type url: string

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
# Additionally, option REQUIRED can be specified.
#
# @public
function(catkin_download_test_data target url)
  _warn_if_skip_testing("catkin_download_test_data")

  catkin_download("${target}" "${url}" ${ARGN} EXCLUDE_FROM_ALL)

  if(TARGET tests)
    add_dependencies(tests ${target})
  endif()
endfunction()
