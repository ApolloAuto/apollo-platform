cmake_minimum_required(VERSION 2.8.3)
project(roslisp)

find_package(catkin REQUIRED)
catkin_package(CFG_EXTRAS roslisp-extras.cmake)

add_subdirectory(env-hooks)

foreach(dir
    load-manifest
    rosbuild
    roslisp-msg-protocol
    src
    s-xml
    s-xml-rpc
    utils)
  install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
      PATTERN ".svn" EXCLUDE
      PATTERN "genmsg_lisp.py" EXCLUDE)
endforeach()

# install legacy infrastructure needed by rosbuild
install(FILES manifest.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    PATTERN ".svn" EXCLUDE
    PATTERN "genmsg_lisp.py" EXCLUDE)
install(FILES rosbuild/roslisp.cmake
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild)
install(PROGRAMS
    rosbuild/scripts/genmsg_lisp.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rosbuild/scripts)
