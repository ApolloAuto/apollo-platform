cmake_minimum_required(VERSION 2.8.3)
project(genlisp)
find_package(catkin REQUIRED COMPONENTS genmsg)

catkin_package(
  CATKIN_DEPENDS genmsg
  CFG_EXTRAS genlisp-extras.cmake
)

add_subdirectory(scripts)

file(WRITE ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/genlisp "LISP")
install(FILES ${CATKIN_DEVEL_PREFIX}/${GENMSG_LANGS_DESTINATION}/genlisp
  DESTINATION ${GENMSG_LANGS_DESTINATION})

# drop marker file to prevent rospack from spending time on crawling this folder
file(WRITE ${CATKIN_DEVEL_PREFIX}/share/common-lisp/rospack_nosubdirs "")
install(FILES ${CATKIN_DEVEL_PREFIX}/share/common-lisp/rospack_nosubdirs
  DESTINATION share/common-lisp)

catkin_python_setup()
