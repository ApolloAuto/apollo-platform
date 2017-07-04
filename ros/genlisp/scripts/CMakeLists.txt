install(
  FILES msg.lisp.template srv.lisp.template
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(
  PROGRAMS gen_lisp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
