file(COPY calib5.ini calib8.ini DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/test)

catkin_add_nosetests(parser.py)
