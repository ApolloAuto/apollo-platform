#!/usr/bin/make -f

# THESE ARE CUSTOM RULES

export DH_VERBOSE=1
export DH_OPTIONS=-v

%:
	dh  $@@

override_dh_auto_configure:
	dh_auto_configure -Scmake -- \
		-DCMAKE_INSTALL_PREFIX="@(CMAKE_INSTALL_PREFIX)" \
		-DCMAKE_PREFIX_PATH="@(CMAKE_PREFIX_PATH)" \
		-DCATKIN_PACKAGE_PREFIX="@(CATKIN_PACKAGE_PREFIX)" \
		-DCATKIN=YES
