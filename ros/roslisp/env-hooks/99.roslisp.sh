# generated from roslisp/env-hooks/99.roslisp.sh.in

# python function to generate ROSLISP package directories containing all devel spaces based on all workspaces
PYTHON_CODE_BUILD_ROSLISP_PACKAGE_DIRECTORIES=$(cat <<EOF
from __future__ import print_function
import os
env_name = 'CMAKE_PREFIX_PATH'
paths = [path for path in os.environ[env_name].split(os.pathsep)] if env_name in os.environ and os.environ[env_name] != '' else []
workspaces = [path for path in paths if os.path.exists(os.path.join(path, '.catkin'))]
paths = []
for workspace in workspaces:
    filename = os.path.join(workspace, '.catkin')
    data = ''
    with open(filename) as f:
        data = f.read()
    if data:
        paths.append(os.path.join(workspace, 'share', 'common-lisp'))
print(os.pathsep.join(paths))
EOF
)
export ROSLISP_PACKAGE_DIRECTORIES="`python -c \"$PYTHON_CODE_BUILD_ROSLISP_PACKAGE_DIRECTORIES\"`"
