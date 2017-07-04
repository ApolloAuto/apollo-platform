import os
import shutil
from test.utils import AbstractCatkinWorkspaceTest, TEMP_DIR, rosinstall, \
    create_catkin_workspace


class AbstractUnstableTest(AbstractCatkinWorkspaceTest):

    """
    Parent class for any Test case that download latest ros core
    stacks from github to build custom stacks against that
    """

    def __init__(self, testCaseName, name):
        # for ROS core integration tests, we reuse the same sources
        # (to save download time), keep in test folder
        super(AbstractUnstableTest, self).__init__(
            testCaseName, os.path.join(TEMP_DIR, name))

    def setupWorkspaceContents(self):
        rosinstall(self.workspacedir,
                   os.path.join(os.path.dirname(__file__),
                                'test.rosinstall'))
        create_catkin_workspace(self.workspacedir)

    def tearDown(self):
        # override parent tearDown which would delete what we
        # rosinstalled
        pass

    def delete_build(self):
        """
        cleans the build folder, run manually in subtests when
        appropriate. We don't to this in setup because it takes so
        long to build all of ros core'
        """
        if os.path.isdir(self.builddir):
            shutil.rmtree(self.builddir)
