from __future__ import division, absolute_import, print_function

import sys
from numpy.core import arange
from numpy.testing import *
import numpy.lib.utils as utils
from numpy.lib import deprecate

if sys.version_info[0] >= 3:
    from io import StringIO
else:
    from StringIO import StringIO

def test_lookfor():
    out = StringIO()
    utils.lookfor('eigenvalue', module='numpy', output=out,
                  import_modules=False)
    out = out.getvalue()
    assert_('numpy.linalg.eig' in out)


@deprecate
def old_func(self, x):
    return x

@deprecate(message="Rather use new_func2")
def old_func2(self, x):
    return x

def old_func3(self, x):
    return x
new_func3 = deprecate(old_func3, old_name="old_func3", new_name="new_func3")

def test_deprecate_decorator():
    assert_('deprecated' in old_func.__doc__)

def test_deprecate_decorator_message():
    assert_('Rather use new_func2' in old_func2.__doc__)

def test_deprecate_fn():
    assert_('old_func3' in new_func3.__doc__)
    assert_('new_func3' in new_func3.__doc__)


def test_safe_eval_nameconstant():
    # Test if safe_eval supports Python 3.4 _ast.NameConstant
    utils.safe_eval('None')


def test_byte_bounds():
    a = arange(12).reshape(3, 4)
    low, high = utils.byte_bounds(a)
    assert_equal(high - low, a.size * a.itemsize)


if __name__ == "__main__":
    run_module_suite()
