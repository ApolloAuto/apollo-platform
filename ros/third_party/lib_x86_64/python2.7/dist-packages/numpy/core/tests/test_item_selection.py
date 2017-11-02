from __future__ import division, absolute_import, print_function

import numpy as np
from numpy.testing import *
import sys, warnings


class TestTake(TestCase):
    def test_simple(self):
        a = [[1, 2], [3, 4]]
        a_str = [[b'1', b'2'], [b'3', b'4']]
        modes = ['raise', 'wrap', 'clip']
        indices = [-1, 4]
        index_arrays = [np.empty(0, dtype=np.intp),
                        np.empty(tuple(), dtype=np.intp),
                        np.empty((1, 1), dtype=np.intp)]
        real_indices = {}
        real_indices['raise'] = {-1:1, 4:IndexError}
        real_indices['wrap'] = {-1:1, 4:0}
        real_indices['clip'] = {-1:0, 4:1}
        # Currently all types but object, use the same function generation.
        # So it should not be necessary to test all. However test also a non
        # refcounted struct on top of object.
        types = np.int, np.object, np.dtype([('', 'i', 2)])
        for t in types:
            # ta works, even if the array may be odd if buffer interface is used
            ta = np.array(a if np.issubdtype(t, np.number) else a_str, dtype=t)
            tresult = list(ta.T.copy())
            for index_array in index_arrays:
                if index_array.size != 0:
                    tresult[0].shape = (2,) + index_array.shape
                    tresult[1].shape = (2,) + index_array.shape
                for mode in modes:
                    for index in indices:
                        real_index = real_indices[mode][index]
                        if real_index is IndexError and index_array.size != 0:
                            index_array.put(0, index)
                            assert_raises(IndexError, ta.take, index_array,
                                          mode=mode, axis=1)
                        elif index_array.size != 0:
                            index_array.put(0, index)
                            res = ta.take(index_array, mode=mode, axis=1)
                            assert_array_equal(res, tresult[real_index])
                        else:
                            res = ta.take(index_array, mode=mode, axis=1)
                            assert_(res.shape == (2,) + index_array.shape)


    def test_refcounting(self):
        objects = [object() for i in range(10)]
        for mode in ('raise', 'clip', 'wrap'):
            a = np.array(objects)
            b = np.array([2, 2, 4, 5, 3, 5])
            a.take(b, out=a[:6])
            del a
            assert_(all(sys.getrefcount(o) == 3 for o in objects))
            # not contiguous, example:
            a = np.array(objects * 2)[::2]
            a.take(b, out=a[:6])
            del a
            assert_(all(sys.getrefcount(o) == 3 for o in objects))


if __name__ == "__main__":
    run_module_suite()
