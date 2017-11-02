"""Test functions for matrix module

"""
from __future__ import division, absolute_import, print_function

from numpy.testing import *

from numpy import ( arange, rot90, add, fliplr, flipud, zeros, ones, eye,
                    array, diag, histogram2d, tri, mask_indices, triu_indices,
                    triu_indices_from, tril_indices, tril_indices_from )

import numpy as np
from numpy.compat import asbytes, asbytes_nested

def get_mat(n):
    data = arange(n)
    data = add.outer(data, data)
    return data

class TestEye(TestCase):
    def test_basic(self):
        assert_equal(eye(4), array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]))
        assert_equal(eye(4, dtype='f'), array([[1, 0, 0, 0],
                                                [0, 1, 0, 0],
                                                [0, 0, 1, 0],
                                                [0, 0, 0, 1]], 'f'))
        assert_equal(eye(3) == 1, eye(3, dtype=bool))

    def test_diag(self):
        assert_equal(eye(4, k=1), array([[0, 1, 0, 0],
                                       [0, 0, 1, 0],
                                       [0, 0, 0, 1],
                                       [0, 0, 0, 0]]))
        assert_equal(eye(4, k=-1), array([[0, 0, 0, 0],
                                        [1, 0, 0, 0],
                                        [0, 1, 0, 0],
                                        [0, 0, 1, 0]]))
    def test_2d(self):
        assert_equal(eye(4, 3), array([[1, 0, 0],
                                     [0, 1, 0],
                                     [0, 0, 1],
                                     [0, 0, 0]]))
        assert_equal(eye(3, 4), array([[1, 0, 0, 0],
                                     [0, 1, 0, 0],
                                     [0, 0, 1, 0]]))
    def test_diag2d(self):
        assert_equal(eye(3, 4, k=2), array([[0, 0, 1, 0],
                                         [0, 0, 0, 1],
                                         [0, 0, 0, 0]]))
        assert_equal(eye(4, 3, k=-2), array([[0, 0, 0],
                                          [0, 0, 0],
                                          [1, 0, 0],
                                          [0, 1, 0]]))

    def test_eye_bounds(self):
        assert_equal(eye(2, 2,  1), [[0, 1], [0, 0]])
        assert_equal(eye(2, 2, -1), [[0, 0], [1, 0]])
        assert_equal(eye(2, 2,  2), [[0, 0], [0, 0]])
        assert_equal(eye(2, 2, -2), [[0, 0], [0, 0]])
        assert_equal(eye(3, 2,  2), [[0, 0], [0, 0], [0, 0]])
        assert_equal(eye(3, 2,  1), [[0, 1], [0, 0], [0, 0]])
        assert_equal(eye(3, 2, -1), [[0, 0], [1, 0], [0, 1]])
        assert_equal(eye(3, 2, -2), [[0, 0], [0, 0], [1, 0]])
        assert_equal(eye(3, 2, -3), [[0, 0], [0, 0], [0, 0]])

    def test_strings(self):
        assert_equal(eye(2, 2, dtype='S3'),
                     asbytes_nested([['1', ''], ['', '1']]))

    def test_bool(self):
        assert_equal(eye(2, 2, dtype=bool), [[True, False], [False, True]])

class TestDiag(TestCase):
    def test_vector(self):
        vals = (100 * arange(5)).astype('l')
        b = zeros((5, 5))
        for k in range(5):
            b[k, k] = vals[k]
        assert_equal(diag(vals), b)
        b = zeros((7, 7))
        c = b.copy()
        for k in range(5):
            b[k, k + 2] = vals[k]
            c[k + 2, k] = vals[k]
        assert_equal(diag(vals, k=2), b)
        assert_equal(diag(vals, k=-2), c)

    def test_matrix(self, vals=None):
        if vals is None:
            vals = (100 * get_mat(5) + 1).astype('l')
        b = zeros((5,))
        for k in range(5):
            b[k] = vals[k, k]
        assert_equal(diag(vals), b)
        b = b * 0
        for k in range(3):
            b[k] = vals[k, k + 2]
        assert_equal(diag(vals, 2), b[:3])
        for k in range(3):
            b[k] = vals[k + 2, k]
        assert_equal(diag(vals, -2), b[:3])

    def test_fortran_order(self):
        vals = array((100 * get_mat(5) + 1), order='F', dtype='l')
        self.test_matrix(vals)

    def test_diag_bounds(self):
        A = [[1, 2], [3, 4], [5, 6]]
        assert_equal(diag(A, k=2), [])
        assert_equal(diag(A, k=1), [2])
        assert_equal(diag(A, k=0), [1, 4])
        assert_equal(diag(A, k=-1), [3, 6])
        assert_equal(diag(A, k=-2), [5])
        assert_equal(diag(A, k=-3), [])

    def test_failure(self):
        self.assertRaises(ValueError, diag, [[[1]]])

class TestFliplr(TestCase):
    def test_basic(self):
        self.assertRaises(ValueError, fliplr, ones(4))
        a = get_mat(4)
        b = a[:, ::-1]
        assert_equal(fliplr(a), b)
        a = [[0, 1, 2],
             [3, 4, 5]]
        b = [[2, 1, 0],
             [5, 4, 3]]
        assert_equal(fliplr(a), b)

class TestFlipud(TestCase):
    def test_basic(self):
        a = get_mat(4)
        b = a[::-1,:]
        assert_equal(flipud(a), b)
        a = [[0, 1, 2],
             [3, 4, 5]]
        b = [[3, 4, 5],
             [0, 1, 2]]
        assert_equal(flipud(a), b)

class TestRot90(TestCase):
    def test_basic(self):
        self.assertRaises(ValueError, rot90, ones(4))

        a = [[0, 1, 2],
             [3, 4, 5]]
        b1 = [[2, 5],
              [1, 4],
              [0, 3]]
        b2 = [[5, 4, 3],
              [2, 1, 0]]
        b3 = [[3, 0],
              [4, 1],
              [5, 2]]
        b4 = [[0, 1, 2],
              [3, 4, 5]]

        for k in range(-3, 13, 4):
            assert_equal(rot90(a, k=k), b1)
        for k in range(-2, 13, 4):
            assert_equal(rot90(a, k=k), b2)
        for k in range(-1, 13, 4):
            assert_equal(rot90(a, k=k), b3)
        for k in range(0, 13, 4):
            assert_equal(rot90(a, k=k), b4)

    def test_axes(self):
        a = ones((50, 40, 3))
        assert_equal(rot90(a).shape, (40, 50, 3))

class TestHistogram2d(TestCase):
    def test_simple(self):
        x = array([ 0.41702200,  0.72032449,  0.00011437481, 0.302332573,  0.146755891])
        y = array([ 0.09233859,  0.18626021,  0.34556073,  0.39676747,  0.53881673])
        xedges = np.linspace(0, 1, 10)
        yedges = np.linspace(0, 1, 10)
        H = histogram2d(x, y, (xedges, yedges))[0]
        answer = array([[0, 0, 0, 1, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [1, 0, 1, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0]])
        assert_array_equal(H.T, answer)
        H = histogram2d(x, y, xedges)[0]
        assert_array_equal(H.T, answer)
        H, xedges, yedges = histogram2d(list(range(10)), list(range(10)))
        assert_array_equal(H, eye(10, 10))
        assert_array_equal(xedges, np.linspace(0, 9, 11))
        assert_array_equal(yedges, np.linspace(0, 9, 11))

    def test_asym(self):
        x = array([1, 1, 2, 3, 4, 4, 4, 5])
        y = array([1, 3, 2, 0, 1, 2, 3, 4])
        H, xed, yed = histogram2d(x, y, (6, 5), range = [[0, 6], [0, 5]], normed=True)
        answer = array([[0., 0, 0, 0, 0],
                        [0, 1, 0, 1, 0],
                        [0, 0, 1, 0, 0],
                        [1, 0, 0, 0, 0],
                        [0, 1, 1, 1, 0],
                        [0, 0, 0, 0, 1]])
        assert_array_almost_equal(H, answer/8., 3)
        assert_array_equal(xed, np.linspace(0, 6, 7))
        assert_array_equal(yed, np.linspace(0, 5, 6))
    def test_norm(self):
        x = array([1, 2, 3, 1, 2, 3, 1, 2, 3])
        y = array([1, 1, 1, 2, 2, 2, 3, 3, 3])
        H, xed, yed = histogram2d(x, y, [[1, 2, 3, 5], [1, 2, 3, 5]], normed=True)
        answer=array([[1, 1, .5],
                     [1, 1, .5],
                     [.5, .5, .25]])/9.
        assert_array_almost_equal(H, answer, 3)

    def test_all_outliers(self):
        r = rand(100)+1.
        H, xed, yed = histogram2d(r, r, (4, 5), range=([0, 1], [0, 1]))
        assert_array_equal(H, 0)

    def test_empty(self):
        a, edge1, edge2 = histogram2d([], [], bins=([0, 1], [0, 1]))
        assert_array_max_ulp(a, array([[ 0.]]))

        a, edge1, edge2 = histogram2d([], [], bins=4)
        assert_array_max_ulp(a, np.zeros((4, 4)))


class TestTri(TestCase):
    def test_dtype(self):
        out = array([[1, 0, 0],
                     [1, 1, 0],
                     [1, 1, 1]])
        assert_array_equal(tri(3), out)
        assert_array_equal(tri(3, dtype=bool), out.astype(bool))


def test_tril_triu():
    for dtype in np.typecodes['AllFloat'] + np.typecodes['AllInteger']:
        a = np.ones((2, 2), dtype=dtype)
        b = np.tril(a)
        c = np.triu(a)
        assert_array_equal(b, [[1, 0], [1, 1]])
        assert_array_equal(c, b.T)
        # should return the same dtype as the original array
        assert_equal(b.dtype, a.dtype)
        assert_equal(c.dtype, a.dtype)


def test_mask_indices():
    # simple test without offset
    iu = mask_indices(3, np.triu)
    a = np.arange(9).reshape(3, 3)
    yield (assert_array_equal, a[iu], array([0, 1, 2, 4, 5, 8]))
    # Now with an offset
    iu1 = mask_indices(3, np.triu, 1)
    yield (assert_array_equal, a[iu1], array([1, 2, 5]))


def test_tril_indices():
    # indices without and with offset
    il1 = tril_indices(4)
    il2 = tril_indices(4, 2)

    a = np.array([[1, 2, 3, 4],
                  [5, 6, 7, 8],
                  [9, 10, 11, 12],
                  [13, 14, 15, 16]])

    # indexing:
    yield (assert_array_equal, a[il1],
           array([ 1,  5,  6,  9, 10, 11, 13, 14, 15, 16]) )

    # And for assigning values:
    a[il1] = -1
    yield (assert_array_equal, a,
    array([[-1,  2,  3,  4],
           [-1, -1,  7,  8],
           [-1, -1, -1, 12],
           [-1, -1, -1, -1]]) )

    # These cover almost the whole array (two diagonals right of the main one):
    a[il2] = -10
    yield (assert_array_equal, a,
    array([[-10, -10, -10,   4],
           [-10, -10, -10, -10],
           [-10, -10, -10, -10],
           [-10, -10, -10, -10]]) )


class TestTriuIndices(object):
    def test_triu_indices(self):
        iu1 = triu_indices(4)
        iu2 = triu_indices(4, 2)

        a = np.array([[1, 2, 3, 4],
                      [5, 6, 7, 8],
                      [9, 10, 11, 12],
                      [13, 14, 15, 16]])

        # Both for indexing:
        yield (assert_array_equal, a[iu1],
               array([1, 2,  3,  4,  6, 7, 8, 11, 12, 16]))

        # And for assigning values:
        a[iu1] = -1
        yield (assert_array_equal, a,
               array([[-1, -1, -1, -1],
                      [ 5, -1, -1, -1],
                      [ 9, 10, -1, -1],
                      [13, 14, 15, -1]])  )

        # These cover almost the whole array (two diagonals right of the main one):
        a[iu2] = -10
        yield ( assert_array_equal, a,
                array([[ -1,  -1, -10, -10],
                       [  5,  -1,  -1, -10],
                       [  9,  10,  -1,  -1],
                       [ 13,  14,  15,  -1]]) )


class TestTrilIndicesFrom(object):
    def test_exceptions(self):
        assert_raises(ValueError, tril_indices_from, np.ones((2,)))
        assert_raises(ValueError, tril_indices_from, np.ones((2, 2, 2)))
        assert_raises(ValueError, tril_indices_from, np.ones((2, 3)))


class TestTriuIndicesFrom(object):
    def test_exceptions(self):
        assert_raises(ValueError, triu_indices_from, np.ones((2,)))
        assert_raises(ValueError, triu_indices_from, np.ones((2, 2, 2)))
        assert_raises(ValueError, triu_indices_from, np.ones((2, 3)))


if __name__ == "__main__":
    run_module_suite()
