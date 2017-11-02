from __future__ import division, absolute_import, print_function

import sys
import numpy as np
from numpy.testing import *

def assert_dtype_equal(a, b):
    assert_equal(a, b)
    assert_equal(hash(a), hash(b),
                 "two equivalent types do not hash to the same value !")

def assert_dtype_not_equal(a, b):
    assert_(a != b)
    assert_(hash(a) != hash(b),
            "two different types hash to the same value !")

class TestBuiltin(TestCase):
    def test_run(self):
        """Only test hash runs at all."""
        for t in [np.int, np.float, np.complex, np.int32, np.str, np.object,
                np.unicode]:
            dt = np.dtype(t)
            hash(dt)

    def test_dtype(self):
        # Make sure equivalent byte order char hash the same (e.g. < and = on
        # little endian)
        for t in [np.int, np.float]:
            dt = np.dtype(t)
            dt2 = dt.newbyteorder("<")
            dt3 = dt.newbyteorder(">")
            if dt == dt2:
                self.assertTrue(dt.byteorder != dt2.byteorder, "bogus test")
                assert_dtype_equal(dt, dt2)
            else:
                self.assertTrue(dt.byteorder != dt3.byteorder, "bogus test")
                assert_dtype_equal(dt, dt3)

    def test_equivalent_dtype_hashing(self):
        # Make sure equivalent dtypes with different type num hash equal
        uintp = np.dtype(np.uintp)
        if uintp.itemsize == 4:
            left = uintp
            right = np.dtype(np.uint32)
        else:
            left = uintp
            right = np.dtype(np.ulonglong)
        self.assertTrue(left == right)
        self.assertTrue(hash(left) == hash(right))

    def test_invalid_types(self):
        # Make sure invalid type strings raise a warning.
        # For now, display a deprecation warning for invalid
        # type sizes. In the future this should be changed
        # to an exception.

        assert_warns(DeprecationWarning, np.dtype, 'O3')
        assert_warns(DeprecationWarning, np.dtype, 'O5')
        assert_warns(DeprecationWarning, np.dtype, 'O7')
        assert_warns(DeprecationWarning, np.dtype, 'b3')
        assert_warns(DeprecationWarning, np.dtype, 'h4')
        assert_warns(DeprecationWarning, np.dtype, 'I5')
        assert_warns(DeprecationWarning, np.dtype, 'e3')
        assert_warns(DeprecationWarning, np.dtype, 'f5')

        if np.dtype('g').itemsize == 8 or np.dtype('g').itemsize == 16:
            assert_warns(DeprecationWarning, np.dtype, 'g12')
        elif np.dtype('g').itemsize == 12:
            assert_warns(DeprecationWarning, np.dtype, 'g16')

        if np.dtype('l').itemsize == 8:
            assert_warns(DeprecationWarning, np.dtype, 'l4')
            assert_warns(DeprecationWarning, np.dtype, 'L4')
        else:
            assert_warns(DeprecationWarning, np.dtype, 'l8')
            assert_warns(DeprecationWarning, np.dtype, 'L8')

        if np.dtype('q').itemsize == 8:
            assert_warns(DeprecationWarning, np.dtype, 'q4')
            assert_warns(DeprecationWarning, np.dtype, 'Q4')
        else:
            assert_warns(DeprecationWarning, np.dtype, 'q8')
            assert_warns(DeprecationWarning, np.dtype, 'Q8')

    def test_bad_param(self):
        # Can't give a size that's too small
        assert_raises(ValueError, np.dtype,
                        {'names':['f0', 'f1'],
                         'formats':['i4', 'i1'],
                         'offsets':[0, 4],
                         'itemsize':4})
        # If alignment is enabled, the alignment (4) must divide the itemsize
        assert_raises(ValueError, np.dtype,
                        {'names':['f0', 'f1'],
                         'formats':['i4', 'i1'],
                         'offsets':[0, 4],
                         'itemsize':9}, align=True)
        # If alignment is enabled, the individual fields must be aligned
        assert_raises(ValueError, np.dtype,
                        {'names':['f0', 'f1'],
                         'formats':['i1', 'f4'],
                         'offsets':[0, 2]}, align=True)

class TestRecord(TestCase):
    def test_equivalent_record(self):
        """Test whether equivalent record dtypes hash the same."""
        a = np.dtype([('yo', np.int)])
        b = np.dtype([('yo', np.int)])
        assert_dtype_equal(a, b)

    def test_different_names(self):
        # In theory, they may hash the same (collision) ?
        a = np.dtype([('yo', np.int)])
        b = np.dtype([('ye', np.int)])
        assert_dtype_not_equal(a, b)

    def test_different_titles(self):
        # In theory, they may hash the same (collision) ?
        a = np.dtype({'names': ['r', 'b'],
                      'formats': ['u1', 'u1'],
                      'titles': ['Red pixel', 'Blue pixel']})
        b = np.dtype({'names': ['r', 'b'],
                      'formats': ['u1', 'u1'],
                      'titles': ['RRed pixel', 'Blue pixel']})
        assert_dtype_not_equal(a, b)

    def test_not_lists(self):
        """Test if an appropriate exception is raised when passing bad values to
        the dtype constructor.
        """
        self.assertRaises(TypeError, np.dtype,
            dict(names=set(['A', 'B']), formats=['f8', 'i4']))
        self.assertRaises(TypeError, np.dtype,
            dict(names=['A', 'B'], formats=set(['f8', 'i4'])))

    def test_aligned_size(self):
        # Check that structured dtypes get padded to an aligned size
        dt = np.dtype('i4, i1', align=True)
        assert_equal(dt.itemsize, 8)
        dt = np.dtype([('f0', 'i4'), ('f1', 'i1')], align=True)
        assert_equal(dt.itemsize, 8)
        dt = np.dtype({'names':['f0', 'f1'],
                       'formats':['i4', 'u1'],
                       'offsets':[0, 4]}, align=True)
        assert_equal(dt.itemsize, 8)
        dt = np.dtype({'f0': ('i4', 0), 'f1':('u1', 4)}, align=True)
        assert_equal(dt.itemsize, 8)
        # Nesting should preserve that alignment
        dt1 = np.dtype([('f0', 'i4'),
                       ('f1', [('f1', 'i1'), ('f2', 'i4'), ('f3', 'i1')]),
                       ('f2', 'i1')], align=True)
        assert_equal(dt1.itemsize, 20)
        dt2 = np.dtype({'names':['f0', 'f1', 'f2'],
                       'formats':['i4',
                                  [('f1', 'i1'), ('f2', 'i4'), ('f3', 'i1')],
                                  'i1'],
                       'offsets':[0, 4, 16]}, align=True)
        assert_equal(dt2.itemsize, 20)
        dt3 = np.dtype({'f0': ('i4', 0),
                       'f1': ([('f1', 'i1'), ('f2', 'i4'), ('f3', 'i1')], 4),
                       'f2': ('i1', 16)}, align=True)
        assert_equal(dt3.itemsize, 20)
        assert_equal(dt1, dt2)
        assert_equal(dt2, dt3)
        # Nesting should preserve packing
        dt1 = np.dtype([('f0', 'i4'),
                       ('f1', [('f1', 'i1'), ('f2', 'i4'), ('f3', 'i1')]),
                       ('f2', 'i1')], align=False)
        assert_equal(dt1.itemsize, 11)
        dt2 = np.dtype({'names':['f0', 'f1', 'f2'],
                       'formats':['i4',
                                  [('f1', 'i1'), ('f2', 'i4'), ('f3', 'i1')],
                                  'i1'],
                       'offsets':[0, 4, 10]}, align=False)
        assert_equal(dt2.itemsize, 11)
        dt3 = np.dtype({'f0': ('i4', 0),
                       'f1': ([('f1', 'i1'), ('f2', 'i4'), ('f3', 'i1')], 4),
                       'f2': ('i1', 10)}, align=False)
        assert_equal(dt3.itemsize, 11)
        assert_equal(dt1, dt2)
        assert_equal(dt2, dt3)

    def test_union_struct(self):
        # Should be able to create union dtypes
        dt = np.dtype({'names':['f0', 'f1', 'f2'], 'formats':['<u4', '<u2', '<u2'],
                        'offsets':[0, 0, 2]}, align=True)
        assert_equal(dt.itemsize, 4)
        a = np.array([3], dtype='<u4').view(dt)
        a['f1'] = 10
        a['f2'] = 36
        assert_equal(a['f0'], 10 + 36*256*256)
        # Should be able to specify fields out of order
        dt = np.dtype({'names':['f0', 'f1', 'f2'], 'formats':['<u4', '<u2', '<u2'],
                        'offsets':[4, 0, 2]}, align=True)
        assert_equal(dt.itemsize, 8)
        dt2 = np.dtype({'names':['f2', 'f0', 'f1'],
                        'formats':['<u2', '<u4', '<u2'],
                        'offsets':[2, 4, 0]}, align=True)
        vals = [(0, 1, 2), (3, -1, 4)]
        vals2 = [(2, 0, 1), (4, 3, -1)]
        a = np.array(vals, dt)
        b = np.array(vals2, dt2)
        assert_equal(a.astype(dt2), b)
        assert_equal(b.astype(dt), a)
        assert_equal(a.view(dt2), b)
        assert_equal(b.view(dt), a)
        # Should not be able to overlap objects with other types
        assert_raises(TypeError, np.dtype,
                {'names':['f0', 'f1'],
                 'formats':['O', 'i1'],
                 'offsets':[0, 2]})
        assert_raises(TypeError, np.dtype,
                {'names':['f0', 'f1'],
                 'formats':['i4', 'O'],
                 'offsets':[0, 3]})
        assert_raises(TypeError, np.dtype,
                {'names':['f0', 'f1'],
                 'formats':[[('a', 'O')], 'i1'],
                 'offsets':[0, 2]})
        assert_raises(TypeError, np.dtype,
                {'names':['f0', 'f1'],
                 'formats':['i4', [('a', 'O')]],
                 'offsets':[0, 3]})
        # Out of order should still be ok, however
        dt = np.dtype({'names':['f0', 'f1'],
                       'formats':['i1', 'O'],
                       'offsets':[np.dtype('intp').itemsize, 0]})

    def test_comma_datetime(self):
        dt = np.dtype('M8[D],datetime64[Y],i8')
        assert_equal(dt, np.dtype([('f0', 'M8[D]'),
                                   ('f1', 'datetime64[Y]'),
                                   ('f2', 'i8')]))

class TestSubarray(TestCase):
    def test_single_subarray(self):
        a = np.dtype((np.int, (2)))
        b = np.dtype((np.int, (2,)))
        assert_dtype_equal(a, b)

        assert_equal(type(a.subdtype[1]), tuple)
        assert_equal(type(b.subdtype[1]), tuple)

    def test_equivalent_record(self):
        """Test whether equivalent subarray dtypes hash the same."""
        a = np.dtype((np.int, (2, 3)))
        b = np.dtype((np.int, (2, 3)))
        assert_dtype_equal(a, b)

    def test_nonequivalent_record(self):
        """Test whether different subarray dtypes hash differently."""
        a = np.dtype((np.int, (2, 3)))
        b = np.dtype((np.int, (3, 2)))
        assert_dtype_not_equal(a, b)

        a = np.dtype((np.int, (2, 3)))
        b = np.dtype((np.int, (2, 2)))
        assert_dtype_not_equal(a, b)

        a = np.dtype((np.int, (1, 2, 3)))
        b = np.dtype((np.int, (1, 2)))
        assert_dtype_not_equal(a, b)

    def test_shape_equal(self):
        """Test some data types that are equal"""
        assert_dtype_equal(np.dtype('f8'), np.dtype(('f8', tuple())))
        assert_dtype_equal(np.dtype('f8'), np.dtype(('f8', 1)))
        assert_dtype_equal(np.dtype((np.int, 2)), np.dtype((np.int, (2,))))
        assert_dtype_equal(np.dtype(('<f4', (3, 2))), np.dtype(('<f4', (3, 2))))
        d = ([('a', 'f4', (1, 2)), ('b', 'f8', (3, 1))], (3, 2))
        assert_dtype_equal(np.dtype(d), np.dtype(d))

    def test_shape_simple(self):
        """Test some simple cases that shouldn't be equal"""
        assert_dtype_not_equal(np.dtype('f8'), np.dtype(('f8', (1,))))
        assert_dtype_not_equal(np.dtype(('f8', (1,))), np.dtype(('f8', (1, 1))))
        assert_dtype_not_equal(np.dtype(('f4', (3, 2))), np.dtype(('f4', (2, 3))))

    def test_shape_monster(self):
        """Test some more complicated cases that shouldn't be equal"""
        assert_dtype_not_equal(
            np.dtype(([('a', 'f4', (2, 1)), ('b', 'f8', (1, 3))], (2, 2))),
            np.dtype(([('a', 'f4', (1, 2)), ('b', 'f8', (1, 3))], (2, 2))))
        assert_dtype_not_equal(
            np.dtype(([('a', 'f4', (2, 1)), ('b', 'f8', (1, 3))], (2, 2))),
            np.dtype(([('a', 'f4', (2, 1)), ('b', 'i8', (1, 3))], (2, 2))))
        assert_dtype_not_equal(
            np.dtype(([('a', 'f4', (2, 1)), ('b', 'f8', (1, 3))], (2, 2))),
            np.dtype(([('e', 'f8', (1, 3)), ('d', 'f4', (2, 1))], (2, 2))))
        assert_dtype_not_equal(
            np.dtype(([('a', [('a', 'i4', 6)], (2, 1)), ('b', 'f8', (1, 3))], (2, 2))),
            np.dtype(([('a', [('a', 'u4', 6)], (2, 1)), ('b', 'f8', (1, 3))], (2, 2))))

class TestMonsterType(TestCase):
    """Test deeply nested subtypes."""
    def test1(self):
        simple1 = np.dtype({'names': ['r', 'b'], 'formats': ['u1', 'u1'],
            'titles': ['Red pixel', 'Blue pixel']})
        a = np.dtype([('yo', np.int), ('ye', simple1),
            ('yi', np.dtype((np.int, (3, 2))))])
        b = np.dtype([('yo', np.int), ('ye', simple1),
            ('yi', np.dtype((np.int, (3, 2))))])
        assert_dtype_equal(a, b)

        c = np.dtype([('yo', np.int), ('ye', simple1),
            ('yi', np.dtype((a, (3, 2))))])
        d = np.dtype([('yo', np.int), ('ye', simple1),
            ('yi', np.dtype((a, (3, 2))))])
        assert_dtype_equal(c, d)

class TestMetadata(TestCase):
    def test_no_metadata(self):
        d = np.dtype(int)
        self.assertEqual(d.metadata, None)

    def test_metadata_takes_dict(self):
        d = np.dtype(int, metadata={'datum': 1})
        self.assertEqual(d.metadata, {'datum': 1})

    def test_metadata_rejects_nondict(self):
        self.assertRaises(TypeError, np.dtype, int, metadata='datum')
        self.assertRaises(TypeError, np.dtype, int, metadata=1)
        self.assertRaises(TypeError, np.dtype, int, metadata=None)

    def test_nested_metadata(self):
        d = np.dtype([('a', np.dtype(int, metadata={'datum': 1}))])
        self.assertEqual(d['a'].metadata, {'datum': 1})

class TestString(TestCase):
    def test_complex_dtype_str(self):
        dt = np.dtype([('top', [('tiles', ('>f4', (64, 64)), (1,)),
                                ('rtile', '>f4', (64, 36))], (3,)),
                       ('bottom', [('bleft', ('>f4', (8, 64)), (1,)),
                                   ('bright', '>f4', (8, 36))])])
        assert_equal(str(dt),
                     "[('top', [('tiles', ('>f4', (64, 64)), (1,)), "
                     "('rtile', '>f4', (64, 36))], (3,)), "
                     "('bottom', [('bleft', ('>f4', (8, 64)), (1,)), "
                     "('bright', '>f4', (8, 36))])]")

        # If the sticky aligned flag is set to True, it makes the
        # str() function use a dict representation with an 'aligned' flag
        dt = np.dtype([('top', [('tiles', ('>f4', (64, 64)), (1,)),
                                ('rtile', '>f4', (64, 36))],
                                (3,)),
                       ('bottom', [('bleft', ('>f4', (8, 64)), (1,)),
                                   ('bright', '>f4', (8, 36))])],
                       align=True)
        assert_equal(str(dt),
                    "{'names':['top','bottom'], "
                     "'formats':[([('tiles', ('>f4', (64, 64)), (1,)), "
                                  "('rtile', '>f4', (64, 36))], (3,)),"
                                 "[('bleft', ('>f4', (8, 64)), (1,)), "
                                  "('bright', '>f4', (8, 36))]], "
                     "'offsets':[0,76800], "
                     "'itemsize':80000, "
                     "'aligned':True}")
        assert_equal(np.dtype(eval(str(dt))), dt)

        dt = np.dtype({'names': ['r', 'g', 'b'], 'formats': ['u1', 'u1', 'u1'],
                        'offsets': [0, 1, 2],
                        'titles': ['Red pixel', 'Green pixel', 'Blue pixel']})
        assert_equal(str(dt),
                    "[(('Red pixel', 'r'), 'u1'), "
                    "(('Green pixel', 'g'), 'u1'), "
                    "(('Blue pixel', 'b'), 'u1')]")

        dt = np.dtype({'names': ['rgba', 'r', 'g', 'b'],
                       'formats': ['<u4', 'u1', 'u1', 'u1'],
                       'offsets': [0, 0, 1, 2],
                       'titles': ['Color', 'Red pixel',
                                  'Green pixel', 'Blue pixel']})
        assert_equal(str(dt),
                    "{'names':['rgba','r','g','b'],"
                    " 'formats':['<u4','u1','u1','u1'],"
                    " 'offsets':[0,0,1,2],"
                    " 'titles':['Color','Red pixel',"
                              "'Green pixel','Blue pixel'],"
                    " 'itemsize':4}")

        dt = np.dtype({'names': ['r', 'b'], 'formats': ['u1', 'u1'],
                        'offsets': [0, 2],
                        'titles': ['Red pixel', 'Blue pixel']})
        assert_equal(str(dt),
                    "{'names':['r','b'],"
                    " 'formats':['u1','u1'],"
                    " 'offsets':[0,2],"
                    " 'titles':['Red pixel','Blue pixel'],"
                    " 'itemsize':3}")

        dt = np.dtype([('a', '<m8[D]'), ('b', '<M8[us]')])
        assert_equal(str(dt),
                    "[('a', '<m8[D]'), ('b', '<M8[us]')]")

    def test_complex_dtype_repr(self):
        dt = np.dtype([('top', [('tiles', ('>f4', (64, 64)), (1,)),
                                ('rtile', '>f4', (64, 36))], (3,)),
                       ('bottom', [('bleft', ('>f4', (8, 64)), (1,)),
                                   ('bright', '>f4', (8, 36))])])
        assert_equal(repr(dt),
                     "dtype([('top', [('tiles', ('>f4', (64, 64)), (1,)), "
                     "('rtile', '>f4', (64, 36))], (3,)), "
                     "('bottom', [('bleft', ('>f4', (8, 64)), (1,)), "
                     "('bright', '>f4', (8, 36))])])")

        dt = np.dtype({'names': ['r', 'g', 'b'], 'formats': ['u1', 'u1', 'u1'],
                        'offsets': [0, 1, 2],
                        'titles': ['Red pixel', 'Green pixel', 'Blue pixel']},
                        align=True)
        assert_equal(repr(dt),
                    "dtype([(('Red pixel', 'r'), 'u1'), "
                    "(('Green pixel', 'g'), 'u1'), "
                    "(('Blue pixel', 'b'), 'u1')], align=True)")

        dt = np.dtype({'names': ['rgba', 'r', 'g', 'b'],
                       'formats': ['<u4', 'u1', 'u1', 'u1'],
                       'offsets': [0, 0, 1, 2],
                       'titles': ['Color', 'Red pixel',
                                  'Green pixel', 'Blue pixel']}, align=True)
        assert_equal(repr(dt),
                    "dtype({'names':['rgba','r','g','b'],"
                    " 'formats':['<u4','u1','u1','u1'],"
                    " 'offsets':[0,0,1,2],"
                    " 'titles':['Color','Red pixel',"
                              "'Green pixel','Blue pixel'],"
                    " 'itemsize':4}, align=True)")

        dt = np.dtype({'names': ['r', 'b'], 'formats': ['u1', 'u1'],
                        'offsets': [0, 2],
                        'titles': ['Red pixel', 'Blue pixel'],
                        'itemsize': 4})
        assert_equal(repr(dt),
                    "dtype({'names':['r','b'], "
                    "'formats':['u1','u1'], "
                    "'offsets':[0,2], "
                    "'titles':['Red pixel','Blue pixel'], "
                    "'itemsize':4})")

        dt = np.dtype([('a', '<M8[D]'), ('b', '<m8[us]')])
        assert_equal(repr(dt),
                    "dtype([('a', '<M8[D]'), ('b', '<m8[us]')])")

    @dec.skipif(sys.version_info[0] >= 3)
    def test_dtype_str_with_long_in_shape(self):
        # Pull request #376
        dt = np.dtype('(1L,)i4')

    def test_base_dtype_with_object_type(self):
        # Issue gh-2798
        a = np.array(['a'], dtype="O").astype(("O", [("name", "O")]))

    def test_empty_string_to_object(self):
        # Pull request #4722
        np.array(["", ""]).astype(object)

class TestDtypeAttributeDeletion(object):

    def test_dtype_non_writable_attributes_deletion(self):
        dt = np.dtype(np.double)
        attr = ["subdtype", "descr", "str", "name", "base", "shape",
                "isbuiltin", "isnative", "isalignedstruct", "fields",
                "metadata", "hasobject"]

        if sys.version[:3] == '2.4':
            error = TypeError
        else:
            error = AttributeError

        for s in attr:
            assert_raises(error, delattr, dt, s)


    def test_dtype_writable_attributes_deletion(self):
        dt = np.dtype(np.double)
        attr = ["names"]
        for s in attr:
            assert_raises(AttributeError, delattr, dt, s)


if __name__ == "__main__":
    run_module_suite()
