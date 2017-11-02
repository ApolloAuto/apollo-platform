from __future__ import division, absolute_import, print_function

from numpy.testing import *
from numpy.core import *
import numpy as np
import sys
from numpy.core.multiarray import _vec_string

from numpy.compat import asbytes, asbytes_nested, sixu

kw_unicode_true = {'unicode': True} # make 2to3 work properly
kw_unicode_false = {'unicode': False}

class TestBasic(TestCase):
    def test_from_object_array(self):
        A = np.array([['abc', 2],
                      ['long   ', '0123456789']], dtype='O')
        B = np.char.array(A)
        assert_equal(B.dtype.itemsize, 10)
        assert_array_equal(B, asbytes_nested([['abc', '2'],
                                              ['long', '0123456789']]))

    def test_from_object_array_unicode(self):
        A = np.array([['abc', sixu('Sigma \u03a3')],
                      ['long   ', '0123456789']], dtype='O')
        self.assertRaises(ValueError, np.char.array, (A,))
        B = np.char.array(A, **kw_unicode_true)
        assert_equal(B.dtype.itemsize, 10 * np.array('a', 'U').dtype.itemsize)
        assert_array_equal(B, [['abc', sixu('Sigma \u03a3')],
                               ['long', '0123456789']])

    def test_from_string_array(self):
        A = np.array(asbytes_nested([['abc', 'foo'],
                                     ['long   ', '0123456789']]))
        assert_equal(A.dtype.type, np.string_)
        B = np.char.array(A)
        assert_array_equal(B, A)
        assert_equal(B.dtype, A.dtype)
        assert_equal(B.shape, A.shape)
        B[0, 0] = 'changed'
        assert_(B[0, 0] != A[0, 0])
        C = np.char.asarray(A)
        assert_array_equal(C, A)
        assert_equal(C.dtype, A.dtype)
        C[0, 0] = 'changed again'
        assert_(C[0, 0] != B[0, 0])
        assert_(C[0, 0] == A[0, 0])

    def test_from_unicode_array(self):
        A = np.array([['abc', sixu('Sigma \u03a3')],
                      ['long   ', '0123456789']])
        assert_equal(A.dtype.type, np.unicode_)
        B = np.char.array(A)
        assert_array_equal(B, A)
        assert_equal(B.dtype, A.dtype)
        assert_equal(B.shape, A.shape)
        B = np.char.array(A, **kw_unicode_true)
        assert_array_equal(B, A)
        assert_equal(B.dtype, A.dtype)
        assert_equal(B.shape, A.shape)
        def fail():
            B = np.char.array(A, **kw_unicode_false)
        self.assertRaises(UnicodeEncodeError, fail)

    def test_unicode_upconvert(self):
        A = np.char.array(['abc'])
        B = np.char.array([sixu('\u03a3')])
        assert_(issubclass((A + B).dtype.type, np.unicode_))

    def test_from_string(self):
        A = np.char.array(asbytes('abc'))
        assert_equal(len(A), 1)
        assert_equal(len(A[0]), 3)
        assert_(issubclass(A.dtype.type, np.string_))

    def test_from_unicode(self):
        A = np.char.array(sixu('\u03a3'))
        assert_equal(len(A), 1)
        assert_equal(len(A[0]), 1)
        assert_equal(A.itemsize, 4)
        assert_(issubclass(A.dtype.type, np.unicode_))

class TestVecString(TestCase):
    def test_non_existent_method(self):
        def fail():
            _vec_string('a', np.string_, 'bogus')
        self.assertRaises(AttributeError, fail)

    def test_non_string_array(self):
        def fail():
            _vec_string(1, np.string_, 'strip')
        self.assertRaises(TypeError, fail)

    def test_invalid_args_tuple(self):
        def fail():
            _vec_string(['a'], np.string_, 'strip', 1)
        self.assertRaises(TypeError, fail)

    def test_invalid_type_descr(self):
        def fail():
            _vec_string(['a'], 'BOGUS', 'strip')
        self.assertRaises(TypeError, fail)

    def test_invalid_function_args(self):
        def fail():
            _vec_string(['a'], np.string_, 'strip', (1,))
        self.assertRaises(TypeError, fail)

    def test_invalid_result_type(self):
        def fail():
            _vec_string(['a'], np.integer, 'strip')
        self.assertRaises(TypeError, fail)

    def test_broadcast_error(self):
        def fail():
            _vec_string([['abc', 'def']], np.integer, 'find', (['a', 'd', 'j'],))
        self.assertRaises(ValueError, fail)


class TestWhitespace(TestCase):
    def setUp(self):
        self.A = np.array([['abc ', '123  '],
                           ['789 ', 'xyz ']]).view(np.chararray)
        self.B = np.array([['abc', '123'],
                           ['789', 'xyz']]).view(np.chararray)

    def test1(self):
        assert_(all(self.A == self.B))
        assert_(all(self.A >= self.B))
        assert_(all(self.A <= self.B))
        assert_(all(negative(self.A > self.B)))
        assert_(all(negative(self.A < self.B)))
        assert_(all(negative(self.A != self.B)))

class TestChar(TestCase):
    def setUp(self):
        self.A = np.array('abc1', dtype='c').view(np.chararray)

    def test_it(self):
        assert_equal(self.A.shape, (4,))
        assert_equal(self.A.upper()[:2].tostring(), asbytes('AB'))

class TestComparisons(TestCase):
    def setUp(self):
        self.A = np.array([['abc', '123'],
                           ['789', 'xyz']]).view(np.chararray)
        self.B = np.array([['efg', '123  '],
                           ['051', 'tuv']]).view(np.chararray)

    def test_not_equal(self):
        assert_array_equal((self.A != self.B), [[True, False], [True, True]])

    def test_equal(self):
        assert_array_equal((self.A == self.B), [[False, True], [False, False]])

    def test_greater_equal(self):
        assert_array_equal((self.A >= self.B), [[False, True], [True, True]])

    def test_less_equal(self):
        assert_array_equal((self.A <= self.B), [[True, True], [False, False]])

    def test_greater(self):
        assert_array_equal((self.A > self.B), [[False, False], [True, True]])

    def test_less(self):
        assert_array_equal((self.A < self.B), [[True, False], [False, False]])

class TestComparisonsMixed1(TestComparisons):
    """Ticket #1276"""

    def setUp(self):
        TestComparisons.setUp(self)
        self.B = np.array([['efg', '123  '],
                           ['051', 'tuv']], np.unicode_).view(np.chararray)

class TestComparisonsMixed2(TestComparisons):
    """Ticket #1276"""

    def setUp(self):
        TestComparisons.setUp(self)
        self.A = np.array([['abc', '123'],
                           ['789', 'xyz']], np.unicode_).view(np.chararray)

class TestInformation(TestCase):
    def setUp(self):
        self.A = np.array([[' abc ', ''],
                           ['12345', 'MixedCase'],
                           ['123 \t 345 \0 ', 'UPPER']]).view(np.chararray)
        self.B = np.array([[sixu(' \u03a3 '), sixu('')],
                           [sixu('12345'), sixu('MixedCase')],
                           [sixu('123 \t 345 \0 '), sixu('UPPER')]]).view(np.chararray)

    def test_len(self):
        assert_(issubclass(np.char.str_len(self.A).dtype.type, np.integer))
        assert_array_equal(np.char.str_len(self.A), [[5, 0], [5, 9], [12, 5]])
        assert_array_equal(np.char.str_len(self.B), [[3, 0], [5, 9], [12, 5]])

    def test_count(self):
        assert_(issubclass(self.A.count('').dtype.type, np.integer))
        assert_array_equal(self.A.count('a'), [[1, 0], [0, 1], [0, 0]])
        assert_array_equal(self.A.count('123'), [[0, 0], [1, 0], [1, 0]])
        # Python doesn't seem to like counting NULL characters
        # assert_array_equal(self.A.count('\0'), [[0, 0], [0, 0], [1, 0]])
        assert_array_equal(self.A.count('a', 0, 2), [[1, 0], [0, 0], [0, 0]])
        assert_array_equal(self.B.count('a'), [[0, 0], [0, 1], [0, 0]])
        assert_array_equal(self.B.count('123'), [[0, 0], [1, 0], [1, 0]])
        # assert_array_equal(self.B.count('\0'), [[0, 0], [0, 0], [1, 0]])

    def test_endswith(self):
        assert_(issubclass(self.A.endswith('').dtype.type, np.bool_))
        assert_array_equal(self.A.endswith(' '), [[1, 0], [0, 0], [1, 0]])
        assert_array_equal(self.A.endswith('3', 0, 3), [[0, 0], [1, 0], [1, 0]])
        def fail():
            self.A.endswith('3', 'fdjk')
        self.assertRaises(TypeError, fail)

    def test_find(self):
        assert_(issubclass(self.A.find('a').dtype.type, np.integer))
        assert_array_equal(self.A.find('a'), [[1, -1], [-1, 6], [-1, -1]])
        assert_array_equal(self.A.find('3'), [[-1, -1], [2, -1], [2, -1]])
        assert_array_equal(self.A.find('a', 0, 2), [[1, -1], [-1, -1], [-1, -1]])
        assert_array_equal(self.A.find(['1', 'P']), [[-1, -1], [0, -1], [0, 1]])

    def test_index(self):
        def fail():
            self.A.index('a')
        self.assertRaises(ValueError, fail)
        assert_(np.char.index('abcba', 'b') == 1)
        assert_(issubclass(np.char.index('abcba', 'b').dtype.type, np.integer))

    def test_isalnum(self):
        assert_(issubclass(self.A.isalnum().dtype.type, np.bool_))
        assert_array_equal(self.A.isalnum(), [[False, False], [True, True], [False, True]])

    def test_isalpha(self):
        assert_(issubclass(self.A.isalpha().dtype.type, np.bool_))
        assert_array_equal(self.A.isalpha(), [[False, False], [False, True], [False, True]])

    def test_isdigit(self):
        assert_(issubclass(self.A.isdigit().dtype.type, np.bool_))
        assert_array_equal(self.A.isdigit(), [[False, False], [True, False], [False, False]])

    def test_islower(self):
        assert_(issubclass(self.A.islower().dtype.type, np.bool_))
        assert_array_equal(self.A.islower(), [[True, False], [False, False], [False, False]])

    def test_isspace(self):
        assert_(issubclass(self.A.isspace().dtype.type, np.bool_))
        assert_array_equal(self.A.isspace(), [[False, False], [False, False], [False, False]])

    def test_istitle(self):
        assert_(issubclass(self.A.istitle().dtype.type, np.bool_))
        assert_array_equal(self.A.istitle(), [[False, False], [False, False], [False, False]])

    def test_isupper(self):
        assert_(issubclass(self.A.isupper().dtype.type, np.bool_))
        assert_array_equal(self.A.isupper(), [[False, False], [False, False], [False, True]])

    def test_rfind(self):
        assert_(issubclass(self.A.rfind('a').dtype.type, np.integer))
        assert_array_equal(self.A.rfind('a'), [[1, -1], [-1, 6], [-1, -1]])
        assert_array_equal(self.A.rfind('3'), [[-1, -1], [2, -1], [6, -1]])
        assert_array_equal(self.A.rfind('a', 0, 2), [[1, -1], [-1, -1], [-1, -1]])
        assert_array_equal(self.A.rfind(['1', 'P']), [[-1, -1], [0, -1], [0, 2]])

    def test_rindex(self):
        def fail():
            self.A.rindex('a')
        self.assertRaises(ValueError, fail)
        assert_(np.char.rindex('abcba', 'b') == 3)
        assert_(issubclass(np.char.rindex('abcba', 'b').dtype.type, np.integer))

    def test_startswith(self):
        assert_(issubclass(self.A.startswith('').dtype.type, np.bool_))
        assert_array_equal(self.A.startswith(' '), [[1, 0], [0, 0], [0, 0]])
        assert_array_equal(self.A.startswith('1', 0, 3), [[0, 0], [1, 0], [1, 0]])
        def fail():
            self.A.startswith('3', 'fdjk')
        self.assertRaises(TypeError, fail)


class TestMethods(TestCase):
    def setUp(self):
        self.A = np.array([[' abc ', ''],
                           ['12345', 'MixedCase'],
                           ['123 \t 345 \0 ', 'UPPER']],
                          dtype='S').view(np.chararray)
        self.B = np.array([[sixu(' \u03a3 '), sixu('')],
                           [sixu('12345'), sixu('MixedCase')],
                           [sixu('123 \t 345 \0 '), sixu('UPPER')]]).view(np.chararray)

    def test_capitalize(self):
        assert_(issubclass(self.A.capitalize().dtype.type, np.string_))
        assert_array_equal(self.A.capitalize(), asbytes_nested([
                [' abc ', ''],
                ['12345', 'Mixedcase'],
                ['123 \t 345 \0 ', 'Upper']]))
        assert_(issubclass(self.B.capitalize().dtype.type, np.unicode_))
        assert_array_equal(self.B.capitalize(), [
                [sixu(' \u03c3 '), ''],
                ['12345', 'Mixedcase'],
                ['123 \t 345 \0 ', 'Upper']])

    def test_center(self):
        assert_(issubclass(self.A.center(10).dtype.type, np.string_))
        widths = np.array([[10, 20]])
        C = self.A.center([10, 20])
        assert_array_equal(np.char.str_len(C), [[10, 20], [10, 20], [12, 20]])
        C = self.A.center(20, asbytes('#'))
        assert_(np.all(C.startswith(asbytes('#'))))
        assert_(np.all(C.endswith(asbytes('#'))))
        C = np.char.center(asbytes('FOO'), [[10, 20], [15, 8]])
        assert_(issubclass(C.dtype.type, np.string_))
        assert_array_equal(C, asbytes_nested([
                ['   FOO    ', '        FOO         '],
                ['      FOO      ', '  FOO   ']]))

    def test_decode(self):
        if sys.version_info[0] >= 3:
            A = np.char.array([asbytes('\\u03a3')])
            assert_(A.decode('unicode-escape')[0] == '\u03a3')
        else:
            A = np.char.array(['736563726574206d657373616765'])
            assert_(A.decode('hex_codec')[0] == 'secret message')

    def test_encode(self):
        B = self.B.encode('unicode_escape')
        assert_(B[0][0] == str(' \\u03a3 ').encode('latin1'))

    def test_expandtabs(self):
        T = self.A.expandtabs()
        assert_(T[2][0] == asbytes('123      345'))

    def test_join(self):
        if sys.version_info[0] >= 3:
            # NOTE: list(b'123') == [49, 50, 51]
            #       so that b','.join(b'123') results to an error on Py3
            A0 = self.A.decode('ascii')
        else:
            A0 = self.A

        A = np.char.join([',', '#'], A0)
        if sys.version_info[0] >= 3:
            assert_(issubclass(A.dtype.type, np.unicode_))
        else:
            assert_(issubclass(A.dtype.type, np.string_))
        assert_array_equal(np.char.join([',', '#'], A0),
                           [
            [' ,a,b,c, ', ''],
            ['1,2,3,4,5', 'M#i#x#e#d#C#a#s#e'],
            ['1,2,3, ,\t, ,3,4,5, ,\x00, ', 'U#P#P#E#R']])

    def test_ljust(self):
        assert_(issubclass(self.A.ljust(10).dtype.type, np.string_))
        widths = np.array([[10, 20]])
        C = self.A.ljust([10, 20])
        assert_array_equal(np.char.str_len(C), [[10, 20], [10, 20], [12, 20]])
        C = self.A.ljust(20, asbytes('#'))
        assert_array_equal(C.startswith(asbytes('#')), [
                [False, True], [False, False], [False, False]])
        assert_(np.all(C.endswith(asbytes('#'))))
        C = np.char.ljust(asbytes('FOO'), [[10, 20], [15, 8]])
        assert_(issubclass(C.dtype.type, np.string_))
        assert_array_equal(C, asbytes_nested([
                ['FOO       ', 'FOO                 '],
                ['FOO            ', 'FOO     ']]))

    def test_lower(self):
        assert_(issubclass(self.A.lower().dtype.type, np.string_))
        assert_array_equal(self.A.lower(), asbytes_nested([
                [' abc ', ''],
                ['12345', 'mixedcase'],
                ['123 \t 345 \0 ', 'upper']]))
        assert_(issubclass(self.B.lower().dtype.type, np.unicode_))
        assert_array_equal(self.B.lower(), [
                [sixu(' \u03c3 '), sixu('')],
                [sixu('12345'), sixu('mixedcase')],
                [sixu('123 \t 345 \0 '), sixu('upper')]])

    def test_lstrip(self):
        assert_(issubclass(self.A.lstrip().dtype.type, np.string_))
        assert_array_equal(self.A.lstrip(), asbytes_nested([
                ['abc ', ''],
                ['12345', 'MixedCase'],
                ['123 \t 345 \0 ', 'UPPER']]))
        assert_array_equal(self.A.lstrip(asbytes_nested(['1', 'M'])),
                           asbytes_nested([
                [' abc', ''],
                ['2345', 'ixedCase'],
                ['23 \t 345 \x00', 'UPPER']]))
        assert_(issubclass(self.B.lstrip().dtype.type, np.unicode_))
        assert_array_equal(self.B.lstrip(), [
                [sixu('\u03a3 '), ''],
                ['12345', 'MixedCase'],
                ['123 \t 345 \0 ', 'UPPER']])

    def test_partition(self):
        P = self.A.partition(asbytes_nested(['3', 'M']))
        assert_(issubclass(P.dtype.type, np.string_))
        assert_array_equal(P, asbytes_nested([
                [(' abc ', '', ''), ('', '', '')],
                [('12', '3', '45'), ('', 'M', 'ixedCase')],
                [('12', '3', ' \t 345 \0 '), ('UPPER', '', '')]]))

    def test_replace(self):
        R = self.A.replace(asbytes_nested(['3', 'a']),
                           asbytes_nested(['##########', '@']))
        assert_(issubclass(R.dtype.type, np.string_))
        assert_array_equal(R, asbytes_nested([
                [' abc ', ''],
                ['12##########45', 'MixedC@se'],
                ['12########## \t ##########45 \x00', 'UPPER']]))

        if sys.version_info[0] < 3:
            # NOTE: b'abc'.replace(b'a', 'b') is not allowed on Py3
            R = self.A.replace(asbytes('a'), sixu('\u03a3'))
            assert_(issubclass(R.dtype.type, np.unicode_))
            assert_array_equal(R, [
                    [sixu(' \u03a3bc '), ''],
                    ['12345', sixu('MixedC\u03a3se')],
                    ['123 \t 345 \x00', 'UPPER']])

    def test_rjust(self):
        assert_(issubclass(self.A.rjust(10).dtype.type, np.string_))
        widths = np.array([[10, 20]])
        C = self.A.rjust([10, 20])
        assert_array_equal(np.char.str_len(C), [[10, 20], [10, 20], [12, 20]])
        C = self.A.rjust(20, asbytes('#'))
        assert_(np.all(C.startswith(asbytes('#'))))
        assert_array_equal(C.endswith(asbytes('#')),
                           [[False, True], [False, False], [False, False]])
        C = np.char.rjust(asbytes('FOO'), [[10, 20], [15, 8]])
        assert_(issubclass(C.dtype.type, np.string_))
        assert_array_equal(C, asbytes_nested([
                ['       FOO', '                 FOO'],
                ['            FOO', '     FOO']]))

    def test_rpartition(self):
        P = self.A.rpartition(asbytes_nested(['3', 'M']))
        assert_(issubclass(P.dtype.type, np.string_))
        assert_array_equal(P, asbytes_nested([
                [('', '', ' abc '), ('', '', '')],
                [('12', '3', '45'), ('', 'M', 'ixedCase')],
                [('123 \t ', '3', '45 \0 '), ('', '', 'UPPER')]]))

    def test_rsplit(self):
        A = self.A.rsplit(asbytes('3'))
        assert_(issubclass(A.dtype.type, np.object_))
        assert_equal(A.tolist(), asbytes_nested([
                [[' abc '], ['']],
                [['12', '45'], ['MixedCase']],
                [['12', ' \t ', '45 \x00 '], ['UPPER']]]))

    def test_rstrip(self):
        assert_(issubclass(self.A.rstrip().dtype.type, np.string_))
        assert_array_equal(self.A.rstrip(), asbytes_nested([
                [' abc', ''],
                ['12345', 'MixedCase'],
                ['123 \t 345', 'UPPER']]))
        assert_array_equal(self.A.rstrip(asbytes_nested(['5', 'ER'])),
                           asbytes_nested([
                [' abc ', ''],
                ['1234', 'MixedCase'],
                ['123 \t 345 \x00', 'UPP']]))
        assert_(issubclass(self.B.rstrip().dtype.type, np.unicode_))
        assert_array_equal(self.B.rstrip(), [
                [sixu(' \u03a3'), ''],
                ['12345', 'MixedCase'],
                ['123 \t 345', 'UPPER']])

    def test_strip(self):
        assert_(issubclass(self.A.strip().dtype.type, np.string_))
        assert_array_equal(self.A.strip(), asbytes_nested([
                ['abc', ''],
                ['12345', 'MixedCase'],
                ['123 \t 345', 'UPPER']]))
        assert_array_equal(self.A.strip(asbytes_nested(['15', 'EReM'])),
                           asbytes_nested([
                [' abc ', ''],
                ['234', 'ixedCas'],
                ['23 \t 345 \x00', 'UPP']]))
        assert_(issubclass(self.B.strip().dtype.type, np.unicode_))
        assert_array_equal(self.B.strip(), [
                [sixu('\u03a3'), ''],
                ['12345', 'MixedCase'],
                ['123 \t 345', 'UPPER']])

    def test_split(self):
        A = self.A.split(asbytes('3'))
        assert_(issubclass(A.dtype.type, np.object_))
        assert_equal(A.tolist(), asbytes_nested([
                [[' abc '], ['']],
                [['12', '45'], ['MixedCase']],
                [['12', ' \t ', '45 \x00 '], ['UPPER']]]))

    def test_splitlines(self):
        A = np.char.array(['abc\nfds\nwer']).splitlines()
        assert_(issubclass(A.dtype.type, np.object_))
        assert_(A.shape == (1,))
        assert_(len(A[0]) == 3)

    def test_swapcase(self):
        assert_(issubclass(self.A.swapcase().dtype.type, np.string_))
        assert_array_equal(self.A.swapcase(), asbytes_nested([
                [' ABC ', ''],
                ['12345', 'mIXEDcASE'],
                ['123 \t 345 \0 ', 'upper']]))
        assert_(issubclass(self.B.swapcase().dtype.type, np.unicode_))
        assert_array_equal(self.B.swapcase(), [
                [sixu(' \u03c3 '), sixu('')],
                [sixu('12345'), sixu('mIXEDcASE')],
                [sixu('123 \t 345 \0 '), sixu('upper')]])

    def test_title(self):
        assert_(issubclass(self.A.title().dtype.type, np.string_))
        assert_array_equal(self.A.title(), asbytes_nested([
                [' Abc ', ''],
                ['12345', 'Mixedcase'],
                ['123 \t 345 \0 ', 'Upper']]))
        assert_(issubclass(self.B.title().dtype.type, np.unicode_))
        assert_array_equal(self.B.title(), [
                [sixu(' \u03a3 '), sixu('')],
                [sixu('12345'), sixu('Mixedcase')],
                [sixu('123 \t 345 \0 '), sixu('Upper')]])

    def test_upper(self):
        assert_(issubclass(self.A.upper().dtype.type, np.string_))
        assert_array_equal(self.A.upper(), asbytes_nested([
                [' ABC ', ''],
                ['12345', 'MIXEDCASE'],
                ['123 \t 345 \0 ', 'UPPER']]))
        assert_(issubclass(self.B.upper().dtype.type, np.unicode_))
        assert_array_equal(self.B.upper(), [
                [sixu(' \u03a3 '), sixu('')],
                [sixu('12345'), sixu('MIXEDCASE')],
                [sixu('123 \t 345 \0 '), sixu('UPPER')]])

    def test_isnumeric(self):
        def fail():
            self.A.isnumeric()
        self.assertRaises(TypeError, fail)
        assert_(issubclass(self.B.isnumeric().dtype.type, np.bool_))
        assert_array_equal(self.B.isnumeric(), [
                [False, False], [True, False], [False, False]])

    def test_isdecimal(self):
        def fail():
            self.A.isdecimal()
        self.assertRaises(TypeError, fail)
        assert_(issubclass(self.B.isdecimal().dtype.type, np.bool_))
        assert_array_equal(self.B.isdecimal(), [
                [False, False], [True, False], [False, False]])


class TestOperations(TestCase):
    def setUp(self):
        self.A = np.array([['abc', '123'],
                           ['789', 'xyz']]).view(np.chararray)
        self.B = np.array([['efg', '456'],
                           ['051', 'tuv']]).view(np.chararray)

    def test_add(self):
        AB = np.array([['abcefg', '123456'],
                       ['789051', 'xyztuv']]).view(np.chararray)
        assert_array_equal(AB, (self.A + self.B))
        assert_(len((self.A + self.B)[0][0]) == 6)

    def test_radd(self):
        QA = np.array([['qabc', 'q123'],
                       ['q789', 'qxyz']]).view(np.chararray)
        assert_array_equal(QA, ('q' + self.A))

    def test_mul(self):
        A = self.A
        for r in (2, 3, 5, 7, 197):
            Ar = np.array([[A[0, 0]*r, A[0, 1]*r],
                           [A[1, 0]*r, A[1, 1]*r]]).view(np.chararray)

            assert_array_equal(Ar, (self.A * r))

        for ob in [object(), 'qrs']:
            try:
                A * ob
            except ValueError:
                pass
            else:
                self.fail("chararray can only be multiplied by integers")

    def test_rmul(self):
        A = self.A
        for r in (2, 3, 5, 7, 197):
            Ar = np.array([[A[0, 0]*r, A[0, 1]*r],
                           [A[1, 0]*r, A[1, 1]*r]]).view(np.chararray)
            assert_array_equal(Ar, (r * self.A))

        for ob in [object(), 'qrs']:
            try:
                ob * A
            except ValueError:
                pass
            else:
                self.fail("chararray can only be multiplied by integers")

    def test_mod(self):
        """Ticket #856"""
        F = np.array([['%d', '%f'], ['%s', '%r']]).view(np.chararray)
        C = np.array([[3, 7], [19, 1]])
        FC = np.array([['3', '7.000000'],
                       ['19', '1']]).view(np.chararray)
        assert_array_equal(FC, F % C)

        A = np.array([['%.3f', '%d'], ['%s', '%r']]).view(np.chararray)
        A1 = np.array([['1.000', '1'], ['1', '1']]).view(np.chararray)
        assert_array_equal(A1, (A % 1))

        A2 = np.array([['1.000', '2'], ['3', '4']]).view(np.chararray)
        assert_array_equal(A2, (A % [[1, 2], [3, 4]]))

    def test_rmod(self):
        assert_(("%s" % self.A) == str(self.A))
        assert_(("%r" % self.A) == repr(self.A))

        for ob in [42, object()]:
            try:
                ob % self.A
            except TypeError:
                pass
            else:
                self.fail("chararray __rmod__ should fail with " \
                          "non-string objects")


def test_empty_indexing():
    """Regression test for ticket 1948."""
    # Check that indexing a chararray with an empty list/array returns an
    # empty chararray instead of a chararray with a single empty string in it.
    s = np.chararray((4,))
    assert_(s[[]].size == 0)


if __name__ == "__main__":
    run_module_suite()
