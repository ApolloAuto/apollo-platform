from __future__ import division, absolute_import, print_function

import sys

from numpy.testing import *
from numpy.core import *
from numpy.compat import asbytes, sixu

# Guess the UCS length for this python interpreter
if sys.version_info[:2] >= (3, 3):
    # Python 3.3 uses a flexible string representation
    ucs4 = False
    def buffer_length(arr):
        if isinstance(arr, unicode):
            arr = str(arr)
            return (sys.getsizeof(arr+"a") - sys.getsizeof(arr)) * len(arr)
        v = memoryview(arr)
        if v.shape is None:
            return len(v) * v.itemsize
        else:
            return prod(v.shape) * v.itemsize
elif sys.version_info[0] >= 3:
    import array as _array
    ucs4 = (_array.array('u').itemsize == 4)
    def buffer_length(arr):
        if isinstance(arr, unicode):
            return _array.array('u').itemsize * len(arr)
        v = memoryview(arr)
        if v.shape is None:
            return len(v) * v.itemsize
        else:
            return prod(v.shape) * v.itemsize
else:
    if len(buffer(sixu('u'))) == 4:
        ucs4 = True
    else:
        ucs4 = False
    def buffer_length(arr):
        if isinstance(arr, ndarray):
            return len(arr.data)
        return len(buffer(arr))

# In both cases below we need to make sure that the byte swapped value (as
# UCS4) is still a valid unicode:
# Value that can be represented in UCS2 interpreters
ucs2_value = sixu('\u0900')
# Value that cannot be represented in UCS2 interpreters (but can in UCS4)
ucs4_value = sixu('\U00100900')


############################################################
#    Creation tests
############################################################

class create_zeros(object):
    """Check the creation of zero-valued arrays"""

    def content_check(self, ua, ua_scalar, nbytes):

        # Check the length of the unicode base type
        self.assertTrue(int(ua.dtype.str[2:]) == self.ulen)
        # Check the length of the data buffer
        self.assertTrue(buffer_length(ua) == nbytes)
        # Small check that data in array element is ok
        self.assertTrue(ua_scalar == sixu(''))
        # Encode to ascii and double check
        self.assertTrue(ua_scalar.encode('ascii') == asbytes(''))
        # Check buffer lengths for scalars
        if ucs4:
            self.assertTrue(buffer_length(ua_scalar) == 0)
        else:
            self.assertTrue(buffer_length(ua_scalar) == 0)

    def test_zeros0D(self):
        """Check creation of 0-dimensional objects"""
        ua = zeros((), dtype='U%s' % self.ulen)
        self.content_check(ua, ua[()], 4*self.ulen)

    def test_zerosSD(self):
        """Check creation of single-dimensional objects"""
        ua = zeros((2,), dtype='U%s' % self.ulen)
        self.content_check(ua, ua[0], 4*self.ulen*2)
        self.content_check(ua, ua[1], 4*self.ulen*2)

    def test_zerosMD(self):
        """Check creation of multi-dimensional objects"""
        ua = zeros((2, 3, 4), dtype='U%s' % self.ulen)
        self.content_check(ua, ua[0, 0, 0], 4*self.ulen*2*3*4)
        self.content_check(ua, ua[-1, -1, -1], 4*self.ulen*2*3*4)


class test_create_zeros_1(create_zeros, TestCase):
    """Check the creation of zero-valued arrays (size 1)"""
    ulen = 1


class test_create_zeros_2(create_zeros, TestCase):
    """Check the creation of zero-valued arrays (size 2)"""
    ulen = 2


class test_create_zeros_1009(create_zeros, TestCase):
    """Check the creation of zero-valued arrays (size 1009)"""
    ulen = 1009


class create_values(object):
    """Check the creation of unicode arrays with values"""

    def content_check(self, ua, ua_scalar, nbytes):

        # Check the length of the unicode base type
        self.assertTrue(int(ua.dtype.str[2:]) == self.ulen)
        # Check the length of the data buffer
        self.assertTrue(buffer_length(ua) == nbytes)
        # Small check that data in array element is ok
        self.assertTrue(ua_scalar == self.ucs_value*self.ulen)
        # Encode to UTF-8 and double check
        self.assertTrue(ua_scalar.encode('utf-8') == \
                     (self.ucs_value*self.ulen).encode('utf-8'))
        # Check buffer lengths for scalars
        if ucs4:
            self.assertTrue(buffer_length(ua_scalar) == 4*self.ulen)
        else:
            if self.ucs_value == ucs4_value:
                # In UCS2, the \U0010FFFF will be represented using a
                # surrogate *pair*
                self.assertTrue(buffer_length(ua_scalar) == 2*2*self.ulen)
            else:
                # In UCS2, the \uFFFF will be represented using a
                # regular 2-byte word
                self.assertTrue(buffer_length(ua_scalar) == 2*self.ulen)

    def test_values0D(self):
        """Check creation of 0-dimensional objects with values"""
        ua = array(self.ucs_value*self.ulen, dtype='U%s' % self.ulen)
        self.content_check(ua, ua[()], 4*self.ulen)

    def test_valuesSD(self):
        """Check creation of single-dimensional objects with values"""
        ua = array([self.ucs_value*self.ulen]*2, dtype='U%s' % self.ulen)
        self.content_check(ua, ua[0], 4*self.ulen*2)
        self.content_check(ua, ua[1], 4*self.ulen*2)

    def test_valuesMD(self):
        """Check creation of multi-dimensional objects with values"""
        ua = array([[[self.ucs_value*self.ulen]*2]*3]*4, dtype='U%s' % self.ulen)
        self.content_check(ua, ua[0, 0, 0], 4*self.ulen*2*3*4)
        self.content_check(ua, ua[-1, -1, -1], 4*self.ulen*2*3*4)


class test_create_values_1_ucs2(create_values, TestCase):
    """Check the creation of valued arrays (size 1, UCS2 values)"""
    ulen = 1
    ucs_value = ucs2_value


class test_create_values_1_ucs4(create_values, TestCase):
    """Check the creation of valued arrays (size 1, UCS4 values)"""
    ulen = 1
    ucs_value = ucs4_value


class test_create_values_2_ucs2(create_values, TestCase):
    """Check the creation of valued arrays (size 2, UCS2 values)"""
    ulen = 2
    ucs_value = ucs2_value


class test_create_values_2_ucs4(create_values, TestCase):
    """Check the creation of valued arrays (size 2, UCS4 values)"""
    ulen = 2
    ucs_value = ucs4_value


class test_create_values_1009_ucs2(create_values, TestCase):
    """Check the creation of valued arrays (size 1009, UCS2 values)"""
    ulen = 1009
    ucs_value = ucs2_value


class test_create_values_1009_ucs4(create_values, TestCase):
    """Check the creation of valued arrays (size 1009, UCS4 values)"""
    ulen = 1009
    ucs_value = ucs4_value


############################################################
#    Assignment tests
############################################################

class assign_values(object):
    """Check the assignment of unicode arrays with values"""

    def content_check(self, ua, ua_scalar, nbytes):

        # Check the length of the unicode base type
        self.assertTrue(int(ua.dtype.str[2:]) == self.ulen)
        # Check the length of the data buffer
        self.assertTrue(buffer_length(ua) == nbytes)
        # Small check that data in array element is ok
        self.assertTrue(ua_scalar == self.ucs_value*self.ulen)
        # Encode to UTF-8 and double check
        self.assertTrue(ua_scalar.encode('utf-8') == \
                     (self.ucs_value*self.ulen).encode('utf-8'))
        # Check buffer lengths for scalars
        if ucs4:
            self.assertTrue(buffer_length(ua_scalar) == 4*self.ulen)
        else:
            if self.ucs_value == ucs4_value:
                # In UCS2, the \U0010FFFF will be represented using a
                # surrogate *pair*
                self.assertTrue(buffer_length(ua_scalar) == 2*2*self.ulen)
            else:
                # In UCS2, the \uFFFF will be represented using a
                # regular 2-byte word
                self.assertTrue(buffer_length(ua_scalar) == 2*self.ulen)

    def test_values0D(self):
        """Check assignment of 0-dimensional objects with values"""
        ua = zeros((), dtype='U%s' % self.ulen)
        ua[()] = self.ucs_value*self.ulen
        self.content_check(ua, ua[()], 4*self.ulen)

    def test_valuesSD(self):
        """Check assignment of single-dimensional objects with values"""
        ua = zeros((2,), dtype='U%s' % self.ulen)
        ua[0] = self.ucs_value*self.ulen
        self.content_check(ua, ua[0], 4*self.ulen*2)
        ua[1] = self.ucs_value*self.ulen
        self.content_check(ua, ua[1], 4*self.ulen*2)

    def test_valuesMD(self):
        """Check assignment of multi-dimensional objects with values"""
        ua = zeros((2, 3, 4), dtype='U%s' % self.ulen)
        ua[0, 0, 0] = self.ucs_value*self.ulen
        self.content_check(ua, ua[0, 0, 0], 4*self.ulen*2*3*4)
        ua[-1, -1, -1] = self.ucs_value*self.ulen
        self.content_check(ua, ua[-1, -1, -1], 4*self.ulen*2*3*4)


class test_assign_values_1_ucs2(assign_values, TestCase):
    """Check the assignment of valued arrays (size 1, UCS2 values)"""
    ulen = 1
    ucs_value = ucs2_value


class test_assign_values_1_ucs4(assign_values, TestCase):
    """Check the assignment of valued arrays (size 1, UCS4 values)"""
    ulen = 1
    ucs_value = ucs4_value


class test_assign_values_2_ucs2(assign_values, TestCase):
    """Check the assignment of valued arrays (size 2, UCS2 values)"""
    ulen = 2
    ucs_value = ucs2_value


class test_assign_values_2_ucs4(assign_values, TestCase):
    """Check the assignment of valued arrays (size 2, UCS4 values)"""
    ulen = 2
    ucs_value = ucs4_value


class test_assign_values_1009_ucs2(assign_values, TestCase):
    """Check the assignment of valued arrays (size 1009, UCS2 values)"""
    ulen = 1009
    ucs_value = ucs2_value


class test_assign_values_1009_ucs4(assign_values, TestCase):
    """Check the assignment of valued arrays (size 1009, UCS4 values)"""
    ulen = 1009
    ucs_value = ucs4_value



############################################################
#    Byteorder tests
############################################################

class byteorder_values:
    """Check the byteorder of unicode arrays in round-trip conversions"""

    def test_values0D(self):
        """Check byteorder of 0-dimensional objects"""
        ua = array(self.ucs_value*self.ulen, dtype='U%s' % self.ulen)
        ua2 = ua.newbyteorder()
        # This changes the interpretation of the data region (but not the
        #  actual data), therefore the returned scalars are not
        #  the same (they are byte-swapped versions of each other).
        self.assertTrue(ua[()] != ua2[()])
        ua3 = ua2.newbyteorder()
        # Arrays must be equal after the round-trip
        assert_equal(ua, ua3)

    def test_valuesSD(self):
        """Check byteorder of single-dimensional objects"""
        ua = array([self.ucs_value*self.ulen]*2, dtype='U%s' % self.ulen)
        ua2 = ua.newbyteorder()
        self.assertTrue(ua[0] != ua2[0])
        self.assertTrue(ua[-1] != ua2[-1])
        ua3 = ua2.newbyteorder()
        # Arrays must be equal after the round-trip
        assert_equal(ua, ua3)

    def test_valuesMD(self):
        """Check byteorder of multi-dimensional objects"""
        ua = array([[[self.ucs_value*self.ulen]*2]*3]*4,
                   dtype='U%s' % self.ulen)
        ua2 = ua.newbyteorder()
        self.assertTrue(ua[0, 0, 0] != ua2[0, 0, 0])
        self.assertTrue(ua[-1, -1, -1] != ua2[-1, -1, -1])
        ua3 = ua2.newbyteorder()
        # Arrays must be equal after the round-trip
        assert_equal(ua, ua3)


class test_byteorder_1_ucs2(byteorder_values, TestCase):
    """Check the byteorder in unicode (size 1, UCS2 values)"""
    ulen = 1
    ucs_value = ucs2_value


class test_byteorder_1_ucs4(byteorder_values, TestCase):
    """Check the byteorder in unicode (size 1, UCS4 values)"""
    ulen = 1
    ucs_value = ucs4_value


class test_byteorder_2_ucs2(byteorder_values, TestCase):
    """Check the byteorder in unicode (size 2, UCS2 values)"""
    ulen = 2
    ucs_value = ucs2_value


class test_byteorder_2_ucs4(byteorder_values, TestCase):
    """Check the byteorder in unicode (size 2, UCS4 values)"""
    ulen = 2
    ucs_value = ucs4_value


class test_byteorder_1009_ucs2(byteorder_values, TestCase):
    """Check the byteorder in unicode (size 1009, UCS2 values)"""
    ulen = 1009
    ucs_value = ucs2_value


class test_byteorder_1009_ucs4(byteorder_values, TestCase):
    """Check the byteorder in unicode (size 1009, UCS4 values)"""
    ulen = 1009
    ucs_value = ucs4_value


if __name__ == "__main__":
    run_module_suite()
