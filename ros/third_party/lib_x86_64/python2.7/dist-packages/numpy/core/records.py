"""
Record Arrays
=============
Record arrays expose the fields of structured arrays as properties.

Most commonly, ndarrays contain elements of a single type, e.g. floats, integers,
bools etc.  However, it is possible for elements to be combinations of these,
such as::

  >>> a = np.array([(1, 2.0), (1, 2.0)], dtype=[('x', int), ('y', float)])
  >>> a
  array([(1, 2.0), (1, 2.0)],
        dtype=[('x', '<i4'), ('y', '<f8')])

Here, each element consists of two fields: x (and int), and y (a float).
This is known as a structured array.  The different fields are analogous
to columns in a spread-sheet.  The different fields can be accessed as
one would a dictionary::

  >>> a['x']
  array([1, 1])

  >>> a['y']
  array([ 2.,  2.])

Record arrays allow us to access fields as properties::

  >>> ar = a.view(np.recarray)

  >>> ar.x
  array([1, 1])

  >>> ar.y
  array([ 2.,  2.])

"""
from __future__ import division, absolute_import, print_function

import sys
import os

from . import numeric as sb
from .defchararray import chararray
from . import numerictypes as nt
from numpy.compat import isfileobj, bytes, long

# All of the functions allow formats to be a dtype
__all__ = ['record', 'recarray', 'format_parser']


ndarray = sb.ndarray

_byteorderconv = {'b':'>',
                  'l':'<',
                  'n':'=',
                  'B':'>',
                  'L':'<',
                  'N':'=',
                  'S':'s',
                  's':'s',
                  '>':'>',
                  '<':'<',
                  '=':'=',
                  '|':'|',
                  'I':'|',
                  'i':'|'}

# formats regular expression
# allows multidimension spec with a tuple syntax in front
# of the letter code '(2,3)f4' and ' (  2 ,  3  )  f4  '
# are equally allowed

numfmt = nt.typeDict
_typestr = nt._typestr

def find_duplicate(list):
    """Find duplication in a list, return a list of duplicated elements"""
    dup = []
    for i in range(len(list)):
        if (list[i] in list[i + 1:]):
            if (list[i] not in dup):
                dup.append(list[i])
    return dup

class format_parser:
    """
    Class to convert formats, names, titles description to a dtype.

    After constructing the format_parser object, the dtype attribute is
    the converted data-type:
    ``dtype = format_parser(formats, names, titles).dtype``

    Attributes
    ----------
    dtype : dtype
        The converted data-type.

    Parameters
    ----------
    formats : str or list of str
        The format description, either specified as a string with
        comma-separated format descriptions in the form ``'f8, i4, a5'``, or
        a list of format description strings  in the form
        ``['f8', 'i4', 'a5']``.
    names : str or list/tuple of str
        The field names, either specified as a comma-separated string in the
        form ``'col1, col2, col3'``, or as a list or tuple of strings in the
        form ``['col1', 'col2', 'col3']``.
        An empty list can be used, in that case default field names
        ('f0', 'f1', ...) are used.
    titles : sequence
        Sequence of title strings. An empty list can be used to leave titles
        out.
    aligned : bool, optional
        If True, align the fields by padding as the C-compiler would.
        Default is False.
    byteorder : str, optional
        If specified, all the fields will be changed to the
        provided byte-order.  Otherwise, the default byte-order is
        used. For all available string specifiers, see `dtype.newbyteorder`.

    See Also
    --------
    dtype, typename, sctype2char

    Examples
    --------
    >>> np.format_parser(['f8', 'i4', 'a5'], ['col1', 'col2', 'col3'],
    ...                  ['T1', 'T2', 'T3']).dtype
    dtype([(('T1', 'col1'), '<f8'), (('T2', 'col2'), '<i4'),
           (('T3', 'col3'), '|S5')])

    `names` and/or `titles` can be empty lists. If `titles` is an empty list,
    titles will simply not appear. If `names` is empty, default field names
    will be used.

    >>> np.format_parser(['f8', 'i4', 'a5'], ['col1', 'col2', 'col3'],
    ...                  []).dtype
    dtype([('col1', '<f8'), ('col2', '<i4'), ('col3', '|S5')])
    >>> np.format_parser(['f8', 'i4', 'a5'], [], []).dtype
    dtype([('f0', '<f8'), ('f1', '<i4'), ('f2', '|S5')])

    """
    def __init__(self, formats, names, titles, aligned=False, byteorder=None):
        self._parseFormats(formats, aligned)
        self._setfieldnames(names, titles)
        self._createdescr(byteorder)
        self.dtype = self._descr

    def _parseFormats(self, formats, aligned=0):
        """ Parse the field formats """

        if formats is None:
            raise ValueError("Need formats argument")
        if isinstance(formats, list):
            if len(formats) < 2:
                formats.append('')
            formats = ','.join(formats)
        dtype = sb.dtype(formats, aligned)
        fields = dtype.fields
        if fields is None:
            dtype = sb.dtype([('f1', dtype)], aligned)
            fields = dtype.fields
        keys = dtype.names
        self._f_formats = [fields[key][0] for key in keys]
        self._offsets = [fields[key][1] for key in keys]
        self._nfields = len(keys)

    def _setfieldnames(self, names, titles):
        """convert input field names into a list and assign to the _names
        attribute """

        if (names):
            if (type(names) in [list, tuple]):
                pass
            elif isinstance(names, str):
                names = names.split(',')
            else:
                raise NameError("illegal input names %s" % repr(names))

            self._names = [n.strip() for n in names[:self._nfields]]
        else:
            self._names = []

        # if the names are not specified, they will be assigned as
        #  "f0, f1, f2,..."
        # if not enough names are specified, they will be assigned as "f[n],
        # f[n+1],..." etc. where n is the number of specified names..."
        self._names += ['f%d' % i for i in range(len(self._names),
                                                 self._nfields)]
        # check for redundant names
        _dup = find_duplicate(self._names)
        if _dup:
            raise ValueError("Duplicate field names: %s" % _dup)

        if (titles):
            self._titles = [n.strip() for n in titles[:self._nfields]]
        else:
            self._titles = []
            titles = []

        if (self._nfields > len(titles)):
            self._titles += [None] * (self._nfields - len(titles))

    def _createdescr(self, byteorder):
        descr = sb.dtype({'names':self._names,
                          'formats':self._f_formats,
                          'offsets':self._offsets,
                          'titles':self._titles})
        if (byteorder is not None):
            byteorder = _byteorderconv[byteorder[0]]
            descr = descr.newbyteorder(byteorder)

        self._descr = descr

class record(nt.void):
    """A data-type scalar that allows field access as attribute lookup.
    """
    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return str(self.item())

    def __getattribute__(self, attr):
        if attr in ['setfield', 'getfield', 'dtype']:
            return nt.void.__getattribute__(self, attr)
        try:
            return nt.void.__getattribute__(self, attr)
        except AttributeError:
            pass
        fielddict = nt.void.__getattribute__(self, 'dtype').fields
        res = fielddict.get(attr, None)
        if res:
            obj = self.getfield(*res[:2])
            # if it has fields return a recarray,
            # if it's a string ('SU') return a chararray
            # otherwise return the object
            try:
                dt = obj.dtype
            except AttributeError:
                return obj
            if dt.fields:
                return obj.view(obj.__class__)
            if dt.char in 'SU':
                return obj.view(chararray)
            return obj
        else:
            raise AttributeError("'record' object has no "
                    "attribute '%s'" % attr)


    def __setattr__(self, attr, val):
        if attr in ['setfield', 'getfield', 'dtype']:
            raise AttributeError("Cannot set '%s' attribute" % attr)
        fielddict = nt.void.__getattribute__(self, 'dtype').fields
        res = fielddict.get(attr, None)
        if res:
            return self.setfield(val, *res[:2])
        else:
            if getattr(self, attr, None):
                return nt.void.__setattr__(self, attr, val)
            else:
                raise AttributeError("'record' object has no "
                        "attribute '%s'" % attr)

    def pprint(self):
        """Pretty-print all fields."""
        # pretty-print all fields
        names = self.dtype.names
        maxlen = max([len(name) for name in names])
        rows = []
        fmt = '%% %ds: %%s' % maxlen
        for name in names:
            rows.append(fmt % (name, getattr(self, name)))
        return "\n".join(rows)

# The recarray is almost identical to a standard array (which supports
#   named fields already)  The biggest difference is that it can use
#   attribute-lookup to find the fields and it is constructed using
#   a record.

# If byteorder is given it forces a particular byteorder on all
#  the fields (and any subfields)

class recarray(ndarray):
    """
    Construct an ndarray that allows field access using attributes.

    Arrays may have a data-types containing fields, analogous
    to columns in a spread sheet.  An example is ``[(x, int), (y, float)]``,
    where each entry in the array is a pair of ``(int, float)``.  Normally,
    these attributes are accessed using dictionary lookups such as ``arr['x']``
    and ``arr['y']``.  Record arrays allow the fields to be accessed as members
    of the array, using ``arr.x`` and ``arr.y``.

    Parameters
    ----------
    shape : tuple
        Shape of output array.
    dtype : data-type, optional
        The desired data-type.  By default, the data-type is determined
        from `formats`, `names`, `titles`, `aligned` and `byteorder`.
    formats : list of data-types, optional
        A list containing the data-types for the different columns, e.g.
        ``['i4', 'f8', 'i4']``.  `formats` does *not* support the new
        convention of using types directly, i.e. ``(int, float, int)``.
        Note that `formats` must be a list, not a tuple.
        Given that `formats` is somewhat limited, we recommend specifying
        `dtype` instead.
    names : tuple of str, optional
        The name of each column, e.g. ``('x', 'y', 'z')``.
    buf : buffer, optional
        By default, a new array is created of the given shape and data-type.
        If `buf` is specified and is an object exposing the buffer interface,
        the array will use the memory from the existing buffer.  In this case,
        the `offset` and `strides` keywords are available.

    Other Parameters
    ----------------
    titles : tuple of str, optional
        Aliases for column names.  For example, if `names` were
        ``('x', 'y', 'z')`` and `titles` is
        ``('x_coordinate', 'y_coordinate', 'z_coordinate')``, then
        ``arr['x']`` is equivalent to both ``arr.x`` and ``arr.x_coordinate``.
    byteorder : {'<', '>', '='}, optional
        Byte-order for all fields.
    aligned : bool, optional
        Align the fields in memory as the C-compiler would.
    strides : tuple of ints, optional
        Buffer (`buf`) is interpreted according to these strides (strides
        define how many bytes each array element, row, column, etc.
        occupy in memory).
    offset : int, optional
        Start reading buffer (`buf`) from this offset onwards.
    order : {'C', 'F'}, optional
        Row-major or column-major order.

    Returns
    -------
    rec : recarray
        Empty array of the given shape and type.

    See Also
    --------
    rec.fromrecords : Construct a record array from data.
    record : fundamental data-type for `recarray`.
    format_parser : determine a data-type from formats, names, titles.

    Notes
    -----
    This constructor can be compared to ``empty``: it creates a new record
    array but does not fill it with data.  To create a record array from data,
    use one of the following methods:

    1. Create a standard ndarray and convert it to a record array,
       using ``arr.view(np.recarray)``
    2. Use the `buf` keyword.
    3. Use `np.rec.fromrecords`.

    Examples
    --------
    Create an array with two fields, ``x`` and ``y``:

    >>> x = np.array([(1.0, 2), (3.0, 4)], dtype=[('x', float), ('y', int)])
    >>> x
    array([(1.0, 2), (3.0, 4)],
          dtype=[('x', '<f8'), ('y', '<i4')])

    >>> x['x']
    array([ 1.,  3.])

    View the array as a record array:

    >>> x = x.view(np.recarray)

    >>> x.x
    array([ 1.,  3.])

    >>> x.y
    array([2, 4])

    Create a new, empty record array:

    >>> np.recarray((2,),
    ... dtype=[('x', int), ('y', float), ('z', int)]) #doctest: +SKIP
    rec.array([(-1073741821, 1.2249118382103472e-301, 24547520),
           (3471280, 1.2134086255804012e-316, 0)],
          dtype=[('x', '<i4'), ('y', '<f8'), ('z', '<i4')])

    """
    def __new__(subtype, shape, dtype=None, buf=None, offset=0, strides=None,
                formats=None, names=None, titles=None,
                byteorder=None, aligned=False, order='C'):

        if dtype is not None:
            descr = sb.dtype(dtype)
        else:
            descr = format_parser(formats, names, titles, aligned, byteorder)._descr

        if buf is None:
            self = ndarray.__new__(subtype, shape, (record, descr), order=order)
        else:
            self = ndarray.__new__(subtype, shape, (record, descr),
                                      buffer=buf, offset=offset,
                                      strides=strides, order=order)
        return self

    def __getattribute__(self, attr):
        try:
            return object.__getattribute__(self, attr)
        except AttributeError: # attr must be a fieldname
            pass
        fielddict = ndarray.__getattribute__(self, 'dtype').fields
        try:
            res = fielddict[attr][:2]
        except (TypeError, KeyError):
            raise AttributeError("record array has no attribute %s" % attr)
        obj = self.getfield(*res)
        # if it has fields return a recarray, otherwise return
        # normal array
        if obj.dtype.fields:
            return obj
        if obj.dtype.char in 'SU':
            return obj.view(chararray)
        return obj.view(ndarray)

# Save the dictionary
#  If the attr is a field name and not in the saved dictionary
#  Undo any "setting" of the attribute and do a setfield
# Thus, you can't create attributes on-the-fly that are field names.

    def __setattr__(self, attr, val):
        newattr = attr not in self.__dict__
        try:
            ret = object.__setattr__(self, attr, val)
        except:
            fielddict = ndarray.__getattribute__(self, 'dtype').fields or {}
            if attr not in fielddict:
                exctype, value = sys.exc_info()[:2]
                raise exctype(value)
        else:
            fielddict = ndarray.__getattribute__(self, 'dtype').fields or {}
            if attr not in fielddict:
                return ret
            if newattr:         # We just added this one
                try:            #  or this setattr worked on an internal
                                #  attribute.
                    object.__delattr__(self, attr)
                except:
                    return ret
        try:
            res = fielddict[attr][:2]
        except (TypeError, KeyError):
            raise AttributeError("record array has no attribute %s" % attr)
        return self.setfield(val, *res)

    def __getitem__(self, indx):
        obj = ndarray.__getitem__(self, indx)
        if (isinstance(obj, ndarray) and obj.dtype.isbuiltin):
            return obj.view(ndarray)
        return obj

    def __repr__(self) :
        ret = ndarray.__repr__(self)
        return ret.replace("recarray", "rec.array", 1)

    def field(self, attr, val=None):
        if isinstance(attr, int):
            names = ndarray.__getattribute__(self, 'dtype').names
            attr = names[attr]

        fielddict = ndarray.__getattribute__(self, 'dtype').fields

        res = fielddict[attr][:2]

        if val is None:
            obj = self.getfield(*res)
            if obj.dtype.fields:
                return obj
            if obj.dtype.char in 'SU':
                return obj.view(chararray)
            return obj.view(ndarray)
        else:
            return self.setfield(val, *res)

    def view(self, dtype=None, type=None):
        if dtype is None:
            return ndarray.view(self, type)
        elif type is None:
            try:
                if issubclass(dtype, ndarray):
                    return ndarray.view(self, dtype)
            except TypeError:
                pass
            dtype = sb.dtype(dtype)
            if dtype.fields is None:
                return self.__array__().view(dtype)
            return ndarray.view(self, dtype)
        else:
            return ndarray.view(self, dtype, type)


def fromarrays(arrayList, dtype=None, shape=None, formats=None,
               names=None, titles=None, aligned=False, byteorder=None):
    """ create a record array from a (flat) list of arrays

    >>> x1=np.array([1,2,3,4])
    >>> x2=np.array(['a','dd','xyz','12'])
    >>> x3=np.array([1.1,2,3,4])
    >>> r = np.core.records.fromarrays([x1,x2,x3],names='a,b,c')
    >>> print r[1]
    (2, 'dd', 2.0)
    >>> x1[1]=34
    >>> r.a
    array([1, 2, 3, 4])
    """

    arrayList = [sb.asarray(x) for x in arrayList]

    if shape is None or shape == 0:
        shape = arrayList[0].shape

    if isinstance(shape, int):
        shape = (shape,)

    if formats is None and dtype is None:
        # go through each object in the list to see if it is an ndarray
        # and determine the formats.
        formats = ''
        for obj in arrayList:
            if not isinstance(obj, ndarray):
                raise ValueError("item in the array list must be an ndarray.")
            formats += _typestr[obj.dtype.type]
            if issubclass(obj.dtype.type, nt.flexible):
                formats += repr(obj.itemsize)
            formats += ','
        formats = formats[:-1]

    if dtype is not None:
        descr = sb.dtype(dtype)
        _names = descr.names
    else:
        parsed = format_parser(formats, names, titles, aligned, byteorder)
        _names = parsed._names
        descr = parsed._descr

    # Determine shape from data-type.
    if len(descr) != len(arrayList):
        raise ValueError("mismatch between the number of fields "
                "and the number of arrays")

    d0 = descr[0].shape
    nn = len(d0)
    if nn > 0:
        shape = shape[:-nn]

    for k, obj in enumerate(arrayList):
        nn = len(descr[k].shape)
        testshape = obj.shape[:len(obj.shape) - nn]
        if testshape != shape:
            raise ValueError("array-shape mismatch in array %d" % k)

    _array = recarray(shape, descr)

    # populate the record array (makes a copy)
    for i in range(len(arrayList)):
        _array[_names[i]] = arrayList[i]

    return _array

# shape must be 1-d if you use list of lists...
def fromrecords(recList, dtype=None, shape=None, formats=None, names=None,
                titles=None, aligned=False, byteorder=None):
    """ create a recarray from a list of records in text form

        The data in the same field can be heterogeneous, they will be promoted
        to the highest data type.  This method is intended for creating
        smaller record arrays.  If used to create large array without formats
        defined

        r=fromrecords([(2,3.,'abc')]*100000)

        it can be slow.

        If formats is None, then this will auto-detect formats. Use list of
        tuples rather than list of lists for faster processing.

    >>> r=np.core.records.fromrecords([(456,'dbe',1.2),(2,'de',1.3)],
    ... names='col1,col2,col3')
    >>> print r[0]
    (456, 'dbe', 1.2)
    >>> r.col1
    array([456,   2])
    >>> r.col2
    chararray(['dbe', 'de'],
          dtype='|S3')
    >>> import pickle
    >>> print pickle.loads(pickle.dumps(r))
    [(456, 'dbe', 1.2) (2, 'de', 1.3)]
    """

    nfields = len(recList[0])
    if formats is None and dtype is None:  # slower
        obj = sb.array(recList, dtype=object)
        arrlist = [sb.array(obj[..., i].tolist()) for i in range(nfields)]
        return fromarrays(arrlist, formats=formats, shape=shape, names=names,
                          titles=titles, aligned=aligned, byteorder=byteorder)

    if dtype is not None:
        descr = sb.dtype((record, dtype))
    else:
        descr = format_parser(formats, names, titles, aligned, byteorder)._descr

    try:
        retval = sb.array(recList, dtype=descr)
    except TypeError:  # list of lists instead of list of tuples
        if (shape is None or shape == 0):
            shape = len(recList)
        if isinstance(shape, (int, long)):
            shape = (shape,)
        if len(shape) > 1:
            raise ValueError("Can only deal with 1-d array.")
        _array = recarray(shape, descr)
        for k in range(_array.size):
            _array[k] = tuple(recList[k])
        return _array
    else:
        if shape is not None and retval.shape != shape:
            retval.shape = shape

    res = retval.view(recarray)

    return res


def fromstring(datastring, dtype=None, shape=None, offset=0, formats=None,
               names=None, titles=None, aligned=False, byteorder=None):
    """ create a (read-only) record array from binary data contained in
    a string"""


    if dtype is None and formats is None:
        raise ValueError("Must have dtype= or formats=")

    if dtype is not None:
        descr = sb.dtype(dtype)
    else:
        descr = format_parser(formats, names, titles, aligned, byteorder)._descr

    itemsize = descr.itemsize
    if (shape is None or shape == 0 or shape == -1):
        shape = (len(datastring) - offset) / itemsize

    _array = recarray(shape, descr, buf=datastring, offset=offset)
    return _array

def get_remaining_size(fd):
    try:
        fn = fd.fileno()
    except AttributeError:
        return os.path.getsize(fd.name) - fd.tell()
    st = os.fstat(fn)
    size = st.st_size - fd.tell()
    return size

def fromfile(fd, dtype=None, shape=None, offset=0, formats=None,
             names=None, titles=None, aligned=False, byteorder=None):
    """Create an array from binary file data

    If file is a string then that file is opened, else it is assumed
    to be a file object.

    >>> from tempfile import TemporaryFile
    >>> a = np.empty(10,dtype='f8,i4,a5')
    >>> a[5] = (0.5,10,'abcde')
    >>>
    >>> fd=TemporaryFile()
    >>> a = a.newbyteorder('<')
    >>> a.tofile(fd)
    >>>
    >>> fd.seek(0)
    >>> r=np.core.records.fromfile(fd, formats='f8,i4,a5', shape=10,
    ... byteorder='<')
    >>> print r[5]
    (0.5, 10, 'abcde')
    >>> r.shape
    (10,)
    """

    if (shape is None or shape == 0):
        shape = (-1,)
    elif isinstance(shape, (int, long)):
        shape = (shape,)

    name = 0
    if isinstance(fd, str):
        name = 1
        fd = open(fd, 'rb')
    if (offset > 0):
        fd.seek(offset, 1)
    size = get_remaining_size(fd)

    if dtype is not None:
        descr = sb.dtype(dtype)
    else:
        descr = format_parser(formats, names, titles, aligned, byteorder)._descr

    itemsize = descr.itemsize

    shapeprod = sb.array(shape).prod()
    shapesize = shapeprod * itemsize
    if shapesize < 0:
        shape = list(shape)
        shape[ shape.index(-1) ] = size / -shapesize
        shape = tuple(shape)
        shapeprod = sb.array(shape).prod()

    nbytes = shapeprod * itemsize

    if nbytes > size:
        raise ValueError(
                "Not enough bytes left in file for specified shape and type")

    # create the array
    _array = recarray(shape, descr)
    nbytesread = fd.readinto(_array.data)
    if nbytesread != nbytes:
        raise IOError("Didn't read as many bytes as expected")
    if name:
        fd.close()

    return _array

def array(obj, dtype=None, shape=None, offset=0, strides=None, formats=None,
          names=None, titles=None, aligned=False, byteorder=None, copy=True):
    """Construct a record array from a wide-variety of objects.
    """

    if (isinstance(obj, (type(None), str)) or isfileobj(obj)) \
           and (formats is None) \
           and (dtype is None):
        raise ValueError("Must define formats (or dtype) if object is "\
                         "None, string, or an open file")

    kwds = {}
    if dtype is not None:
        dtype = sb.dtype(dtype)
    elif formats is not None:
        dtype = format_parser(formats, names, titles,
                              aligned, byteorder)._descr
    else:
        kwds = {'formats': formats,
                'names' : names,
                'titles' : titles,
                'aligned' : aligned,
                'byteorder' : byteorder
                }

    if obj is None:
        if shape is None:
            raise ValueError("Must define a shape if obj is None")
        return recarray(shape, dtype, buf=obj, offset=offset, strides=strides)

    elif isinstance(obj, bytes):
        return fromstring(obj, dtype, shape=shape, offset=offset, **kwds)

    elif isinstance(obj, (list, tuple)):
        if isinstance(obj[0], (tuple, list)):
            return fromrecords(obj, dtype=dtype, shape=shape, **kwds)
        else:
            return fromarrays(obj, dtype=dtype, shape=shape, **kwds)

    elif isinstance(obj, recarray):
        if dtype is not None and (obj.dtype != dtype):
            new = obj.view(dtype)
        else:
            new = obj
        if copy:
            new = new.copy()
        return new

    elif isfileobj(obj):
        return fromfile(obj, dtype=dtype, shape=shape, offset=offset)

    elif isinstance(obj, ndarray):
        if dtype is not None and (obj.dtype != dtype):
            new = obj.view(dtype)
        else:
            new = obj
        if copy:
            new = new.copy()
        res = new.view(recarray)
        if issubclass(res.dtype.type, nt.void):
            res.dtype = sb.dtype((record, res.dtype))
        return res

    else:
        interface = getattr(obj, "__array_interface__", None)
        if interface is None or not isinstance(interface, dict):
            raise ValueError("Unknown input type")
        obj = sb.array(obj)
        if dtype is not None and (obj.dtype != dtype):
            obj = obj.view(dtype)
        res = obj.view(recarray)
        if issubclass(res.dtype.type, nt.void):
            res.dtype = sb.dtype((record, res.dtype))
        return res
