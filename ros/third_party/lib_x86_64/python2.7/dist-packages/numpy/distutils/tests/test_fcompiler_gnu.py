from __future__ import division, absolute_import, print_function

from numpy.testing import *

import numpy.distutils.fcompiler

g77_version_strings = [
    ('GNU Fortran 0.5.25 20010319 (prerelease)', '0.5.25'),
    ('GNU Fortran (GCC 3.2) 3.2 20020814 (release)', '3.2'),
    ('GNU Fortran (GCC) 3.3.3 20040110 (prerelease) (Debian)', '3.3.3'),
    ('GNU Fortran (GCC) 3.3.3 (Debian 20040401)', '3.3.3'),
    ('GNU Fortran (GCC 3.2.2 20030222 (Red Hat Linux 3.2.2-5)) 3.2.2'
       ' 20030222 (Red Hat Linux 3.2.2-5)', '3.2.2'),
]

gfortran_version_strings = [
    ('GNU Fortran 95 (GCC 4.0.3 20051023 (prerelease) (Debian 4.0.2-3))',
     '4.0.3'),
    ('GNU Fortran 95 (GCC) 4.1.0', '4.1.0'),
    ('GNU Fortran 95 (GCC) 4.2.0 20060218 (experimental)', '4.2.0'),
    ('GNU Fortran (GCC) 4.3.0 20070316 (experimental)', '4.3.0'),
    ('GNU Fortran (rubenvb-4.8.0) 4.8.0', '4.8.0'),
]

class TestG77Versions(TestCase):
    def test_g77_version(self):
        fc = numpy.distutils.fcompiler.new_fcompiler(compiler='gnu')
        for vs, version in g77_version_strings:
            v = fc.version_match(vs)
            assert_(v == version, (vs, v))

    def test_not_g77(self):
        fc = numpy.distutils.fcompiler.new_fcompiler(compiler='gnu')
        for vs, _ in gfortran_version_strings:
            v = fc.version_match(vs)
            assert_(v is None, (vs, v))

class TestGortranVersions(TestCase):
    def test_gfortran_version(self):
        fc = numpy.distutils.fcompiler.new_fcompiler(compiler='gnu95')
        for vs, version in gfortran_version_strings:
            v = fc.version_match(vs)
            assert_(v == version, (vs, v))

    def test_not_gfortran(self):
        fc = numpy.distutils.fcompiler.new_fcompiler(compiler='gnu95')
        for vs, _ in g77_version_strings:
            v = fc.version_match(vs)
            assert_(v is None, (vs, v))


if __name__ == '__main__':
    run_module_suite()
