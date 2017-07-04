import os
import unittest
import tempfile
import shutil

import imp
imp.load_source('download_checkmd5',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'cmake', 'test', 'download_checkmd5.py'))

from download_checkmd5 import download_md5, checkmd5, main


class DowloadCheckMd5Test(unittest.TestCase):

    def test_download(self):
        try:
            rootdir = tempfile.mkdtemp()
            src_file = os.path.join(rootdir, 'testfile')
            check_file = os.path.join(rootdir, 'checkfile')
            with open(src_file, 'w') as fhand:
                fhand.write('foo')
            download_md5('file://%s' % src_file, check_file)
            self.assertTrue(os.path.isfile(check_file))
        finally:
            shutil.rmtree(rootdir)

    def test_checkmd5(self):
        try:
            rootdir = tempfile.mkdtemp()
            src_file = os.path.join(rootdir, 'testfile')
            realmd5 = 'acbd18db4cc2f85cedef654fccc4a4d8'
            with open(src_file, 'w') as fhand:
                fhand.write('foo')
            result, hexdig = checkmd5(src_file, "hello")
            self.assertFalse(result)
            self.assertEqual(realmd5, hexdig)
            result, hexdig = checkmd5(src_file, realmd5)
            self.assertTrue(result)
            self.assertEqual(realmd5, hexdig)
        finally:
            shutil.rmtree(rootdir)

    def test_main(self):
        try:
            rootdir = tempfile.mkdtemp()
            src_file = os.path.join(rootdir, 'testfile')
            check_file = os.path.join(rootdir, 'checkfile')
            realmd5 = 'acbd18db4cc2f85cedef654fccc4a4d8'
            with open(src_file, 'w') as fhand:
                fhand.write('foo')
            main([os.path.join('file://localhost', src_file), check_file])
            self.assertTrue(os.path.isfile(check_file))
            os.remove(check_file)
            self.assertFalse(os.path.isfile(check_file))
            self.assertNotEqual(main([os.path.join('file://localhost', src_file), check_file, "hello"]), 0)
            main([os.path.join('file://localhost', src_file), check_file, realmd5])

        finally:
            shutil.rmtree(rootdir)
