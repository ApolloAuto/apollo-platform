import os
import unittest
import tempfile
import shutil

try:
    from catkin.tidy_xml import tidy_xml, _SAFE_XML_REGEX
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))

try:
    char = unichr
except NameError:
    char = chr


class TidyXmlTest(unittest.TestCase):

    def test_safe_xml_regex(self):
        for data in [char(0), char(14)]:
            self.assertIsNotNone(_SAFE_XML_REGEX.match(data))

    def test_tiny_xml(self):
        try:
            rootdir = tempfile.mkdtemp()
            not_exist_file = os.path.join(rootdir, 'not_exist')
            self.assertRaises(ValueError, tidy_xml, not_exist_file)

            utf8_file = os.path.join(rootdir, 'utf8.xml')
            with open(utf8_file, 'ab') as fhand:
                fhand.write(char(0).encode('utf8)'))
            tidy_xml(utf8_file)
            with open(utf8_file, 'r') as fhand:
                contents = fhand.read()
            self.assertEqual('?', contents)

            iso_file = os.path.join(rootdir, 'iso.xml')
            with open(iso_file, 'ab') as fhand:
                fhand.write(char(0).encode('ascii'))
            tidy_xml(iso_file)
            with open(iso_file, 'r') as fhand:
                contents = fhand.read()
            self.assertEqual('?', contents)
        finally:
            shutil.rmtree(rootdir)
