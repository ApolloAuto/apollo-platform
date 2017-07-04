import os
import unittest

import imp
imp.load_source('catkin_find',
                os.path.join(os.path.dirname(__file__),
                             '..', '..', 'bin', 'catkin_find'))

from catkin_find import parse_args


class CatkinFindTest(unittest.TestCase):

    def test_parse_args_empty(self):
        args = parse_args([])
        self.assertEqual(False, args.first_only)
        self.assertIsNone(args.path)
        self.assertIsNone(args.project)
        self.assertIsNone(args.install_folders)

    def test_parse_args_folders(self):
        args = parse_args(['--etc', '--lib', '--bin'])
        self.assertEqual(False, args.first_only)
        self.assertIsNone(args.path)
        self.assertIsNone(args.project)
        self.assertEqual(['etc', 'lib', 'bin'], args.install_folders)
        args = parse_args(['--etc', '--bin', '--lib'])
        self.assertEqual(['etc', 'bin', 'lib'], args.install_folders)

    def test_parse_args_first(self):
        args = parse_args(['--first-only'])
        self.assertEqual(True, args.first_only)
        self.assertIsNone(args.path)
        self.assertIsNone(args.project)
        self.assertIsNone(args.install_folders)
