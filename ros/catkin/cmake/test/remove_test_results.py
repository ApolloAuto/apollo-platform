#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import sys


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Remove all test result files found within a given path.')
    parser.add_argument('path', help='The path to recursively process')
    args = parser.parse_args(argv)

    print("Removing test result files from '%s'" % os.path.abspath(args.path))
    if not os.path.exists(args.path):
        return 0

    for dirpath, dirnames, filenames in os.walk(args.path):
        # do not recurse into folders starting with a dot
        dirnames[:] = [d for d in dirnames if not d.startswith('.')]
        for filename in [f for f in filenames if f.endswith('.xml')]:
            filename_abs = os.path.join(dirpath, filename)
            print("- removing '%s'" % filename_abs)
            os.remove(filename_abs)

    return 0


if __name__ == '__main__':
    sys.exit(main())
