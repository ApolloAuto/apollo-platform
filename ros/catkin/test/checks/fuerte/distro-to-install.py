#!/usr/bin/env python

import rospkg.distro
import sys

def go(argv):
  if len(argv) != 2:
    print 'wrong number args'
    sys.exit(1)

  d=rospkg.distro.load_distro(argv[1])
  ri=rospkg.distro.distro_to_rosinstall(d, 'release')
  print ri

if __name__ == '__main__':
  go(sys.argv)

