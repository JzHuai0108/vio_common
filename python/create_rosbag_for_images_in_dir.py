#!/usr/bin/env python

from __future__ import print_function
import sys

import kalibr_bagcreator


def main():
    if len(sys.argv) < 3:
        print("Usage: {} <folder from which images will be recursively found> <output bag path> "
              "[image downscale factor, must be powers of 2]".format(sys.argv[0]))
        sys.exit(1)
    downscalefactor = 1
    if len(sys.argv) == 4:
        downscalefactor = int(sys.argv[3])
    kalibr_bagcreator.create_rosbag_for_images_in_dir(sys.argv[1], sys.argv[2], downscalefactor=downscalefactor)


if __name__ == "__main__":
    main()
