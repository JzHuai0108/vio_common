#!/usr/bin/env python

from __future__ import print_function
import sys

import kalibr_bagcreater


def main():
    if len(sys.argv) < 3:
        print("Usage: {} <folder from which images will be recursively found> <output bag path>".format(sys.argv[0]))
        sys.exit(1)
    kalibr_bagcreater.create_rosbag_for_images_in_dir(sys.argv[1], sys.argv[2])


if __name__ == "__main__":
    main()
