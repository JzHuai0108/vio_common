import os
import subprocess
import sys

if __name__ == "__main__":
    """Examine status of bags under bags_dir."""

    if len(sys.argv) < 2:
        print('Usage: {} bags_dir'.format(sys.argv[0]))
        sys.exit(1)
    _, bags_dir = sys.argv

    bagname_list = []
    for root, dirs, files in os.walk(bags_dir, topdown=False):
        for name in files:
            if name.endswith('.bag'):
                bagname_list.append(os.path.join(root, name))

    print('Found #bags: {}'.format(len(bagname_list)))
    for i, bag in enumerate(bagname_list):
        print("{}: {}".format(i, bag))

    for i, bagname in enumerate(bagname_list):
        # Run command with arguments and return its output as a byte string.
        try:
            status = subprocess.check_output(['rosbag', 'info', bagname],
                                             stdin=None,
                                             stderr=None,
                                             shell=False,
                                             universal_newlines=False)
            print("{} {}: OK".format(i, bagname))
        except Exception as inst:
            print("{} {}: {}".format(i, bagname, inst.args))

