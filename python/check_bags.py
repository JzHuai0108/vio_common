import os
import subprocess
import sys

if __name__ == "__main__":
    """Examine status of bags listed in bagnames_file which are
    stored under bags_dir without subfolders."""  # pylint: disable=pointless-string-statement

    if len(sys.argv) < 3:
        print('Usage: {} bagnames_file bags_dir'.format(sys.argv[0]))
        sys.exit(1)
    script, bagname_file, output_dir = sys.argv

    bagname_list = []
    with open(bagname_file, 'r') as stream:
        for line in stream:
            line = line.strip()
            if line:
                bagname = os.path.basename(line)
                bagname_list.append(os.path.join(output_dir, bagname))

    print('Found #bagnames: {}'.format(len(bagname_list)))

    for bagname in bagname_list:
        # Run command with arguments and return its output as a byte string.
        try:
            status = subprocess.check_output(['rosbag', 'info', bagname],
                                             stdin=None,
                                             stderr=None,
                                             shell=False,
                                             universal_newlines=False)
            # print(status)
        except Exception as inst:
            print(inst.args)
