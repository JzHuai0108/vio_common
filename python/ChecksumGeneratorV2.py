# ChecksumGeneratorV2.py
# Python 2.7.6
# adapted from the version downloaded https://gist.github.com/vinovator/d864555d9e82d25e52fd
"""
Generate md5/sha256 checksum for all files placed under a folder
"""

import hashlib
import os
import sys


def generate_checksum(fname, chunk_size=4096, method='md5'):
    """
    Function which takes a file name and returns checksum of the file
    """
    if method == 'md5':
        hash = hashlib.md5()
    else:
        hash = hashlib.sha256()
    with open(fname, "rb") as f:
        # Read the 1st block of the file
        chunk = f.read(chunk_size)
        # Keep reading the file until the end and update hash
        while chunk:
            hash.update(chunk)
            chunk = f.read(chunk_size)

    # Return the hex checksum
    return hash.hexdigest()


if __name__ == "__main__":
    """
    Starting block of the script
    """
    nargs = len(sys.argv)
    if nargs < 4:
        print('Usage: {} src-folder filename-extension md5(sha256).\n'
              'The program will generate md5sum or sha256sum file '
              'for files with filename-extension under src-folder.'.format(sys.argv[0]))
        sys.exit(1)

    src_folder = sys.argv[1]
    extension = sys.argv[2]
    method = sys.argv[3]

    sum_dict = dict()

    # Iterate through all files under source folder
    for path, dirs, files in os.walk(src_folder):
        for file_name in files:
            if file_name.endswith(extension):
                print("Generating checksum for {}".format(file_name))
                sum = generate_checksum(
                    os.path.join(src_folder, file_name), 4096, method)
                sum_dict[file_name] = sum

                with open(os.path.join(src_folder, file_name + '.' + method), "w") as f:
                    f.write("{} {}\n".format(sum, file_name))
                    print("{} {}".format(sum, file_name))
