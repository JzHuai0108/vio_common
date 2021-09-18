import glob
import os
import tempfile

DEBUG = False

def setup_debug_dir(dir_name=None):
    """Creates a debug directory and required subdirectories.

    Each subdirectory contains images from a different step in the process.

    Args:
        dir_name: The directory to create.  If none is specified, a temp
        directory is created.

    Returns:
        The name of the directory that is used.
    """
    if dir_name is None:
        dir_name = tempfile.mkdtemp()
    else:
        force_mkdir(dir_name)
    print('Saving debugging files to "%s"' % dir_name)
    # For original captured images.
    force_mkdir(dir_name + '/raw', clean=True)
    # For monochrome images.
    force_mkdir(dir_name + '/mono', clean=True)
    # For contours generated from monochrome images.
    force_mkdir(dir_name + '/contour', clean=True)
    # For post-contour debugging information.
    force_mkdir(dir_name + '/scratch', clean=True)
    return dir_name


def force_mkdir(dir_name, clean=False):
    """Creates a directory if it doesn't already exist.

    Args:
        dir_name: Name of the directory to create.
        clean:    (optional) If set to true, cleans image files from the
                  directory (if it already exists).
    """
    if os.path.exists(dir_name):
        if clean:
            for image in glob.glob('%s/*.png' % dir_name):
                os.remove(image)
    else:
        os.makedirs(dir_name)

