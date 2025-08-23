
import os
import zipfile
from pathlib import Path


def zip_folder(folder, exclude_filenames):
    exclude_filenames = set(exclude_filenames or [])
    zip_path = f"{folder}.zip"

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for root, dirs, files in os.walk(folder):
            # Skip entire directories
            dirs[:] = [d for d in dirs if d not in exclude_filenames]

            for file in files:
                if file in exclude_filenames:
                    continue
                abs_path = os.path.join(root, file)
                rel_path = os.path.relpath(abs_path, folder)
                zf.write(abs_path, rel_path)
    return zip_path


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Zip a folder, excluding specified files.")
    parser.add_argument("folder_path", type=str, help="Path to the folder to zip.")
    parser.add_argument("--exclude", nargs='*', default=None, help="Files to exclude from the zip.")

    args = parser.parse_args()
    zip_file_path = zip_folder(args.folder_path, args.exclude)
    print(f"Created ZIP file: {zip_file_path}")
