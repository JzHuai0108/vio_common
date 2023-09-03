
# iterate through all files in a folder
def main():
    if len(sys.argv) != 2:
        print('Usage: python rename_files.py folder. \nRename files to 00001.xxx')
        sys.exit(1)
    folder = sys.argv[1]

folder = '/home/username/Downloads'

for filename in os.listdir(folder):
    # get the file extension
    ext = os.path.splitext(filename)[1]
    # rename the file
    os.rename(os.path.join(folder, filename), os.path.join(folder, filename.replace(ext, '.png')))

if __name__ == '__main__':
    main()