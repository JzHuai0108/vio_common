import os

import download_files_from_web
import sys

output_path = sys.argv[1]
rooturl="https://data.ciirc.cvut.cz/public/projects/2020VisualLocalization"

datasets = ["Aachen-Day-Night",
            "CMU-Seasons",
            "Cross-Seasons-Correspondence",
            "Extended-CMU-Seasons",
            "InLoc",
            "InLocCIIRC",
            "RobotCar-Seasons",
            "carltoft",
            "kapture"]

links = [os.path.join(rooturl, name) for name in datasets]

print('Downloading {} files from {}'.format(len(links), rooturl))
for l, link in enumerate(links):
    print("{}:{}".format(l, link))

print('Downloading files to {}'.format(output_path))
commands = []
for link in links:
    cmd = 'cd {}; wget -c -r -np -R "index.html*" {};'.format(output_path, link)
    commands.append(cmd)
download_files_from_web.subprocess_parallel(commands, 10)
print('Downloading finishes!')
