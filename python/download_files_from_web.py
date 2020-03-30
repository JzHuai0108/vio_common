#!/usr/bin/env python
import os
import subprocess
import sys
import requests
from bs4 import BeautifulSoup
"""URL of the archive web-page which provides link to
all file lectures. It would have been tiring to
download each file manually.
In this example, we first crawl the webpage to extract
all the links and then download files."""  # pylint: disable=pointless-string-statement


def get_file_links(archive_url, file_ext_in):
    """create response object"""
    r = requests.get(archive_url)

    # create beautiful-soup object
    soup = BeautifulSoup(r.content, 'html5lib')

    # find all links on web-page
    links = soup.findAll('a')

    # filter the link sending with .mp4
    file_links_out = []
    dot_index = archive_url.rfind('.')
    slash_index = archive_url.rfind('/')
    if slash_index == -1:
        raise Exception('No slash found in URL:{}'.format(archive_url))
    dir_url = archive_url
    if dot_index != -1:
        if dot_index > slash_index:
            dir_url = dir_url[:slash_index + 1]

    for link in links:
        if link['href'].endswith(file_ext_in):
            file_links_out.append(dir_url + link['href'])

    return file_links_out


def subprocess_cmd(command):
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    proc_stdout = process.communicate()[0].strip()
    print(proc_stdout)


def wget_file_series(file_links_in, save_to_path_in):
    # create the bash script
    bash_script = os.path.join(save_to_path_in, 'download.sh')
    with open(bash_script, 'w') as stream:
        stream.write('{}'.format('\n'.join(file_links_in)))

    # invoke the bash script
    print('Downloading files to {}'.format(save_to_path_in))
    command = 'cd {}; wget -i {};'.format(save_to_path_in, bash_script)
    subprocess_cmd(command)
    print('Downloading finishes!')


if __name__ == "__main__":
    nargs = len(sys.argv)
    if nargs < 3:
        print('Usage: {} webpage_url file_ext(eg., "zip" or "indoor.zip") '
              'save_to_path'.format(sys.argv[0]))
        sys.exit(1)

    webpage_url = sys.argv[1]
    file_ext = sys.argv[2]
    save_to_path = sys.argv[3]

    # getting all file links
    file_links = get_file_links(webpage_url, file_ext)
    print('Found links:\n{}'.format('\n'.join(file_links)))

    # download all files
    wget_file_series(file_links, save_to_path)
