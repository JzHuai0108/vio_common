#!/usr/bin/env python3
from itertools import islice
import os
import subprocess
import sys
import requests


"""First crawl the webpage to extract all links and then download files sequentially.
To install requirements,
pip install BeautifulSoup4
pip install html5lib
"""  # pylint: disable=pointless-string-statement


def has_all_keys(url, key_list):
    return all(key in url for key in key_list)


def get_file_links(archive_url, key_list):
    """create response object"""
    from bs4 import BeautifulSoup
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
        try:
            if has_all_keys(link['href'], key_list):
                if dir_url in link['href']:
                    file_links_out.append(link['href'])
                else:
                    file_links_out.append(dir_url + link['href'])
        except KeyError:
            print("href key not found in link:{}".format(link))

    return file_links_out


def subprocess_cmd(command):
    process = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    proc_stdout = process.communicate()[0].strip()
    print(proc_stdout)


def subprocess_parallel(commands, max_workers=2):
    """
    https://stackoverflow.com/questions/14533458/python-threading-multiple-bash-subprocesses
    max_workers = 2  # no more than 2 concurrent processes
    """
    processes = (subprocess.Popen(cmd, shell=True) for cmd in commands)
    running_processes = list(islice(processes, max_workers))  # start new processes
    while running_processes:
        for i, process in enumerate(running_processes):
            if process.poll() is not None:  # the process has finished
                running_processes[i] = next(processes, None)  # start new process
                if running_processes[i] is None: # no new processes
                    del running_processes[i]
                    break


def wget_file_parallel(links, path_prefix):
    print('Downloading files to {}'.format(path_prefix))
    commands = []
    for link in links:
        cmd = 'cd {}; wget {};'.format(path_prefix, link)
        commands.append(cmd)
    subprocess_parallel(commands, 10)
    print('Downloading finishes!')


def wget_file_series(links, path_prefix):
    # create wget script
    link_file = os.path.join(path_prefix, 'links.txt')
    with open(link_file, 'w') as stream:
        stream.write('{}'.format('\n'.join(links)))

    # invoke the bash script
    print('Downloading files to {}'.format(path_prefix))
    command = 'cd {}; wget -i {};'.format(path_prefix, link_file)
    subprocess_cmd(command)
    print('Downloading finishes!')


if __name__ == "__main__":
    nargs = len(sys.argv)
    if nargs < 3:
        print('Usage: {} webpage_url comma_separated_filename_keys('
              'eg., "zip,indoor") save_to_path'.format(sys.argv[0]))
        sys.exit(1)

    webpage_url = sys.argv[1]
    filename_keys = sys.argv[2]
    save_to_path = sys.argv[3]

    # getting all file links
    filename_key_list = filename_keys.split(',')
    file_links = get_file_links(webpage_url, filename_key_list)
    print('Found links:\n{}'.format('\n'.join(file_links)))

    # download all files
    wget_file_parallel(file_links, save_to_path)
