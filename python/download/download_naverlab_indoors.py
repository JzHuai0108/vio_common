import sys
import download_files_from_web

"""
NAVER LABS - Indoor Localization Dataset
Dataset Download >> https://www.naverlabs.com/en/storyDetail/211

Indoor localization dataset v1.0 description
https://www.naverlabs.com/en/storyDetail/194

dataset specs and conversion tools
https://github.com/naver/kapture
"""


def generate_links():
    rooturl = "https://challenge.naverlabs.com/kapture"
    datanames = {"GangnamStation_B1", "GangnamStation_B2", "HyundaiDepartmentStore_1F",
                 "HyundaiDepartmentStore_4F", "HyundaiDepartmentStore_B1"}
    datatypes = {"mapping", "test", "validation", "mapping_lidar_only"}
    formats = {".tar.gz", ".tar.gz.sha256sum"}

    links = []
    for name in datanames:
        for type in datatypes:
            for fm in formats:
                link = rooturl + '/' + name + '_release_' + type + fm
                links.append(link)
    return links


if __name__ == "__main__":
    nargs = len(sys.argv)
    if nargs < 2:
        print('Usage: {} output-path'.format(sys.argv[0]))
        sys.exit(1)

    output_path = sys.argv[1]
    links = generate_links()
    print('Downloading {} files'.format(len(links)))
    for l, link in enumerate(links):
        print("{}:{}".format(l, link))
    download_files_from_web.wget_file_parallel(links, output_path)
