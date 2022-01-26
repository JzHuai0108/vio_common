#!/usr/bin/env python3

import download_files_from_web

dates = {
    'office-loop': [
        '2020-03-24_17-36-22',
        '2020-03-24_17-45-31',
        '2020-04-07_10-20-32',
        '2020-06-12_10-10-57',
        '2021-01-07_12-04-03',
        '2021-02-25_13-51-57'],
    'neighborhood': [
        '2020-03-26_13-32-55',
        '2020-10-07_14-47-51',
        '2020-10-07_14-53-52',
        '2020-12-22_11-54-24',
        '2021-02-25_13-25-15',
        '2021-05-10_18-02-12',
        '2021-05-10_18-32-32'],

    'business-campus': [
        '2020-10-08_09-30-57',
        '2021-01-07_13-12-23',
        '2021-02-25_14-16-43'],

    'countryside': [
        '2020-04-07_11-33-45',
        '2020-06-12_11-26-43',
        '2020-10-08_09-57-28',
        '2021-01-07_13-30-07'],

    'cityloop': [
        '2020-12-22_11-33-15',
        '2021-01-07_14-36-17',
        '2021-02-25_11-09-49'],

    'oldtown': [
        '2020-10-08_11-53-41',
        '2021-01-07_10-49-45',
        '2021-02-25_12-34-08',
        '2021-05-10_21-32-00'],

    'parkinggarage': [
        '2020-12-22_12-04-35',
        '2021-02-25_13-39-06',
        '2021-05-10_19-15-19']
}

datatypes = ['imu_gnss.zip', 'point_clouds.zip', 'reference_poses.zip',
             'stereo_images_distorted.zip', 'stereo_images_undistorted.zip']

rooturl = 'https://vision.cs.tum.edu/webshare/g/4seasons-dataset/dataset'
output_path = '/media/jhuai/SeagateData/jhuai/data/4seasons'

links = []
for name, dtlist in dates.items():
    for dt in dtlist:
        for type in datatypes:
            link = '{}/recording_{}/recording_{}_{}'.format(rooturl, dt, dt, type)
            links.append(link)

print('Downloading {} files from {}'.format(len(links), rooturl))
for l, link in enumerate(links):
    print("{}:{}".format(l, link))
download_files_from_web.wget_file_parallel(links, output_path)
