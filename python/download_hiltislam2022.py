import download_files_from_web

links = ["https://storage.googleapis.com/hsc2022/april_grid_2500x1500_7x12_15cm.yaml",
         "https://storage.googleapis.com/hsc2022/calib_03_2022-03-02-11-27-22.bag",
         "https://storage.googleapis.com/hsc2022/calibration/2022-04-25-12-52-59-stationary-imu.bag",
         "https://storage.googleapis.com/hsc2022/exp01_construction_ground_level.bag",
         "https://storage.googleapis.com/hsc2022/exp02_construction_multilevel.bag",
         "https://storage.googleapis.com/hsc2022/exp03_construction_stairs.bag",
         "https://storage.googleapis.com/hsc2022/groundtruth/exp_04_construction_upper_level_imu.txt",
         "https://storage.googleapis.com/hsc2022/exp04_construction_upper_level.bag",
         "https://storage.googleapis.com/hsc2022/groundtruth/exp_05_construction_upper_level_2_imu.txt",
         "https://storage.googleapis.com/hsc2022/exp05_construction_upper_level_2.bag",
         "https://storage.googleapis.com/hsc2022/groundtruth/exp_06_construction_upper_level_3_imu.txt",
         "https://storage.googleapis.com/hsc2022/exp06_construction_upper_level_3.bag",
         "https://storage.googleapis.com/hsc2022/exp07_long_corridor.bag",
         "https://storage.googleapis.com/hsc2022/exp09_cupola.bag",
         "https://storage.googleapis.com/hsc2022/exp11_lower_gallery.bag",
         "https://storage.googleapis.com/hsc2022/exp15_attic_to_upper_gallery.bag",
         "https://storage.googleapis.com/hsc2022/exp21_outside_building.bag",
         "https://storage.googleapis.com/hsc2022/calibration/2022322_calibration_files.zip",
         "https://storage.googleapis.com/hsc2022/april_grid_2500x1500_7x12_15cm.yaml",
         "https://storage.googleapis.com/hsc2022/cad/Phasma25.step",
         "https://storage.googleapis.com/hsc2022/cad/Phasma25.stl",
         "https://storage.googleapis.com/hsc2022/cad/Phasma25_GOM_high_res.stl"]

output_path = "/media/jhuai/Data1/hiltislam2022"

print('Downloading {} files'.format(len(links)))
for l, link in enumerate(links):
    print("{}:{}".format(l, link))
download_files_from_web.wget_file_parallel(links, output_path)
