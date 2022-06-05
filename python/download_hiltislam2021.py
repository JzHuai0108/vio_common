import sys

import download_files_from_web

if __name__ == "__main__":
    nargs = len(sys.argv)
    if nargs < 2:
        print('Usage: {} output-path'.format(sys.argv[0]))
        sys.exit(1)

    output_path = sys.argv[1]

    links = ["https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2_imu.txt",
             "https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag",
             "https://storage.googleapis.com/hilti_challenge/IC_Office_1.bag",
             "https://storage.googleapis.com/hilti_challenge/Office_Mitte_1.bag",
             "https://storage.googleapis.com/hilti_challenge/Parking_1.bag",
             "https://storage.googleapis.com/hilti_challenge/Basement_1_pole.txt",
             "https://storage.googleapis.com/hilti_challenge/Basement_1.bag",
             "https://storage.googleapis.com/hilti_challenge/Basement_3.bag",
             "https://storage.googleapis.com/hilti_challenge/Basement_4_prism.txt",
             "https://storage.googleapis.com/hilti_challenge/Basement_4.bag",
             "https://storage.googleapis.com/hilti_challenge/LAB_Survey_2_imu.txt",
             "https://storage.googleapis.com/hilti_challenge/LAB_Survey_2.bag",
             "https://storage.googleapis.com/hilti_challenge/Construction_Site_1.bag",
             "https://storage.googleapis.com/hilti_challenge/Construction_Site_2_prism.txt",
             "https://storage.googleapis.com/hilti_challenge/Construction_Site_2.bag",
             "https://storage.googleapis.com/hilti_challenge/Campus_1.bag",
             "https://storage.googleapis.com/hilti_challenge/Campus_2_prism.txt",
             "https://storage.googleapis.com/hilti_challenge/Campus_2.bag",
             "https://storage.googleapis.com/hilti_challenge/5588054_00_Sensor_stick_IROS.stp",
             "https://storage.googleapis.com/hilti_challenge/Calibration/calibration_02.yaml",
             "https://storage.googleapis.com/hilti_challenge/5588054_00_Sensor_stick_IROS.stp"]

    print('Downloading {} files'.format(len(links)))
    for l, link in enumerate(links):
        print("{}:{}".format(l, link))
    download_files_from_web.wget_file_parallel(links, output_path)
