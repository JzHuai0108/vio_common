#!/usr/bin/env python3
import os, sys, csv
from pathlib import Path
import argparse
import rospy
import shutil

# ---- import your timestamp corrector (pybind11 module) ----
MYMOD_PATH = os.path.abspath('../timestamp_corrector/build')
if MYMOD_PATH not in sys.path:
    sys.path.append(MYMOD_PATH)
import TimestampCorrector as TC  # provides TimestampCorrector()

def to_time_ns(x):
    """Accept float seconds or int nanoseconds and return int nanoseconds."""
    # If it's obviously ns-scale, keep it; else treat as seconds.
    try:
        xf = float(x)
    except Exception:
        raise ValueError(f"Bad timestamp value: {x!r}")
    if xf > 1e12:   # already in ns
        return int(xf)
    return int(round(xf * 1e9))

def ns_to_rospy_time(ns):
    return rospy.Time(secs=ns // 1_000_000_000, nsecs=ns % 1_000_000_000)

def read_host_and_sensor_times(csv_path):
    host_times, sensor_times = [], []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        # skip header lines starting with '#'
        for row in reader:
            if not row:
                continue
            if row[0].strip().startswith('#'):
                continue
            if len(row) < 2:
                print(f"[WARNING] Skipping malformed row: {row}")
                continue
            host_time = to_time_ns(row[0])
            sensor_time = to_time_ns(row[1])
            host_times.append(host_time); sensor_times.append(sensor_time)
    # ensure ascending order of timestamps
    if any(t2 < t1 for t1, t2 in zip(host_times, host_times[1:])):
        paired = sorted(zip(host_times, sensor_times))
        host_times = [h for h, _ in paired]
        sensor_times = [s for _, s in paired]
    return host_times, sensor_times

def img_tstamps_associate(host_1_ns, host_2_ns, imgs_1, imgs_2, dt=6e6):
    """
    Align left and right image timestamps to the nearest host timestamp.
    Returns aligned lists of image paths and timestamps.

    jhuai: The images and sensor times should be one to one, but unfortunately, our recording aborts saving early,
     causing different number of times and images. We should delete these extra images manually to make them one to one match.

    """
    start_j = 0
    associated_imgs_1, associated_imgs_2 = [], []
    host_associate_1, host_associate_2 = [], []
    for i, t in enumerate(host_1_ns):
        for j in range(start_j, len(host_2_ns)):
            if t + 40e6 < host_2_ns[j]:
                break
            if abs(t - host_2_ns[j]) < dt:  
                img_1_file = imgs_1[i]
                img_2_file = imgs_2[j]
                associated_imgs_1.append(img_1_file)
                host_associate_1.append(t)
                associated_imgs_2.append(img_2_file)
                host_associate_2.append(host_2_ns[j])
                start_j = j + 1
                break
    return associated_imgs_1, associated_imgs_2, host_associate_1, host_associate_2

def imread_with_tstamp_ns(img_folder): 
    imgs_l = sorted(Path(os.path.join(img_folder, "left_thermal/image")).glob('*.png')) 
    imgs_r = sorted(Path(os.path.join(img_folder, "right_thermal/image")).glob('*.png')) 
    return imgs_l, imgs_r

def quick_stats(corrected_ns, host_ns):
    diffs_ms = [(c - h) / 1e6 for c, h in zip(corrected_ns, host_ns)]
    diffs_sorted = sorted(diffs_ms)
    mid = len(diffs_sorted) // 2
    median = (diffs_sorted[mid] if len(diffs_sorted) % 2 else
            0.5 * (diffs_sorted[mid - 1] + diffs_sorted[mid]))
    mean = sum(diffs_ms) / len(diffs_ms)
    print(f"[STATS] Δ = corrected_host - origin_host (ms): "
        f"mean={mean:.3f}  min={min(diffs_ms):.3f}  max={max(diffs_ms):.3f}  median={median:.3f}")
    
def imgs_rename(new_path, imgs, corrected_ns):
    """
    Rename images in imgs to new_path with corrected timestamps.
    """
    for img, t in zip(imgs, corrected_ns):
        new_name = f"{t//1_000_000_000}.{t % 1_000_000_000:09d}.png"
        new_full_path = os.path.join(new_path, new_name)
        shutil.copy(str(img), new_full_path)

def main():
    ap = argparse.ArgumentParser(description='Rewrite Thermal Stereo image times using TimestampCorrector.')
    ap.add_argument('dataset_folder', help='Folder with thermal stereo dataset')
    ap.add_argument('--ref-offset-sec', type=float, default=100.0,
                    help='Seconds to subtract from first host time to build a reference (default: 100)')
    args = ap.parse_args()


    # -------- PASS 1: collect times and read images with tstamp--------
    host_l_ns, sensor_l_ns = read_host_and_sensor_times(os.path.join(args.dataset_folder, "left_thermal/timestamps.csv"))
    print(f"[INFO] Collected sensor_l_ns: {len(sensor_l_ns)}")
    host_r_ns, sensor_r_ns = read_host_and_sensor_times(os.path.join(args.dataset_folder, "right_thermal/timestamps.csv"))
    print(f"[INFO] Collected sensor_r_ns: {len(sensor_r_ns)}")

    first_diff = host_l_ns[0] - sensor_l_ns[0]
    run_time_correction = False
    if abs(first_diff) > 100e9:
        run_time_correction = True

    # -------- PASS 2: run TimestampCorrector --------
    if not run_time_correction:
        corrected_l_ns = sensor_l_ns
        corrected_r_ns = sensor_r_ns
    else:
        host_l_ref_ns = host_l_ns[0] - int(args.ref_offset_sec * 1e9)
        host_r_ref_ns = host_r_ns[0] - int(args.ref_offset_sec * 1e9)
        sens_l_f = [s / 1e9 for s in sensor_l_ns]              
        sens_r_f = [s / 1e9 for s in sensor_r_ns] 
        host_l_f = [(h - host_l_ref_ns) / 1e9 for h in host_l_ns]   
        host_r_f = [(h - host_r_ref_ns) / 1e9 for h in host_r_ns]
    
        TCor_l = TC.TimestampCorrector()
        TCor_r = TC.TimestampCorrector()
        for s, h in zip(sens_l_f, host_l_f):
            TCor_l.correctTimestamp(s, h)
        for s, h in zip(sens_r_f, host_r_f):
            TCor_r.correctTimestamp(s, h)

        corrected_l_ns = []
        corrected_r_ns = []
        for s in sens_l_f:
            corr_s = float(TCor_l.getLocalTime(s))           
            corrected_l_ns.append(int(round(host_l_ref_ns + corr_s * 1e9)))
        for s in sens_r_f:
            corr_s = float(TCor_r.getLocalTime(s))       
            corrected_r_ns.append(int(round(host_r_ref_ns + corr_s * 1e9)))

        # Quick stats
        quick_stats(corrected_l_ns, host_l_ns)
        quick_stats(corrected_r_ns, host_r_ns)

        print(f"[INFO] Corrected tstamps_left: {len(corrected_l_ns)}, tstamps_right: {len(corrected_r_ns)}")

    # -------- PASS 3: thermal stereo timestamps associate --------
    imgs_l, imgs_r = imread_with_tstamp_ns(args.dataset_folder)
    print(f"[INFO] Read images_left: {len(imgs_l)}, images_right: {len(imgs_r)}")

    if len(imgs_l) > len(imgs_r):
        imgs_r_associated, imgs_l_associated, associate_r_ns, associate_l_ns = img_tstamps_associate(corrected_r_ns, corrected_l_ns, imgs_r, imgs_l)
    else:
        imgs_l_associated, imgs_r_associated, associate_l_ns, associate_r_ns = img_tstamps_associate(corrected_l_ns, corrected_r_ns, imgs_l, imgs_r)

    print(f"[INFO] Associated images_left: {len(imgs_l_associated)}, images_right: {len(imgs_r_associated)}")

    # -------- PASS 4: Output associated thermal stereo images with corrected times --------
    associated_img_left = os.path.join(args.dataset_folder, "left_thermal/associated_left")
    associated_img_right = os.path.join(args.dataset_folder, "right_thermal/associated_right")
    os.makedirs(associated_img_left, exist_ok=True)
    os.makedirs(associated_img_right, exist_ok=True)

    imgs_rename(associated_img_left, imgs_l_associated, associate_l_ns)
    imgs_rename(associated_img_right, imgs_r_associated, associate_r_ns)
    print(f"[INFO] Thermal stereo images with corrected timestamps saved to {associated_img_left} and {associated_img_right}")

if __name__ == '__main__':
    main()
