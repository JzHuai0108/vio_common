# A sample shell script to invoke the procedures to calculate the rolling shutter skew.
# Input: A video captured looking at the LED panel in rolling shutter continuous mode.
# Output: Rolling shutter skew estimates for each video.

DIR="/ksf-data/honorv10-led-panel"
DATA_LIST=(
    "2020_07_13_18_08_43"
    "2020_07_13_18_12_29"
    "2020_07_13_21_23_41"
    "2020_07_13_21_24_01"
    "2020_07_13_21_24_42"
    "2020_07_13_21_24_53"
    "2020_07_13_21_24_56"
    "2020_07_13_21_25_05")
LED_TIME=(1 1 2 2 1 1 1 1)
VIO_PYTHON="/msckf_ws/src/vio_common/python"

FRAME_SCRIPT="$VIO_PYTHON/video2frames.py"
TR_SCRIPT="$VIO_PYTHON/rolling_shutter_skew/test_rolling_shutter_skew.py"

for i in "${!DATA_LIST[@]}"; do
    echo "data dir $DIR/${DATA_LIST[$i]} and led time ${LED_TIME[$i]}"
    # python2 $FRAME_SCRIPT "$DIR/$DATA/movie.mp4" --output-folder="$DIR/$DATA/raw" --video-from-to 2 1000 \
    #    --choose-every-n 3 --save_rgb
    python2 $TR_SCRIPT -l ${LED_TIME[$i]} --read_dir="$DIR/${DATA_LIST[$i]}/raw" \
        --img_size=1280,720 -d -o="$DIR/${DATA_LIST[$i]}" >> "$DIR/${DATA_LIST[$i]}/rolling-shutter-skew.txt"
done

