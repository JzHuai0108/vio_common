#!/bin/bash
# Step 1: Spatially subsample result.las to 2 cm using CloudCompare, producing
# result_spatialsub_2cm.bin (CloudCompare binary) and result_spatialsub_2cm.las
# (for MATLAB). Also converts the reference .bin clouds to .las for MATLAB.
#
# Usage: bash step1_downsample.sh
# Requires: CloudCompare (snap package)

set -e

CC=cloudcompare.CloudCompare
BASE_DIR=/media/jhuai/T5EVO/jhuai/results/hiltislam2022_rss26

METHODS=(balm2 balm3 base nolc)
SEQS=(exp04 exp05 exp06 exp14 exp16 exp18)

echo "=== Step 1a: Downsample result.las to 2 cm ==="
for method in "${METHODS[@]}"; do
    for seq in "${SEQS[@]}"; do
        input="$BASE_DIR/$method/$seq/result.las"
        output_bin="$BASE_DIR/$method/$seq/result_spatialsub_2cm.bin"
        output_las="$BASE_DIR/$method/$seq/result_spatialsub_2cm.las"

        if [ ! -f "$input" ]; then
            echo "  [SKIP] Missing input: $input"
            continue
        fi

        "$CC" -SILENT -AUTO_SAVE OFF -O "$input" -SS SPATIAL 0.02 -C_EXPORT_FMT LAS -SAVE_CLOUDS FILE "$output_las"

    done
done

echo ""
echo "=== Step 1b: Convert reference .e57 clouds to .las ==="
REF_DIR="$BASE_DIR/ref"

for ref_e57 in \
    "$REF_DIR/construction_site_full.e57" \
    "$REF_DIR/sheldonian_full.e57"
do
    ref_las="${ref_e57%.e57}.las"
    echo "  Converting $(basename "$ref_e57") -> .las ..."
    "$CC" -SILENT -AUTO_SAVE OFF -O "$ref_e57" -SS SPATIAL 0.02 -REMOVE_ALL_SFS -C_EXPORT_FMT LAS -SAVE_CLOUDS FILE "$ref_las"
done

echo ""
echo "Done. All clouds ready for MATLAB evaluation."
