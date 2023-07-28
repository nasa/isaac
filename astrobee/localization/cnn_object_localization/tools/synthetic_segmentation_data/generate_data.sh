#!/bin/bash

# Parse command line options.
function abs_path() {
    echo "$(cd "$(dirname "$1")"; pwd -P)/$(basename "$1")"
}
CONFIG_NAME=""
OUTPUT_DIR=""
while getopts ':c:o:' OPTION; do
    case "$OPTION" in
        c)
            CONFIG_NAME=$OPTARG
            ;;
        o)
            OUTPUT_DIR=$(abs_path "$OPTARG")
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            ;;
    esac
done
shift $(($OPTIND -1))
if [[ -z "$CONFIG_NAME" ]] || [[ -z "$OUTPUT_DIR" ]]; then
    echo "Usage: $(basename $0) -c <config_name> -o <output_dir>"
    exit 1
fi

# Cleanup function to delete the zeroth image and shift all the other image filenames down by one
function cleanup {
    echo ""
    echo "Fixing indexing for output files..."
    rm -f "${OUTPUT_DIR}/colored_maps/colored_0000000.png"
    rm -f "${OUTPUT_DIR}/images/image_0000000.png"
    rm -f "${OUTPUT_DIR}/labels_maps/labels_0000000.png"
    for directory in colored_maps images labels_maps; do
        for old_file in $(ls "${OUTPUT_DIR}/${directory}"/*.png | sort -V); do
            base_name=$(basename "${old_file}" .png)
            index=${base_name#*_}
            index=$((10#$index))
            index=$((index - 1))
            new_file=$(printf "${OUTPUT_DIR}/${directory}/%s_%07d.png" "${base_name%_*}" "${index}")
            mv "${old_file}" "${new_file}"
        done
    done
}
trap cleanup SIGINT

# Figure out directories and filepaths of interest
SCRIPT_DIR="$( cd "$(dirname "$0")" ; pwd -P )"
CONFIG_FILE="${SCRIPT_DIR}/config/${CONFIG_NAME}.config"

# Export environment variables for C++ data generation plugin
export SYNTHETIC_SEGMENTATION_DATA_CONFIG_FILE=${CONFIG_FILE}
export SYNTHETIC_SEGMENTATION_DATA_OUTPUT_DIR=${OUTPUT_DIR}
export SYNTHETIC_SEGMENTATION_DATA_SCRIPT_DIR=${SCRIPT_DIR}

# Export environment variables for Ignition itself
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/ign_plugins/build"
export IGN_GAZEBO_RESOURCE_PATH="${SCRIPT_DIR}/models"
export MESA_GL_VERSION_OVERRIDE=3.3

echo $IGN_GAZEBO_RESOURCE_PATH

# Create a copy of the world file specified in the config file
# The copy has the data output directory filled in
# This is hacky but works I guess...
WORLD_FILENAME=$(grep -oP '^WORLD_FILENAME=\K.*' "$CONFIG_FILE")
WORLD_TEMPLATE_FILE="${SCRIPT_DIR}/worlds/templates/${WORLD_FILENAME}"
WORLD_GENERATE_FILE="${SCRIPT_DIR}/worlds/generated/${WORLD_FILENAME}"
mkdir -p "$(dirname "$WORLD_GENERATE_FILE")"
sed "s#__OUTPUT_DIR__#$OUTPUT_DIR#g" "$WORLD_TEMPLATE_FILE" > "$WORLD_GENERATE_FILE"

# Make data output directory
mkdir -p ${SYNTHETIC_SEGMENTATION_DATA_OUTPUT_DIR}

# Run Ignition Gazebo
echo "Running Ignition Gazebo. Remember to subscribe to the segmentation camera topic!"
ign gazebo $WORLD_GENERATE_FILE

