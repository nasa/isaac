#!/bin/bash

# Parse command line options.
function abs_path() {
    echo "$(cd "$(dirname "$1")"; pwd -P)/$(basename "$1")"
}
CONFIG_NAME=""
OUTPUT_DIR=""
GENERATE_ANNOTATED=""
GENERATE_MASKS=""
GENERATE_JSON=""
while getopts ':c:o:amj' OPTION; do
    case "$OPTION" in
        c)
            CONFIG_NAME=$OPTARG
            ;;
        o)
            OUTPUT_DIR=$(abs_path "$OPTARG")
            ;;
        a)
            GENERATE_ANNOTATED="--generate_annotated"
            ;;
        m)
            GENERATE_MASKS="--generate_masks"
            ;;
        j)
            GENERATE_JSON="--generate_json"
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
    echo "Usage: $(basename $0) -c <config_name> -o <output_dir> [-a] [-m] [-j]"
    exit 1
fi

# Figure out directories and filepaths of interest
SCRIPT_DIR="$( cd "$(dirname "$0")" ; pwd -P )"

# Run Python keypoint annotation
python3 "${SCRIPT_DIR}/src/annotate_keypoints.py" --tool_dir $SCRIPT_DIR --config_name $CONFIG_NAME --data_dir $OUTPUT_DIR $GENERATE_ANNOTATED $GENERATE_MASKS $GENERATE_JSON
