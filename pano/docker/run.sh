#!/bin/sh

SCRIPT_DIR=$(dirname "$0")
ISAAC_SRC=$(realpath "${SCRIPT_DIR}/../../")

if [ "$ISAAC_PANO_INPUT" = "" ] || [ ! -d "$ISAAC_PANO_INPUT" ]; then
    echo "ISAAC_PANO_INPUT env var must point to input folder"
    exit 1
fi
if [ "$ISAAC_PANO_OUTPUT" = "" ] || [ ! -d "$ISAAC_PANO_OUTPUT" ]; then
    echo "ISAAC_PANO_OUTPUT env var must point to output folder"
    exit 1
fi

set -x
cd "${ISAAC_SRC}"
docker run \
       -it --rm \
       --name isaac_pano \
       --mount type=bind,source=${ISAAC_PANO_INPUT},target=/input,readonly \
       --mount type=bind,source=${ISAAC_PANO_OUTPUT},target=/output \
       --mount type=bind,source=${ISAAC_SRC},target=/src/isaac/src \
       isaac/pano
