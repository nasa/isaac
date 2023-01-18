#!/bin/sh
if [ "$ISAAC_PANO_INPUT" = "" ] || [ ! -d "$ISAAC_PANO_INPUT" ]; then
    echo "ISAAC_PANO_INPUT env var must point to input folder"
    exit 1
fi
if [ "$ISAAC_PANO_STITCH" = "" ] || [ ! -d "$ISAAC_PANO_STITCH" ]; then
    echo "ISAAC_PANO_STITCH env var must point to output folder"
    exit 1
fi

set -x

cd ${HOME}/isaac/src
docker run \
       -it --rm \
       --name isaac_pano_stitch \
       --mount type=bind,source=${ISAAC_PANO_INPUT},target=/input,readonly \
       --mount type=bind,source=${ISAAC_PANO_STITCH},target=/stitch \
       --mount type=bind,source=$(pwd),target=/src/isaac/src \
       isaac/pano_stitch
