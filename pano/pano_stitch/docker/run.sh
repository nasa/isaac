#!/bin/sh
cd ${HOME}/isaac/src
docker run -it --rm --name isaac_pano_stitch isaac/pano_stitch
