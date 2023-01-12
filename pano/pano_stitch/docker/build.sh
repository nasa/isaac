#!/bin/sh
cd ${HOME}/isaac/src
docker build . -f pano/pano_stitch/docker/Dockerfile.pano_stitch -t isaac/pano_stitch

