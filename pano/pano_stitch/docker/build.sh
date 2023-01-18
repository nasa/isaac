#!/bin/sh
set -x
cd ${HOME}/isaac/src
docker build . -f pano/pano_stitch/docker/pano_stitch.Dockerfile -t isaac/pano_stitch
