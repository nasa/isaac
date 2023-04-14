#!/bin/sh

SCRIPT_DIR=$(dirname "$0")
ISAAC_SRC=$(realpath "${SCRIPT_DIR}/../../")

set -x
cd "${ISAAC_SRC}"
docker build . -f pano/docker/pano.Dockerfile -t isaac/pano
