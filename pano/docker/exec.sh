#!/bin/sh
set -x
docker exec -it isaac_pano /bin/bash -ic "$*"
