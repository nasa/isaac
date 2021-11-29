#!/bin/sh

DIR="$(dirname "$0")"

cd $DIR/../..
mkdir -p mast
vcs import --input isaac/scripts/isaac.repos --debug
