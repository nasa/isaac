#!/bin/bash
#

# generate pto file with images
pto_gen -o panorama.pto -f 62.0 *.jpg

# generate control points
cpfind --multirow -o panorama.pto panorama.pto

# set optimization variables, optimize
pto_var --opt y,p,r,b -o panorama.pto panorama.pto
autooptimiser -n -o panorama.pto panorama.pto

# clean outliers
cpclean --max-distance=3 -o panorama.pto panorama.pto

# optimize pohotometric parameters
pto_var --opt y,p,r,TrX,TrY,TrZ,b -o panorama.pto panorama.pto
autooptimiser -n -o panorama.pto panorama.pto


autooptimiser -m -o panorama.pto panorama.pto

# configure image output
pano_modify -o panorama.pto --center --canvas=AUTO --projection=2 --fov=360x180 panorama.pto

# generate panorama
hugin_executor --stitching --prefix=prefix panorama.pto