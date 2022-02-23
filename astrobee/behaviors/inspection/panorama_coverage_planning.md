\page inspection Panorama coverage planning

A panorama coverage plan is a sequence of image center pan/tilt
values. The objective of panorama coverage planning is to generate a
plan that completely covers the specified range of pan/tilt values with
sufficient image-to-image overlap to permit downstream processing (e.g.,
Hugin panorama stitching), and is sufficiently robust to attitude
error. Within those constraints, we want to optimize the plan for
minimum image count, as a proxy for minimum run time.

# Relevant files

- [pano_orientations.py](./scripts/pano_orientations.py): This was the
  initial reference implementation of a panorama planner. It has been
  superseded by improvements in the C++ code and may eventually go away.
- [pano.cc](./src/pano.cc): The latest C++ implementation of the
  coverage planner.
- [test_pano.cc](./tools/test_pano.cc): A tool that invokes the coverage
  planner on the specified test cases.
- [pano_test_cases.csv](./scripts/pano_test_cases.csv): The test cases
  used by `test_pano.cc`.
- [plot_pano.py](./scripts/plot_pano.py): A script that checks the
  output of the `test_pano` tool and produces plots for debugging.
- [field_of_view_calculator.py](./scripts/field_of_view_calculator.py):
  A script to calculate the field of view for various Astrobee cameras.

# Panorama design approach

Panorama planning starts from the concept of a rectangular grid of image
centers, evenly spaced so as to completely cover the specified
(rectangular) imaging area with the desired overlap and attitude
tolerance.

The collection order of images in the grid follows a column-major
raster pattern: alternating columns are collected top-to-bottom and
bottom-to-top. Column-major is preferred because it makes it easier
for crew to stay behind the robot during panorama collection, if they
care to do so. Following an alternating raster pattern minimizes
large attitude changes that are challenging for Astrobee localization.

A further consideration is that, when viewing a plot of the coverage
using a typical equirectangular projection, the rectangular area of each
image becomes increasingly warped as the tilt value approaches the poles
(+/-90 degrees).

The primary effect of the warping is to make the effective image
coverage wider nearer the poles. We take advantage of this effect by
reducing the number of images in grid rows nearer the poles. A downside
of reducing the image count is that the images no longer form a grid, so
the column-major raster sequencing is only approximate.

A secondary effect of the warping is that it complicates determining how
to position the warped rectangles of individual image coverage so that
together they cover the boundaries of the rectangular desired imaging
area.

# ISAAC panorama surveys

There are different styles of panorama that could be useful for
different applications.

## `5_mapper_and_hugin`

The parameters in this test case are recommended as a potential
"workhorse" panorama type for doing complete module surveys. The design
criteria were:

1. Capture a panorama with complete spherical coverage.
2. Tolerate up to 5 degrees (TBC) of attitude error without violating
   complete coverage or overlap requirements. We don't yet have a good
   estimate of actual attitude error during panorama collection.
3. Enable Hugin auto-stitch of a SciCam panorama. This requires complete
   SciCam coverage with at least 30% overlap. This is the strictest
   spacing criterion that ends up driving the panorama plan.
4. Enable post-activity bundle adjustment for localization. This
   requires complete NavCam coverage with sufficient overlap. At least
   75-80% overlap is needed between consecutive NavCam images in time
   sequence, but because the NavCam images are recorded continuously at
   ~5 Hz, as long as the robot angular velocity is low enough, this
   requirement will always be met regardless of the panorama plan. It
   also helps to have > ~30% overlap between adjacent columns of the
   panorama, which is not difficult because the NavCam FOV is so
   wide. [Not the strictest/driving requirement.]
5. Enable ISAAC geometry mapper dense mapping to produce a 3D mesh with
   SciCam texture.  This requires both the NavCam bundle adjustment
   listed above for registration, as well as complete SciCam and HazCam
   coverage, but without a requirement for them to overlap. [Not the
   strictest/driving requirement.]

As of this writing (2022/02), using the experimental
pano_orientations2() planner, the resulting panorama plan has 56 images
in 7 rows, with at most 10 images in a row.

## `4_mapper`

This test case examines the scenario of relaxing the Hugin auto-stitch
requirement #2 above. In that case, requirement #5 becomes the driving
requirement. Because the panorama motion is vertical and HazCam images
are acquired continuously at ~5 Hz, the HazCam vertical spacing is not a
driving constraint, even though the HazCam has a smaller FOV than the
SciCam.  As a result, we specify the VFOV from the SciCam, the HFOV from
the the HazCam, and as a bit of a hack, we pad the tilt radius slightly
to make doubly sure the HazCam gets complete coverage near the poles,
despite its smaller VFOV.

The resulting panorama plan has far fewer images than
`5_mapper_and_hugin`, 30 vs. 56, which is attractive. The downsides are
that it may not be compatible with Hugin auto-stitch (although it may be
feasible to pass pan/tilt parameters from NavCam bundle adjustment to
Hugin instead), and probably more importantly, it would be less robust
to excessive robot pointing error. If time permits, it might be useful
to try to capture a panorama in this more aggressive mode to evaluate
whether the data is sufficient for downstream analysis.

## `6_nav_hugin`

This test case examines the scenario of relaxing both requirements #2
and #5 above, so that the NavCam overlap requirement #4 becomes the
driving requirement. HazCam and SciCam coverage would be incomplete, so
the geometry mapper could not build a full 3D mesh, but the resulting
NavCam imagery could be used to build a low-resolution NavCam panorama
with Hugin auto-stitch.

The resulting panorama plan has only 15 images. This type of panorama
could occasionally be suitable for a fast low-resolution survey.
