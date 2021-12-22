\page volumetric_mapper Volumetric Mapper
Volumetric Mapper
==============

Overview
---------

The wifi mapper subscribes to all the wifi signal strength messages and prcesses them for 3D visualization.
Customization os the produced maps is defined in isaac/config/dense_map/wifi_mapper.config

Trace Map
---------

The trace map is drawn at a certain resolution, within this resolution, every signal received is averaged. The result is depicted using MarkerArray's where the cube size is the map resolution.

Parameters:
plot_trace - enable or disable the trace map calculation
resolution - resolution of the trace map where whithin it, the measurements will be averaged


Wifi 3D Map
---------
The Wifi Mapper uses Gaussian Process Regression to mapp the ISS regarding the wifi signal strength. It makes use of the libgp library.
When a new measure is obtained, the value is recorded in the GP object. When the timer responsible for calculating the wifi interpolation is enables, all the recorded data is processed.

Parameters:
plot_map - enable or disable the 3D map interpolation calculation