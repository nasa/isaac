\page docker Docker

## Demos

There are currently 3 demos available to showcase some aspects of the ISAAC functionality

### Trigger anomaly

This demo will trigger a C02 anomaly, the levels of C02 will start to increase. The mast detects the anomaly and sends astrobee to inspect a vent. Astrobee will undock, calculate the optimal inspection pose to observe the target and move towards that pose, replanning if any obstacle is found. When the robot has the vent of interest in sight, it will take a picture and run it through a trained CNN, identifying whether the vent is obstructed, free or inconclusive result. After inspection Astrobee will dock autonomously.

### Trigger geometric mapping

This demo will trigger a geometric mapping inspection event. The geometric mapper collects pictures from several poses and creates a 3d mesh of the ISS.
The robot will undock, follow a trajectory taking pictures at the specified waypoints and dock again. For the geometric mapper, the trajectory followed is defined in astrobee/behaviors/inspection/resources/jpm_sliced.txt. The geometric mapper will map a section of the jem containing the entry node.

### Trigger wifi mapping

This demo will trigger a volumetric mapping inspection event. The volumetric mapper collects information from an onboard sensor of Astrobee and interpolates the data in a specified area.
The robot will undock, follow a trajectory and dock again. For the wifi mapper, the trajectory followed is defined in astrobee/behaviors/inspection/resources/wifi.txt.