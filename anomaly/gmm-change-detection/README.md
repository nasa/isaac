\page gmm GMM Change Detection

# Overview

This implementation of a GMM-based anomaly detection algorithm was created by Jamie Santos, for the purposes of a [Master thesis]().
This algorithm is able to detect changes on environments such as the ISS using 3D point depth cloud data.

# Requirements
pip3 install pulp
pip3 install scikit-learn
pip install pyntcloud
pip install pandas
pip install open3d
apt-get install glpk-utils

## Usage

	rosrun gmm gmm_change_detection.py


