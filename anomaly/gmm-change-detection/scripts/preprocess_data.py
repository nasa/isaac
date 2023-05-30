# Converts ROS sensor_msgs:PointCloud2 to a
# python-PCL-filtered numpy array

import math
import time

import geometry_msgs
import matplotlib.pyplot as plt
import numpy as np
import pcl
import pcl.pcl_visualization
import ros_numpy
import rosbag
import rospy
import sensor_msgs
from PIL import Image


# See https://stackoverflow.com/questions/39772424
def convert_pc2_pcl(data):
    data.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
    offset_sorted = {f.offset: f for f in data.fields}
    data.fields = [f for (_, f) in sorted(offset_sorted.items())]

    pc = ros_numpy.numpify(data)
    height = pc.shape[0]
    if len(pc.shape) == 1:  # Unordered PC2 structure
        np_points = np.zeros((height, 3), dtype=np.float32)
        np_points[:, 0] = np.resize(pc["x"], height)
        np_points[:, 1] = np.resize(pc["y"], height)
        np_points[:, 2] = np.resize(pc["z"], height)
        p = pcl.PointCloud(np.array(np_points, dtype=np.float32))

    else:  # Ordered PC2 structure
        width = pc.shape[1]
        np_points = np.zeros((height * width, 3), dtype=np.float32)
        np_points[:, 0] = np.resize(pc["x"], height * width)
        np_points[:, 1] = np.resize(pc["y"], height * width)
        np_points[:, 2] = np.resize(pc["z"], height * width)
        p = pcl.PointCloud(np.array(np_points, dtype=np.float32))
    return p


# Outlier and downsample filtering of data
def filter_pcl(pcl_data):
    p = pcl_data
    pcl.save(p, "normal.pcd")

    # Statistical Outlier Filter
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(1.0)

    # pcl.save(fil.filter(), "inliers.pcd")
    # fil.set_negative(True)
    # pcl.save(fil.filter(), "outliers.pcd")

    # Downsample outlier filtered data
    # fil = fil.make_voxel_grid_filter()
    # fil.set_leaf_size(0.01, 0.01, 0.01)
    # pcl.save(fil.filter(), "downsample.pcd")

    # Conversion from pcl to numpy array
    np_arr = fil.filter().to_array()
    # np_arr = p.to_array()
    return np_arr


# Obtain and concatenate point clouds remapped with ground truth transform
def concat_ground_truth_msgs(bagfile):
    skipped = 100  # Number of messages to skip over
    n_points = 171 * 224  # 171 rows x 224 columns
    n_msgs = rosbag.Bag(bagfile).get_message_count("/hw/depth_haz/points/ground_truth")
    n_msgs_used = math.floor(n_msgs / skipped)
    merged_pcl = np.empty((n_msgs_used * n_points, 3), dtype=np.float32)

    count = 0
    i = 0
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        if topic == "/hw/depth_haz/points/ground_truth":
            count += 1
            if count % skipped == 0:
                np_arr = convert_pc2_pcl(msg).to_array()
                merged_pcl[i : i + n_points, :] = np_arr
                i += n_points

    p = pcl.PointCloud(merged_pcl)
    pcl.save(p, "ground_truth_run5.pcd")
    return p


# Process data from bagfile
def read_pc2_msgs(bagfile):
    for topic, msg, t in rosbag.Bag(bagfile).read_messages():
        if topic == "/hw/depth_haz/extended/amplitude_int":
            np_im = ros_numpy.image.image_to_numpy(msg)
            data = Image.fromarray(np_im)

        if (
            topic == "/hw/depth_haz/points"
            or topic == "/hw/depth_haz/points/ground_truth"
        ):
            p = convert_pc2_pcl(msg)
            np_arr = filter_pcl(p)

            if "data" in locals():
                data.show()
                data.save("image.png")
                return np_arr
    return np_arr


# Process PCD point cloud directly
def read_pcd(pcdfile):
    p = pcl.load(pcdfile)
    np_arr = filter_pcl(p)
    return np_arr


if __name__ == "__main__":
    # filtered_data = read_pc2_msgs('/home/jcsanto3/bagfile-data/groundtruth/run5_precut.bag')
    concat_ground_truth_msgs("./groundtruth_run5.bag")
