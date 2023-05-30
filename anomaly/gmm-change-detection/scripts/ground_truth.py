#!/usr/bin/python2

# Python2 script to convert a series of haz_cam point clouds
# to a single merged point cloud in the world reference frame

import geometry_msgs
import numpy as np
import ros_numpy
import rosbag
import rospy
import sensor_msgs
import tf2_py as tf2
import tf2_ros
from tf import transformations as ts
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


# Transform the pc2 msg camera coordinates from the haz_cam to body:
def translate_cam_frame(pc2_msg):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = pc2_msg.header.stamp
    t.header.frame_id = "haz_cam"
    t.child_frame_id = "body"

    t.transform.translation.x = 0.036
    t.transform.translation.y = -0.083
    t.transform.translation.z = -0.133

    # In quaternions:
    t.transform.rotation.x = 0.500
    t.transform.rotation.y = -0.500
    t.transform.rotation.z = 0.500
    t.transform.rotation.w = -0.500

    body_tf = do_transform_cloud(pc2_msg, t)
    return body_tf


# Transform point cloud coordinates to map ground truth (from Marina and Ryan)
def ground_truth(msg, pc2_msg):
    # Get transformation info from hazcam pc2 message
    t = geometry_msgs.msg.TransformStamped()
    # t.header.stamp = rospy.Time.now()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "body"
    t.child_frame_id = "world"

    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    cloud_out = do_transform_cloud(pc2_msg, t)
    return cloud_out


def convert_bag(bagfile, output_bag_name, save_all_topics=False):
    topics = [
        "/gnc/ekf",
        "/hw/depth_haz/points",
        "/hw/depth_haz/extended/amplitude_int",
    ]
    output_bag = rosbag.Bag(output_bag_name, "w")
    topics_bag = [] if save_all_topics else topics

    pc2_msg = None
    with rosbag.Bag(bagfile, "r") as bag:
        for topic, msg, t in bag.read_messages(topics_bag):
            if topic == "/hw/depth_haz/points":
                pc2_msg = msg
            if topic == "/gnc/ekf" and pc2_msg is not None:
                new_topic_name = "/hw/depth_haz/points/ground_truth"
                pc2_body = translate_cam_frame(pc2_msg)
                pc2_gt = ground_truth(msg, pc2_body)
                output_bag.write(topic, msg, t)
                output_bag.write(new_topic_name, pc2_gt, t)
                pc2_msg = None
            if save_all_topics:
                output_bag.write(topic, msg, t)
    output_bag.close()


if __name__ == "__main__":
    convert_bag(
        "/home/jcsanto3/bagfile-data/ground_truth/groundtruth_20230419_1836_survey_run5.bag",
        "./groundtruth_run5.bag",
    )
