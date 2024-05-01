# Copyright © 2021, United States Government, as represented by the Administrator of the
# National Aeronautics and Space Administration. All rights reserved.
#
# The “ISAAC - Integrated System for Autonomous and Adaptive Caretaking platform” software is
# licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under the
# License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific language governing
# permissions and limitations under the License.
#

import json
import os
import subprocess

import numpy as np
import quaternion
from pyArango.connection import *


# Get nested dictionary key values
def get_nested_value(data, keys):
    value = data
    for key in keys:
        if key in value:
            value = value[key]
        else:
            return None
    return value


def query_image(
    target_position,
    target_attitude,
    ros_topic_pose,
    ros_topic_image,
    max_distance,
    min_distance,
    max_angle,
    target_size_y,
    target_size_z,
    bag,
):
    # Connect to the database
    addresses = ["http://iui_arangodb:8529", "http://127.0.0.1:8529"]
    conn = None
    for address in addresses:
        try:
            conn = Connection(
                arangoURL=address, username="root", password="isaac", max_retries=5, timeout=300
            )
            break  # Connection successful, exit the loop
        except Exception as e:
            continue

    # Open the isaac database / create it if it does not exist
    if not conn.hasDatabase("isaac"):
        print("There is no isaac database, did you load it?")
    else:
        db = conn["isaac"]
        print("Connected to isaac database")

    
    # Query pose topic for the closest timestamps to the image topic & only get nearby pairs
    query = (
        "FOR doc_image IN "
        + ros_topic_image
        + "  LET image_stamp = doc_image.header.stamp.secs + doc_image.header.stamp.nsecs * 0.000000001"
        + "  LET closest_pose = ("
        + "    FOR doc_pose IN "
        + ros_topic_pose
        + "      FILTER doc_pose.bagfile == '{}'".format(bag)
        + "      FILTER doc_pose.header.stamp.secs + doc_pose.header.stamp.nsecs * 0.000000001 >= image_stamp - 1.0"
        + "      FILTER doc_pose.header.stamp.secs + doc_pose.header.stamp.nsecs * 0.000000001 <= image_stamp + 1.0"
        + "      SORT ABS((doc_pose.header.stamp.secs + doc_pose.header.stamp.nsecs * 0.000000001) - image_stamp)"
        + "      LIMIT 1"
        + "      RETURN doc_pose"
        + "    )"
        + "  FILTER closest_pose[0].pose.position.x >= "
        + str(target_position[0] - max_distance)
        + "    AND closest_pose[0].pose.position.x <= "
        + str(target_position[0] + max_distance)
        + "    AND closest_pose[0].pose.position.y >= "
        + str(target_position[1] - max_distance)
        + "    AND closest_pose[0].pose.position.y <= "
        + str(target_position[1] + max_distance)
        + "    AND closest_pose[0].pose.position.z >= "
        + str(target_position[2] - max_distance)
        + "    AND closest_pose[0].pose.position.z <= "
        + str(target_position[2] + max_distance)
        + "\n"
        + "  RETURN {doc_image, closest_pose: closest_pose[0]}"
        
    )

    result = list(db.AQLQuery(query, rawResults=True))

    print("From database got " + str(len(result)) + " matches")
    # Extract the rotated vector
    target_vector = quaternion.rotate_vectors(target_attitude, np.array([1, 0, 0]))

    for i in range(len(result) - 1, -1, -1):

        # Calculate the first vector from the difference between the points
        robot_position = np.array(
            [
                result[i]["closest_pose"]["pose"]["position"]["x"],
                result[i]["closest_pose"]["pose"]["position"]["y"],
                result[i]["closest_pose"]["pose"]["position"]["z"],
            ]
        )
        robot_target_vector = target_position - robot_position

        # Calculate the angle between the two vectors
        angle = np.arccos(
            np.dot(robot_target_vector, target_vector)
            / (np.linalg.norm(robot_target_vector) * np.linalg.norm(target_vector))
        )

        if np.degrees(angle) > max_angle:
            del result[i]
            continue

        # Perform the quaternion rotation
        robot_attitude = np.quaternion(
            result[i]["closest_pose"]["pose"]["orientation"]["w"],
            result[i]["closest_pose"]["pose"]["orientation"]["x"],
            result[i]["closest_pose"]["pose"]["orientation"]["y"],
            result[i]["closest_pose"]["pose"]["orientation"]["z"],
        )
        # Extract the rotated vector
        robot_vector = quaternion.rotate_vectors(robot_attitude, np.array([1, 0, 0]))

        # Calculate the angle between the two vectors
        angle = np.arccos(
            np.dot(robot_vector, target_vector)
            / (np.linalg.norm(robot_vector) * np.linalg.norm(target_vector))
        )
        if np.degrees(angle) > max_angle:
            del result[i]
            continue

    print("From first filtering got " + str(len(result)) + " matches")

    # Prep results for camera projection
    selected_keys = [
        "doc_image/header",
        "closest_pose/pose/position/x",
        "closest_pose/pose/position/y",
        "closest_pose/pose/position/z",
        "closest_pose/pose/orientation/x",
        "closest_pose/pose/orientation/y",
        "closest_pose/pose/orientation/z",
        "closest_pose/pose/orientation/w",
    ]
    selected_data = [
        [
            item[key] if "/" not in key else get_nested_value(item, key.split("/"))
            for key in selected_keys
        ]
        for item in result
    ]
    # Append three numbers after each the dictionary

    c1 = target_position + quaternion.rotate_vectors(
        target_attitude, np.array([0.0, target_size_y, target_size_z])
    )
    c2 = target_position + quaternion.rotate_vectors(
        target_attitude, np.array([0.0, target_size_y, -target_size_z])
    )
    c3 = target_position + quaternion.rotate_vectors(
        target_attitude, np.array([0.0, -target_size_y, -target_size_z])
    )
    c4 = target_position + quaternion.rotate_vectors(
        target_attitude, np.array([0.0, -target_size_y, target_size_z])
    )

    for element in selected_data:
        element[0] = (
            str(element[0]["stamp"]["secs"])
            + "."
            + "%03d" % (element[0]["stamp"]["nsecs"] * 0.000001)
            + ".jpg"
        )
        element.append(target_position.tolist())
        element.append(c1.tolist())
        element.append(c2.tolist())
        element.append(c3.tolist())
        element.append(c4.tolist())

    # Convert the input list to a JSON string
    input_json = json.dumps(selected_data)

    os.environ["ASTROBEE_CONFIG_DIR"] = "/src/astrobee/src/astrobee/config"
    os.environ["ASTROBEE_RESOURCE_DIR"] = "/src/astrobee/src/astrobee/resource"
    os.environ["ASTROBEE_ROBOT"] = "bsharp"
    os.environ["ASTROBEE_WORLD"] = "granite"

    cmd = ["rosrun", "analyst_notebook", "query_view_points", input_json]

    # Execute the C++ script and pass the input JSON as a command-line argument
    stdout = ""
    stderr = ""
    popen = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True
    )
    for stdout_line in iter(popen.stdout.readline, ""):
        stdout += stdout_line

    popen.stdout.close()
    data_list = json.loads(stdout)
    print("Query successful, got " + str(len(result)) + " matches")

    return data_list
