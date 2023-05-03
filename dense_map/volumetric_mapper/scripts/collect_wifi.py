#!/usr/bin/env python
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

"""
Finds bags with ISAAC Wi-Fi data, converts each bag to a wifi + pose
CSV. This format is convenient for Chatwin Lansdowne's ISS/Gateway
Wi-Fi modeling team to use.

This script is designed to run in a Vagrant VM on hivemind where:
- The VM is limited to use four cores. (If not, you may want to update NUM_JOBS.)
- The /testsessions is mapped to /home/p-astrobee/webdir/testsessions
- The /shared path is mapped to an arbitrary folder in the host
  (output will be written here).

The current version hard-codes some paths unnecessarily. In theory it
would be easy to add more command-line flexibility, but it's not worth
it to invest in that improvement right now because the testing is very
slow, and we don't know of any definite need to run the script
frequently.
"""

import argparse
import collections
import csv
import glob
import json
import multiprocessing as mp
import os
import subprocess

import yaml

ROBOTS = {"SN003": "bumble", "SN004": "honey", "SN005": "queen"}

POSE_TOPIC = "/loc/pose"
WIFI_TOPIC = "/hw/wifi"
TOPICS = [POSE_TOPIC, WIFI_TOPIC]

NUM_JOBS = 4

OUT_PATH = "/shared/wifi_analysis"

SCAN_PATH = os.path.join(OUT_PATH, "bags_with_wifi.txt")

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
ACTIVITIES_PATH = os.path.jois(THIS_DIR, "isaac_activities.csv")


def dosys(cmd):
    print("+" + cmd)
    ret = os.system(cmd)
    if ret != 0:
        print("command exited with non-zero return value: %s" % ret)
    return ret


def has_wifi(bag_path):
    print()
    print("### has_wifi %s" % bag_path)

    cmd = "rosbag info --yaml %s" % bag_path
    print("+" + cmd)
    proc = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    proc.wait()
    if proc.returncode != 0:
        print(
            "WARNING: Could not determine topics in bag %s, bag probably not fixed"
            % bag_path
        )
        return bag_path, False

    bag_info = yaml.safe_load(proc.stdout)
    bag_topics = [entry["topic"] for entry in bag_info["topics"]]
    bag_has_wifi = all([need_topic in bag_topics for need_topic in TOPICS])
    print("bag_has_wifi: %s" % bag_has_wifi)
    return bag_path, bag_has_wifi


def scan_wifi():
    if os.path.exists(SCAN_PATH):
        print("Output file %s exists, not overwriting" % SCAN_PATH)
        return

    all_bags = []
    with open(ACTIVITIES_PATH, "r") as activities:
        rows = csv.reader(activities)
        for row in rows:
            activity_date = row[1]
            robot_data_dir = glob.glob("/testsessions/%s_*/robot_data" % activity_date)[
                0
            ]
            for sn, name in ROBOTS.items():
                bag_paths = glob.glob("%s/%s/bags/*.bag" % (robot_data_dir, sn))
                bags = [bag for bag in bag_paths if not "ars_default" in bag]
                all_bags += bags

    with mp.Pool(processes=NUM_JOBS) as pool:
        result = pool.imap(has_wifi, all_bags)
        bags_with_wifi = [bag_path for bag_path, bag_has_wifi in result if bag_has_wifi]

    scan_dir = os.path.dirname(os.path.realpath(SCAN_PATH))
    if not os.path.isdir(scan_dir):
        dosys("mkdir -p %s" % scan_dir)

    with open(SCAN_PATH, "w") as out:
        for bag in bags_with_wifi:
            out.write(bag + "\n")


def process_wifi(bag_path):
    print()
    print("### process_wifi %s" % bag_path)

    # parse bag_path to extract fields we care about
    path_elts = bag_path.split("/")
    session = path_elts[2]
    robot_sn = path_elts[4]
    robot_label = "%s_%s" % (robot_sn, ROBOTS[robot_sn])
    bag_prefix = os.path.splitext(path_elts[6])[0]

    out_dir = os.path.join(OUT_PATH, session, robot_label)
    out_path = os.path.join(out_dir, bag_prefix + ".csv")

    if os.path.exists(out_path):
        print("Output file %s exists, skipping" % out_path)
        return

    tmp_path = os.path.realpath(os.path.join(OUT_PATH, "tmp", bag_prefix))
    if not os.path.isdir(tmp_path):
        dosys("mkdir -p %s" % tmp_path)

    topic_args = " ".join(["-a %s" % topic for topic in TOPICS])
    filtered = os.path.join(tmp_path, "filtered.bag")
    dosys(
        "rosrun bag_processing rosbag_topic_filter.py %s %s -o %s"
        % (bag_path, topic_args, filtered)
    )

    join_files = []
    for topic in TOPICS:
        fname = topic.lstrip("/").replace("/", "_")
        csv_path = os.path.join(tmp_path, fname + ".csv")
        dosys("rostopic echo --bag=%s -p %s > %s" % (filtered, topic, csv_path))
        join_files.append(csv_path)

    out_path_part = out_path + ".part"
    if not os.path.isdir(out_dir):
        dosys("mkdir -p %s" % out_dir)

    join_files_str = " ".join(join_files)
    dosys("rosrun bag_processing csv_join.py %s %s" % (join_files_str, out_path_part))
    dosys("mv %s %s" % (out_path_part, out_path))


def collect_wifi():
    if not os.path.exists(SCAN_PATH):
        scan_wifi()

    print("reading %s" % SCAN_PATH)
    with open(SCAN_PATH, "r") as bags_in:
        bags = [b.rstrip() for b in bags_in.readlines()]
    print("found %s bags to process" % len(bags))

    with mp.Pool(processes=NUM_JOBS) as pool:
        result_iterator = pool.imap_unordered(process_wifi, bags)
        # exhaust iterator
        collections.deque(result_iterator, maxlen=0)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    args = parser.parse_args()

    collect_wifi()


if __name__ == "__main__":
    main()
