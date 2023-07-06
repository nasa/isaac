import glob
import re

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def get_all_files(folder):
    files = glob.glob(folder + "*_locations.txt")
    return files


def graph(files):
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")

    x = []
    y = []
    z = []
    for file in files:
        with open(file, "r") as f:
            for line in f:
                if "PCL Intersection" in line:
                    nums = re.findall(
                        r"[-+]?\d*\.\d+|\d+", line
                    )  # [x, y, z, roll, pitch, yaw]
                    x.append(float(nums[0]))
                    y.append(float(nums[1]))
                    z.append(float(nums[2]))

    ax.scatter(x, y, z)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title("Label Locations on ISS")

    return plt


if __name__ == "__main__":
    files = get_all_files(
        "/home/rlu3/isaac/src/anomaly/image_str/scripts/image_str/result/beehive/queen/"
    )
    plt = graph(files)
    plt.show()
