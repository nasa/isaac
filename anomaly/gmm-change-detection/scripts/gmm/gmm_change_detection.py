#!/usr/bin/env python
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
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


import argparse
import os
import sys

import numpy as np
from gmm.artificial_data import generate_data
from gmm.gmm_edit import *
from gmm.gmm_mml import GmmMml
from gmm.preprocess_data import *
from gmm.utilities import *
from gmm.visualization import *

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)


def gmm_change_detection(
    input1,
    input2,
    output_directory,
    test,
    visualize,
    verbose,
    n_start=0,
    n_disappearances=0,
    n_appearances=0,
):

    # Path to input data (.pk for pickled GMMs, PCL file,
    # or bag files cropped to same positions at different times)

    # input1 = './data/ground_truth/ground_truth_run5.pcd'
    # input2 = './data/ground_truth/ground_truth_run4.pcd'

    # input2 = "./data/single_frame_bags/20230313_1901_survey_no_bag_panorama_precut.bag"
    # input1 = "./data/single_frame_bags/20230313_1908_survey_bag_panorama_precut.bag"

    # input1 = './data/single_frame_bags/run5_astrobee.bag'
    # input2 = './data/single_frame_bags/run1_astrobee.bag'

    # input1 = './data/image_reconstruction/run5.pcd'
    # input2 = './data/image_reconstruction/run2.pcd'

    # input1 = './saved_models/fake_data_t0_1.pk'
    # input2 = './saved_models/fake_data_t1_1.pk'

    # input1 = './saved_models/20230313_1901_survey_no_bag_panorama_precut_1.pk'
    # input2 = './saved_models/20230313_1908_survey_bag_panorama_precut_1.pk'

    # input1 = 'data/image_reconstruction/run5.pcd'
    # input2 = 'data/image_reconstruction/run4.pcd'

    # Get file extension
    ext = os.path.splitext(os.path.basename(input1))[1]
    current_dir = os.getcwd()

    #############################
    # Set up caching the models #
    #############################

    # Load pre-clustered GMMs if provided
    if ext == ".pk":
        gmm1_init, predictions1, points1 = read_pre_clustered(input1)
        gmm2_init, predictions2, points2 = read_pre_clustered(input2)
    else:
        if test:
            filename_1 = get_filename("test_data_t0")
            filename_2 = get_filename("test_data_t1")
        else:
            filename_1 = get_filename(os.path.splitext(os.path.basename(input1))[0])
            filename_2 = get_filename(os.path.splitext(os.path.basename(input2))[0])

        ###################
        # Format the data #
        ###################

        if test:
            points1, points2 = generate_data(n_start, n_disappearances, n_appearances)

        elif ext == ".bag":  # sensor_msgs::PointCloud2 data from bagfile
            points1 = read_pc2_msgs(input1)
            points2 = read_pc2_msgs(input2)

        elif ext == ".ply":  # PCL formatted file (e.g. from reconstructed map)
            points1 = read_ply(input1)
            points2 = read_ply(input2)
        else:
            sys.exit("Invalid file format")

        if not os.path.exists(output_directory):
            os.mkdir(output_directory)
        os.chdir(output_directory)

        # Plot the figures
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, projection="3d")
        ax1.scatter(points1[:, 0], points1[:, 1], points1[:, 2], s=0.7, alpha=0.07)
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111, projection="3d")
        ax2.scatter(points2[:, 0], points2[:, 1], points2[:, 2], s=0.7, alpha=0.07)
        plt.show()

        #####################################
        # Cluster the point clouds int GMMs #
        #####################################

        # Run split-and-merge expectation-maximization algorithm
        # described in "Unsupervised Learning of Finite Mixture Models" by Figueiredo et al.
        print("Fitting Gamma")
        gmm1_init = GmmMml()
        gmm1_init = gmm1_init.fit(points1, verb=verbose)
        predictions1 = gmm1_init.predict(points1)

        print("Fitting Theta")
        gmm2_init = GmmMml()
        gmm2_init = gmm2_init.fit(points2, verb=verbose)
        predictions2 = gmm2_init.predict(points2)

    print("Gamma number of Gaussians: " + str(gmm1_init.bestk))
    print("Theta number of Gaussians: " + str(gmm2_init.bestk))

    # Move the GMMs to a new data structure to be able to remove
    # Gaussians from GMMs for change detection
    gamma_t0 = GMM(gmm1_init.bestpp[0, :], gmm1_init.bestmu, gmm1_init.bestcov)
    theta_t1 = GMM(gmm2_init.bestpp[0, :], gmm2_init.bestmu, gmm2_init.bestcov)

    ######################################
    # Generate GMM with Detected Changes #
    ######################################
    pi_appearances = change_detection(gamma_t0, theta_t1)
    pi_disappearances = change_detection(theta_t1, gamma_t0)

    # Output change Gaussian information
    print("\nCHANGE GAUSSIANS")
    print("################################")
    print("Clusters associated with appearances: " + str(pi_appearances.n_gaussians))
    print(
        "Clusters associated with disappearances: " + str(pi_disappearances.n_gaussians)
    )

    if visualize:
        plot_gmm_results(
            gmm1_init,
            gmm2_init,
            points1,
            points2,
            predictions1,
            predictions2,
            pi_appearances,
            pi_disappearances,
        )

    ########################
    # Visualize the Output #
    ########################

    # Save model parameters for future use
    if ext != ".pk":
        save_pre_clustered(filename_1 + ".pk", gmm1_init, predictions1, points1)
        save_pre_clustered(filename_2 + ".pk", gmm2_init, predictions2, points2)

        print("Saved to: ")
        print("    " + str(filename_1 + ".pk"))
        print("    " + str(filename_2 + ".pk"))

    plt.show()

    os.chdir(current_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--input1",
        default="./data/single_frame_bags/20230313_1901_survey_no_bag_panorama_precut.bag",
        help="Filter suffix for bagfiles to merge. Bags should all be in the current working directory.",
    )
    parser.add_argument(
        "--input2",
        default="./data/single_frame_bags/20230313_1908_survey_bag_panorama_precut.bag",
        help="Filter suffix for bagfiles to merge. Bags should all be in the current working directory.",
    )
    parser.add_argument(
        "--test",
        dest="test",
        action="store_true",
        help="Test the GMM algorithm using generated data.",
    )
    parser.add_argument(
        "--visualize",
        dest="visualize",
        action="store_true",
        help="Visualize the results.",
    )
    parser.add_argument(
        "--verbose",
        dest="verbose",
        action="store_true",
        help="Print maximum output.",
    )
    parser.add_argument(
        "--n-start",
        dest="n_start",
        type=int,
        default=4,  # Set default value here
        help="Input double value. Default: 0.0",
    )
    parser.add_argument(
        "--n-disappearances",
        dest="n_disappearances",
        type=int,
        default=0,  # Set default value here
        help="Input double value. Default: 0.0",
    )
    parser.add_argument(
        "--n-appearances",
        dest="n_appearances",
        type=int,
        default=3,  # Set default value here
        help="Input double value. Default: 0.0",
    )
    parser.add_argument("-o", "--output-directory", default="gmm_saved_models")
    args = parser.parse_args()

    gmm_change_detection(
        args.input1,
        args.input2,
        args.output_directory,
        args.test,
        args.visualize,
        args.verbose,
        args.n_start,
        args.n_disappearances,
        args.n_appearances,
    )
