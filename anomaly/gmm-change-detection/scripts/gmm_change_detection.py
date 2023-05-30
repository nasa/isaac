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

import copy
import os
import pickle
import sys

import numpy as np
from artificial_data import generate_data
from emd_gmm import *
from gmm import *
from gmm_mml import GmmMml
from preprocess_data import *
from visualization import *

np.set_printoptions(threshold=sys.maxsize)
np.set_printoptions(suppress=True)

############
# Settings #
############

visualize = True  # Visualize GMMs

fake_data = True  # Generate point clouds
n_start = 4
n_disappearances = 0
n_appearances = 3

# Path to input data (.pk for pickled GMMs, PCL file,
# or bag files cropped to same positions at different times)

# t_0 = './data/ground_truth/ground_truth_run5.pcd'
# t_1 = './data/ground_truth/ground_truth_run4.pcd'

t_1 = "./data/single_frame_bags/20230313_1901_survey_no_bag_panorama_precut.bag"
t_0 = "./data/single_frame_bags/20230313_1908_survey_bag_panorama_precut.bag"

# t_0 = './data/single_frame_bags/run5_astrobee.bag'
# t_1 = './data/single_frame_bags/run1_astrobee.bag'

# t_0 = './data/image_reconstruction/run5.pcd'
# t_1 = './data/image_reconstruction/run2.pcd'

# t_0 = './saved_models/fake_data_t0_1.pk'
# t_1 = './saved_models/fake_data_t1_1.pk'

# t_0 = './saved_models/20230313_1901_survey_no_bag_panorama_precut_1.pk'
# t_1 = './saved_models/20230313_1908_survey_bag_panorama_precut_1.pk'

# t_0 = 'data/image_reconstruction/run5.pcd'
# t_1 = 'data/image_reconstruction/run4.pcd'

# Get file extension
ext = os.path.splitext(os.path.basename(t_0))[1]

#############################
# Set up caching the models #
#############################
def get_filename(original_file):
    """Make sure the filename is unique"""
    counter = 1
    filename = original_file + "_{}"
    while os.path.isfile(".saved_models/" + filename.format(counter)):
        counter += 1
    filename = filename.format(counter)
    return filename


# Load pre-clustered GMMs if provided
if ext == ".pk":
    with open(t_0, "rb") as fi:
        (
            gmm1_init_bestk,
            gmm1_init_bestpp,
            gmm1_init_bestcov,
            gmm1_init_bestmu,
            predictions1,
        ) = pickle.load(fi)
    with open(t_1, "rb") as fi:
        (
            gmm2_init_bestk,
            gmm2_init_bestpp,
            gmm2_init_bestcov,
            gmm2_init_bestmu,
            predictions2,
        ) = pickle.load(fi)
else:
    if fake_data:
        filename_1 = get_filename("fake_data_t0")
        filename_2 = get_filename("fake_data_t1")
    else:
        filename_1 = get_filename(os.path.splitext(os.path.basename(t_0))[0])
        filename_2 = get_filename(os.path.splitext(os.path.basename(t_1))[0])

    t0_save_file = (
        os.path.dirname(os.path.realpath(__file__))
        + "/saved_models/"
        + filename_1
        + ".pk"
    )
    t1_save_file = (
        os.path.dirname(os.path.realpath(__file__))
        + "/saved_models/"
        + filename_2
        + ".pk"
    )

    ###################
    # Format the data #
    ###################

    if fake_data:
        points1, points2 = generate_data(n_start, n_disappearances, n_appearances)

    elif ext == ".bag":  # sensor_msgs::PointCloud2 data from bagfile
        points1 = read_pc2_msgs(t_0)
        points2 = read_pc2_msgs(t_1)

    elif ext == ".pcd":  # PCL formatted file (e.g. from reconstructed map)
        points1 = read_pcd(t_0)
        points2 = read_pcd(t_1)
    else:
        sys.exit("Invalid file format")

    # Plot the figures
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection="3d")
    ax1.scatter(points1[:, 0], points1[:, 1], points1[:, 2], s=0.7, alpha=0.07)
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection="3d")
    ax2.scatter(points2[:, 0], points2[:, 1], points2[:, 2], s=0.7, alpha=0.07)
    # plt.show()

    #####################################
    # Cluster the point clouds int GMMs #
    #####################################

    # Run split-and-merge expectation-maximization algorithm
    # described in "Unsupervised Learning of Finite Mixture Models" by Figueiredo et al.
    print("Fitting Gamma")
    gmm1_init = GmmMml()
    gmm1_init = gmm1_init.fit(points1, verb=True)
    gmm1_init_bestk = gmm1_init.bestk
    gmm1_init_bestpp = gmm1_init.bestpp
    gmm1_init_bestcov = gmm1_init.bestcov
    gmm1_init_bestmu = gmm1_init.bestmu
    predictions1 = gmm1_init.predict(points1)

    print("Fitting Theta")
    gmm2_init = GmmMml()
    gmm2_init = gmm2_init.fit(points2, verb=True)
    gmm2_init_bestk = gmm2_init.bestk
    gmm2_init_bestpp = gmm2_init.bestpp
    gmm2_init_bestcov = gmm2_init.bestcov
    gmm2_init_bestmu = gmm2_init.bestmu
    predictions2 = gmm2_init.predict(points2)


print("Gamma number of Gaussians: " + str(gmm1_init_bestk))
print("Theta number of Gaussians: " + str(gmm2_init_bestk))

# Move the GMMs to a new data structure to be able to remove
# Gaussians from GMMs for change detection
gamma_t0 = GMM(gmm1_init_bestpp[0, :], gmm1_init_bestmu, gmm1_init_bestcov)
theta_t1 = GMM(gmm2_init_bestpp[0, :], gmm2_init_bestmu, gmm2_init_bestcov)

######################################
# Generate GMM with Detected Changes #
######################################


def find_emd(gmm1, gmm2):
    """Find the Earth Mover's Distance between two GMMs"""
    emdgmm = EMDGMM(gmm1.weights, gmm2.weights)
    emdgmm.get_distance(gmm1.means, gmm2.means)
    emdgmm.calculate_emd()
    return emdgmm.emd


def greedy_select_gmm(gamma, theta, start_emd):
    """Find the Gaussian in Theta that contributes the
    highest degree of positive change (i.e., removal results
    in the lowest EMD) and return "best" Theta GMM from which such
    Gaussian has been removed
    """

    lowest_emd = start_emd  # Metric for degree of change
    best_theta = theta
    best_gauss = GMM(
        np.array([]), np.array([]).reshape((0, 3)), np.array([]).reshape((0, 3))
    )  # GMM w/ k=1

    # Remove Gaussians one at a time and track which contributes most change
    for gauss in range(theta.n_gaussians - 1):
        theta_temp = copy.deepcopy(theta)
        removed_gauss = theta_temp.remove_gaussian(gauss)  # Remove the next Gaussian
        new_emd = find_emd(gamma, theta_temp)

        # print("NEW EMD: " + str(new_emd) + " LOWEST: " + str(lowest_emd))

        if new_emd < lowest_emd:
            lowest_emd = new_emd
            best_theta = theta_temp
            best_gauss = removed_gauss

    return lowest_emd, best_theta, best_gauss


def change_detection(gamma, theta):
    """Greedily remove Gaussians that contribute the most change
    and place them into a tertiary "change" GMM
    """

    # Initialize empty Pi (change) GMM
    pi = GMM(np.array([]), np.array([]).reshape((0, 3)), None)

    # Iteratively remove Gaussians from "after" GMM until EMD is the same
    dGMM_old = find_emd(gamma, theta)
    dGMM, new_theta, best_gauss = greedy_select_gmm(gamma, theta, dGMM_old)

    for i in range(new_theta.n_gaussians):
        if best_gauss.n_gaussians == 0:
            break

        # Update Pi with new Gaussian
        pi.add_gaussian(best_gauss)

        # Obtain "best Gaussian" that produces highest degree of change
        dGMM, new_theta, best_gauss = greedy_select_gmm(gamma, new_theta, dGMM_old)
        if dGMM_old > dGMM:
            dGMM_old = dGMM
        else:
            break

    # Map the change Gaussians back to the original Theta GMM
    theta_changes = []
    for gauss in range(pi.n_gaussians):
        change_gauss = np.where(np.prod(gmm2_init_bestmu == pi.means[gauss], axis=-1))
        theta_changes.append(change_gauss)

    # Readjust the Pi covariance shape back to [3,3,K]
    if pi.n_gaussians != 0:
        pi.covariances = pi.covariances.reshape(pi.n_gaussians, 3, 3).T
    return pi


pi_appearances = change_detection(gamma_t0, theta_t1)
pi_disappearances = change_detection(theta_t1, gamma_t0)

# Output change Gaussian information
print("\nCHANGE GAUSSIANS")
print("################################")
print("Clusters associated with appearances: " + str(pi_appearances.n_gaussians))
print("Clusters associated with disappearances: " + str(pi_disappearances.n_gaussians))


def get_diag(covs, k):
    """Get diagonal covariance from full covariance
    matrix for plotting"""

    diag_covs = []
    for i in range(k):
        cov = covs[:, :, i]
        l = len(cov[0])
        diag = [cov[j][j] for j in range(l)]
        diag_covs.append(diag)
    return np.array(diag_covs)


########################
# Visualize the Output #
########################
if visualize:
    gmm1_k = gmm1_init_bestk
    gmm2_k = gmm2_init_bestk

    fig = plt.figure(figsize=(12, 4))
    ax1 = fig.add_subplot(141, projection="3d")
    ax1.set(xlabel="X", ylabel="Y", zlabel="Z")
    ax1.view_init(elev=-75, azim=-90)
    ax1.title.set_text("Gamma (Before)")

    ax2 = fig.add_subplot(142, projection="3d")
    ax2.set(xlabel="X", ylabel="Y", zlabel="Z")
    ax2.view_init(elev=-75, azim=-90)
    ax2.title.set_text("Theta (After)")

    ax3 = fig.add_subplot(143, projection="3d")
    ax3.set(xlabel="X", ylabel="Y", zlabel="Z")
    ax3.view_init(elev=-75, azim=-90)
    ax3.title.set_text("Pi (Appearances)")

    ax4 = fig.add_subplot(144, projection="3d")
    ax4.set(xlabel="X", ylabel="Y", zlabel="Z")
    ax4.view_init(elev=-75, azim=-90)
    ax4.title.set_text("Pi (Disappearances)")

    # GMM 1 (Gamma)
    diag_covs1 = get_diag(gmm1_init_bestcov, gmm1_init_bestk)
    visualize_3d_gmm(
        points1,
        predictions1,
        gmm1_init_bestpp[0],
        gmm1_init_bestmu.T,
        np.sqrt(diag_covs1).T,
        gmm1_k,
        ax1,
    )

    # GMM 2 (Theta)
    diag_covs2 = get_diag(gmm2_init_bestcov, gmm2_init_bestk)
    visualize_3d_gmm(
        points2,
        predictions2,
        gmm2_init_bestpp[0],
        gmm2_init_bestmu.T,
        np.sqrt(diag_covs2).T,
        gmm2_k,
        ax2,
    )

    # Appearance GMM (Pi 1), compared to GMM2 original points
    piagonal = get_diag(pi_appearances.covariances, pi_appearances.weights.shape[0])
    visualize_3d_gmm(
        points2,
        predictions2,
        pi_appearances.weights,
        pi_appearances.means.T,
        np.sqrt(piagonal).T,
        gmm2_k,
        ax3,
    )

    # Disappearance GMM (Pi 2), compared to GMM2 original points
    piagonal_dis = get_diag(
        pi_disappearances.covariances, pi_disappearances.weights.shape[0]
    )
    visualize_3d_gmm(
        points2,
        predictions2,
        pi_disappearances.weights,
        pi_disappearances.means.T,
        np.sqrt(piagonal_dis).T,
        gmm2_k,
        ax4,
    )

    # Save model parameters for future use
    if ext != ".pk":
        with open(t0_save_file, "wb") as fi:
            pickle.dump(
                [
                    gmm1_init.bestk,
                    gmm1_init.bestpp,
                    gmm1_init.bestcov,
                    gmm1_init.bestmu,
                    predictions1,
                    points1,
                ],
                fi,
            )
        with open(t1_save_file, "wb") as fi:
            pickle.dump(
                [
                    gmm2_init.bestk,
                    gmm2_init.bestpp,
                    gmm2_init.bestcov,
                    gmm2_init.bestmu,
                    predictions2,
                    points2,
                ],
                fi,
            )

        print("Saved to: ")
        print("    " + str(t0_save_file))
        print("    " + str(t1_save_file))

plt.show()
