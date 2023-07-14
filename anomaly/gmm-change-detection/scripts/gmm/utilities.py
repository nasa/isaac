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


import copy
import os
import pickle

from gmm.emd_gmm import *
from gmm.gmm2 import *


def get_filename(original_file):
    """Make sure the filename is unique"""
    counter = 1
    filename = original_file + "_{}"
    while os.path.isfile(".saved_models/" + filename.format(counter)):
        counter += 1
    filename = filename.format(counter)
    return filename


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
        change_gauss = np.where(np.prod(theta.means == pi.means[gauss], axis=-1))
        theta_changes.append(change_gauss)

    # Readjust the Pi covariance shape back to [3,3,K]
    if pi.n_gaussians != 0:
        pi.covariances = pi.covariances.reshape(pi.n_gaussians, 3, 3).T
    return pi


def read_pre_clustered(filename):
    with open(filename, "rb") as fi:
        (
            gmm_init,
            predictions,
            points,
        ) = pickle.load(fi)
    return gmm_init, predictions, points


def save_pre_clustered(filename, gmm_init, predictions, points):
    with open(filename, "wb") as fi:
        pickle.dump(
            [
                gmm_init,
                predictions,
                points,
            ],
            fi,
        )
