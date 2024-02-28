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

# Based on the GMM visualization code:
#    https://github.com/sitzikbs/gmm_tutorial/blob/master/visualization.py
# with modifications to accomodate using the model data
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes._axes import _log as matplotlib_axes_logger
from mpl_toolkits.mplot3d import Axes3D

# Suppress matplotlib warnings
matplotlib_axes_logger.setLevel("ERROR")


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


def plot_sphere(w=0, c=[0, 0, 0], r=[1, 1, 1], subdev=10, ax=None, sigma_multiplier=3):
    """
    plot a sphere surface
    Input:
        c: 3 elements list, sphere center
        r: 3 element list, sphere original scale in each axis ( allowing to draw elipsoids)
        subdiv: scalar, number of subdivisions (subdivision^2 points sampled on the surface)
        ax: optional pyplot axis object to plot the sphere in.
        sigma_multiplier: sphere additional scale (choosing an std value when plotting gaussians)
    Output:
        ax: pyplot axis object
    """

    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
    pi = np.pi
    cos = np.cos
    sin = np.sin
    phi, theta = np.mgrid[
        0.0 : pi : complex(0, subdev), 0.0 : 2.0 * pi : complex(0, subdev)
    ]
    x = sigma_multiplier * r[0] * sin(phi) * cos(theta) + c[0]
    y = sigma_multiplier * r[1] * sin(phi) * sin(theta) + c[1]
    z = sigma_multiplier * r[2] * cos(phi) + c[2]
    cmap = cmx.ScalarMappable()
    cmap.set_cmap("jet")
    c = cmap.to_rgba(w)

    ax.plot_surface(x, y, z, color=c, alpha=0.2, linewidth=1)

    return ax


def visualize_3d_gmm(points, predictions, w, mu, stdev, n_gaussians, axes=None):
    """
    plots points and their corresponding gmm model in 3D
    Input:
        points: N X 3, sampled points
        w: n_gaussians, gmm weights
        mu: 3 X n_gaussians, gmm means
        stdev: 3 X n_gaussians, gmm standard deviation (assuming diagonal covariance matrix)
    Output:
        None
    """

    N = int(np.round(points.shape[0] / n_gaussians))

    # Visualize data
    axes = axes or plt.gca()
    plt.set_cmap("Set1")
    colors = cmx.Set1(np.linspace(0, 1, n_gaussians))
    for i in range(n_gaussians):
        idx = np.where(predictions == i)
        axes.scatter(
            points[idx, 0],
            points[idx, 1],
            points[idx, 2],
            alpha=0.07,
            c=colors[i],
            s=0.7,
        )

    for i in range(w.shape[0]):
        plot_sphere(w=w[i], c=mu[:, i], r=stdev[:, i], ax=axes)

    return axes
    # plt.title('3D GMM')
    # axes.set_xlabel('X')
    # axes.set_ylabel('Y')
    # axes.set_zlabel('Z')
    # plt.show()


def plot_gmm_results(
    gmm1_init,
    gmm2_init,
    points1,
    points2,
    predictions1,
    predictions2,
    pi_appearances,
    pi_disappearances,
):

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
    diag_covs1 = get_diag(gmm1_init.bestcov, gmm1_init.bestk)
    visualize_3d_gmm(
        points1,
        predictions1,
        gmm1_init.bestpp[0],
        gmm1_init.bestmu.T,
        np.sqrt(diag_covs1).T,
        gmm1_init.bestk,
        ax1,
    )

    # GMM 2 (Theta)
    diag_covs2 = get_diag(gmm2_init.bestcov, gmm2_init.bestk)
    visualize_3d_gmm(
        points2,
        predictions2,
        gmm2_init.bestpp[0],
        gmm2_init.bestmu.T,
        np.sqrt(diag_covs2).T,
        gmm2_init.bestk,
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
        gmm2_init.bestk,
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
        gmm2_init.bestk,
        ax4,
    )
