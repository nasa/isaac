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

import numpy as np


def generate_data(n_start, n_disappearances, n_appearances):
    N = 1000  # N points in each cluster
    point_set_1 = []
    point_set_2 = []

    # Define first set of points
    means_1 = np.random.uniform(-2, 2, (n_start, 3)).round(2)
    covs_1 = np.zeros(shape=(n_start, 3, 3))
    for i in range(n_start):
        covs_1[i] = np.diag(np.random.uniform(0.0, 0.1, (1, 3))[0].round(2))

    # Remove old clusters in second set of points
    means_2 = copy.deepcopy(means_1)
    covs_2 = copy.deepcopy(covs_1)

    for i in range(n_disappearances):
        means_2 = np.delete(means_2, 0, 0)
        covs_2 = np.delete(covs_2, 0, 0)

    # Add new clusters in second set of points
    means_appearances = np.random.uniform(-2, 2, (n_appearances, 3)).round(2)
    covs_appearances = np.zeros(shape=(n_appearances, 3, 3))
    for i in range(n_appearances):
        covs_appearances[i] = np.diag(np.random.uniform(0.0, 0.1, (1, 3))[0].round(2))
    means_2 = np.vstack((means_2, means_appearances))
    covs_2 = np.vstack((covs_2, covs_appearances))

    # Concatenate clusters into point clouds
    for i in range(len(means_1)):
        x = np.random.multivariate_normal(means_1[i], covs_1[i], N)
        point_set_1.append(x)
    points_1 = np.concatenate(point_set_1)

    for i in range(len(means_2)):
        x = np.random.multivariate_normal(means_2[i], covs_2[i], N)
        point_set_2.append(x)
    points_2 = np.concatenate(point_set_2)

    return points_1, points_2


if __name__ == "__main__":
    points_1, points_2 = generate_data(5, 3, 9)
    print(points_1.shape, points_2.shape)
