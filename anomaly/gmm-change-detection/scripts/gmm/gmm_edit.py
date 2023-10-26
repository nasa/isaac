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


# Small class that holds precalculated GMM and
# allows clusters to be edited and removed

import numpy as np


class GMM:
    def __init__(self, weights, means, covariances):
        self.weights = weights
        self.means = means
        self.covariances = covariances
        self.n_gaussians = self.weights.size

    def remove_gaussian(self, index):  # Index of gaussian to remove
        # removed_gauss = GMM(self.weights[index], self.means[index,:].reshape((1,3)), self.covariances[index,:].reshape((1,3)))
        removed_gauss = GMM(
            self.weights[index],
            self.means[index, :].reshape((1, 3)),
            np.array(self.covariances[:, :, index].reshape((3, 3))),
        )
        new_weights = np.delete(self.weights, index, 0)
        self.weights = new_weights / np.linalg.norm(new_weights, ord=1)
        self.means = np.delete(self.means, index, 0)
        self.covariances = np.delete(self.covariances, index, -1)
        self.n_gaussians = self.weights.size
        return removed_gauss

    def add_gaussian(self, gaussian):  # Gaussian in form of GMM, k=1
        self.weights = np.append(
            self.weights, gaussian.weights
        )  # PLACEHOLDER (weights are now incorrect)
        self.means = np.append(self.means, gaussian.means, axis=0)
        # print("Gauss type: " + str(type(gaussian.covariances.reshape((3, 3)))))
        if type(self.covariances) is not np.ndarray:
            self.covariances = gaussian.covariances.reshape((3, 3))
            # print("Pi cov initialized: " + str(self.covariances.shape))
        else:
            # print("Curr cov size: " + str(self.covariances.shape))
            # print("Gauss cov size: " + str(gaussian.covariances.reshape((3, 3)).shape))
            # self.covariances = np.vstack([[self.covariances], [gaussian.covariances.reshape((3,3))]])
            self.covariances = np.concatenate(
                [self.covariances, gaussian.covariances], axis=0
            )
        self.n_gaussians = self.weights.size
