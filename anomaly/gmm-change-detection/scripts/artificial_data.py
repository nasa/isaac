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
        covs_1[i] = np.diag(np.random.uniform(-0.1, 0.1, (1, 3))[0].round(2))

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
        covs_appearances[i] = np.diag(np.random.uniform(-0.1, 0.1, (1, 3))[0].round(2))
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

# if fake_data:
#    # Generate 3D data with 4 clusters
#    # set Gaussian centers and covariances in 3D
#    means = np.array([[1, 0.0, 0.0],
#              [0.0, 0.0, 0.0],
#                      [-0.5, -0.5, -0.5],
#                      [-0.8, 0.3, 0.4]])
#    covs = np.array([np.diag([0.01, 0.01, 0.03]),
#                     np.diag([0.08, 0.01, 0.01]),
#                     np.diag([0.01, 0.05, 0.01]),
#                     np.diag([0.03, 0.07, 0.01])])
#
#    N = 1000 #Number of points to be generated for each cluster.
#    points_a = []
#    points_b = []
#
#    for i in range(len(means)):
#        x = np.random.multivariate_normal(means[i], covs[i], N )
#        points_a.append(x)
#        points_b.append(x)
#
#    points1 = np.concatenate(points_a)
#
#    if appearance:
#        # Add an extra Gaussian
#        means2 = np.array([[1.5, 1.5, 1.5],
#                          [0.2, 0.2, 0.2],
#                          [0.8, -.03, -0.4]])
#        covs2 = np.array([np.diag([0.01, 0.01, 0.01]),
#                         np.diag([0.02, 0.01, 0.03]),
#                         np.diag([0.03, 0.02, 0.01])])
#
#        for i in range(len(means2)):
#            x = np.random.multivariate_normal(means2[i], covs2[i], N )
#            points_b.append(x)
#
#        points2 = np.concatenate(points_b)
#
#    else:
#        # Remove an extra Gaussian
#        means2 = np.array([[1, 0.0, 0.0],
#                          [0.0, 0.0, 0.0],
#                          [-0.5, -0.5, -0.5]])
#        covs2 = np.array([np.diag([0.01, 0.01, 0.03]),
#                         np.diag([0.08, 0.01, 0.01]),
#                         np.diag([0.01, 0.05, 0.01])])
#        points_b = []
#
#        for i in range(len(means2)):
#            x2 = np.random.multivariate_normal(means2[i], covs2[i], N )
#            points_b.append(x2)
#
#        points2 = np.concatenate(points_b)
#
