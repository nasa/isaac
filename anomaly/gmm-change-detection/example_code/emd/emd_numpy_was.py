import numpy as np
import matplotlib.pyplot as plt

def emd(m1, m2):
    """Computes the Earth Movers Distance between two 3D Gaussian mixture models.

    Args:
        m1 (numpy.ndarray): The first 3D Gaussian mixture model.
        m2 (numpy.ndarray): The second 3D Gaussian mixture model.

    Returns:
        numpy.ndarray: The Earth Movers Distance between the two models.
    """
    # Compute the ground distance matrix.
    d = np.linalg.norm(m1 - m2, axis=2)

    # Compute the Earth Movers Distance.
    EMD = np.sum(np.min(d, d.T))

    return EMD

def wasserstein(m1, m2):
    """Computes the Wasserstein Distance between two 3D Gaussian mixture models.

    Args:
        m1 (numpy.ndarray): The first 3D Gaussian mixture model.
        m2 (numpy.ndarray): The second 3D Gaussian mixture model.

    Returns:
        numpy.ndarray: The Wasserstein Distance between the two models.
    """
    # Compute the optimal transport matrix.
    T = np.argmin(np.sum(m1 ** 2) + np.sum(m2 ** 2) - 2 * np.dot(m1, m2))

    # Compute the Wasserstein Distance.
    Wasserstein = np.sum(np.dot(m1, T) * d)

    return Wasserstein

# Generate two 3D Gaussian mixture models.
m1 = np.random.multivariate_normal(mean=np.array([0, 0, 0]), cov=np.eye(3), size=100)
m2 = np.random.multivariate_normal(mean=np.array([1, 1, 1]), cov=np.eye(3), size=100)

# Compute the Earth Movers Distance and Wasserstein Distance.
#EMD = emd(m1, m2)
Wasserstein = wasserstein(m1, m2)
