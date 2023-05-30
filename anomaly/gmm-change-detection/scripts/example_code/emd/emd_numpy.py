import numpy as np


def earth_movers_distance(m1, m2):
    """Compute the Earth Movers Distance between two Gaussian mixture models.

    Parameters
    ----------
    m1 : np.ndarray
        Array of means and covariances of the first Gaussian mixture model.
    m2 : np.ndarray
        Array of means and covariances of the second Gaussian mixture model.

    Returns
    -------
    np.ndarray
        Earth Movers Distance between the two Gaussian mixture models.
    """
    # Compute the mean and covariance of the combined Gaussian mixture model.
    mean_combined = (m1.mean() + m2.mean()) / 2
    covariance_combined = (np.cov(m1) + np.cov(m2)) / 2

    # Compute the Earth Movers Distance between the two Gaussian mixture models.
    earth_movers_distance = np.linalg.norm(mean_combined - m1.mean()) + np.linalg.norm(
        mean_combined - m2.mean()
    )

    return earth_movers_distance


def main():
    m1 = np.random.multivariate_normal((0, 0, 0), np.eye(3))
    m2 = np.random.multivariate_normal((1, 2, 3), np.eye(3))
    print(m1.shape)
    print(m1)

    # Compute the Earth Movers Distance between the two Gaussian mixture models.
    emd = earth_movers_distance(m1, m2)

    # Print the Earth Movers Distance.
    print(emd)


if __name__ == "__main__":
    main()
