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
    covariance_combined = (m1.covariance() + m2.covariance()) / 2

    # Compute the Earth Movers Distance between the two Gaussian mixture models.
    earth_movers_distance = np.linalg.norm(mean_combined - m1.mean()) + np.linalg.norm(
        mean_combined - m2.mean()
    )

    return earth_movers_distance


def main():
    m1 = np.random.multivariate_normal(mean=[0, 0, 0], covariance=np.eye(3))
    m2 = np.random.multivariate_normal(mean=[1, 2, 3], covariance=np.eye(3))

    # Compute the Earth Movers Distance between the two Gaussian mixture models.
    earth_movers_distance = earth_movers_distance(m1, m2)

    # Print the Earth Movers Distance.
    print(
        "The Earth Movers Distance between the two Gaussian mixture models is "
        + earth_movers_distance
    )


if __name__ == "__main__":
    main()
