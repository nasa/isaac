import numpy as np
import pyemd
from sklearn.mixture import GaussianMixture

# Generate some random data for two Gaussian mixture models
np.random.seed(0)
n_samples = 1000
X1 = np.concatenate(
    [
        np.random.normal(-1, 1, int(0.3 * n_samples)),
        np.random.normal(5, 1, int(0.7 * n_samples)),
    ]
).reshape(-1, 1)
X2 = np.concatenate(
    [
        np.random.normal(0, 2, int(0.4 * n_samples)),
        np.random.normal(3, 1, int(0.6 * n_samples)),
    ]
).reshape(-1, 1)

# Fit Gaussian mixture models to the data
gm1 = GaussianMixture(n_components=2, random_state=0)
gm1.fit(X1)
gm2 = GaussianMixture(n_components=2, random_state=0)
gm2.fit(X2)

# Get the means and covariances of the components
means1 = gm1.means_.ravel()
covs1 = gm1.covariances_.ravel()
means2 = gm2.means_.ravel()
covs2 = gm2.covariances_.ravel()

# Create the distance matrix between the means
D = pyemd.euclidean_distances(means1.reshape(-1, 1), means2.reshape(-1, 1))

# Create the flow matrix between the components
flow = pyemd.emd(
    gm1.weights_, gm2.weights_, D, covs1.reshape(-1, 1), covs2.reshape(-1, 1)
)

# Print the EMD between the two Gaussian mixture models
print("EMD between the two Gaussian mixture models: {:.2f}".format(flow))
