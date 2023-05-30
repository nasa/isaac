import numpy as np
from scipy.stats import wasserstein_distance
from sklearn.mixture import GaussianMixture

# Generate some data
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
weights1 = gm1.weights_

means2 = gm2.means_.ravel()
covs2 = gm2.covariances_.ravel()
weights2 = gm2.weights_

# Calculate the EMD between the GMMs
emd = wasserstein_distance(
    means1, means2, u_weights=weights1, v_weights=weights2, metric="euclidean"
)
print("The EMD between the two GMMs is: " + emd)
