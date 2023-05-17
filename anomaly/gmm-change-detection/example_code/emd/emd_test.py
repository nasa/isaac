import numpy as np
from sklearn.mixture import GaussianMixture

# Define the first GMM
gmm1 = GaussianMixture(n_components=2, covariance_type='full', random_state=42)
X1 = np.random.randn(100, 2)
gmm1.fit(X1)

# Define the second GMM
gmm2 = GaussianMixture(n_components=3, covariance_type='full', random_state=42)
X2 = np.random.randn(100, 2)
gmm2.fit(X2)

from scipy.stats import wasserstein_distance

# Extract the means and weights from the first GMM
means1 = gmm1.means_
weights1 = gmm1.weights_

# Extract the means and weights from the second GMM
means2 = gmm2.means_
weights2 = gmm2.weights_

emd = wasserstein_distance(weights1, weights2, np.linalg.norm(means1[:, np.newaxis] - means2, axis=-1))
print(emd)

