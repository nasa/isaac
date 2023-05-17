# Based on the GMM visualization code:
#    https://github.com/sitzikbs/gmm_tutorial/blob/master/visualization.py
# with modifications to accomodate using the model data
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cmx
from matplotlib.axes._axes import _log as matplotlib_axes_logger

# Suppress matplotlib warnings
matplotlib_axes_logger.setLevel('ERROR')

def plot_sphere(w=0, c=[0,0,0], r=[1, 1, 1], subdev=10, ax=None, sigma_multiplier=3):
    '''
        plot a sphere surface
        Input: 
            c: 3 elements list, sphere center
            r: 3 element list, sphere original scale in each axis ( allowing to draw elipsoids)
            subdiv: scalar, number of subdivisions (subdivision^2 points sampled on the surface)
            ax: optional pyplot axis object to plot the sphere in.
            sigma_multiplier: sphere additional scale (choosing an std value when plotting gaussians)
        Output:
            ax: pyplot axis object
    '''

    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    pi = np.pi
    cos = np.cos
    sin = np.sin
    phi, theta = np.mgrid[0.0:pi:complex(0,subdev), 0.0:2.0 * pi:complex(0,subdev)]
    x = sigma_multiplier*r[0] * sin(phi) * cos(theta) + c[0]
    y = sigma_multiplier*r[1] * sin(phi) * sin(theta) + c[1]
    z = sigma_multiplier*r[2] * cos(phi) + c[2]
    cmap = cmx.ScalarMappable()
    cmap.set_cmap('jet')
    c = cmap.to_rgba(w)

    ax.plot_surface(x, y, z, color=c, alpha=0.2, linewidth=1)

    return ax

def visualize_3d_gmm(points, predictions, w, mu, stdev, n_gaussians, axes=None):
    '''
    plots points and their corresponding gmm model in 3D
    Input: 
        points: N X 3, sampled points
        w: n_gaussians, gmm weights
        mu: 3 X n_gaussians, gmm means
        stdev: 3 X n_gaussians, gmm standard deviation (assuming diagonal covariance matrix)
    Output:
        None
    '''

    N = int(np.round(points.shape[0] / n_gaussians))

    # Visualize data
    axes = axes or plt.gca()
    plt.set_cmap('Set1')
    colors = cmx.Set1(np.linspace(0, 1, n_gaussians))
    for i in range(n_gaussians):
        idx = np.where(predictions == i)
        axes.scatter(points[idx, 0], points[idx, 1], points[idx, 2], alpha=0.07, c=colors[i], s = 0.7)
    
    for i in range(w.shape[0]):
        plot_sphere(w=w[i], c=mu[:, i], r=stdev[:, i], ax=axes)

    return axes
    #plt.title('3D GMM')
    #axes.set_xlabel('X')
    #axes.set_ylabel('Y')
    #axes.set_zlabel('Z')
    #plt.show()

