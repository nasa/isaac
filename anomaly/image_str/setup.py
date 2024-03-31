from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["image_str"],
    package_dir={"": "scripts"},
    install_requires=[
        "strhub>=1.2.0",
        "craft>=0.0.1",
        "gdown>=4.7.1",
        "ipython>=8.12.2",
        "ipywidgets>=8.0.7",
        "jellyfish>=1.0.0",
        "matplotlib>=3.7.2",
        "numpy>=1.24.4",
        "opencv_python>=4.8.0.74",
        "pandas>=2.0.3",
        "Pillow>=10.0.0",
        "scikit-image>=0.21.0",
        "torch>=2.0.1",
        "pytorch_lightning>=1.6.5",
        "tqdm>=4.65.0",
    ],
)

setup(**d)
