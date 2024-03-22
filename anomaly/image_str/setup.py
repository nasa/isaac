from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["image_str"],
    package_dir={"": "scripts"},
    install_requires=[
        "strhub>=4.7.1",
        "craft>=4.7.1",
        "gdown>=4.7.1",
        "ipython>=8.12.2",
        "ipywidgets>=8.0.7",
        "jellyfish>=1.0.0",
        "matplotlib>=3.7.2",
        "numpy>=1.24.4",
        "opencv_python>=4.8.0.74",
        "pandas>=2.0.3",
        "Pillow>=10.0.0",
        "skimage>=0.0",
        "torch>=2.0.1",
        "tqdm>=4.65.0",
    ],
)

setup(**d)
