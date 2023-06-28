from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["image_str", "craft", "parseq"], package_dir={"": "scripts"}, 
    install_requires=[
        'CRAFT-pytorch @ git+ssh://git@github.com/clovaai/CRAFT-pytorch.git',
        'parseq @ git+ssh://git@github.com/baudm/parseq.git'
    ]
)

setup(**d)
