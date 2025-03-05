from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

d = generate_distutils_setup(
    packages=['softhand_ros'],
    package_dir={'': 'python'}
)

setup(**d)
