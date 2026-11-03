from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dynamixel_detachable_hand_common', 'hand_controller'],
    package_dir={'': 'src'},
)
setup(**d)
