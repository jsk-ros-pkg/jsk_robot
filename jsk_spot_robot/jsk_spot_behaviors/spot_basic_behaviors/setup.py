from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['spot_basic_behaviors'],
        package_dir={'': 'src'}
        )

setup(**d)
