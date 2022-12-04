from distutils.core import setup
from importlib.resources import Package
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['SG_PR'],
    package_dir={'':'src'}
)

setup(**d)