from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['maze_example_solution'],
    scripts=['scripts'],
    package_dir={'': 'scripts'}
)

setup(**d)