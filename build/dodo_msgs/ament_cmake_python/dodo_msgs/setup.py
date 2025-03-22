from setuptools import find_packages
from setuptools import setup

setup(
    name='dodo_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('dodo_msgs', 'dodo_msgs.*')),
)
