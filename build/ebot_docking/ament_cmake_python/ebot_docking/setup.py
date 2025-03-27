from setuptools import find_packages
from setuptools import setup

setup(
    name='ebot_docking',
    version='0.0.0',
    packages=find_packages(
        include=('ebot_docking', 'ebot_docking.*')),
)
