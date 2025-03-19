from setuptools import find_packages
from setuptools import setup

setup(
    name='usb_servo',
    version='0.0.0',
    packages=find_packages(
        include=('usb_servo', 'usb_servo.*')),
)
