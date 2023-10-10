from setuptools import find_packages
from setuptools import setup

setup(
    name='carbot_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('carbot_interfaces', 'carbot_interfaces.*')),
)
