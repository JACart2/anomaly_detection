from setuptools import find_packages
from setuptools import setup

setup(
    name='anomaly_msg',
    version='0.0.1',
    packages=find_packages(
        include=('anomaly_msg', 'anomaly_msg.*')),
)
