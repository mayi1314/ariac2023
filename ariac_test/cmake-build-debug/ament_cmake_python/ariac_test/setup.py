import os
from setuptools import find_packages
from setuptools import setup

setup(
    name='ariac_test',
    version='0.0.1',
    packages=find_packages(
        include=('ariac_test', 'ariac_test.*')),
)
