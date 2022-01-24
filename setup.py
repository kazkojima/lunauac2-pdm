#!/usr/bin/env python3

from setuptools import setup
from setuptools import find_packages


setup(
    name="lunauac2-pdm",
    description="A LiteX module implementing a USB Audio module with PDM I/O",
    test_suite="test",
    license="BSD",
    python_requires="~=3.6",
    packages=find_packages(exclude=("test*", "sim*", "doc*", "examples*")),
    package_data={
        'lunauac2-pdm': ['verilog/**'],
    },
    include_package_data=True,
)
