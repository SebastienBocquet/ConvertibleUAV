#!/usr/bin/env python
# coding: utf-8

import io
from setuptools import setup, find_packages


# http://blog.ionelmc.ro/2014/05/25/python-packaging/
setup(
    name="pyUavFlightMech",
    version="0.0",
    description="Python Flight Mechanics for Convertible UAV",
    author="Sebastien Bocquet",
    author_email="sbocquet@gmail.com",
    url="",
    download_url="",
    license="",
    keywords=[
      "aero", "UAV", "VTOL",
      "flight mechanics"],
    python_requires=">=3.7",
    install_requires=[
        "numpy",
        "scipy",
        "matplotlib",
        "pandas"
    ],
    tests_require=[
        "pytest"
    ],
    packages=find_packages('uav_model'),
    package_dir={'': 'uav_model'},
    classifiers=[
      "Development Status :: Pre-Alpha",
      "Intended Audience :: Education",
      "Intended Audience :: Science/Research",
      "Operating System :: OS Independent",
      "Programming Language :: Python :: 3.7",
      "Topic :: Scientific/Engineering",
      "Topic :: Scientific/Engineering :: Physics",
      "Topic :: Scientific/Engineering :: UAV",
      "Topic :: Scientific/Engineering :: VTOL",
    ],
    long_description=io.open('README.rst', encoding='utf-8').read(),
    include_package_data=True,
    zip_safe=False,
)
