#!/usr/bin/env python3

from setuptools import setup

setup(
    name='config',
    description='config for webcam-webui',
    author='Sergio Schvezov <sergio.schvezov@canonical.com>',
    license='GPLv3',
    install_requires=[
        'pyyaml',
    ],
    scripts=[
        'config.py',
    ],
)
