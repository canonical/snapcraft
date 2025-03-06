#!/usr/bin/env python3

from distutils.core import setup

setup(
    name="hello",
    version="0.1",
    entry_points={"console_scripts": ["hello=hello:main"]},
    packages=["hello"],
)
