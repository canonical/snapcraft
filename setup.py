#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import codecs
import os
import re

from setuptools import setup

version = 'devel'
# look/set what version we have
changelog = 'debian/changelog'
if os.path.exists(changelog):
    head = codecs.open(changelog, encoding='utf-8').readline()
    match = re.compile('.*\((.*)\).*').match(head)
    if match:
        version = match.group(1)


setup(
    name='snapcraft',
    version=version,
    description='Easily craft snaps from multiple sources',
    author_email='snapcraft@lists.snapcraft.io',
    url='https://github.com/snapcore/snapcraft',
    packages=['snapcraft',
              'snapcraft.internal',
              'snapcraft.internal.states',
              'snapcraft.plugins',
              'snapcraft.storeapi'],
    package_data={'snapcraft.internal': ['manifest.txt']},
    scripts=['bin/snapcraft', 'bin/snapcraft-parser'],
    data_files=[
        ('share/snapcraft/schema',
            ['schema/' + x for x in os.listdir('schema')]),
        ('share/snapcraft/libraries',
            ['libraries/' + x for x in os.listdir('libraries')]),
    ],
    dependency_links=[
        'git+git://anonscm.debian.org/apt/python-apt.git#egg=python_apt-0.0.0',
        'https://launchpad.net/python-distutils-extra/trunk/2.39/+download/python-distutils-extra-2.39.tar.gz',
    ],
    install_requires=[
        'configparser',
        'docopt',
        'file-magic',
        'jsonschema',
        'oauthlib',
        'pkg-resources',
        'py3-progressbar',
        'pyxdg',
        'PyYAML',
        'requests',
        'python-apt',
        'python-distutils-extra',
        'requests-oauthlib',
        'requests-toolbelt',
    ],
    test_suite='snapcraft.tests',
    license='GPL v3',
    classifiers=(
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Intended Audience :: System Administrators',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Natural Language :: English',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Topic :: Software Development :: Build Tools',
        'Topic :: System :: Software Distribution',
    ),
)
