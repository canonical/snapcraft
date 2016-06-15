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
        'https://launchpad.net/ubuntu/+archive/primary/+files/python-apt_1.1.0~beta1build1.tar.xz#egg=python_apt-0.0.0',
        'https://launchpad.net/python-distutils-extra/trunk/2.39/+download/python-distutils-extra-2.39.tar.gz',
    ],
    install_requires=[
        'configparser',
        'docopt==0.6.2',
        'file-magic',
        'jsonschema==2.5.1',
        'oauthlib==1.0.3',
        'pkg-resources',
        'progressbar33==2.4',
        'PyYAML==3.11',
        'pyxdg==0.25',
        'requests==2.9.1',
        'python-apt==0.0.0',
        'python-distutils-extra==2.39',
        'requests-oauthlib==0.4.0',
        'requests-toolbelt==0.6.0',
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
