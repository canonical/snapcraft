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

import os
import subprocess

from setuptools import setup
from setuptools.command.test import test


setup(name="snapcraft",
      version='0.4',
      description="Easily craft snaps",
      author_email="snappy-devel@lists.ubuntu.com",
      url="https://launchpad.net/snapcraft",
      packages=['snapcraft',
                'snapcraft.plugins'],
      package_data={'snapcraft': ['manifest.txt']},
      scripts=['bin/snapcraft'],
      data_files=[
          ('share/snapcraft/schema',
              ['schema/' + x for x in os.listdir('schema')]),
      ],
      test_suite='snapcraft.tests',
      )
