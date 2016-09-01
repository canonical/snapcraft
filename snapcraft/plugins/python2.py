# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

"""The python2 plugin can be used for python 2 based parts.

The python2 plugin can be used for python 2 projects where you would
want to do:

    - import python modules with a requirements.txt
    - build a python project that has a setup.py
    - install sources straight from pip

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - requirements:
      (string)
      path to a requirements.txt file
    - constraints:
      (string)
      path to a constraints file
    - python-packages:
      (list)
      A list of dependencies to get from PyPi
"""

import os

from snapcraft.plugins import python3


class Python2Plugin(python3.Python3Plugin):

    @property
    def plugin_build_packages(self):
        return [
            'python-dev',
            'python-pip',
            'python-pkg-resources',
            'python-setuptools',
        ]

    @property
    def plugin_stage_packages(self):
        return ['python']

    @property
    def python_version(self):
        return 'python2'

    @property
    def system_pip_command(self):
        return os.path.join(os.path.sep, 'usr', 'bin', 'pip')

    def snap_fileset(self):
        fileset = super().snap_fileset()
        # This is a major cause of inter part conflict.
        fileset.append('-**/*.pyc')
        return fileset
