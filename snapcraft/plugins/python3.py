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

"""The python3 plugin can be used for python 3 based parts.

The python3 plugin can be used for python 3 projects where you would
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
    - python-packages:
      (list)
      A list of dependencies to get from PyPi
"""

import os
import re

import snapcraft
from snapcraft import common


class Python3Plugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['requirements'] = {
            'type': 'string',
        }
        schema['properties']['python-packages'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string'
            },
            'default': [],
        }
        schema['properties']['process-dependency-links'] = {
            'type': 'boolean',
            'default': False,
        }
        schema.pop('required')

        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        schema['pull-properties'].extend(['requirements', 'python-packages'])

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend([
            'python3-dev',
            'python3-pkg-resources',
            'python3-setuptools',
        ])
        self.stage_packages.extend(['python3'])

    def env(self, root):
        env = ['PYTHONUSERBASE={}'.format(root)]

        python_path = self._get_python_path(root)
        if python_path:
            env.append('PYTHONPATH={}'.format(python_path))

        return env

    def pull(self):
        super().pull()

        setup = 'setup.py'
        if os.listdir(self.sourcedir):
            setup = os.path.join(self.sourcedir, 'setup.py')

        self._run_pip(setup)

    def _get_pip_command(self):
        pip3 = os.path.join(os.path.sep, 'usr', 'bin', 'pip3')
        pip_install = [pip3, 'install', '--user']

        if self.options.process_dependency_links:
            pip_install.append('--process-dependency-links')

        return pip_install

    def _run_pip(self, setup):
        pip_install = self._get_pip_command()

        if self.options.requirements:
            self.run(pip_install + [
                '--requirement',
                os.path.join(self.sourcedir, self.options.requirements),
            ])

        if self.options.python_packages:
            self.run(pip_install + ['--upgrade'] +
                     self.options.python_packages)

        if os.path.exists(setup):
            self.run(pip_install + ['.'], cwd=self.sourcedir)

    def build(self):
        super().build()

        setup_file = os.path.join(self.builddir, 'setup.py')
        self._run_pip(setup_file)

        # Fix all shebangs to use the in-snap python.
        common.replace_in_file(self.installdir, re.compile(r''),
                               re.compile(r'#!.*python'),
                               r'#!/usr/bin/env python')

    def snap_fileset(self):
        fileset = super().snap_fileset()
        # .pth files are only read from the built-in site-packages directory.
        # We use PYTHONPATH for everything so not needed.
        fileset.append('-**/*.pth')
        # Holds all the .pyc files. It is a major cause of inter part
        # conflict.
        fileset.append('-**/__pycache__')
        return fileset

    def _get_python_path(self, root):
        python_path = ''
        python_lib_dir = os.path.join('usr', 'lib', 'python3')
        site_packages_dir = os.path.join(root, python_lib_dir, 'site-packages')
        if os.path.exists(site_packages_dir):
            python_path += site_packages_dir

        # This means we are in builder mode
        if root == self.installdir:
            dist_packages_dir = os.path.join(
                os.path.sep, python_lib_dir, 'dist-packages')
            if python_path:
                python_path += ':{}'.format(dist_packages_dir)
            else:
                python_path = dist_packages_dir

        return python_path
