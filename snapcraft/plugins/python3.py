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
    - constraints:
      (string)
      path to a constraints file
    - python-packages:
      (list)
      A list of dependencies to get from PyPi
"""

import os
import re
import subprocess
from glob import glob

import snapcraft
from snapcraft import common


class Python3Plugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['requirements'] = {
            'type': 'string',
        }
        schema['properties']['constraints'] = {
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
        schema['pull-properties'].extend([
            'requirements',
            'constraints',
            'python-packages'
        ])

        return schema

    @property
    def plugin_build_packages(self):
        return [
            'python3-dev',
            'python3-pip',
            'python3-pkg-resources',
            'python3-setuptools',
        ]

    @property
    def plugin_stage_packages(self):
        return ['python3']

    @property
    def python_version(self):
        return 'python3'

    @property
    def system_pip_command(self):
        return os.path.join(os.path.sep, 'usr', 'bin', 'pip3')

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(self.plugin_build_packages)
        self.stage_packages.extend(self.plugin_stage_packages)

    def env(self, root):
        return [
            'PYTHONUSERBASE={}'.format(root),
            'PYTHONHOME={}'.format(os.path.join(root, 'usr'))
        ]

    def pull(self):
        super().pull()

        setup = 'setup.py'
        if os.listdir(self.sourcedir):
            setup = os.path.join(self.sourcedir, 'setup.py')

        self._run_pip(setup)

    def _install_pip(self):
        env = os.environ.copy()
        env['PYTHONUSERBASE'] = self.installdir

        subprocess.check_call([
            self.system_pip_command, 'install', '--user',
            '--ignore-installed',
            '--disable-pip-version-check',
            'pip', 'setuptools', 'wheel'], env=env)

    def _get_pip_command(self):
        self._install_pip()

        pip_install = ['pip', 'install', '--user',
                       '--disable-pip-version-check']

        if self.options.constraints:
            pip_install = pip_install + [
                '--constraint',
                os.path.join(self.sourcedir, self.options.constraints),
            ]

        if self.options.process_dependency_links:
            pip_install.append('--process-dependency-links')

        return pip_install

    def _get_build_env(self):
        env = os.environ.copy()
        headers = glob(os.path.join(
            os.path.sep, 'usr', 'include', '{}*'.format(self.python_version)))
        if headers:
            env['CPPFLAGS'] = '-I{} {}'.format(
                headers[0], env.get('CPPFLAGS', ''))

        return env

    def _run_pip(self, setup):
        pip_install = self._get_pip_command()

        env = self._get_build_env()

        if self.options.requirements:
            self.run(pip_install + [
                '--requirement',
                os.path.join(self.sourcedir, self.options.requirements),
            ], env=env)

        if self.options.python_packages:
            self.run(pip_install + self.options.python_packages, env=env)

        if os.path.exists(setup):
            self.run(pip_install + ['.'], cwd=self.sourcedir, env=env)

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
        fileset.append('-bin/pip*')
        fileset.append('-bin/easy_install*')
        fileset.append('-bin/wheel')
        # .pth files are only read from the built-in site-packages directory.
        # We use PYTHONPATH for everything so not needed.
        fileset.append('-**/*.pth')
        # Holds all the .pyc files. It is a major cause of inter part
        # conflict.
        fileset.append('-**/__pycache__')
        return fileset
