# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

"""The python plugin can be used for python 2 or 3 based parts.

It can be used for python projects where you would want to do:

    - import python modules with a requirements.txt
    - build a python project that has a setup.py
    - install packages straight from pip

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - requirements:
      (string)
      Path to a requirements.txt file
    - constraints:
      (string)
      Path to a constraints file
    - process-dependency-links:
      (bool; default: false)
      Enable the processing of dependency links.
    - python-packages:
      (list)
      A list of dependencies to get from PyPi
    - python-version:
      (string; default: python3)
      The python version to use. Valid options are: python2 and python3
"""

import os
import re
import stat
import subprocess
from glob import glob

import snapcraft
from snapcraft import file_utils
from snapcraft.common import isurl


class PythonPlugin(snapcraft.BasePlugin):

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
        schema['properties']['python-version'] = {
            'type': 'string',
            'default': 'python3',
            'enum': ['python2', 'python3']
        }
        schema.pop('required')

        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        schema['pull-properties'].extend([
            'requirements',
            'constraints',
            'python-packages',
            'python-version',
        ])

        return schema

    @property
    def plugin_build_packages(self):
        if self.options.python_version == 'python3':
            return [
                'python3-dev',
                'python3-pip',
                'python3-pkg-resources',
                'python3-setuptools',
            ]
        elif self.options.python_version == 'python2':
            return [
                'python-dev',
                'python-pip',
                'python-pkg-resources',
                'python-setuptools',
            ]

    @property
    def plugin_stage_packages(self):
        if self.options.python_version == 'python3':
            return ['python3']
        elif self.options.python_version == 'python2':
            return ['python']

    @property
    def system_pip_command(self):
        if self.options.python_version == 'python3':
            return os.path.join(os.path.sep, 'usr', 'bin', 'pip3')
        else:
            return os.path.join(os.path.sep, 'usr', 'bin', 'pip')

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
            self.system_pip_command, 'install', '--user', '--no-compile',
            '--ignore-installed',
            '--disable-pip-version-check',
            'pip', 'setuptools', 'wheel'], env=env)

    def _get_pip_command(self):
        self._install_pip()

        pip_install = ['pip', 'install', '--user', '--no-compile',
                       '--disable-pip-version-check']

        if self.options.constraints:
            if isurl(self.options.constraints):
                constraints = self.options.constraints
            else:
                constraints = os.path.join(self.sourcedir,
                                           self.options.constraints)
            pip_install = pip_install + ['--constraint', constraints]

        if self.options.process_dependency_links:
            pip_install.append('--process-dependency-links')

        return pip_install

    def _get_build_env(self):
        env = os.environ.copy()
        headers = glob(os.path.join(
            os.path.sep, 'usr', 'include', '{}*'.format(
                self.options.python_version)))
        if headers:
            env['CPPFLAGS'] = '-I{} {}'.format(
                headers[0], env.get('CPPFLAGS', ''))

        return env

    def _run_pip(self, setup):
        pip_install = self._get_pip_command()

        env = self._get_build_env()

        if self.options.requirements:
            if isurl(self.options.requirements):
                requirements = self.options.requirements
            else:
                requirements = os.path.join(self.sourcedir,
                                            self.options.requirements)
            self.run(pip_install + ['--requirement', requirements], env=env)

        if self.options.python_packages:
            self.run(pip_install + self.options.python_packages, env=env)

        if os.path.exists(setup):
            cwd = os.path.dirname(setup)
            self.run(pip_install + ['.'], cwd=cwd, env=env)

    def _fix_permissions(self):
        for root, dirs, files in os.walk(self.installdir):
            for filename in files:
                _replicate_owner_mode(os.path.join(root, filename))
            for dirname in dirs:
                _replicate_owner_mode(os.path.join(root, dirname))

    def build(self):
        super().build()

        setup_file = os.path.join(self.builddir, 'setup.py')
        self._run_pip(setup_file)

        self._fix_permissions()

        # Fix all shebangs to use the in-snap python.
        file_utils.replace_in_file(self.installdir, re.compile(r''),
                                   re.compile(r'^#!.*python'),
                                   r'#!/usr/bin/env python')

    def snap_fileset(self):
        fileset = super().snap_fileset()
        fileset.append('-bin/pip*')
        fileset.append('-bin/easy_install*')
        fileset.append('-bin/wheel')
        # Holds all the .pyc files. It is a major cause of inter part
        # conflict.
        fileset.append('-**/__pycache__')
        fileset.append('-**/*.pyc')
        return fileset


def _replicate_owner_mode(path):
    if not os.path.exists(path):
        return

    file_mode = os.stat(path).st_mode
    new_mode = file_mode & stat.S_IWUSR
    if file_mode & stat.S_IXUSR:
        new_mode |= stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH
    if file_mode & stat.S_IRUSR:
        new_mode |= stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH
    os.chmod(path, new_mode)
