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
import shutil
import stat
import subprocess
import tempfile
from contextlib import contextmanager
from glob import glob
from shutil import which
from textwrap import dedent

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

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return [
            'requirements',
            'constraints',
            'python-packages',
            'python-version',
        ]

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
        self._python_package_dir = os.path.join(self.partdir, 'packages')

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
        with simple_env_bzr(os.path.join(self.installdir, 'bin')):
            self._run_pip(setup, download=True)

    def clean_pull(self):
        super().clean_pull()

        if os.path.isdir(self._python_package_dir):
            shutil.rmtree(self._python_package_dir)

    def _install_pip(self, download):
        env = os.environ.copy()
        env['PYTHONUSERBASE'] = self.installdir

        args = ['pip', 'setuptools', 'wheel']

        pip = _Pip(exec_func=subprocess.check_call,
                   runnable=self.system_pip_command,
                   package_dir=self._python_package_dir, env=env,
                   extra_install_args=['--ignore-installed'])

        if download:
            pip.download(args)
        pip.wheel(args)
        pip.install(args)

    def _get_build_env(self):
        env = os.environ.copy()
        headers = glob(os.path.join(
            os.path.sep, 'usr', 'include', '{}*'.format(
                self.options.python_version)))
        if headers:
            env['CPPFLAGS'] = '-I{} {}'.format(
                headers[0], env.get('CPPFLAGS', ''))

        return env

    def _get_commands(self, setup):
        commands = []
        if self.options.requirements:
            requirements = self.options.requirements
            if not isurl(requirements):
                requirements = os.path.join(self.sourcedir,
                                            self.options.requirements)

            commands.append(dict(args=['--requirement', requirements]))

        if self.options.python_packages:
            commands.append(dict(args=self.options.python_packages))

        if os.path.exists(setup):
            cwd = os.path.dirname(setup)
            commands.append(dict(args=['.'], cwd=cwd))

        return commands

    def _run_pip(self, setup, download=False):
        self._install_pip(download)

        env = self._get_build_env()

        constraints = []
        if self.options.constraints:
            if isurl(self.options.constraints):
                constraints = self.options.constraints
            else:
                constraints = os.path.join(self.sourcedir,
                                           self.options.constraints)

        pip = _Pip(exec_func=self.run, runnable='pip',
                   package_dir=self._python_package_dir, env=env,
                   constraints=constraints,
                   dependency_links=self.options.process_dependency_links)

        commands = self._get_commands(setup)

        if download:
            for command in commands:
                pip.download(**command)
        else:
            for command in commands:
                wheels = pip.wheel(**command)
                installed = pip.list(self.run_output)
                wheel_names = [os.path.basename(w).split('-')[0]
                               for w in wheels]
                # we want to avoid installing what is already provided in
                # stage-packages
                need_install = [k for k in wheel_names if k not in installed]
                pip.install(need_install + ['--no-deps', '--upgrade'])

    def _fix_permissions(self):
        for root, dirs, files in os.walk(self.installdir):
            for filename in files:
                _replicate_owner_mode(os.path.join(root, filename))
            for dirname in dirs:
                _replicate_owner_mode(os.path.join(root, dirname))

    def build(self):
        super().build()

        setup_file = os.path.join(self.builddir, 'setup.py')
        with simple_env_bzr(os.path.join(self.installdir, 'bin')):
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


class _Pip:

    def __init__(self, *, exec_func, runnable, package_dir, env,
                 constraints=None, dependency_links=None,
                 extra_install_args=None):
        self._exec_func = exec_func
        self._runnable = runnable
        self._package_dir = package_dir
        self._env = env

        self._extra_install_args = extra_install_args or []

        self._extra_pip_args = []
        if constraints:
            self._extra_pip_args.extend(['--constraint', constraints])

        if dependency_links:
            self._extra_pip_args.append('--process-dependency-links')

    def list(self, exec_func=None):
        """Return a dict of installed python packages with versions."""
        if not exec_func:
            exec_func = self._exec_func
        cmd = [self._runnable, 'list']

        output = exec_func(cmd, env=self._env)
        package_listing = {}
        version_regex = re.compile('\((.+)\)')
        for line in output.splitlines():
            line = line.split()
            m = version_regex.search(line[1])
            package_listing[line[0]] = m.group(1)

        return package_listing

    def wheel(self, args, **kwargs):
        cmd = [
            self._runnable, 'wheel',
            '--disable-pip-version-check', '--no-index',
            '--find-links', self._package_dir,
        ]
        cmd.extend(self._extra_pip_args)

        os.makedirs(self._package_dir, exist_ok=True)

        wheels = []
        with tempfile.TemporaryDirectory() as temp_dir:
            cmd.extend(['--wheel-dir', temp_dir])
            cmd.extend(args)
            self._exec_func(cmd, env=self._env, **kwargs)
            wheels = os.listdir(temp_dir)
            for wheel in wheels:
                file_utils.link_or_copy(
                    os.path.join(temp_dir, wheel),
                    os.path.join(self._package_dir, wheel))

        return [os.path.join(self._package_dir, wheel) for wheel in wheels]

    def download(self, args, **kwargs):
        cmd = [
            self._runnable, 'download',
            '--disable-pip-version-check',
            '--dest', self._package_dir,
        ]
        cmd.extend(self._extra_pip_args)
        cmd.extend(args)

        os.makedirs(self._package_dir, exist_ok=True)
        self._exec_func(cmd, env=self._env, **kwargs)

    def install(self, args, **kwargs):
        cmd = [
            self._runnable, 'install', '--user', '--no-compile',
            '--disable-pip-version-check', '--no-index',
            '--find-links', self._package_dir,
        ]
        cmd.extend(self._extra_pip_args)
        cmd.extend(self._extra_install_args)
        cmd.extend(args)

        self._exec_func(cmd, env=self._env, **kwargs)


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


@contextmanager
def simple_env_bzr(bin_dir):
    """Create an appropriate environment to run bzr.

       The python plugin sets up PYTHONUSERBASE and PYTHONHOME which
       conflicts with bzr when using python3 as those two environment
       variables will make bzr look for modules in the wrong location.
       """
    os.makedirs(bin_dir, exist_ok=True)
    bzr_bin = os.path.join(bin_dir, 'bzr')
    real_bzr_bin = which('bzr')
    if real_bzr_bin:
        exec_line = 'exec {} "$@"'.format(real_bzr_bin)
    else:
        exec_line = 'echo bzr needs to be in PATH; exit 1'
    with open(bzr_bin, 'w') as f:
        f.write(dedent(
            """#!/bin/sh
               unset PYTHONUSERBASE
               unset PYTHONHOME
               {}
            """.format(exec_line)))
    os.chmod(bzr_bin, 0o777)
    try:
        yield
    finally:
        os.remove(bzr_bin)
        if not os.listdir(bin_dir):
            os.rmdir(bin_dir)
