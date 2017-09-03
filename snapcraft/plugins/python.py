# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
      Enable the processing of dependency links in pip, which allow one
      project to provide places to look for another project
    - python-packages:
      (list)
      A list of dependencies to get from PyPI
    - python-version:
      (string; default: python3)
      The python version to use. Valid options are: python2 and python3

If the plugin finds a python interpreter with a basename that matches
`python-version` in the <stage> directory on the following fixed path:
`<stage-dir>/usr/bin/<python-interpreter>` then this interpreter would
be preferred instead and no interpreter would be brought in through
`stage-packages` mechanisms.
"""

import collections
import json
import os
import re
import shutil
import stat
import subprocess
import tempfile
from contextlib import contextmanager, suppress
from glob import glob
from shutil import which
from textwrap import dedent

import requests

import snapcraft
from snapcraft import file_utils
from snapcraft.common import isurl


_SITECUSTOMIZE_TEMPLATE = dedent("""\
    import site
    import os

    snap_dir = os.getenv("SNAP")
    snapcraft_stage_dir = os.getenv("SNAPCRAFT_STAGE")
    snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")

    for d in (snap_dir, snapcraft_stage_dir, snapcraft_part_install):
        if d:
            site_dir = os.path.join(d, "{site_dir}")
            site.addsitedir(site_dir)

    if snap_dir:
        site.ENABLE_USER_SITE = False""")


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
    def stage_packages(self):
        if not os.path.exists(self._get_python_command()):
            return super().stage_packages + self.plugin_stage_packages
        else:
            return super().stage_packages

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(self.plugin_build_packages)
        self._python_package_dir = os.path.join(self.partdir, 'packages')
        self._manifest = collections.OrderedDict()

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

    def _get_python_command(self):
        python_command = os.path.join(
            'usr', 'bin', self.options.python_version)

        # staged as in project.stage_dir, not from a stage-packages entry
        # in this part.
        staged_python = os.path.join(self.project.stage_dir, python_command)
        unstaged_python = os.path.join(self.installdir, python_command)

        if os.path.exists(staged_python):
            return staged_python
        else:
            return unstaged_python

    def _get_python_headers(self):
        base_match = os.path.join('usr', 'include', '{}*'.format(
            self.options.python_version))
        unstaged_python = glob(os.path.join(os.path.sep, base_match))
        staged_python = glob(os.path.join(self.project.stage_dir, base_match))

        if staged_python:
            return staged_python[0]
        elif unstaged_python:
            return unstaged_python[0]
        else:
            return ''

    def _install_pip(self, download):
        env = self._get_build_env()
        # since we are using an independent env we need to export this too
        # TODO: figure out if we can move back to common.run
        env['SNAPCRAFT_STAGE'] = self.project.stage_dir
        env['SNAPCRAFT_PART_INSTALL'] = self.installdir
        # If python_command is not from stage we don't have pip, which means
        # we are going to need to resort to the pip installed on the system
        # that came from build-packages. This shouldn't be a problem as
        # stage-packages and build-packages should match.
        if not self._get_python_command().startswith(self.project.stage_dir):
            env['PYTHONHOME'] = '/usr'

        args = ['pip', 'setuptools', 'wheel']

        pip_command = [self._get_python_command(), '-m', 'pip']

        pip = _Pip(exec_func=subprocess.check_call,
                   runnable=pip_command,
                   package_dir=self._python_package_dir, env=env,
                   extra_install_args=['--ignore-installed'])

        if download:
            pip.download(args)
        pip.install(args)

    def _get_build_env(self):
        env = os.environ.copy()
        env['PYTHONUSERBASE'] = self.installdir
        if self._get_python_command().startswith(self.project.stage_dir):
            env['PYTHONHOME'] = os.path.join(self.project.stage_dir, 'usr')
        else:
            env['PYTHONHOME'] = os.path.join(self.installdir, 'usr')

        env['PATH'] = '{}:{}'.format(
            os.path.join(self.installdir, 'usr', 'bin'),
            os.path.expandvars('$PATH'))

        headers = self._get_python_headers()
        if headers:
            current_cppflags = env.get('CPPFLAGS', '')
            env['CPPFLAGS'] = '-I{}'.format(headers)
            if current_cppflags:
                env['CPPFLAGS'] = '{} {}'.format(
                    env['CPPFLAGS'], current_cppflags)

        return env

    def _get_commands(self, setup):
        args = []
        cwd = None
        if self.options.requirements:
            requirements = self.options.requirements
            if not isurl(requirements):
                requirements = os.path.join(self.sourcedir,
                                            self.options.requirements)

            args.extend(['--requirement', requirements])
        if os.path.exists(setup):
            args.append('.')
            cwd = os.path.dirname(setup)

        if self.options.python_packages:
            args.extend(self.options.python_packages)

        if args:
            return [dict(args=args, cwd=cwd)]
        else:
            return []

    def _setup_tools_install(self, setup_file):
        command = [
            self._get_python_command(),
            os.path.basename(setup_file), '--no-user-cfg', 'install',
            '--single-version-externally-managed',
            '--user', '--record', 'install.txt']
        cwd = os.path.dirname(setup_file)
        self.run(
            command, env=self._get_build_env(),
            cwd=cwd)

    def _run_pip(self, setup, download=False):
        self._install_pip(download)

        env = self._get_build_env()

        pip_command = [self._get_python_command(), '-m', 'pip']

        constraints = []
        if self.options.constraints:
            if isurl(self.options.constraints):
                constraints = self.options.constraints
            else:
                constraints = os.path.join(self.sourcedir,
                                           self.options.constraints)

        pip = _Pip(exec_func=self.run,
                   runnable=pip_command,
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
            if os.path.exists(setup):
                # pbr and others don't work using `pip install .`
                # LP: #1670852
                # There is also a chance that this setup.py is distutils based
                # in which case we will rely on the `pip install .` ran before
                #  this.
                with suppress(subprocess.CalledProcessError):
                    self._setup_tools_install(setup)
        return pip.list(self.run_output)

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
            installed_pipy_packages = self._run_pip(setup_file)
        # We record the requirements and constraints files only if they are
        # remote. If they are local, they are already tracked with the source.
        if self.options.requirements:
            self._manifest['requirements-contents'] = (
                self._get_file_contents(self.options.requirements))
        if self.options.constraints:
            self._manifest['constraints-contents'] = (
                self._get_file_contents(self.options.constraints))
        self._manifest['python-packages'] = [
            '{}={}'.format(name, installed_pipy_packages[name])
            for name in installed_pipy_packages
        ]

        self._fix_permissions()

        # Fix all shebangs to use the in-snap python.
        file_utils.replace_in_file(self.installdir, re.compile(r''),
                                   re.compile(r'^#!.*python'),
                                   r'#!/usr/bin/env python')

        self._setup_sitecustomize()

    def _get_file_contents(self, path):
        if isurl(path):
            return requests.get(path).text
        else:
            file_path = os.path.join(self.sourcedir, path)
            with open(file_path) as _file:
                return _file.read()

    def _setup_sitecustomize(self):
        # This avoids needing to leak PYTHONUSERBASE
        # USER_SITE and USER_BASE default to base of SNAP for when used in
        # runtime and to SNAPCRAFT_STAGE to support chaining dependencies
        # when used with the `after` keyword.
        site_dir = self._get_user_site_dir()
        sitecustomize_path = self._get_sitecustomize_path()

        # python from the archives has a sitecustomize symlinking to /etc which
        # is distro specific and not needed for a snap.
        if os.path.islink(sitecustomize_path):
            os.unlink(sitecustomize_path)

        # Now create our sitecustomize
        os.makedirs(os.path.dirname(sitecustomize_path), exist_ok=True)
        with open(sitecustomize_path, 'w') as f:
            f.write(_SITECUSTOMIZE_TEMPLATE.format(site_dir=site_dir))

    def _get_user_site_dir(self):
        user_site_dir = glob(os.path.join(
            self.installdir, 'lib', '{}*'.format(self.options.python_version),
            'site-packages'))[0]

        return user_site_dir[len(self.installdir)+1:]

    def _get_sitecustomize_path(self):
        if self._get_python_command().startswith(self.project.stage_dir):
            base_dir = self.project.stage_dir
        else:
            base_dir = self.installdir

        python_site = glob(os.path.join(
            base_dir, 'usr', 'lib',
            '{}*'.format(self.options.python_version),
            'site.py'))[0]
        python_site_dir = os.path.dirname(python_site)

        return os.path.join(self.installdir,
                            python_site_dir[len(base_dir)+1:],
                            'sitecustomize.py')

    def get_manifest(self):
        return self._manifest

    def snap_fileset(self):
        fileset = super().snap_fileset()
        fileset.append('-bin/pip*')
        fileset.append('-bin/easy_install*')
        fileset.append('-bin/wheel')
        # Holds all the .pyc files. It is a major cause of inter part
        # conflict.
        fileset.append('-**/__pycache__')
        fileset.append('-**/*.pyc')
        # The RECORD files include hashes useful when uninstalling packages.
        # In the snap they will cause conflicts when more than one part uses
        # the python plugin.
        fileset.append('-lib/python*/site-packages/*/RECORD')
        return fileset


class _Pip:

    def __init__(self, *, exec_func, runnable, package_dir, env,
                 constraints=None, dependency_links=None,
                 extra_install_args=None):
        self._exec_func = exec_func
        if isinstance(runnable, str):
            self._runnable = [runnable]
        else:
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
        cmd = [*self._runnable, 'list', '--format=json']

        output = exec_func(cmd, env=self._env)
        packages = collections.OrderedDict()
        for package in json.loads(
                output, object_pairs_hook=collections.OrderedDict):
            packages[package['name']] = package['version']
        return packages

    def wheel(self, args, **kwargs):
        cmd = [
            *self._runnable, 'wheel',
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
            *self._runnable, 'download',
            '--disable-pip-version-check',
            '--dest', self._package_dir,
        ]
        cmd.extend(self._extra_pip_args)
        cmd.extend(args)

        os.makedirs(self._package_dir, exist_ok=True)
        self._exec_func(cmd, env=self._env, **kwargs)

    def install(self, args, **kwargs):
        cmd = [
            *self._runnable, 'install', '--user', '--no-compile',
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
