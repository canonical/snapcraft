# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import contextlib
import os
import logging
import shutil
import subprocess

from snapcraft import (
    repo,
    sources,
)

logger = logging.getLogger(__name__)

_ROS2_URL_TEMPLATE = (
    'https://raw.githubusercontent.com/ros2/ros2/{version}/ros2.repos'
)


_INSTALL_TOOLS_STEP = 'install-tools'
_FETCH_ROS2_STEP = 'fetch-ros2'
_BUILD_ROS2_STEP = 'build-ros2'


class Bootstrapper:
    def __init__(self, *, version, bootstrap_path, ubuntu_sources, project):
        self._version = version
        self._ubuntu_sources = ubuntu_sources
        self._project = project

        self._bootstrap_path = bootstrap_path
        self._tool_dir = os.path.join(self._bootstrap_path, 'tools')
        self._state_dir = os.path.join(self._bootstrap_path, 'state')
        self._underlay_dir = os.path.join(self._bootstrap_path, 'underlay')
        self._install_dir = os.path.join(self._bootstrap_path, 'install')
        self._build_dir = os.path.join(self._bootstrap_path, 'build')
        self._source_dir = os.path.join(self._underlay_dir, 'src')

    def get_build_packages(self):
        # Include dependencies for FastRTPS
        packages = ['cmake']

        # Dependencies for FastRTPS
        packages.extend(['libasio-dev', 'libtinyxml2-dev'])

        # Dependencies for the rest of ros2
        packages.extend([
            'libopencv-dev', 'libpoco-dev', 'libpocofoundation9v5',
            'libpocofoundation9v5-dbg', 'python-empy', 'python3-dev',
            'python3-empy', 'python3-nose', 'python3-pip',
            'python3-setuptools', 'python3-yaml', 'libtinyxml-dev',
            'libeigen3-dev'])

        return packages

    def get_stage_packages(self):
        # Ament is a python3 tool, and it requires setuptools at runtime
        packages = ['python3', 'python3-setuptools']

        return packages

    def pull(self):
        self._run_step(
            self._install_tools,
            step=_INSTALL_TOOLS_STEP,
            skip_message='Tools already installed. Skipping...')
        self._run_step(
            self._fetch_ros2,
            step=_FETCH_ROS2_STEP,
            skip_message='ros2 already fetched. Skipping...')

    def build(self):
        self._run_step(
            self._build_ros2,
            step=_BUILD_ROS2_STEP,
            skip_message='ros2 already built. Skipping...')

        return self._install_dir

    def _run(self, command):
        env = os.environ.copy()
        env['PATH'] = env['PATH'] + ':' + os.path.join(
            self._tool_dir, 'usr', 'bin')
        env['PYTHONPATH'] = os.path.join(
            self._tool_dir, 'usr', 'lib', 'python3', 'dist-packages')

        subprocess.check_call(command, env=env)

    def _step_done(self, step):
        return os.path.isfile(os.path.join(self._state_dir, step))

    def _set_step_done(self, step):
        os.makedirs(self._state_dir, exist_ok=True)
        open(os.path.join(self._state_dir, step), 'w').close()

    def _run_step(self, callable, *, step, skip_message=None):
        if self._step_done(step):
            if skip_message:
                logger.debug(skip_message)
        else:
            callable()
            self._set_step_done(step)

    def clean(self):
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._bootstrap_path)

    def _install_tools(self):
        logger.info('Preparing to fetch vcstool...')
        ubuntu = repo.Ubuntu(
            self._bootstrap_path, sources=self._ubuntu_sources,
            project_options=self._project)

        logger.info('Fetching vcstool...')
        ubuntu.get(['python3-vcstool'])

        logger.info('Installing vcstool...')
        ubuntu.unpack(self._tool_dir)

    def _fetch_ros2(self):
        os.makedirs(self._source_dir, exist_ok=True)
        sources.Script(
            _ROS2_URL_TEMPLATE.format(version=self._version),
            self._underlay_dir).download()

        logger.info('Fetching ros2 sources....')
        ros2_repos = os.path.join(self._underlay_dir, 'ros2.repos')
        self._run(
            ['vcs', 'import', '--input', ros2_repos, self._source_dir])

    def _build_ros2(self):
        logger.info('Building ros2 underlay...')
        ament_path = os.path.join(
            self._source_dir, 'ament', 'ament_tools', 'scripts',
            'ament.py')
        self._run(
            [ament_path, 'build', self._source_dir, '--build-space',
             self._build_dir, '--install-space', self._install_dir])
