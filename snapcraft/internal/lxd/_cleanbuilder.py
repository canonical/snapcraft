#!/usr/bin/python3
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

import logging
import os
import petname
import subprocess

from ._containerbuild import Containerbuild
from snapcraft.internal.errors import ContainerConnectionError

logger = logging.getLogger(__name__)


class Cleanbuilder(Containerbuild):

    def __init__(self, *, output=None, source, project_options,
                 metadata=None, remote=None):
        container_name = petname.Generate(3, '-')
        super().__init__(output=output, source=source,
                         project_options=project_options, metadata=metadata,
                         container_name=container_name, remote=remote)

    def _ensure_container(self):
        try:
            subprocess.check_call([
                'lxc', 'launch', '-e', self._image, self._container_name])
        except subprocess.CalledProcessError as e:
            raise ContainerConnectionError('Failed to setup container')
        self._configure_container()
        self._wait_for_network()
        self._container_run(['apt-get', 'update'])
        # Because of https://bugs.launchpad.net/snappy/+bug/1628289
        # Needed to run snapcraft as a snap and build-snaps
        self._container_run(['apt-get', 'install', 'squashfuse', '-y'])
        self._inject_snapcraft(new_container=True)

    def _setup_project(self):
        logger.info('Setting up container with project assets')
        tar_filename = self._source
        # os.sep needs to be `/` and on Windows it will be set to `\`
        dst = '{}/{}'.format(self._project_folder,
                             os.path.basename(tar_filename))
        self._container_run(['mkdir', self._project_folder])
        self._push_file(tar_filename, dst)
        self._container_run(['tar', 'xvf', os.path.basename(tar_filename)],
                            cwd=self._project_folder)

    def _finish(self):
        # os.sep needs to be `/` and on Windows it will be set to `\`
        src = '{}/{}'.format(self._project_folder, self._snap_output)
        self._pull_file(src, self._snap_output)
        logger.info('Retrieved {}'.format(self._snap_output))
