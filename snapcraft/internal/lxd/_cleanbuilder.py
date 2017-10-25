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

from . import Containerbuild

logger = logging.getLogger(__name__)


class Cleanbuilder(Containerbuild):

    def __init__(self, *, output=None, source, project_options,
                 metadata=None, remote=None):
        container_name = petname.Generate(3, '-')
        super().__init__(output=output, source=source,
                         project_options=project_options, metadata=metadata,
                         container_name=container_name, remote=remote)

    def _push_file(self, src, dst):
        subprocess.check_call(['lxc', 'file', 'push',
                              src, '{}{}'.format(self._container_name, dst)])

    def _pull_file(self, src, dst):
        subprocess.check_call(['lxc', 'file', 'pull',
                               '{}{}'.format(self._container_name, src), dst])

    def _ensure_container(self):
        subprocess.check_call([
            'lxc', 'launch', '-e', self._image, self._container_name])
        self._configure_container()

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
