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


import logging
import shutil
import subprocess

from snapcraft.internal.deltas import BaseDeltasGenerator


logger = logging.getLogger(__name__)


class XDeltaGenerator(BaseDeltasGenerator):

    def __init__(self, *, source_path, target_path):
        delta_format = 'xdelta'
        delta_tool_path = shutil.which(delta_format)
        super().__init__(source_path=source_path,
                         target_path=target_path,
                         delta_file_extname='xdelta',
                         delta_format=delta_format,
                         delta_tool_path=delta_tool_path)

    def get_delta_cmd(self, source_path, target_path, delta_file):
        return [
            self.delta_tool_path,
            'delta',
            source_path,
            target_path,
            delta_file
        ]

    def is_returncode_unexpected(self, proc):
        # Success is exiting with 0 or 1. Yes, really. I know.
        # https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=212189
        return proc.returncode not in (0, 1)

    def log_delta_file(self, delta_file):
        xdelta_output = subprocess.check_output(
            [self.delta_tool_path, 'info', delta_file],
            universal_newlines=True)
        logger.debug('xdelta delta diff generation:\n{}'.format(xdelta_output))
