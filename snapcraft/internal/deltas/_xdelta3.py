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
import subprocess

from ._deltas import BaseDeltasGenerator
from snapcraft import file_utils

logger = logging.getLogger(__name__)


class XDelta3Generator(BaseDeltasGenerator):
    def __init__(self, *, source_path, target_path):
        delta_format = "xdelta3"
        delta_tool_path = file_utils.get_tool_path("xdelta3")
        super().__init__(
            source_path=source_path,
            target_path=target_path,
            delta_file_extname="xdelta3",
            delta_format=delta_format,
            delta_tool_path=delta_tool_path,
        )

    def get_delta_cmd(self, source_path, target_path, delta_file):
        return [self.delta_tool_path, "-s", source_path, target_path, delta_file]

    def log_delta_file(self, delta_file):
        xdelta_output = subprocess.check_output(
            [self.delta_tool_path, "printhdr", delta_file], universal_newlines=True
        )
        logger.debug("xdelta3 delta diff generation:\n{}".format(xdelta_output))
