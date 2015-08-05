#!/usr/bin/python3
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

import logging
import sys


_COLOR_BOLD = '\033[1m'
_COLOR_END = '\033[0m'


class _StdoutFilter(logging.Filter):

    def filter(self, record):
        return record.levelno <= logging.WARNING


class _StderrFilter(logging.Filter):

    def filter(self, record):
        return record.levelno >= logging.ERROR


def configure(logger_name=None):
    stdout_handler = logging.StreamHandler(stream=sys.stdout)
    stdout_handler.addFilter(_StdoutFilter())
    stderr_handler = logging.StreamHandler(stream=sys.stderr)
    stderr_handler.addFilter(_StderrFilter())
    handlers = [stdout_handler, stderr_handler]

    formatter = logging.Formatter(
        _COLOR_BOLD + '{msg}' + _COLOR_END, style='{')
    logger = logging.getLogger(logger_name)
    for handler in handlers:
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    logger.setLevel(logging.INFO)
