#!/usr/bin/python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

from snapcraft.internal.indicators import is_dumb_terminal


class _StdoutFilter(logging.Filter):
    def filter(self, record):
        return record.levelno <= logging.WARNING


class _StderrFilter(logging.Filter):
    def filter(self, record):
        return record.levelno >= logging.ERROR


class _ColoredFormatter(logging.Formatter):
    RESET = "\033[0m"
    LEVEL_COLORS = {
        "INFO": "\033[0;32m",  # Green
        "WARNING": "\033[1;33m",  # Yellow
        "ERROR": "\033[0;31m",  # Dark red
        "CRITICAL": "\033[1;31m",  # Light red
    }

    def format(self, record):
        color = self.LEVEL_COLORS.get(record.levelname, None)
        log_message = super().format(record)
        if color:
            return "{color}{message}{reset}".format(
                color=color, message=log_message, reset=self.RESET
            )

        return log_message


def configure(logger_name=None, log_level=None):
    if not log_level:
        log_level = logging.INFO

    stdout_handler = logging.StreamHandler(stream=sys.stdout)
    stdout_handler.addFilter(_StdoutFilter())
    stderr_handler = logging.StreamHandler(stream=sys.stderr)
    stderr_handler.addFilter(_StderrFilter())
    handlers = [stdout_handler, stderr_handler]

    if is_dumb_terminal():
        formatter = logging.Formatter(style="{")
    else:
        formatter = _ColoredFormatter(style="{")
    logger = logging.getLogger(logger_name)
    for handler in handlers:
        handler.setFormatter(formatter)
        logger.addHandler(handler)

    logger.setLevel(log_level)

    # INFO by default for the requests lib as it is too noisy
    if log_level == logging.DEBUG:
        logging.getLogger("requests").setLevel(log_level)
    else:
        logging.getLogger("requests").setLevel(logging.WARNING)
