# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
import subprocess

import testtools


class HasArchitecture:
    """Match if the file was built for the expected architecture"""

    def __init__(self, expected_arch):
        self._expected_arch = expected_arch

    def __str__(self):
        return "HasArchitecture()"

    def match(self, file_path):
        cmd = ["file", file_path]
        try:
            output = subprocess.check_output(cmd).decode()
        except subprocess.CalledProcessError as cmd_err:
            raise ValueError(
                "Failed to get architecture info: {}", cmd_err
            ) from cmd_err

        try:
            # output is of the form
            # ELF 64-bit LSB executable, ARM aarch64, version 1 (SYSV), \
            # dynamically linked, interpreter /lib/ld-linux-aarch64.so.1, \
            # for GNU/Linux 3.7.0, \
            # BuildID[sha1]=58b8495bef7a6c9e5f403879e29dc63ad4ed2d11, \
            # not stripped
            arch = output.split(",")[1]
        except IndexError as e:
            raise ValueError("Failed to parse magic {!r}".format(output)) from e
        if self._expected_arch not in arch:
            return testtools.matchers.Mismatch(
                "Expected {!r} to be in {!r}".format(self._expected_arch, arch)
            )
