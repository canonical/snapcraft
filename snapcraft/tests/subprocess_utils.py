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

import errno
import fcntl
import os
import select
import subprocess
import sys


def call(cmd):
    subprocess.check_call(cmd, stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL)


def call_with_output(cmd):
    return subprocess.check_output(cmd).decode('utf-8').strip()


def _set_nonblocking(fd):
    fcntl.fcntl(
        fd, fcntl.F_SETFL, fcntl.fcntl(fd, fcntl.F_GETFL) | os.O_NONBLOCK)


def _read_str(fd):
    try:
        snippet = fd.read()
    except IOError as e:
        if e.errno != errno.EAGAIN:
            raise e
        else:
            snippet = ''

    if snippet:
        return snippet.decode(sys.getfilesystemencoding())


class PopenAsyncOutput:
    """Run an external process while capturing both stdout and stderr."""

    def __init__(self, command, *, stdout_callback=None, stderr_callback=None,
                 **kwargs):
        """Initialize a new PopenAsyncOutput.

        :param list command: The command to run
        :param callable stdout_callback: Callback for when stdout is received
        :param callable stderr_callback: Callback for when stderr is received

        Any other kwargs are passed to Popen.
        """
        self._command = command
        self._stdout_callback = stdout_callback
        self._stderr_callback = stderr_callback
        self._kwargs = kwargs

        self.stdout = ''
        self.stderr = ''
        self.combined_output = ''
        self.return_code = None

    def run(self):
        """Run the command, calling callbacks as output is received.

        Note that this function will block until the command completes.

        :raises subprocess.CalledProcessError: If command exits non-zero. The
                                               combined stdout/stderr is
                                               available in the 'output'
                                               attribute.
        """

        process = subprocess.Popen(
            self._command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            **self._kwargs)

        try:
            # Set both stdout and stderr to be non-blocking
            _set_nonblocking(process.stdout)
            _set_nonblocking(process.stderr)

            while self.return_code is None:
                # Block until either stdout or stderr to be available
                select.select([process.stdout, process.stderr], [], [])

                # Check stdout for data. If available, save it and trigger
                # callback
                snippet = _read_str(process.stdout)
                if snippet:
                    self.stdout += snippet
                    self.combined_output += snippet
                    if self._stdout_callback:
                        self._stdout_callback(snippet)

                # Check stderr for data. If available, save it and trigger
                # callback
                snippet = _read_str(process.stderr)
                if snippet:
                    self.stderr += snippet
                    self.combined_output += snippet
                    if self._stderr_callback:
                        self._stderr_callback(snippet)

                self.return_code = process.poll()
        finally:
            process.stdout.close()
            process.stderr.close()

        if self.return_code != 0:
            raise subprocess.CalledProcessError(
                self.return_code, self._command, self.combined_output)
