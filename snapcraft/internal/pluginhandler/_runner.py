# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2018 Canonical Ltd
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
import json
import os
import subprocess
import sys
import tempfile
import textwrap
import time
from typing import Any, Callable, Dict  # noqa

from snapcraft.internal import (
    common,
    errors,
)


class Runner:
    """The Runner class is responsible for orchestrating scriptlets."""

    # FIXME: Need to quote builtin_functions typing because of
    # https://github.com/python/typing/issues/259 which is fixed in Python
    # 3.5.3.
    def __init__(self, *, part_properties: Dict[str, Any], builddir: str,
                 builtin_functions: 'Dict[str, Callable[..., None]]') -> None:
        """Create a new Runner.
        :param dict part_properties: YAML properties set for this part.
        :param str builddir: The build directory for this part.
        :param dict builtin_functions: Dict of builtin function names to
                                       actual callables.
        """
        self._builddir = builddir
        self._builtin_functions = builtin_functions

        self._prepare_scriptlet = part_properties.get('prepare')
        self._build_scriptlet = part_properties.get('build')
        self._install_scriptlet = part_properties.get('install')

    def prepare(self) -> None:
        """Run prepare scriptlet."""
        if self._prepare_scriptlet:
            self._run_scriptlet(
                'prepare', self._prepare_scriptlet, self._builddir)

    def build(self) -> None:
        """Run build scriptlet."""
        if self._build_scriptlet:
            self._run_scriptlet(
                'build', self._build_scriptlet, self._builddir)

    def install(self) -> None:
        """Run install scriptlet."""
        if self._install_scriptlet:
            self._run_scriptlet(
                'install', self._install_scriptlet, self._builddir)

    def _run_scriptlet(self, scriptlet_name: str, scriptlet: str,
                       workdir: str) -> None:
        with tempfile.TemporaryDirectory() as tempdir:
            call_fifo = _NonBlockingRWFifo(
                os.path.join(tempdir, 'function_call'))
            feedback_fifo = _NonBlockingRWFifo(
                os.path.join(tempdir, 'call_feedback'))

            script = textwrap.dedent("""\
                export SNAPCRAFTCTL_CALL_FIFO={call_fifo}
                export SNAPCRAFTCTL_FEEDBACK_FIFO={feedback_fifo}
                {env}
                {scriptlet}
            """.format(
                call_fifo=call_fifo.path, feedback_fifo=feedback_fifo.path,
                env=common.assemble_env(), scriptlet=scriptlet))

            process = subprocess.Popen(
                ['/bin/sh', '-e', '-c', script], cwd=self._builddir)

            status = None
            try:
                while status is None:
                    function_call = call_fifo.read()
                    if function_call:
                        self._handle_builtin_function(
                            scriptlet_name, function_call.strip())
                        # Let caller know that function call has been handled
                        # (must contain at least a newline)
                        feedback_fifo.write('\n')
                    status = process.poll()

                    # Don't loop TOO busily
                    time.sleep(0.1)
            finally:
                call_fifo.close()
                feedback_fifo.close()

            if status:
                raise errors.ScriptletRunError(
                    scriptlet_name=scriptlet_name, code=status)

    def _handle_builtin_function(self, scriptlet_name, function_call):
        try:
            function_json = json.loads(function_call)
        except json.decoder.JSONDecodeError as e:
            # This means a snapcraft developer messed up adding a new
            # snapcraftctl function. Should never be encountered in real life.
            raise ValueError(
                '{!r} scriptlet called a function with invalid json: '
                '{}'.format(scriptlet_name, function_call)) from e

        try:
            function_name = function_json['function']
            function_args = function_json['args']
        except KeyError as e:
            # This means a snapcraft developer messed up adding a new
            # snapcraftctl function. Should never be encountered in real life.
            raise ValueError(
                '{!r} scriptlet missing expected json field {!s} in args for '
                'function call {!r}: {}'.format(
                    scriptlet_name, e, function_name, function_args)) from e

        try:
            function = self._builtin_functions[function_name]
            function(**function_args)
        except KeyError as e:
            # This means a snapcraft developer messed up adding a new
            # snapcraftctl function. Should never be encountered in real life.
            raise ValueError(
                '{!r} scriptlet called an undefined builtin function: '
                '{}'.format(scriptlet_name, function_name)) from e


class _NonBlockingRWFifo:

    def __init__(self, path) -> None:
        os.mkfifo(path)
        self.path = path

        # Using RDWR for every FIFO just so we can open them reliably whenever
        # (i.e. write-only FIFOs can't be opened successfully until the reader
        # is in place)
        self._fd = os.open(self.path, os.O_RDWR | os.O_NONBLOCK)

    def read(self) -> str:
        total_read = ''
        with contextlib.suppress(BlockingIOError):
            value = os.read(self._fd, 1024)
            while value:
                total_read += value.decode(sys.getfilesystemencoding())
                value = os.read(self._fd, 1024)
        return total_read

    def write(self, data: str) -> int:
        return os.write(self._fd, data.encode(sys.getfilesystemencoding()))

    def close(self) -> None:
        if self._fd is not None:
            os.close(self._fd)
