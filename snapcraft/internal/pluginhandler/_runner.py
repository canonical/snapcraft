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
from typing import Any, Callable, Dict

from snapcraft.internal import common, errors, steps


class Runner:
    """The Runner class is responsible for orchestrating scriptlets."""

    # FIXME: Need to quote builtin_functions typing because of
    # https://github.com/python/typing/issues/259 which is fixed in Python
    # 3.5.3.
    def __init__(
        self,
        *,
        part_properties: Dict[str, Any],
        partdir: str,
        sourcedir: str,
        builddir: str,
        stagedir: str,
        primedir: str,
        builtin_functions: Dict[str, Callable[..., None]],
        env_generator: Callable[..., str],
        shell: str = "/bin/sh",
        shell_flags: str = "set -ex",
    ) -> None:
        """Create a new Runner.
        :param dict part_properties: YAML properties set for this part.
        :param str partdir: the root directory for this part.
        :param str sourcedir: The source directory for this part.
        :param str builddir: The build directory for this part.
        :param str stagedir: The staging area.
        :param str primedir: The priming area.
        :param dict builtin_functions: Dict of builtin function names to
                                       actual callables.
        :param str env_generator: a callable to provide environment settings
                                  for the part.
        """
        self._partdir = partdir
        self._sourcedir = sourcedir
        self._builddir = builddir
        self._stagedir = stagedir
        self._primedir = primedir
        self._builtin_functions = builtin_functions
        self._env_generator = env_generator

        self._override_pull_scriptlet = part_properties.get("override-pull")
        self._override_build_scriptlet = part_properties.get("override-build")
        self._override_stage_scriptlet = part_properties.get("override-stage")
        self._override_prime_scriptlet = part_properties.get("override-prime")

        self._shell = shell
        self._shell_flags = shell_flags

    def pull(self) -> None:
        """Run override-pull scriptlet."""
        if self._override_pull_scriptlet:
            self._run_scriptlet(
                "override-pull",
                self._override_pull_scriptlet,
                self._sourcedir,
                steps.PULL,
            )

    def build(self) -> None:
        """Run override-build scriptlet."""
        if self._override_build_scriptlet:
            self._run_scriptlet(
                "override-build",
                self._override_build_scriptlet,
                self._builddir,
                steps.BUILD,
            )

    def stage(self) -> None:
        """Run override-stage scriptlet."""
        if self._override_stage_scriptlet:
            self._run_scriptlet(
                "override-stage",
                self._override_stage_scriptlet,
                self._stagedir,
                steps.STAGE,
            )

    def prime(self) -> None:
        """Run override-prime scriptlet."""
        if self._override_prime_scriptlet:
            self._run_scriptlet(
                "override-prime",
                self._override_prime_scriptlet,
                self._primedir,
                steps.PRIME,
            )

    def _run_scriptlet(
        self, scriptlet_name: str, scriptlet: str, workdir: str, step: steps.Step
    ) -> None:
        if common.is_snap():
            # Since the snap is classic, there is no $PATH pointing into the snap, which
            # means snapcraftctl won't be found. We can't use aliases since they don't
            # persist into subshells. However, we know that snapcraftctl lives in its own
            # directory, so adding that to the PATH should have no ill side effects.
            snapcraftctl_env = 'export PATH="$PATH:$SNAP/bin/scriptlet-bin"'
        else:
            snapcraftctl_env = ""

        with tempfile.TemporaryDirectory(dir=self._partdir) as tempdir:
            call_fifo = _NonBlockingRWFifo(os.path.join(tempdir, "function_call"))
            feedback_fifo = _NonBlockingRWFifo(os.path.join(tempdir, "call_feedback"))

            # snapcraftctl only works consistently if it's using the exact same
            # interpreter as that used by snapcraft itself, thus the definition
            # of SNAPCRAFT_INTERPRETER.
            script = textwrap.dedent(
                """\
                set -e
                export SNAPCRAFTCTL_CALL_FIFO={call_fifo}
                export SNAPCRAFTCTL_FEEDBACK_FIFO={feedback_fifo}
                export SNAPCRAFT_INTERPRETER={interpreter}
                {snapcraftctl_env}

                {env}

                {shell_flags}

                {scriptlet}"""
            ).format(
                shell_flags=self._shell_flags,
                interpreter=sys.executable,
                call_fifo=call_fifo.path,
                feedback_fifo=feedback_fifo.path,
                scriptlet=scriptlet,
                snapcraftctl_env=snapcraftctl_env,
                env=self._env_generator(step),
            )

            with tempfile.TemporaryFile(mode="w+") as script_file:
                print(script, file=script_file)
                script_file.flush()
                script_file.seek(0)

                process = subprocess.Popen(
                    [self._shell], stdin=script_file, cwd=workdir
                )

            status = None
            try:
                while status is None:
                    function_call = call_fifo.read()
                    if function_call:
                        # Handle the function and let caller know that function
                        # call has been handled (must contain at least a
                        # newline, anything beyond is considered an error by
                        # snapcraftctl)
                        self._handle_builtin_function(
                            scriptlet_name, function_call.strip()
                        )
                        feedback_fifo.write("\n")

                    status = process.poll()

                    # Don't loop TOO busily
                    time.sleep(0.1)
            except Exception as error:
                feedback_fifo.write(f"{error!s}\n")
                raise error
            finally:
                call_fifo.close()
                feedback_fifo.close()

            if process.returncode != 0:
                raise errors.ScriptletRunError(
                    scriptlet_name=scriptlet_name, code=status
                )

    def _handle_builtin_function(self, scriptlet_name, function_call) -> None:
        try:
            function_json = json.loads(function_call)
        except json.decoder.JSONDecodeError as e:
            # This means a snapcraft developer messed up adding a new
            # snapcraftctl function. Should never be encountered in real life.
            raise ValueError(
                "{!r} scriptlet called a function with invalid json: "
                "{}".format(scriptlet_name, function_call)
            ) from e

        try:
            function_name = function_json["function"]
            function_args = function_json["args"]
        except KeyError as e:
            # This means a snapcraft developer messed up adding a new
            # snapcraftctl function. Should never be encountered in real life.
            raise ValueError(
                "{!r} scriptlet missing expected json field {!s} in args for "
                "function call {!r}: {}".format(
                    scriptlet_name, e, function_name, function_args
                )
            ) from e

        try:
            function = self._builtin_functions[function_name]
        except KeyError as e:
            # This means a snapcraft developer messed up adding a new
            # snapcraftctl function. Should never be encountered in real life.
            raise ValueError(
                "{!r} scriptlet called an undefined builtin function: "
                "{}".format(scriptlet_name, function_name)
            ) from e

        function(**function_args)


class _NonBlockingRWFifo:
    def __init__(self, path) -> None:
        os.mkfifo(path)
        self.path = path

        # Using RDWR for every FIFO just so we can open them reliably whenever
        # (i.e. write-only FIFOs can't be opened successfully until the reader
        # is in place)
        self._fd = os.open(self.path, os.O_RDWR | os.O_NONBLOCK)

    def read(self) -> str:
        total_read = ""
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
