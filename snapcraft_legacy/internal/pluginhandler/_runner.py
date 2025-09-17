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

import os
import subprocess
import sys
import tempfile
import textwrap
from typing import Any, Callable, Dict

from snapcraft_legacy.internal import common, errors, steps


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
        # builtin_functions removed - snapcraftctl deprecated in core22
        :param str env_generator: a callable to provide environment settings
                                  for the part.
        """
        self._partdir = partdir
        self._sourcedir = sourcedir
        self._builddir = builddir
        self._stagedir = stagedir
        self._primedir = primedir
        # _builtin_functions removed - snapcraftctl deprecated in core22
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
        # snapcraftctl compatibility removed - deprecated in core22
        snapcraftctl_env = ""

        # FIFO handling removed with snapcraftctl compatibility
        
        script = textwrap.dedent(
            """\
            set -e
            export SNAPCRAFT_INTERPRETER={interpreter}
            {snapcraftctl_env}

            {env}

            {shell_flags}

            {scriptlet}"""
        ).format(
            shell_flags=self._shell_flags,
            interpreter=sys.executable,
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
            status = process.wait()

        if status != 0:
            raise errors.ScriptletRunError(
                scriptlet_name=scriptlet_name, code=status
            )

