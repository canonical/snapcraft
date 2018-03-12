# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
import tempfile
import contextlib
from typing import Any, Callable, Dict

from snapcraft.internal import common, deprecations


class Runner:
    """The Runner class is responsible for orchestrating scriptlets."""

    def __init__(self, *, part_properties: Dict[str, Any], builddir: str,
                 default_pre_build: Callable[[], None]=None,
                 default_post_build: Callable[[], None]=None,
                 default_build: Callable[[], None]=None) -> None:
        """Create a new Runner.

        :param dict part_properties: YAML properties set for this part.
        :param str builddir: The build directory for this part.
        :param callable default_pre_build: Default action if pre-build isn't
                                           specified.
        :param callable default_post_build: Default action if post-build isn't
                                            specified.
        :param callable default_build: Default action if build isn't specified.
        """
        self._builddir = builddir
        self._default_pre_build = default_pre_build
        self._default_post_build = default_post_build
        self._default_build = default_build

        # Note that we don't need to worry about handling both prepare and
        # pre-build at the same time, since the schema won't allow it.
        self._pre_build_scriplet = part_properties.get('pre-build')
        if not self._pre_build_scriplet:
            self._pre_build_scriplet = part_properties.get('prepare')
            if self._pre_build_scriplet:
                deprecations.handle_deprecation_notice('dn7')

        # Note that we don't need to worry about handling both install and
        # post-build at the same time, since the schema won't allow it.
        self._post_build_scriplet = part_properties.get('post-build')
        if not self._post_build_scriplet:
            self._post_build_scriplet = part_properties.get('install')
            if self._post_build_scriplet:
                deprecations.handle_deprecation_notice('dn8')

        self._build_scriptlet = part_properties.get('build')

    def pre_build(self) -> None:
        """Run pre-build scriptlet, or default pre-build action."""
        if self._pre_build_scriplet:
            _run_scriptlet(self._pre_build_scriplet, self._builddir)
        elif self._default_pre_build:
            self._default_pre_build()

    def build(self) -> None:
        """Run build scriptlet, or default build action."""
        if self._build_scriptlet:
            _run_scriptlet(self._build_scriptlet, self._builddir)
        elif self._default_build:
            self._default_build()

    def post_build(self) -> None:
        """Run post-build scriptlet, or default post-build action."""
        if self._post_build_scriplet:
            _run_scriptlet(self._post_build_scriplet, self._builddir)
        elif self._default_post_build:
            self._default_post_build()


def _run_scriptlet(scriptlet: str, workdir: str) -> None:
    try:
        with tempfile.NamedTemporaryFile(mode='w+', delete=False) as f:
            f.write('#!/bin/sh -e\n')
            f.write(scriptlet)
            f.flush()
            scriptlet_path = f.name

        os.chmod(scriptlet_path, 0o755)
        common.run([scriptlet_path], cwd=workdir)
    finally:
        with contextlib.suppress(FileNotFoundError):
            os.unlink(scriptlet_path)
