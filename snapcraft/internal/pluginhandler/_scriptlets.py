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

import os
import tempfile
from contextlib import suppress

from snapcraft.internal.common import run


class ScriptRunner:
    """Runs /bin/sh scriptlets around the build step for a part."""

    def __init__(self, *, builddir):
        self._builddir = builddir

    def run(self, *, scriptlet):
        """Runs the specified scriptlet."""
        if not scriptlet:
            return

        try:
            with tempfile.NamedTemporaryFile(mode='w+', delete=False) as f:
                f.write('#!/bin/sh -e\n')
                f.write(scriptlet)
                f.flush()
                scriptlet_path = f.name

            os.chmod(scriptlet_path, 0o755)
            run([scriptlet_path], cwd=self._builddir)
        finally:
            with suppress(FileNotFoundError):
                os.unlink(scriptlet_path)
