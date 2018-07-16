# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

from snapcraft import shell_utils
from snapcraft.internal.meta import _errors
from snapcraft.internal import sources


logger = logging.getLogger(__name__)


def get_version(version: str, version_script: str = None) -> str:
    new_version = version
    if version_script:
        logger.info("Determining the version from the project repo (version-script).")
        try:
            new_version = shell_utils.run_script(version_script).strip()
            if not new_version:
                raise _errors.CommandError("The version-script produced no output")
        except subprocess.CalledProcessError as e:
            raise _errors.CommandError(
                "The version-script failed to run (exit code {})".format(e.returncode)
            )
    # we want to whitelist what we support here.
    elif version == "git":
        logger.info("Determining the version from the project repo (version: git).")
        vcs_handler = sources.get_source_handler_from_type("git")
        new_version = vcs_handler.generate_version()

    if new_version != version:
        logger.info("The version has been set to {!r}".format(new_version))
    return new_version
