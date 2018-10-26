#!/usr/bin/env python3
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

import codecs
import locale
import os
import subprocess
import sys

from ._runner import run as run_snapcraft
from .snapcraftctl._runner import run as run_snapcraftctl  # noqa
from .echo import warning
from snapcraft import project, yaml_utils
from snapcraft.internal import common

# If the locale ends up being ascii, Click will barf. Let's try to prevent that
# here by using C.UTF-8 as a last-resort fallback. This mostly happens in CI,
# using LXD or Docker. This is the same logic used by Click to error out.
if codecs.lookup(locale.getpreferredencoding()).name == "ascii" and os.name == "posix":
    output = subprocess.check_output(["locale", "-a"]).decode("ascii", "replace")

    for line in output.splitlines():
        this_locale = line.strip()
        if this_locale.lower() in ("c.utf8", "c.utf-8"):
            warning("Locale not set! Snapcraft will temporarily use C.UTF-8")
            os.environ["LC_ALL"] = "C.UTF-8"
            os.environ["LANG"] = "C.UTF-8"
            break


def _needs_legacy() -> bool:
    if not os.path.isdir(common.get_legacy_snapcraft_dir()):
        return False

    try:
        # Early bootstrapping does not allow us to use the existing utilities we
        # have to manage this check.
        if os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT") == "managed-host":
            base_dir = os.path.expanduser(os.path.join("~", "project"))
        else:
            base_dir = None
        snapcraft_yaml_path = project.get_snapcraft_yaml(base_dir=base_dir)
        with open(snapcraft_yaml_path, "r") as f:
            data = yaml_utils.load(f)
        return data.get("base") is None
    except Exception:
        # If there are issues loading/parsing the YAML, just pass off to the current version
        # where the error should be properly handled
        return False


def run_legacy_snapcraft() -> None:
    legacy_python = os.path.join(
        common.get_legacy_snapcraft_dir(), "usr", "bin", "python3"
    )
    legacy_snapcraft = os.path.join(
        common.get_legacy_snapcraft_dir(), "bin", "snapcraft"
    )

    os.execv(legacy_python, [legacy_python, legacy_snapcraft] + sys.argv[1:])


def run():
    if _needs_legacy():
        run_legacy_snapcraft()
    else:
        run_snapcraft(prog_name="snapcraft")


if __name__ == "__main__":
    run()
