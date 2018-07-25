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

from ._runner import run
from .snapcraftctl._runner import run as run_snapcraftctl  # noqa
from .echo import warning

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

if __name__ == "__main__":
    run(prog_name="snapcraft")
