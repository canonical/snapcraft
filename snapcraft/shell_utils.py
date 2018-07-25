# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

"""Utilities to run inside a correct snapcrafting environment."""


import tempfile

from snapcraft.internal import common


def which(command, **kwargs):
    """Returns the result of `which` run with the correct environment."""
    return run_script("which {}".format(command), **kwargs)


def getenv(envvar, **kwargs):
    """Return the specified envrionment variable."""
    return run_script("echo ${}".format(envvar), **kwargs)


def run_script(script, **kwargs):
    """Run the script and return the output."""
    with tempfile.NamedTemporaryFile("w+") as tempf:
        print("#!/bin/sh", file=tempf)
        print(script, file=tempf)
        tempf.flush()
        return common.run_output(["/bin/sh", tempf.name], **kwargs)
