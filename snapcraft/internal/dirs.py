# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import os.path


def setup_dirs() -> None:
    """
    Ensure that snapcraft.common plugindir is setup correctly
    and support running out of a development snapshot
    """
    from snapcraft.internal import common

    topdir = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
    # Only change the default if we are running from a checkout or from the
    # snap.
    if os.path.exists(os.path.join(topdir, "setup.py")):
        common.set_plugindir(os.path.join(topdir, "snapcraft", "plugins"))
        common.set_schemadir(os.path.join(topdir, "schema"))
        common.set_librariesdir(os.path.join(topdir, "libraries"))
        common.set_extensionsdir(os.path.join(topdir, "extensions"))

    # The default paths are relative to sys.prefix, which works well for
    # Snapcraft as a deb or in a venv. However, the Python plugin installs
    # packages into $SNAP/ as a prefix, while Python itself is contained in
    # $SNAP/usr/. As a result, using sys.prefix (which is '/usr') to find these
    # files won't work in the snap.
    elif common.is_snap():
        parent_dir = os.path.join(os.environ.get("SNAP"), "share", "snapcraft")
        common.set_plugindir(os.path.join(parent_dir, "plugins"))
        common.set_schemadir(os.path.join(parent_dir, "schema"))
        common.set_librariesdir(os.path.join(parent_dir, "libraries"))
        common.set_extensionsdir(os.path.join(parent_dir, "extensions"))
