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


def setup_dirs():
    """
    Ensure that snapcraft.common plugindir is setup correctly
    and support running out of a development snapshot
    """
    from snapcraft.internal import common
    topdir = os.path.abspath(os.path.join(__file__, '..', '..', '..'))
    # only change the default if we are running from a checkout
    if os.path.exists(os.path.join(topdir, 'setup.py')):
        common.set_plugindir(os.path.join(topdir, 'snapcraft', 'plugins'))
        common.set_schemadir(os.path.join(topdir, 'schema'))
        common.set_librariesdir(os.path.join(topdir, 'libraries'))
        common.set_tourdir(os.path.join(topdir, 'tour'))
