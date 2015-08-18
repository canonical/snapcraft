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

import snapcraft


class GoPlugin(snapcraft.BasePlugin):

    def __init__(self, name, options):
        super().__init__(name, options)

    def env(self, root):
        # usr/lib/go/bin on newer Ubuntus, usr/bin on trusty
        return [
            "GOPATH={}/go".format(root),
        ]

    def snap_files(self):
        return ([], [])  # Go statically links everything, no need for runtime
