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
import os
from distutils import util
from snapcraft.internal import errors, lxd


class ContainerConfig:

    def __init__(self):
        """
        Determines if a container should be used and which remote to choose

        Checks the environment variable SNAPCRAFT_CONTAINER_BUILDS:
          1. SNAPCRAFT_CONTAINER_BUILDS=1 enables local containers
          2. SNAPCRAFT_CONTAINER_BUILDS=foobar uses the value as a remote
          3. SNAPCRAFT_CONTAINER_BUILDS=0 or unset, no container is used
        """

        container_builds = os.environ.get('SNAPCRAFT_CONTAINER_BUILDS', '0')
        # Default remote if it's a truthy value - otherwise it's a remote name
        try:
            self._use_container = util.strtobool(container_builds)
            self._remote = None
        except ValueError:
            self._use_container = True
            # Verbatim name of a remote
            if not lxd._remote_is_valid(container_builds):
                raise errors.InvalidContainerRemoteError(container_builds)
            self._remote = container_builds

    @property
    def use_container(self):
        return self._use_container

    @property
    def remote(self):
        return self._remote


def get_container_config():
    return ContainerConfig()
