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

from . import echo
from snapcraft.internal import errors


class BuilderEnvironmentConfig:
    """Handle the chosen build provider."""

    def __init__(self) -> None:
        use_lxd = None
        container_builds = os.environ.get('SNAPCRAFT_CONTAINER_BUILDS')
        if container_builds:
            echo.warning(
                'The flag SNAPCRAFT_CONTAINER_BUILDS has been deprecated. '
                'Use SNAPCRAFT_BUILD_ENVIRONMENT=lxd instead.')
            try:
                use_lxd = util.strtobool(container_builds)
            except ValueError:
                raise errors.SnapcraftEnvironmentError(
                    'The experimental feature of using non-local LXD remotes '
                    'with SNAPCRAFT_CONTAINER_BUILDS has been dropped.')

        build_provider = os.environ.get('SNAPCRAFT_BUILD_ENVIRONMENT')
        if build_provider and use_lxd:
            raise errors.SnapcraftEnvironmentError(
                'SNAPCRAFT_BUILD_ENVIRONMENT and SNAPCRAFT_CONTAINER_BUILDS '
                'cannot be used together.\n'
                'Unset one of them from the environment and try again.')

        if use_lxd:
            build_provider = 'lxd'
        elif not build_provider:
            echo.warning('Using the host as the build environment.')
            build_provider = 'host'
        # TODO add multipass
        elif build_provider not in ['host', 'lxd']:
            raise errors.SnapcraftEnvironmentError(
                'SNAPCRAFT_BUILD_ENVIRONMENT must be one of: host or lxd.')

        self.provider = build_provider
        self.is_host = build_provider == 'host'
