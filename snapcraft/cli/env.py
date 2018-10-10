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

from snapcraft.internal import errors
from snapcraft.formatting_utils import humanize_list


class BuilderEnvironmentConfig:
    """Handle the chosen build provider.

    To determine the build environment, SNAPCRAFT_BUILD_ENVIRONMENT is
    retrieved from the environment and used to determine the build
    provider. If it is not set, a value of `host` is assumed.

    Valid values are:

    - host: the host will drive the build.
    - multipass: a vm driven by multipass will be created to drive the build.
    """

    def __init__(self, *, default="host") -> None:
        """Instantiate a BuildEnvironmentConfig.

        :param str default: the default provider to use among the list of valid
                            ones.
        :param str additional_providers: Additional providers allowed in the
                                         environment.
        """
        valid_providers = ["host", "multipass", "managed-host"]
        build_provider = os.environ.get("SNAPCRAFT_BUILD_ENVIRONMENT")

        if not build_provider:
            build_provider = default
        elif build_provider not in valid_providers:
            raise errors.SnapcraftEnvironmentError(
                "SNAPCRAFT_BUILD_ENVIRONMENT must be one of: {}.".format(
                    humanize_list(items=valid_providers, conjunction="or")
                )
            )

        self.provider = build_provider
        self.is_host = build_provider == "host"
        self.is_multipass = build_provider == "multipass"
        self.is_managed_host = build_provider == "managed-host"
