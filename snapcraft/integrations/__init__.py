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

"""Snapcraft integrations layer.

Defines 'enable-ci' command infrastructure to support multiple integrations
systems in an isolated form.
"""
import importlib


SUPPORTED_CI_SYSTEMS = (
    'travis',
)


def enable_ci(ci_system):
    if not ci_system:
        # XXX cprov 20161116: we could possibly auto-detect currently
        # integration systems in master ?
        raise EnvironmentError(
            'Please select one of the supported integration systems: '
            '{}.'.format(','.join(SUPPORTED_CI_SYSTEMS))
        )
    elif ci_system not in SUPPORTED_CI_SYSTEMS:
        raise EnvironmentError(
            '"{}" integration is not supported by snapcraft.\n'
            'Please select one of the supported integration systems: '
            '{}.'.format(ci_system, ','.join(SUPPORTED_CI_SYSTEMS))
        )

    module = importlib.import_module(
        'snapcraft.integrations.{}'.format(ci_system))

    print(module.__doc__)

    if input('Continue (y/N): ') == 'y':
        module.enable()
