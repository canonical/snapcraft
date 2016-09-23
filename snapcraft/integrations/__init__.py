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
from contextlib import contextmanager
import importlib
import os
import subprocess


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

    if input('Continue (y/N):') == 'y':
        module.enable()


@contextmanager
def requires_command_success(command, not_found_err=None, failure_err=None):
    if isinstance(command, str):
        cmd_list = command.split()
    else:
        raise TypeError('command must be a string.')
    not_found_err = not_found_err or EnvironmentError(
        '`{}` not found.'.format(cmd_list[0]))
    failure_err = failure_err or EnvironmentError(
        '`{}` failed.'.format(command))
    try:
        subprocess.check_call(
            cmd_list, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except FileNotFoundError as err:
        raise not_found_err
    except subprocess.CalledProcessError as err:
        raise failure_err
    yield


@contextmanager
def requires_path_exists(path, error_message=None):
    if error_message is None:
        error_message = 'Required path does not exist: {}'.format(path)
    if not os.path.exists(path):
        raise EnvironmentError(error_message)
    yield
