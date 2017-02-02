# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import logging
import os
import shutil
import subprocess

import apt

import snapcraft.internal.indicators
import snapcraft.internal.errors

from ._repo import (  # noqa
    fix_pkg_config,
    PackageNotFoundError,
    Ubuntu,
)

logger = logging.getLogger(__name__)


def is_package_installed(package):
    """Return True if a package is installed on the system.

    :param str package: the deb package to query for.
    :returns: True if the package is installed, False if not.
    """
    with apt.Cache() as apt_cache:
        return apt_cache[package].installed


def install_build_packages(packages):
    unique_packages = set(packages)
    new_packages = []
    with apt.Cache() as apt_cache:
        for pkg in unique_packages:
            try:
                if not apt_cache[pkg].installed:
                    new_packages.append(pkg)
            except KeyError as e:
                raise EnvironmentError(
                    'Could not find a required package in '
                    '\'build-packages\': {}'.format(str(e)))
    if new_packages:
        new_packages.sort()
        logger.info(
            'Installing build dependencies: %s', ' '.join(new_packages))
        env = os.environ.copy()
        env.update({
            'DEBIAN_FRONTEND': 'noninteractive',
            'DEBCONF_NONINTERACTIVE_SEEN': 'true',
        })

        apt_command = ['sudo', 'apt-get',
                       '--no-install-recommends', '-y']
        if not snapcraft.internal.indicators.is_dumb_terminal():
            apt_command.extend(['-o', 'Dpkg::Progress-Fancy=1'])
        apt_command.append('install')

        subprocess.check_call(apt_command + new_packages, env=env)

        try:
            subprocess.check_call(['sudo', 'apt-mark', 'auto'] +
                                  new_packages, env=env)
        except subprocess.CalledProcessError as e:
            logger.warning(
                'Impossible to mark packages as auto-installed: {}'
                .format(e))


def get_packages_for_source_type(source_type):
    """Return a list with required packages to handle the source_type.

    :param source: the snapcraft source type
    """
    if source_type == 'bzr':
        packages = 'bzr'
    elif source_type == 'git':
        packages = 'git'
    elif source_type == 'tar':
        packages = 'tar'
    elif source_type == 'hg' or source_type == 'mercurial':
        packages = 'mercurial'
    elif source_type == 'subversion' or source_type == 'svn':
        packages = 'subversion'
    else:
        packages = []

    return packages


def check_for_command(command):
    if not shutil.which(command):
        raise snapcraft.internal.errors.MissingCommandError([command])
