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

import snapcraft
from snapcraft import formatting_utils
from snapcraft.internal import (
    common,
    libraries,
)


def runtime_env(root, arch_triplet):
    """Set the environment variables required for running binaries."""
    env = []

    env.append('PATH="' + ':'.join([
        '{0}/usr/sbin',
        '{0}/usr/bin',
        '{0}/sbin',
        '{0}/bin',
        '$PATH'
    ]).format(root) + '"')

    # Add the default LD_LIBRARY_PATH
    paths = common.get_library_paths(root, arch_triplet)
    if paths:
        env.append(formatting_utils.format_path_variable(
            'LD_LIBRARY_PATH', paths, prepend='', separator=':'))

    # Add more specific LD_LIBRARY_PATH from staged packages if necessary
    ld_library_paths = libraries.determine_ld_library_path(root)
    if ld_library_paths:
        env.append('LD_LIBRARY_PATH="' + ':'.join(ld_library_paths) +
                   ':$LD_LIBRARY_PATH"')

    return env


def build_env(root, snap_name, confinement, arch_triplet,
              core_dynamic_linker=None):
    """Set the environment variables required for building.

    This is required for the current parts installdir due to stage-packages
    and also to setup the stagedir.
    """
    env = []

    paths = common.get_include_paths(root, arch_triplet)
    if paths:
        for envvar in ['CPPFLAGS', 'CFLAGS', 'CXXFLAGS']:
            env.append(formatting_utils.format_path_variable(
                envvar, paths, prepend='-I', separator=' '))

    if confinement == 'classic':
        if not core_dynamic_linker:
            raise snapcraft.internal.errors.SnapcraftEnvironmentError(
                'classic confinement requires the core snap to be installed. '
                'Install it by running `snap install core`.')

        core_path = common.get_core_path()
        core_rpaths = common.get_library_paths(core_path, arch_triplet,
                                               existing_only=False)
        snap_path = os.path.join('/snap', snap_name, 'current')
        snap_rpaths = common.get_library_paths(snap_path, arch_triplet,
                                               existing_only=False)

        # snap_rpaths before core_rpaths to prefer libraries from the snap.
        rpaths = formatting_utils.combine_paths(
            snap_rpaths + core_rpaths, prepend='', separator=':')
        env.append('LDFLAGS="$LDFLAGS '
                   # Building tools to continue the build becomes problematic
                   # with nodefaultlib.
                   # '-Wl,-z,nodefaultlib '
                   '-Wl,--dynamic-linker={0} '
                   '-Wl,-rpath,{1}"'.format(core_dynamic_linker, rpaths))

    paths = common.get_library_paths(root, arch_triplet)
    if paths:
        env.append(formatting_utils.format_path_variable(
            'LDFLAGS', paths, prepend='-L', separator=' '))

    paths = common.get_pkg_config_paths(root, arch_triplet)
    if paths:
        env.append(formatting_utils.format_path_variable(
            'PKG_CONFIG_PATH', paths, prepend='', separator=':'))

    return env


def build_env_for_stage(stagedir, snap_name, confinement,
                        arch_triplet, core_dynamic_linker=None):
    env = build_env(stagedir, snap_name, confinement,
                    arch_triplet, core_dynamic_linker)
    env.append('PERL5LIB={0}/usr/share/perl5/'.format(stagedir))

    return env
