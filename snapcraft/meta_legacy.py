# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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
import logging

import yaml

logger = logging.getLogger(__name__)


_MANDATORY_PACKAGE_KEYS = [
    'name',
    'version',
]

_OPTIONAL_PACKAGE_KEYS = [
    'architectures',
    'type',
    'icon',
    'license-version',
]


_SERVICE_TRANSLATION_MAP = {
    'stop-command': 'stop',
    'stop-timeout': 'stop-timeout',
}


def create(meta_dir, snap_yaml):
    """Creates the legacy package.yaml and readme.md.
    Creates package.yaml and readme.md for legacy purposes until the
    transition to snap.yaml is complete.

    :param dict config_data: project values defined in snapcraft.yaml.
    :return: meta_dir.
    """
    _write_package_yaml(meta_dir, snap_yaml)
    _write_readme_md(meta_dir, snap_yaml)


def _write_package_yaml(meta_dir, snap_yaml):
    package_yaml_path = os.path.join(meta_dir, 'package.yaml')
    package_yaml = _compose_package_yaml(meta_dir, snap_yaml)

    with open(package_yaml_path, 'w') as f:
        yaml.dump(package_yaml, stream=f, default_flow_style=False)


def _write_readme_md(meta_dir, snap_yaml):
    readme_md_path = os.path.join(meta_dir, 'readme.md')
    readme_md = _compose_readme(snap_yaml)

    with open(readme_md_path, 'w') as f:
        f.write(readme_md)


def _compose_package_yaml(meta_dir, snap_yaml):
    '''
    Creates a dictionary that can be used to yaml.dump a package.yaml using
    config_data.
    If provided arches, is a list of arches.

    Missing key exceptions will be raised if config_data does not hold
    MANDATORY_KEYS, config_data can be validated against the snapcraft schema.
    '''
    package_yaml = {}

    for key_name in _MANDATORY_PACKAGE_KEYS:
        package_yaml[key_name] = snap_yaml[key_name]

    for key_name in _OPTIONAL_PACKAGE_KEYS:
        if key_name in snap_yaml:
            package_yaml[key_name] = snap_yaml[key_name]

    if snap_yaml.get('license-agreement', '') == 'explicit':
        package_yaml['explicit-license-agreement'] = 'yes'

    # This is rather inelegant but it is legacy code that will go away
    # soon enought. It supports the only 2 extensions allowed for an icon.
    if os.path.exists(os.path.join(meta_dir, 'icon.png')):
        package_yaml['icon'] = 'meta/icon.png'
    elif os.path.exists(os.path.join(meta_dir, 'icon.svg')):
        package_yaml['icon'] = 'meta/icon.svg'

    if 'apps' in snap_yaml:
        package_yaml.update(_repack_apps(snap_yaml['apps']))

    return package_yaml


def _repack_binaries(binaries):
    repack = []
    for name in binaries:
        command = binaries[name].pop('command')
        binaries[name].update({'name': name, 'exec': command})
        repack.append(binaries[name])

    return repack


def _repack_services(services):
    repack = []
    for name in services:
        s = {
            'name': name,
            'start': services[name].pop('command'),
            'description': 'service for {}'.format(name),
        }

        for k in _SERVICE_TRANSLATION_MAP:
            v = services[name].pop(k, None)
            if v:
                s[_SERVICE_TRANSLATION_MAP[k]] = v
        if services[name]['daemon'] == 'forking':
            s['forking'] = 'yes'
        services[name].update(s)
        repack.append(services[name])

    return repack


def _repack_apps(apps):
    binaries = {k: apps[k] for k in apps if 'daemon' not in apps[k]}
    services = {k: apps[k] for k in apps if 'daemon' in apps[k]}

    apps = {}
    if binaries:
        apps['binaries'] = _repack_binaries(binaries)
    if services:
        apps['services'] = _repack_services(services)

    return apps


def _compose_readme(snap_yaml):
    s = '{config[summary]}\n{config[description]}\n'
    return s.format(config=snap_yaml)
