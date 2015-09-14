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

import os
import logging
import shlex
import shutil
import tempfile
import yaml

from snapcraft import common

logger = logging.getLogger(__name__)


_MANDATORY_PACKAGE_KEYS = [
    'name',
    'version',
    'vendor',
    'icon',
]

_OPTIONAL_PACKAGE_KEYS = [
    'frameworks',
    'type',
]


class InvalidConfigHookError(Exception):

    @property
    def config(self):
        return self._config

    def __init__(self, config):
        self._config = config


def create(config_data, arches=None):
    '''
    Create  the meta directory and provision it with package.yaml and readme.md
    in the snap dir using information from config_data and arches.
    If provided arches, is a list of arches.

    Returns meta_dir.
    '''
    # TODO keys for using apparmor, setting an icon missing.

    meta_dir = os.path.join(common.get_snapdir(), 'meta')
    os.makedirs(meta_dir, exist_ok=True)

    config_data['icon'] = _copy_icon(meta_dir, config_data['icon'])

    _write_package_yaml(meta_dir, config_data, arches)
    _write_readme_md(meta_dir, config_data)

    if 'config' in config_data:
        _setup_config_hook(meta_dir, config_data['config'])

    return meta_dir


def _write_package_yaml(meta_dir, config_data, arches):
    package_yaml_path = os.path.join(meta_dir, 'package.yaml')
    package_yaml = _compose_package_yaml(config_data, arches)

    with open(package_yaml_path, 'w') as f:
        yaml.dump(package_yaml, stream=f, default_flow_style=False)


def _write_readme_md(meta_dir, config_data):
    readme_md_path = os.path.join(meta_dir, 'readme.md')
    readme_md = _compose_readme(config_data)

    with open(readme_md_path, 'w') as f:
        f.write(readme_md)


def _setup_config_hook(meta_dir, config):
    hooks_dir = os.path.join(meta_dir, 'hooks')
    os.makedirs(hooks_dir)

    if not os.path.exists(os.path.join(meta_dir, config)):
        raise InvalidConfigHookError(config)

    os.symlink(os.path.join('..', '..', config), os.path.join(hooks_dir, 'config'))


def _copy_icon(meta_dir, icon_path):
    new_icon_path = os.path.join(meta_dir, os.path.basename(icon_path))
    shutil.copyfile(icon_path, new_icon_path)

    return os.path.join('meta', os.path.basename(icon_path))


def _compose_package_yaml(config_data, arches):
    '''
    Creates a dictionary that can be used to yaml.dump a package.yaml using
    config_data.
    If provided arches, is a list of arches.

    Missing key exceptions will be raised if config_data does not hold
    MANDATORY_KEYS, config_data can be validated against the snapcraft schema.
    '''
    package_yaml = {}

    for key_name in _MANDATORY_PACKAGE_KEYS:
        package_yaml[key_name] = config_data[key_name]

    for key_name in _OPTIONAL_PACKAGE_KEYS:
        if key_name in config_data:
            package_yaml[key_name] = config_data[key_name]

    if arches:
        package_yaml['architectures'] = arches

    if 'binaries' in config_data:
        package_yaml['binaries'] = _wrap_binaries(config_data['binaries'])

    if 'services' in config_data:
        package_yaml['services'] = _wrap_services(config_data['services'])

    return package_yaml


def _compose_readme(config_data):
    return '{config[summary]}\n{config[description]}\n'.format(config=config_data)


def _replace_cmd(execparts, cmd):
        newparts = [cmd] + execparts[1:]
        return ' '.join([shlex.quote(x) for x in newparts])


def _wrap_exe(relexepath):
    snap_dir = common.get_snapdir()
    exepath = os.path.join(snap_dir, relexepath)
    wrappath = exepath + '.wrapper'

    # TODO talk to original author if the exception to be captured here is
    # FileNotFoundError, the original code was a general catch all
    try:
        os.remove(wrappath)
    except FileNotFoundError:
        pass

    wrapexec = '$SNAP_APP_PATH/{}'.format(relexepath)
    if not os.path.exists(exepath) and '/' not in relexepath:
        # If it doesn't exist it might be in the path
        logger.debug('Checking to see if "{}" is in the $PATH'.format(relexepath))
        with tempfile.NamedTemporaryFile('w+') as tempf:
            script = ('#!/bin/sh\n' +
                      '{}\n'.format(common.assemble_env()) +
                      'which "{}"\n'.format(relexepath))
            tempf.write(script)
            tempf.flush()
            if common.run(['/bin/sh', tempf.name], cwd=snap_dir):
                wrapexec = relexepath
            else:
                logger.warning('Warning: unable to find "{}" in the path'.format(relexepath))

    assembled_env = common.assemble_env().replace(snap_dir, '$SNAP_APP_PATH')
    script = ('#!/bin/sh\n' +
              '{}\n'.format(assembled_env) +
              'exec "{}" $*\n'.format(wrapexec))

    with open(wrappath, 'w+') as f:
        f.write(script)

    os.chmod(wrappath, 0o755)

    return os.path.relpath(wrappath, snap_dir)


def _wrap_binaries(binaries):
    for binary in binaries:
        execparts = shlex.split(binary.get('exec', binary['name']))
        execwrap = _wrap_exe(execparts[0])
        if 'exec' in binary:
            binary['exec'] = _replace_cmd(execparts, execwrap)
        else:
            binary['name'] = os.path.basename(binary['name'])
            binary['exec'] = _replace_cmd(execparts, execwrap)

    return binaries


def _wrap_services(services):
    for binary in services:
        startpath = binary.get('start')
        if startpath:
            startparts = shlex.split(startpath)
            startwrap = _wrap_exe(startparts[0])
            binary['start'] = _replace_cmd(startparts, startwrap)
        stoppath = binary.get('stop')
        if stoppath:
            stopparts = shlex.split(stoppath)
            stopwrap = _wrap_exe(stopparts[0])
            binary['stop'] = _replace_cmd(stopparts, stopwrap)

    return services
