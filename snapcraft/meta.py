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
import tempfile
import yaml

from snapcraft import common

logger = logging.getLogger(__name__)


_package_keys = [
    'name',
    'version',
    'vendor',
]

_optional_package_keys = [
    'frameworks',
    'type',
]


def create(config_data, arches=None):
    '''Creates meta in snap_dir from config_data.'''
    # TODO keys for using apparmor, setting an icon missing.

    meta_dir = os.path.join(common.get_snapdir(), 'meta')
    os.makedirs(meta_dir, exist_ok=True)

    package_yaml_path = os.path.join(meta_dir, 'package.yaml')
    package_yaml = compose_package_yaml(config_data, arches)

    with open(package_yaml_path, 'w') as f:
        yaml.dump(package_yaml, stream=f, default_flow_style=False)

    readme_md_path = os.path.join(meta_dir, 'readme.md')
    readme_md = compose_readme(config_data)

    with open(readme_md_path, 'w') as f:
        f.write(readme_md)


def compose_package_yaml(config_data, arches):
    '''Uses config_data to compose a proper package.yaml and returns.'''

    package_yaml = {}

    for key_name in _package_keys:
        package_yaml[key_name] = config_data[key_name]

    for key_name in _optional_package_keys:
        if key_name in config_data:
            package_yaml[key_name] = config_data[key_name]

    if arches:
        package_yaml['architectures'] = arches

    if 'binaries' in config_data:
        package_yaml['binaries'] = _wrap_binaries(config_data['binaries'])

    if 'services' in config_data:
        package_yaml['services'] = _wrap_services(config_data['services'])

    return package_yaml


def compose_readme(config_data):
    return '{}\n{}\n'.format(config_data['summary'], config_data['description'])


def _replace_cmd(execparts, cmd):
        newparts = [cmd] + execparts[1:]
        return ' '.join([shlex.quote(x) for x in newparts])


def _wrap_exe(relexepath):
    snap_dir = common.get_snapdir()
    exepath = os.path.join(snap_dir, relexepath)
    wrappath = exepath + '.wrapper'

    try:
        os.remove(wrappath)
    except Exception:
        pass

    wrapexec = '$SNAP_APP_PATH/{}'.format(relexepath)
    if not os.path.exists(exepath) and '/' not in relexepath:
        # If it doesn't exist it might be in the path
        logger.info('Checking to see if "{}" is in the $PATH'.format(relexepath))
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
