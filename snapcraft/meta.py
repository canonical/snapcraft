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
import re
import shlex
import shutil
import tempfile

import yaml

from snapcraft import common

logger = logging.getLogger(__name__)


_MANDATORY_PACKAGE_KEYS = [
    'name',
    'version',
]

_OPTIONAL_PACKAGE_KEYS = [
    'frameworks',
    'type',
    'icon',
    'license-version',
]

_OPTIONAL_HOOKS = [
    'config',
    'license',
]


def create(config_data):
    '''
    Create  the meta directory and provision it with package.yaml and readme.md
    in the snap dir using information from config_data and arches.
    If provided arches, is a list of arches.

    Returns meta_dir.
    '''

    meta_dir = os.path.join(common.get_snapdir(), 'meta')
    os.makedirs(meta_dir, exist_ok=True)

    if 'icon' in config_data:
        config_data['icon'] = _copy(meta_dir, config_data['icon'])

    if 'framework-policy' in config_data:
        _copy(meta_dir, config_data['framework-policy'], 'framework-policy')

    _write_package_yaml(meta_dir, config_data, config_data['architectures'])
    _write_readme_md(meta_dir, config_data)

    _setup_hooks(meta_dir, config_data)

    return meta_dir


def _write_package_yaml(meta_dir, config_data, arches):
    package_yaml_path = os.path.join(meta_dir, 'package.yaml')
    package_yaml = _compose_package_yaml(meta_dir, config_data, arches)

    with open(package_yaml_path, 'w') as f:
        yaml.dump(package_yaml, stream=f, default_flow_style=False)


def _write_readme_md(meta_dir, config_data):
    readme_md_path = os.path.join(meta_dir, 'readme.md')
    readme_md = _compose_readme(config_data)

    with open(readme_md_path, 'w') as f:
        f.write(readme_md)


def _setup_hooks(meta_dir, config_data):
    if any(key in config_data for key in _OPTIONAL_HOOKS):
        hooks_dir = os.path.join(meta_dir, 'hooks')
        os.makedirs(hooks_dir)

    if 'config' in config_data:
        _setup_config_hook(hooks_dir, config_data['config'])

    if 'license' in config_data:
        _setup_license_hook(hooks_dir, config_data['license'])


def _setup_config_hook(hooks_dir, config):
    config_hook_path = os.path.join(hooks_dir, 'config')

    execparts = shlex.split(config)
    execwrap = _wrap_exe(execparts[0], args=execparts[1:])
    os.rename(os.path.join(common.get_snapdir(), execwrap), config_hook_path)


def _setup_license_hook(hooks_dir, license):
    license_hook_path = os.path.join(hooks_dir, 'license')

    shutil.copyfile(license, license_hook_path)


def _copy(meta_dir, relpath, new_relpath=None):
    new_base = new_relpath or os.path.basename(relpath)
    new_relpath = os.path.join(meta_dir, new_base)

    if os.path.isdir(relpath):
        shutil.copytree(relpath, new_relpath)
    else:
        shutil.copyfile(relpath, new_relpath)

    return os.path.join('meta', os.path.basename(relpath))


def _copy_security_profiles(meta_dir, runnables):
    for runnable in runnables:
        for entry in ('security-policy', 'security-override'):
            if entry in runnable:
                runnable[entry]['apparmor'] = \
                    _copy(meta_dir, runnable[entry]['apparmor'])
                runnable[entry]['seccomp'] = \
                    _copy(meta_dir, runnable[entry]['seccomp'])

    return runnables


def _compose_package_yaml(meta_dir, config_data, arches):
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
        binaries = config_data['binaries']
        binaries = _wrap_binaries(binaries)
        package_yaml['binaries'] = \
            _copy_security_profiles(meta_dir, _repack_names(binaries))

    if 'services' in config_data:
        services = config_data['services']
        services = _wrap_services(services)
        package_yaml['services'] = \
            _copy_security_profiles(meta_dir, _repack_names(services))

    if config_data.get('license-agreement', '') == 'explicit':
        package_yaml['explicit-license-agreement'] = 'yes'

    return package_yaml


def _repack_names(names):
    repack = []
    for name in names:
        names[name].update({'name': name})
        repack.append(names[name])
    return repack


def _compose_readme(config_data):
    s = '{config[summary]}\n{config[description]}\n'
    return s.format(config=config_data)


def _replace_cmd(execparts, cmd):
        newparts = [cmd] + execparts[1:]
        return ' '.join([shlex.quote(x) for x in newparts])


def _write_wrap_exe(wrapexec, wrappath, shebang=None, args=None, cwd=None):
    args = ' '.join(args) + ' $*' if args else '$*'
    cwd = 'cd {}'.format(cwd) if cwd else ''

    snap_dir = common.get_snapdir()
    assembled_env = common.assemble_env().replace(snap_dir, '$SNAP_APP_PATH')
    replace_path = r'{}/[a-z0-9][a-z0-9+-]*/install'.format(
        common.get_partsdir())
    assembled_env = re.sub(replace_path, '$SNAP_APP_PATH', assembled_env)
    executable = '"{}"'.format(wrapexec)
    if shebang is not None:
        new_shebang = re.sub(replace_path, '$SNAP_APP_PATH', shebang)
        if new_shebang != shebang:
            # If the shebang was pointing to and executable within the
            # local 'parts' dir, have the wrapper script execute it
            # directly, since we can't use $SNAP_APP_PATH in the shebang
            # itself.
            executable = '"{}" "{}"'.format(new_shebang, wrapexec)
    script = ('#!/bin/sh\n' +
              '{}\n'.format(assembled_env) +
              '{}\n'.format(cwd) +
              'exec {} {}\n'.format(executable, args))

    with open(wrappath, 'w+') as f:
        f.write(script)

    os.chmod(wrappath, 0o755)


def _wrap_exe(relexepath, args=None):
    snap_dir = common.get_snapdir()
    exepath = os.path.join(snap_dir, relexepath)
    wrappath = exepath + '.wrapper'
    shebang = None

    # TODO talk to original author if the exception to be captured here is
    # FileNotFoundError, the original code was a general catch all
    try:
        os.remove(wrappath)
    except FileNotFoundError:
        pass

    wrapexec = '$SNAP_APP_PATH/{}'.format(relexepath)
    if not os.path.exists(exepath) and '/' not in relexepath:
        # If it doesn't exist it might be in the path
        logger.debug('Checking to see if "{}" is in the $PATH'.format(
            relexepath))
        with tempfile.NamedTemporaryFile('w+') as tempf:
            script = ('#!/bin/sh\n' +
                      '{}\n'.format(common.assemble_env()) +
                      'which "{}"\n'.format(relexepath))
            tempf.write(script)
            tempf.flush()
            common.run(['/bin/sh', tempf.name], cwd=snap_dir)
            wrapexec = relexepath
    else:
        with open(exepath, 'rb') as exefile:
            # If the file has a she-bang, the path might be pointing to
            # the local 'parts' dir. Extract it so that _write_wrap_exe
            # will have a chance to rewrite it.
            if exefile.read(2) == b'#!':
                shebang = exefile.readline().strip().decode('utf-8')

    _write_wrap_exe(wrapexec, wrappath, shebang=shebang, args=args)

    return os.path.relpath(wrappath, snap_dir)


def _wrap_binaries(binaries):
    for name in binaries:
        execparts = shlex.split(binaries[name]['exec'])
        execwrap = _wrap_exe(execparts[0])
        binaries[name]['exec'] = _replace_cmd(execparts, execwrap)

    return binaries


def _wrap_services(services):
    for name in services:
        startpath = services[name].get('start')
        if startpath:
            startparts = shlex.split(startpath)
            startwrap = _wrap_exe(startparts[0])
            services[name]['start'] = _replace_cmd(startparts, startwrap)
        stoppath = services[name].get('stop')
        if stoppath:
            stopparts = shlex.split(stoppath)
            stopwrap = _wrap_exe(stopparts[0])
            services[name]['stop'] = _replace_cmd(stopparts, stopwrap)

    return services
