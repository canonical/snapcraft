# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import subprocess
import tempfile

import yaml

from snapcraft import file_utils
from snapcraft.internal import common
from snapcraft.internal.errors import MissingGadgetError


logger = logging.getLogger(__name__)


_MANDATORY_PACKAGE_KEYS = [
    'name',
    'version',
    'description',
    'summary',
]

_OPTIONAL_PACKAGE_KEYS = [
    'architectures',
    'assumes',
    'type',
    'plugs',
    'slots',
    'confinement',
    'epoch',
    'grade',
]


class CommandError(Exception):
    pass


def create_snap_packaging(config_data, snap_dir, parts_dir):
    """Create snap.yaml and related assets in meta.
    Create  the meta directory and provision it with snap.yaml
    in the snap dir using information from config_data.

    :param dict config_data: project values defined in snapcraft.yaml.
    :return: meta_dir.
    """
    packaging = _SnapPackaging(config_data, snap_dir, parts_dir)
    packaging.write_snap_yaml()
    packaging.setup_assets()

    return packaging.meta_dir


class _SnapPackaging:

    @property
    def meta_dir(self):
        return self._meta_dir

    def __init__(self, config_data, snap_dir, parts_dir):
        self._snap_dir = snap_dir
        self._parts_dir = parts_dir
        self._meta_dir = os.path.join(self._snap_dir, 'meta')
        self._config_data = config_data

        os.makedirs(self._meta_dir, exist_ok=True)

    def write_snap_yaml(self):
        package_snap_path = os.path.join(self.meta_dir, 'snap.yaml')
        snap_yaml = self._compose_snap_yaml()

        with open(package_snap_path, 'w') as f:
            yaml.dump(snap_yaml, stream=f, default_flow_style=False)

        return snap_yaml

    def setup_assets(self):
        # We do _setup_from_setup first since it is legacy and let the
        # declarative items take over.
        self._setup_from_setup()

        if 'icon' in self._config_data:
            # TODO: use developer.ubuntu.com once it has updated documentation.
            icon_ext = self._config_data['icon'].split(os.path.extsep)[1]
            icon_dir = os.path.join(self.meta_dir, 'gui')
            icon_path = os.path.join(icon_dir, 'icon.{}'.format(icon_ext))
            if not os.path.exists(icon_dir):
                os.mkdir(icon_dir)
            if os.path.exists(icon_path):
                os.unlink(icon_path)
            os.link(self._config_data['icon'], icon_path)

        if self._config_data.get('type', '') == 'gadget':
            if not os.path.exists('gadget.yaml'):
                raise MissingGadgetError()
            file_utils.link_or_copy(
                'gadget.yaml', os.path.join(self.meta_dir, 'gadget.yaml'))

    def _setup_from_setup(self):
        setup_dir = 'setup'
        if not os.path.exists(setup_dir):
            return

        gui_src = os.path.join(setup_dir, 'gui')
        gui_dst = os.path.join(self.meta_dir, 'gui')
        if os.path.exists(gui_dst) and os.path.exists(gui_src):
            shutil.rmtree(gui_dst)
        if os.path.exists(gui_src):
            shutil.copytree(gui_src, gui_dst)

    def _compose_snap_yaml(self):
        """Create a new dictionary from config_data to obtain snap.yaml.

        Missing key exceptions will be raised if config_data does not contain
        all the _MANDATORY_PACKAGE_KEYS, config_data can be validated against
        the snapcraft schema.

        Keys that are in _OPTIONAL_PACKAGE_KEYS are ignored if not there.
        """
        snap_yaml = {}

        for key_name in _MANDATORY_PACKAGE_KEYS:
            snap_yaml[key_name] = self._config_data[key_name]

        for key_name in _OPTIONAL_PACKAGE_KEYS:
            if key_name in self._config_data:
                snap_yaml[key_name] = self._config_data[key_name]

        if 'apps' in self._config_data:
            snap_yaml['apps'] = self._wrap_apps(self._config_data['apps'])

        return snap_yaml

    def _write_wrap_exe(self, wrapexec, wrappath,
                        shebang=None, args=None, cwd=None):
        args = ' '.join(args) + ' "$@"' if args else '"$@"'
        cwd = 'cd {}'.format(cwd) if cwd else ''

        assembled_env = common.assemble_env().replace(self._snap_dir, '$SNAP')
        replace_path = r'{}/[a-z0-9][a-z0-9+-]*/install'.format(
            self._parts_dir)
        assembled_env = re.sub(replace_path, '$SNAP', assembled_env)
        executable = '"{}"'.format(wrapexec)
        if shebang is not None:
            new_shebang = re.sub(replace_path, '$SNAP', shebang)
            if new_shebang != shebang:
                # If the shebang was pointing to and executable within the
                # local 'parts' dir, have the wrapper script execute it
                # directly, since we can't use $SNAP in the shebang itself.
                executable = '"{}" "{}"'.format(new_shebang, wrapexec)
        script = ('#!/bin/sh\n' +
                  '{}\n'.format(assembled_env) +
                  '{}\n'.format(cwd) +
                  'LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH\n'
                  'exec {} {}\n'.format(executable, args))

        with open(wrappath, 'w+') as f:
            f.write(script)

        os.chmod(wrappath, 0o755)

    def _wrap_exe(self, command, basename=None):
        execparts = shlex.split(command)
        exepath = os.path.join(self._snap_dir, execparts[0])
        if basename:
            wrappath = os.path.join(self._snap_dir, basename) + '.wrapper'
        else:
            wrappath = exepath + '.wrapper'
        shebang = None

        if os.path.exists(wrappath):
            os.remove(wrappath)

        wrapexec = '$SNAP/{}'.format(execparts[0])
        if not os.path.exists(exepath) and '/' not in execparts[0]:
            _find_bin(execparts[0], self._snap_dir)
            wrapexec = execparts[0]
        else:
            with open(exepath, 'rb') as exefile:
                # If the file has a she-bang, the path might be pointing to
                # the local 'parts' dir. Extract it so that _write_wrap_exe
                # will have a chance to rewrite it.
                if exefile.read(2) == b'#!':
                    shebang = exefile.readline().strip().decode('utf-8')

        self._write_wrap_exe(wrapexec, wrappath,
                             shebang=shebang, args=execparts[1:])

        return os.path.relpath(wrappath, self._snap_dir)

    def _wrap_apps(self, apps):
        for app in apps:
            cmds = (k for k in ('command', 'stop-command') if k in apps[app])
            for k in cmds:
                try:
                    apps[app][k] = self._wrap_exe(
                        apps[app][k], '{}-{}'.format(k, app))
                except CommandError as e:
                    raise EnvironmentError(
                        'The specified command {!r} defined in the app {!r} '
                        'does not exist or is not executable'.format(
                            str(e), app))
        return apps


def _find_bin(binary, basedir):
    # If it doesn't exist it might be in the path
    logger.debug('Checking that {!r} is in the $PATH'.format(binary))
    script = ('#!/bin/sh\n' +
              '{}\n'.format(common.assemble_env()) +
              'which "{}"\n'.format(binary))
    with tempfile.NamedTemporaryFile('w+') as tempf:
        tempf.write(script)
        tempf.flush()
        try:
            common.run(['/bin/sh', tempf.name], cwd=basedir,
                       stdout=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            raise CommandError(binary)
