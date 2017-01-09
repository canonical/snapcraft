# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017 Canonical Ltd
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

import contextlib
import os
import configparser
import logging
import re
import shlex
import shutil
import stat
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
    'hooks',
]


class CommandError(Exception):
    pass


def create_snap_packaging(config_data, snap_dir, parts_dir):
    """Create snap.yaml and related assets in meta.

    Create the meta directory and provision it with snap.yaml in the snap dir
    using information from config_data. Also copy in the local 'snap'
    directory, and generate wrappers for hooks coming from parts.

    :param dict config_data: project values defined in snapcraft.yaml.
    :return: meta_dir.
    """
    packaging = _SnapPackaging(config_data, snap_dir, parts_dir)
    packaging.write_snap_yaml()
    packaging.setup_assets()
    packaging.generate_hook_wrappers()
    packaging.write_snap_directory()

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

    def write_snap_directory(self):
        # First migrate the snap directory. It will overwrite any conflicting
        # files.
        for root, directories, files in os.walk('snap'):
            for directory in directories:
                source = os.path.join(root, directory)
                destination = os.path.join(self._snap_dir, source)
                file_utils.create_similar_directory(source, destination)

            for file_path in files:
                source = os.path.join(root, file_path)
                destination = os.path.join(self._snap_dir, source)
                with contextlib.suppress(FileNotFoundError):
                    os.remove(destination)
                file_utils.link_or_copy(source, destination)

        # Now copy the hooks contained within the snap directory directly into
        # meta (they don't get wrappers like the ones that come from parts).
        snap_hooks_dir = os.path.join('snap', 'hooks')
        hooks_dir = os.path.join(self._snap_dir, 'meta', 'hooks')
        if os.path.isdir(snap_hooks_dir):
            os.makedirs(hooks_dir, exist_ok=True)
            for hook_name in os.listdir(snap_hooks_dir):
                source = os.path.join(snap_hooks_dir, hook_name)
                destination = os.path.join(hooks_dir, hook_name)

                # First, verify that the hook is actually executable
                if not os.stat(source).st_mode & stat.S_IEXEC:
                    raise CommandError('hook {!r} is not executable'.format(
                        hook_name))

                with contextlib.suppress(FileNotFoundError):
                    os.remove(destination)

                file_utils.link_or_copy(source, destination)

    def generate_hook_wrappers(self):
        snap_hooks_dir = os.path.join(self._snap_dir, 'snap', 'hooks')
        hooks_dir = os.path.join(self._snap_dir, 'meta', 'hooks')
        if os.path.isdir(snap_hooks_dir):
            os.makedirs(hooks_dir, exist_ok=True)
            for hook_name in os.listdir(snap_hooks_dir):
                file_path = os.path.join(snap_hooks_dir, hook_name)
                # First, verify that the hook is actually executable
                if not os.stat(file_path).st_mode & stat.S_IEXEC:
                    raise CommandError('hook {!r} is not executable'.format(
                        hook_name))

                hook_exec = os.path.join('$SNAP', 'snap', 'hooks', hook_name)
                hook_path = os.path.join(hooks_dir, hook_name)
                with contextlib.suppress(FileNotFoundError):
                    os.remove(hook_path)

                self._write_wrap_exe(hook_exec, hook_path)

    def _setup_from_setup(self):
        setup_dir = 'setup'
        if not os.path.exists(setup_dir):
            return

        gui_src = os.path.join(setup_dir, 'gui')
        gui_dst = os.path.join(self.meta_dir, 'gui')
        if os.path.exists(gui_src):
            for f in os.listdir(gui_src):
                if not os.path.exists(gui_dst):
                    os.mkdir(gui_dst)
                shutil.copy2(os.path.join(gui_src, f), gui_dst)

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

        # If we are dealing with classic confinement it means all our
        # binaries are linked with `nodefaultlib` so this is harmless.
        # We do however want to be on the safe side and make sure no
        # ABI breakage happens by accidentally loading a library from
        # the classic system.
        include_library_paths = self._config_data['confinement'] != 'classic'
        assembled_env = common.assemble_env(include_library_paths)
        assembled_env = assembled_env.replace(self._snap_dir, '$SNAP')
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
        gui_dir = os.path.join(self.meta_dir, 'gui')
        if not os.path.exists(gui_dir):
            os.mkdir(gui_dir)
        for f in os.listdir(gui_dir):
            if os.path.splitext(f)[1] == '.desktop':
                os.remove(os.path.join(gui_dir, f))
        for app in apps:
            self._wrap_app(app, apps[app])
        return apps

    def _wrap_app(self, name, app):
        cmds = (k for k in ('command', 'stop-command') if k in app)
        for k in cmds:
            try:
                app[k] = self._wrap_exe(app[k], '{}-{}'.format(k, name))
            except CommandError as e:
                raise EnvironmentError(
                    'The specified command {!r} defined in the app {!r} '
                    'does not exist or is not executable'.format(str(e), name))
        if 'desktop' in app:
            self._reformat_desktop(name, app)

    def _reformat_desktop(self, name, app):
        gui_dir = os.path.join(self.meta_dir, 'gui')
        desktop_file = os.path.join(self._snap_dir, app['desktop'])
        if not os.path.exists(desktop_file):
            raise EnvironmentError(
                'The specified desktop file {!r} defined in the app '
                '{!r} does not exist'.format(desktop_file, name))
        desktop_contents = configparser.ConfigParser(interpolation=None)
        desktop_contents.optionxform = str
        desktop_contents.read(desktop_file)
        section = 'Desktop Entry'
        if section not in desktop_contents.sections():
            raise EnvironmentError(
                'The specified desktop file {!r} is not a valid '
                'desktop file'.format(desktop_file))
        if 'Exec' not in desktop_contents[section]:
            raise EnvironmentError(
                'The specified desktop file {!r} is missing the '
                '"Exec" key'.format(desktop_file))
        # XXX: do we want to allow more parameters for Exec?
        exec_value = '{}.{} %U'.format(self._config_data['name'], name)
        desktop_contents[section]['Exec'] = exec_value
        if 'Icon' in desktop_contents[section]:
            icon = desktop_contents[section]['Icon']
            if icon.startswith('/'):
                icon = icon.lstrip('/')
                if os.path.exists(os.path.join(self._snap_dir, icon)):
                    desktop_contents[section]['Icon'] = \
                        '${{SNAP}}/{}'.format(icon)
                else:
                    logger.warning(
                        'Icon {} specified in desktop file {} not found '
                        'in prime directory'.format(icon, app['desktop']))
        target = os.path.join(gui_dir, os.path.basename(desktop_file))
        if os.path.exists(target):
            raise EnvironmentError(
                'Conflicting desktop file referenced by more than one '
                'app: {!r}'.format(desktop_file))
        with open(target, 'w') as f:
            desktop_contents.write(f, space_around_delimiters=False)


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
