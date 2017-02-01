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
from snapcraft.internal import common, project_loader
from snapcraft.internal.errors import MissingGadgetError
from snapcraft.internal.deprecations import handle_deprecation_notice


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


def create_snap_packaging(config_data, project_options):
    """Create snap.yaml and related assets in meta.

    Create the meta directory and provision it with snap.yaml in the snap dir
    using information from config_data. Also copy in the local 'snap'
    directory, and generate wrappers for hooks coming from parts.

    :param dict config_data: project values defined in snapcraft.yaml.
    :return: meta_dir.
    """
    packaging = _SnapPackaging(config_data, project_options)
    packaging.write_snap_yaml()
    packaging.setup_assets()
    packaging.generate_hook_wrappers()
    packaging.write_snap_directory()

    return packaging.meta_dir


class _SnapPackaging:

    @property
    def meta_dir(self):
        return self._meta_dir

    def __init__(self, config_data, project_options):
        self._snap_dir = project_options.snap_dir
        self._parts_dir = project_options.parts_dir
        self._arch_triplet = project_options.arch_triplet

        self._meta_dir = os.path.join(self._snap_dir, 'meta')
        self._config_data = config_data.copy()

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
        self._setup_gui()

        if 'icon' in self._config_data:
            # TODO: use developer.ubuntu.com once it has updated documentation.
            icon_ext = self._config_data['icon'].split(os.path.extsep)[-1]
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

    def _ensure_snapcraft_yaml(self):
        source = project_loader.get_snapcraft_yaml()
        destination = os.path.join(self._snap_dir, 'snap', 'snapcraft.yaml')

        with contextlib.suppress(FileNotFoundError):
            os.remove(destination)

        os.makedirs(os.path.dirname(destination), exist_ok=True)
        file_utils.link_or_copy(source, destination)

    def _ensure_no_build_artifacts(self):
        # TODO: rename _snap_dir to _prime_dir
        snap_dir = os.path.join(self._snap_dir, 'snap')

        artifacts = ['snapcraft.yaml']

        for artifact in artifacts:
            artifact_path = os.path.join(snap_dir, artifact)
            if os.path.isfile(artifact_path):
                os.unlink(artifact_path)

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

        # Now copy the assets contained within the snap directory directly into
        # meta.
        for origin in ['gui', 'hooks']:
            src_dir = os.path.join('snap', origin)
            dst_dir = os.path.join(self.meta_dir, origin)
            if os.path.isdir(src_dir):
                os.makedirs(dst_dir, exist_ok=True)
                for asset in os.listdir(src_dir):
                    source = os.path.join(src_dir, asset)
                    destination = os.path.join(dst_dir, asset)

                    # First, verify that the hook is actually executable
                    if origin == 'hooks':
                        _validate_hook(source)

                    with contextlib.suppress(FileNotFoundError):
                        os.remove(destination)

                    file_utils.link_or_copy(source, destination)

        # FIXME hide this functionality behind a feature flag for now
        if os.environ.get('SNAPCRAFT_BUILD_INFO'):
            self._ensure_snapcraft_yaml()
        else:
            self._ensure_no_build_artifacts()

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

    def _setup_gui(self):
        # Handles the setup directory which only contains gui assets.
        setup_dir = 'setup'
        if not os.path.exists(setup_dir):
            return

        handle_deprecation_notice('dn2')

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
        classic_library_paths = self._config_data['confinement'] == 'classic'
        assembled_env = common.assemble_env(classic_library_paths,
                                            self._arch_triplet)
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
        desktop_file_name = app.pop('desktop', '')
        if desktop_file_name:
            desktop_file = _DesktopFile(
                name=name, filename=desktop_file_name,
                snap_name=self._config_data['name'], prime_dir=self._snap_dir)
            desktop_file.parse_and_reformat()
            desktop_file.write(gui_dir=os.path.join(self.meta_dir, 'gui'))


class _DesktopFile:

    def __init__(self, *, name, filename, snap_name, prime_dir):
        self._name = name
        self._filename = filename
        self._snap_name = snap_name
        self._prime_dir = prime_dir
        self._path = os.path.join(prime_dir, filename)
        if not os.path.exists(self._path):
            raise EnvironmentError(
                'The specified desktop file {!r} defined in the app '
                '{!r} does not exist'.format(filename, name))

    def parse_and_reformat(self):
        self._parser = configparser.ConfigParser(interpolation=None)
        self._parser.optionxform = str
        self._parser.read(self._path)
        section = 'Desktop Entry'
        if section not in self._parser.sections():
            raise EnvironmentError(
                'The specified desktop file {!r} is not a valid '
                'desktop file'.format(self._filename))
        if 'Exec' not in self._parser[section]:
            raise EnvironmentError(
                'The specified desktop file {!r} is missing the '
                '"Exec" key'.format(self._filename))
        # XXX: do we want to allow more parameters for Exec?
        if self._name == self._snap_name:
            exec_value = '{} %U'.format(self._name)
        else:
            exec_value = '{}.{} %U'.format(self._snap_name, self._name)
        self._parser[section]['Exec'] = exec_value
        if 'Icon' in self._parser[section]:
            icon = self._parser[section]['Icon']
            if icon.startswith('/'):
                icon = icon.lstrip('/')
                if os.path.exists(os.path.join(self._prime_dir, icon)):
                    self._parser[section]['Icon'] = '${{SNAP}}/{}'.format(icon)
                else:
                    logger.warning(
                        'Icon {} specified in desktop file {} not found '
                        'in prime directory'.format(icon, self._filename))

    def write(self, *, gui_dir):
        # Rename the desktop file to match the app name. This will help
        # unity8 associate them (https://launchpad.net/bugs/1659330).
        target_filename = '{}.desktop'.format(self._name)
        target = os.path.join(gui_dir, target_filename)
        if os.path.exists(target):
            # Unlikely. A desktop file in setup/gui/ already existed for
            # this app. Let's pretend it wasn't there and overwrite it.
            os.remove(target)
        with open(target, 'w') as f:
            self._parser.write(f, space_around_delimiters=False)


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


def _validate_hook(hook_path):
    if not os.stat(hook_path).st_mode & stat.S_IEXEC:
        asset = os.path.basename(hook_path)
        raise CommandError('hook {!r} is not executable'.format(asset))
