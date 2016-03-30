# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

# Data/methods shared between plugins and snapcraft

import glob
import logging
import multiprocessing
import os
import platform
import subprocess
import sys
import tempfile
import urllib


SNAPCRAFT_FILES = ['snapcraft.yaml', 'parts', 'stage', 'snap']
COMMAND_ORDER = ['pull', 'build', 'stage', 'strip']
_DEFAULT_ENABLE_PARALLEL_BUILDS = True
_enable_parallel_builds = _DEFAULT_ENABLE_PARALLEL_BUILDS
_DEFAULT_PLUGINDIR = '/usr/share/snapcraft/plugins'
_plugindir = _DEFAULT_PLUGINDIR
_DEFAULT_SCHEMADIR = '/usr/share/snapcraft/schema'
_schemadir = _DEFAULT_SCHEMADIR
_DEFAULT_LIBRARIESDIR = 'usr/share/snapcraft/libraries'
_librariesdir = _DEFAULT_LIBRARIESDIR

host_machine = platform.machine()
target_machine = host_machine

env = []

logger = logging.getLogger(__name__)


def assemble_env():
    return '\n'.join(['export ' + e for e in env])


def run(cmd, **kwargs):
    assert isinstance(cmd, list), 'run command must be a list'
    # FIXME: This is gross to keep writing this, even when env is the same
    with tempfile.NamedTemporaryFile(mode='w+') as f:
        f.write(assemble_env())
        f.write('\n')
        f.write('exec $*')
        f.flush()
        subprocess.check_call(['/bin/sh', f.name] + cmd, **kwargs)


def run_output(cmd, **kwargs):
    assert isinstance(cmd, list), 'run command must be a list'
    # FIXME: This is gross to keep writing this, even when env is the same
    with tempfile.NamedTemporaryFile(mode='w+') as f:
        f.write(assemble_env())
        f.write('\n')
        f.write('exec $*')
        f.flush()
        output = subprocess.check_output(['/bin/sh', f.name] + cmd, **kwargs)
        try:
            return output.decode(sys.getfilesystemencoding()).strip()
        except UnicodeEncodeError:
            logger.warning('Could not decode output for {!r} correctly'.format(
                cmd))
            return output.decode('latin-1', 'surrogateescape').strip()


_ARCH_TRANSLATIONS = {
    'armv7l': {
        'kernel': 'arm',
        'deb': 'armhf',
        'cross-compiler-prefix': 'arm-linux-gnueabihf-',
        'cross-build-packages': ['gcc-arm-linux-gnueabihf'],
        'triplet': 'arm-linux-gnueabihf',
    },
    'aarch64': {
        'kernel': 'arm64',
        'deb': 'arm64',
        'cross-compiler-prefix': 'aarch64-linux-gnu-',
        'cross-build-packages': ['gcc-aarch64-linux-gnu'],
        'triplet': 'aarch64-linux-gnu',
    },
    'i686': {
        'kernel': 'x86',
        'deb': 'i386',
        'triplet': 'i386-linux-gnu',
    },
    'ppc64le': {
        'kernel': 'powerpc',
        'deb': 'ppc64el',
        'cross-compiler-prefix': 'gcc-powerpc64-linux-gnu',
        'cross-build-packages': ['gcc-powerpc64-linux-gnu'],
        'triplet': 'powerpc64le-linux-gnu',
    },
    'x86_64': {
        'kernel': 'x86',
        'deb': 'amd64',
        'triplet': 'x86_64-linux-gnu',
    }
}


class PlatformError(Exception):

    def __init__(self):
        super().__init__(
            '{0} is not supported, please log a bug at '
            'https://bugs.launchpad.net/snapcraft/+filebug?'
            'field.title=please+add+support+for+{0}'.format(target_machine))


def get_machine_info(machine):
    try:
        return _ARCH_TRANSLATIONS[machine]
    except KeyError:
        raise PlatformError()


def set_target_machine(deb_arch):
    global target_machine
    for machine in _ARCH_TRANSLATIONS:
        if _ARCH_TRANSLATIONS[machine].get('deb', '') == deb_arch:
            logger.info('Setting target machine to {!r}'.format(machine))
            target_machine = machine
            return

    raise EnvironmentError(
        'Cannot set machine from deb_arch {!r}'.format(deb_arch))


def get_arch():
    try:
        return _ARCH_TRANSLATIONS[target_machine]['deb']
    except KeyError:
        raise PlatformError()


def get_arch_triplet():
    try:
        return _ARCH_TRANSLATIONS[target_machine]['triplet']
    except KeyError:
        raise PlatformError()


def format_snap_name(snap):
    snap['arch'] = (snap['architectures'][0]
                    if len(snap['architectures']) == 1 else 'multi')
    return '{name}_{version}_{arch}.snap'.format(**snap)


def get_partsdir():
    return os.path.join(os.getcwd(), 'parts')


def get_stagedir():
    return os.path.join(os.getcwd(), 'stage')


def get_snapdir():
    return os.path.join(os.getcwd(), 'snap')


def get_local_plugindir():
    return os.path.abspath(os.path.join(get_partsdir(), 'plugins'))


def set_plugindir(plugindir):
    global _plugindir
    _plugindir = plugindir


def get_plugindir():
    return _plugindir


def set_schemadir(schemadir):
    global _schemadir
    _schemadir = schemadir


def get_schemadir():
    return _schemadir


def set_enable_parallel_builds(enable):
    global _enable_parallel_builds
    _enable_parallel_builds = enable


def get_enable_parallel_builds():
    return _enable_parallel_builds


def get_parallel_build_count():
    build_count = 1
    if get_enable_parallel_builds():
        try:
            build_count = multiprocessing.cpu_count()
        except NotImplementedError:
            logger.warning('Unable to determine CPU count; disabling parallel '
                           'builds')

    return build_count


def set_librariesdir(librariesdir):
    global _librariesdir
    _librariesdir = librariesdir


def get_librariesdir():
    return _librariesdir


def get_python2_path(root):
    """Return a valid PYTHONPATH or raise an exception."""
    python_paths = glob.glob(os.path.join(
        root, 'usr', 'lib', 'python2*', 'dist-packages'))
    try:
        return python_paths[0]
    except IndexError:
        raise EnvironmentError(
            'PYTHONPATH cannot be set for {!r}'.format(root))


def isurl(url):
    return urllib.parse.urlparse(url).scheme != ''


def reset_env():
    global env
    env = []


def replace_in_file(directory, file_pattern, search_pattern, replacement):
    """Searches and replaces patterns that match a file pattern.
    :param str directory: The directory to look for files.
    :param str file_pattern: The file pattern to match inside directory.
    :param search_pattern: A re.compile'd pattern to search for within
                           matching files.
    :param str replacement: The string to replace the matching search_pattern
                            with.
    """
    for root, directories, files in os.walk(directory):
        for file_name in files:
            if file_pattern.match(file_name):
                _search_and_replace_contents(os.path.join(root, file_name),
                                             search_pattern, replacement)


def _search_and_replace_contents(file_path, search_pattern, replacement):
    with open(file_path, 'r+') as f:
        try:
            original = f.read()
        except UnicodeDecodeError:
            # This was probably a binary file. Skip it.
            return

        replaced = search_pattern.sub(replacement, original)
        if replaced != original:
            f.seek(0)
            f.truncate()
            f.write(replaced)
