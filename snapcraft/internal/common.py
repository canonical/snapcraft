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

from contextlib import suppress
import glob
import logging
import math
import os
import shutil
import subprocess
import sys
import tempfile
import urllib


SNAPCRAFT_FILES = ['snapcraft.yaml', '.snapcraft.yaml', 'parts', 'stage',
                   'prime', 'snap']
COMMAND_ORDER = ['pull', 'build', 'stage', 'prime']
_DEFAULT_PLUGINDIR = '/usr/share/snapcraft/plugins'
_plugindir = _DEFAULT_PLUGINDIR
_DEFAULT_SCHEMADIR = '/usr/share/snapcraft/schema'
_schemadir = _DEFAULT_SCHEMADIR
_DEFAULT_LIBRARIESDIR = '/usr/share/snapcraft/libraries'
_librariesdir = _DEFAULT_LIBRARIESDIR
_DEFAULT_TOURDIR = '/usr/share/snapcraft/tour'
_tourdir = _DEFAULT_TOURDIR

MAX_CHARACTERS_WRAP = 120

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
        f.write('exec "$@"')
        f.flush()
        subprocess.check_call(['/bin/sh', f.name] + cmd, **kwargs)


def run_output(cmd, **kwargs):
    assert isinstance(cmd, list), 'run command must be a list'
    # FIXME: This is gross to keep writing this, even when env is the same
    with tempfile.NamedTemporaryFile(mode='w+') as f:
        f.write(assemble_env())
        f.write('\n')
        f.write('exec "$@"')
        f.flush()
        output = subprocess.check_output(['/bin/sh', f.name] + cmd, **kwargs)
        try:
            return output.decode(sys.getfilesystemencoding()).strip()
        except UnicodeEncodeError:
            logger.warning('Could not decode output for {!r} correctly'.format(
                cmd))
            return output.decode('latin-1', 'surrogateescape').strip()


def format_snap_name(snap):
    if 'arch' not in snap:
        snap['arch'] = snap.get('architectures', None)
    if not snap['arch']:
        snap['arch'] = 'all'
    elif len(snap['arch']) == 1:
        snap['arch'] = snap['arch'][0]
    else:
        snap['arch'] = 'multi'

    return '{name}_{version}_{arch}.snap'.format(**snap)


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


def get_arch_triplet():
    raise EnvironmentError(
        "This plugin is outdated, use 'project.arch_triplet'")


def get_arch():
    raise EnvironmentError(
        "This plugin is outdated, use 'project.deb_arch'")


def get_parallel_build_count():
    raise EnvironmentError(
        "This plugin is outdated, use 'parallel_build_count'")


def set_librariesdir(librariesdir):
    global _librariesdir
    _librariesdir = librariesdir


def get_librariesdir():
    return _librariesdir


def set_tourdir(tourdir):
    global _tourdir
    _tourdir = tourdir


def get_tourdir():
    return _tourdir


def get_python2_path(root):
    """Return a valid PYTHONPATH or raise an exception."""
    python_paths = glob.glob(os.path.join(
        root, 'usr', 'lib', 'python2*', 'dist-packages'))
    try:
        return python_paths[0]
    except IndexError:
        raise EnvironmentError(
            'PYTHONPATH cannot be set for {!r}'.format(root))


def get_url_scheme(url):
    return urllib.parse.urlparse(url).scheme


def isurl(url):
    return get_url_scheme(url) != ''


def reset_env():
    global env
    env = []


def get_terminal_width(max_width=MAX_CHARACTERS_WRAP):
    if os.isatty(sys.stdout.fileno()):
        width = shutil.get_terminal_size().columns
    else:
        width = MAX_CHARACTERS_WRAP
    if max_width:
        width = min(max_width, width)
    return width


def format_output_in_columns(elements_list, max_width=MAX_CHARACTERS_WRAP,
                             num_col_spaces=2):
    """Return a formatted list of strings ready to be printed line by line

    elements_list is the list of elements ready to be printed on the output
    max_width is the number of caracters the output shouldn't exceed
    num_col_spaces is the number of spaces set between 2 columns"""

    # First, try to get the starting point in term of number of lines
    total_num_chars = sum([len(elem) for elem in elements_list])
    num_lines = math.ceil((total_num_chars +
                           (len(elements_list) - 1) * num_col_spaces) /
                          max_width)
    sep = ' '*num_col_spaces

    candidate_output = []
    while not candidate_output:
        # dispatch elements in resulting list until num_lines
        for i, element in enumerate(elements_list):
            # for new columns, get the maximum width of this column
            if i % num_lines == 0:
                col_width = 0
                for j in range(i, i+num_lines):
                    # ignore non existing elements at the end
                    with suppress(IndexError):
                        col_width = max(len(elements_list[j]), col_width)

            if i < num_lines:
                candidate_output.append([])
            candidate_output[i % num_lines].append(element.ljust(col_width))

        # check that any line (like the first one) is still smaller than
        # max_width
        if len(sep.join(candidate_output[0])) > max_width:
            # reset and try with one more line
            num_lines += 1
            candidate_output = []

    result_output = []
    for i, line in enumerate(candidate_output):
        result_output.append(sep.join(candidate_output[i]))

    return result_output


def get_include_paths(root, arch_triplet):
    paths = [
        os.path.join(root, 'include'),
        os.path.join(root, 'usr', 'include'),
        os.path.join(root, 'include', arch_triplet),
        os.path.join(root, 'usr', 'include', arch_triplet),
    ]

    return [p for p in paths if os.path.exists(p)]


def get_library_paths(root, arch_triplet):
    paths = [
        os.path.join(root, 'lib'),
        os.path.join(root, 'usr', 'lib'),
        os.path.join(root, 'lib', arch_triplet),
        os.path.join(root, 'usr', 'lib', arch_triplet),
    ]

    return [p for p in paths if os.path.exists(p)]


def get_pkg_config_paths(root, arch_triplet):
    paths = [
        os.path.join(root, 'lib', 'pkgconfig'),
        os.path.join(root, 'lib', arch_triplet, 'pkgconfig'),
        os.path.join(root, 'usr', 'lib', 'pkgconfig'),
        os.path.join(root, 'usr', 'lib', arch_triplet, 'pkgconfig'),
        os.path.join(root, 'usr', 'share', 'pkgconfig'),
        os.path.join(root, 'usr', 'local', 'lib', 'pkgconfig'),
        os.path.join(root, 'usr', 'local', 'lib', arch_triplet, 'pkgconfig'),
        os.path.join(root, 'usr', 'local', 'share', 'pkgconfig'),
    ]

    return [p for p in paths if os.path.exists(p)]
