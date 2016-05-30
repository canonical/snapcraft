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


SNAPCRAFT_FILES = ['snapcraft.yaml', 'parts', 'stage', 'snap']
COMMAND_ORDER = ['pull', 'build', 'stage', 'strip']
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
        "This plugin is outdated, use 'project.parallel_build_count'")


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


def isurl(url):
    return urllib.parse.urlparse(url).scheme != ''


def reset_env():
    global env
    env = []


def link_or_copy(source, destination, follow_symlinks=False):
    # Hard-link this file to the source. It's possible for this to
    # fail (e.g. a cross-device link, or permission denied), so as a backup
    # plan we'll just copy it.
    try:
        # Note that follow_symlinks doesn't seem to work for os.link, so we'll
        # implement this logic ourselves using realpath.
        source_path = source
        if follow_symlinks:
            source_path = os.path.realpath(source)

        # Setting follow_symlinks=False in case this bug is ever fixed
        # upstream-- we want this function to continue supporting NOT following
        # symlinks.
        os.link(source_path, destination, follow_symlinks=False)
    except OSError:
        shutil.copy2(source, destination, follow_symlinks=follow_symlinks)


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
