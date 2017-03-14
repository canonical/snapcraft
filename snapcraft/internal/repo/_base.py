# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import fileinput
import itertools
import logging
import os
import re
import shutil
import stat

from snapcraft import file_utils

_BIN_PATHS = (
    'bin',
    'sbin',
    'usr/bin',
    'usr/sbin',
)


logger = logging.getLogger(__name__)


class BaseRepo:
    """Base implementation for a platform specific repo handler.

    Generally any new repo handling system would inherit from this and
    implement `get` and `unpack`. At the end of the `unpack` method
    `normalize` needs to be called to adapt the artifacts downloaded
    to be generic enough for building a snap."""

    @classmethod
    def get_package_libraries(cls, package_name):
        """Return a list of libraries in package_name.

        Given the contents of package_name, return the subset of what are
        considered libraries from those contents, be it static or shared.

        :param str package: package name to get library contents from.
        :returns: a list of libraries that package_name provides. This includes
                  directories.
        :rtype: set.
        """
        raise NotImplemented()

    def __init__(self, rootdir, *args, **kwargs):
        """Initialize a repository handler.

        :param str rootdir: the root directory to work on. This may well be to
                            create repo handling specific caches or download
                            artifacts.
        """
        self.rootdir = rootdir

    def get(self, package_names):
        """Get the packages.

        This method needs to be implemented by the inheriting class. It
        is called all along the core of snapcraft to obtain the
        package_names and store them in some repo defined location to
        later unpack.

        :param str package_names: list of packages to obtain.
        :returns: list of packages obtained, versioned if possible.
        :rtype: list of strings.
        :raises snapcraft.repo.errors.PackageNotFoundError:
            when a package in package_names is not found.
        """
        raise NotImplemented()

    def unpack(self, rootdir):
        """Unpack obtained packages into rootdir.

        This method needs to be implemented by the inheriting class. It
        is called all along the core of snapcraft to unpack the previously
        obtained package_names in get into rootdir.

        After the unpack logic is executed, normalize should be called.

        :param str rootdir: target directory to unpack packages to.
        """
        raise NotImplemented()

    def normalize(self):
        """Normalize artifacts.

        Repo specific packages are generally created to live in a specific
        distro. What normalize does is scan through the unpacked artifacts
        and slightly modifies them to work better with snapcraft projects
        when building and to also work within a snap's environment.
        """
        self._fix_artifacts()
        self._fix_xml_tools()
        self._fix_shebangs()

    def _fix_artifacts(self):
        """Perform various modifications to unpacked artifacts.

        Sometimes distro packages will contain absolute symlinks (e.g. if the
        relative path would go all the way to root, they just do absolute). We
        can't have that, so instead clean those absolute symlinks.

        Some unpacked items will also contain suid binaries which we do not
        want in the resulting snap.
        """
        for root, dirs, files in os.walk(self.rootdir):
            # Symlinks to directories will be in dirs, while symlinks to
            # non-directories will be in files.
            for entry in itertools.chain(files, dirs):
                path = os.path.join(root, entry)
                if os.path.islink(path) and os.path.isabs(os.readlink(path)):
                    self._fix_symlink(path, root)
                elif os.path.exists(path):
                    _fix_filemode(path)

                if path.endswith('.pc') and not os.path.islink(path):
                    fix_pkg_config(self.rootdir, path)

    def _fix_xml_tools(self):
        xml2_config_path = os.path.join(
            self.rootdir, 'usr', 'bin', 'xml2-config')
        with contextlib.suppress(FileNotFoundError):
            file_utils.search_and_replace_contents(
                xml2_config_path, re.compile(r'prefix=/usr'),
                'prefix={}/usr'.format(self.rootdir))

        xslt_config_path = os.path.join(
            self.rootdir, 'usr', 'bin', 'xslt-config')
        with contextlib.suppress(FileNotFoundError):
            file_utils.search_and_replace_contents(
                xslt_config_path, re.compile(r'prefix=/usr'),
                'prefix={}/usr'.format(self.rootdir))

    def _fix_symlink(self, path, root):
        host_target = os.readlink(path)
        if host_target in self.get_package_libraries('libc6'):
            logger.debug(
                "Not fixing symlink {!r}: it's pointing to libc".format(
                    host_target))
            return

        target = os.path.join(path, os.readlink(path)[1:])
        if (not os.path.exists(host_target) and not
                _try_copy_local(path, target)):
            return
        os.remove(path)
        os.symlink(os.path.relpath(target, root), path)

    def _fix_shebangs(self):
        """Changes hard coded shebangs for files in _BIN_PATHS to use env."""
        paths = [p for p in _BIN_PATHS
                 if os.path.exists(os.path.join(self.rootdir, p))]
        for p in [os.path.join(self.rootdir, p) for p in paths]:
            file_utils.replace_in_file(p, re.compile(r''),
                                       re.compile(r'#!.*python\n'),
                                       r'#!/usr/bin/env python\n')


def _try_copy_local(path, target):
    real_path = os.path.realpath(path)
    if os.path.exists(real_path):
        logger.warning(
            'Copying needed target link from the system {}'.format(real_path))
        os.makedirs(os.path.dirname(target), exist_ok=True)
        shutil.copyfile(os.readlink(path), target)
        return True
    else:
        logger.warning(
            '{} will be a dangling symlink'.format(path))
        return False


def fix_pkg_config(root, pkg_config_file, prefix_trim=None):
    """Opens a pkg_config_file and prefixes the prefix with root."""
    pattern_trim = None
    if prefix_trim:
        pattern_trim = re.compile(
            '^prefix={}(?P<prefix>.*)'.format(prefix_trim))
    pattern = re.compile('^prefix=(?P<prefix>.*)')

    with fileinput.input(pkg_config_file, inplace=True) as input_file:
        for line in input_file:
            match = pattern.search(line)
            if prefix_trim:
                match_trim = pattern_trim.search(line)
            if prefix_trim and match_trim:
                print('prefix={}{}'.format(root, match_trim.group('prefix')))
            elif match:
                print('prefix={}{}'.format(root, match.group('prefix')))
            else:
                print(line, end='')


def _fix_filemode(path):
    mode = stat.S_IMODE(os.stat(path, follow_symlinks=False).st_mode)
    if mode & 0o4000 or mode & 0o2000:
        logger.warning('Removing suid/guid from {}'.format(path))
        os.chmod(path, mode & 0o1777)
