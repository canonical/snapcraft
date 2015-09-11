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

import apt
import glob
import itertools
import os
import subprocess


class PackageNotFoundError(Exception):

    @property
    def message(self):
        return 'The Ubuntu package \'%s\' was not found' % self.package_name

    def __init__(self, package_name):
        self.package_name = package_name


class UnpackError(Exception):

    @property
    def message(self):
        return 'Error while provisioning \'%s\'' % self.package_name

    def __init__(self, package_name):
        self.package_name = package_name


class Ubuntu:

    def __init__(self, download_dir, recommends=False, sources=None):
        if not sources:
            self.apt_cache = apt.Cache()
        else:
            os.makedirs(os.path.join(download_dir, 'etc', 'apt'), exist_ok=True)
            srcfile = os.path.join(download_dir, 'etc', 'apt', 'sources.list')
            with open(srcfile, 'w') as f:
                f.write(sources)
            progress=apt.progress.text.AcquireProgress()
            self.apt_cache = apt.Cache(rootdir=download_dir, memonly=True)
            self.apt_cache.update(fetch_progress=progress, sources_list=srcfile)
            self.apt_cache.open()
        self.manifest_dep_names = self._manifest_dep_names()
        self.recommends = recommends
        self.download_dir = download_dir

    def get(self, package_names):
        for name in package_names:
            self.apt_cache[name].mark_install()
            
        for pkg in self.apt_cache:
            # those should be already on each system, it also prevents
            # diving into downloading libc6
            if pkg.candidate.priority in ("essential", "important"):
                print("Skipping priority essential/imporant %s" % pkg.name)
                continue
            if pkg.marked_install:
                pkg.candidate.fetch_binary(destdir=self.download_dir)

        return 

    def unpack(self, root_dir):
        pkgs_abs_path = glob.glob(os.path.join(self.download_dir, '*.deb'))
        for pkg in pkgs_abs_path:
            # TODO needs elegance and error control
            try:
                subprocess.check_call(['dpkg-deb', '--extract', pkg, root_dir])
            except subprocess.CalledProcessError:
                raise UnpackError(pkg)

        _fix_symlinks(root_dir)

    def _manifest_dep_names(self):
        manifest_dep_names = set()

        with open(os.path.abspath(os.path.join(__file__, '..', 'manifest.txt'))) as f:
            for line in f:
                pkg = line.strip()
                if pkg in self.apt_cache:
                    manifest_dep_names.add(pkg)

        return manifest_dep_names


def _fix_symlinks(debdir):
    '''
    Sometimes debs will contain absolute symlinks (e.g. if the relative
    path would go all the way to root, they just do absolute).  We can't
    have that, so instead clean those absolute symlinks.
    '''
    for root, dirs, files in os.walk(debdir):
        # Symlinks to directories will be in dirs, while symlinks to
        # non-directories will be in files.
        for entry in itertools.chain(files, dirs):
            path = os.path.join(root, entry)
            if os.path.islink(path) and os.path.isabs(os.readlink(path)):
                target = os.path.join(debdir, os.readlink(path)[1:])
                if os.path.exists(target):
                    os.remove(path)
                    os.symlink(os.path.relpath(target, root), path)
