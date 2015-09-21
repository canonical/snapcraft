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

import glob
import itertools
import os
import string
import subprocess
import urllib
import urllib.request

import apt
from xml.etree import ElementTree

_DEFAULT_SOURCES = '''deb http://${mirror}archive.ubuntu.com/ubuntu/ vivid main restricted
deb http://${mirror}archive.ubuntu.com/ubuntu/ vivid-updates main restricted
deb http://${mirror}archive.ubuntu.com/ubuntu/ vivid universe
deb http://${mirror}archive.ubuntu.com/ubuntu/ vivid-updates universe
deb http://${mirror}archive.ubuntu.com/ubuntu/ vivid multiverse
deb http://${mirror}archive.ubuntu.com/ubuntu/ vivid-updates multiverse
deb http://security.ubuntu.com/ubuntu vivid-security main restricted
deb http://security.ubuntu.com/ubuntu vivid-security universe
deb http://security.ubuntu.com/ubuntu vivid-security multiverse
'''
_GEOIP_SERVER = "http://geoip.ubuntu.com/lookup"


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

    def __init__(self, rootdir, recommends=False, sources=_DEFAULT_SOURCES):
        self.downloaddir = os.path.join(rootdir, 'download')
        self.rootdir = rootdir
        self.apt_cache = _setup_apt_cache(rootdir, sources)
        self.recommends = recommends

    def get(self, package_names):
        os.makedirs(self.downloaddir, exist_ok=True)

        manifest_dep_names = self._manifest_dep_names()

        for name in package_names:
            try:
                self.apt_cache[name].mark_install()
            except KeyError:
                raise PackageNotFoundError(name)

        for pkg in self.apt_cache:
            # those should be already on each system, it also prevents
            # diving into downloading libc6
            if (pkg.candidate.priority in 'essential'
               and pkg.name not in package_names):
                print('Skipping priority essential/imporant %s' % pkg.name)
                continue
            if (pkg.name in manifest_dep_names and pkg.name not in package_names):
                print('Skipping blacklisted from manifest package %s' % pkg.name)
                continue
            if pkg.marked_install:
                pkg.candidate.fetch_binary(destdir=self.downloaddir)

    def unpack(self, rootdir):
        pkgs_abs_path = glob.glob(os.path.join(self.downloaddir, '*.deb'))
        for pkg in pkgs_abs_path:
            # TODO needs elegance and error control
            try:
                subprocess.check_call(['dpkg-deb', '--extract', pkg, rootdir])
            except subprocess.CalledProcessError:
                raise UnpackError(pkg)

        _fix_symlinks(rootdir)

    def _manifest_dep_names(self):
        manifest_dep_names = set()

        with open(os.path.abspath(os.path.join(__file__, '..', 'manifest.txt'))) as f:
            for line in f:
                pkg = line.strip()
                if pkg in self.apt_cache:
                    manifest_dep_names.add(pkg)

        return manifest_dep_names


def get_geoip_country_code_prefix():
    try:
        with urllib.request.urlopen(_GEOIP_SERVER) as f:
            xml_data = f.read()
        et = ElementTree.fromstring(xml_data)
        cc = et.find("CountryCode")
        if cc is None:
            return ""
        return cc.text.lower() + "."
    except (ElementTree.ParseError, urllib.error.URLError):
        pass
    return ""


def _setup_apt_cache(rootdir, sources):
    os.makedirs(os.path.join(rootdir, 'etc', 'apt'), exist_ok=True)
    srcfile = os.path.join(rootdir, 'etc', 'apt', 'sources.list')
    with open(srcfile, 'w') as f:
        mirror_prefix = get_geoip_country_code_prefix()
        sources_list = string.Template(sources).substitute(
            {"mirror": mirror_prefix})
        f.write(sources_list)
    progress = apt.progress.text.AcquireProgress()
    apt_cache = apt.Cache(rootdir=rootdir, memonly=True)
    apt_cache.update(fetch_progress=progress, sources_list=srcfile)
    apt_cache.open()

    return apt_cache


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
