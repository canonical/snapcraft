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

import fileinput
import glob
import itertools
import logging
import os
import platform
import re
import shutil
import stat
import string
import subprocess
import urllib
import urllib.request

import apt
from xml.etree import ElementTree

import snapcraft
from snapcraft.internal import common


_BIN_PATHS = (
    'bin',
    'sbin',
    'usr/bin',
    'usr/sbin',
)

logger = logging.getLogger(__name__)

_DEFAULT_SOURCES = \
    '''deb http://${prefix}.ubuntu.com/${suffix}/ ${release} main restricted
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates main restricted
deb http://${prefix}.ubuntu.com/${suffix}/ ${release} universe
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates universe
deb http://${prefix}.ubuntu.com/${suffix}/ ${release} multiverse
deb http://${prefix}.ubuntu.com/${suffix}/ ${release}-updates multiverse
deb http://${security}.ubuntu.com/${suffix} ${release}-security main restricted
deb http://${security}.ubuntu.com/${suffix} ${release}-security universe
deb http://${security}.ubuntu.com/${suffix} ${release}-security multiverse
'''
_GEOIP_SERVER = "http://geoip.ubuntu.com/lookup"


def is_package_installed(package):
    """Return True if a package is installed on the system.

    :param str package: the deb package to query for.
    :returns: True if the package is installed, False if not.
    """
    with apt.Cache() as apt_cache:
        return apt_cache[package].installed


def install_build_packages(packages):
    unique_packages = set(packages)
    new_packages = []
    with apt.Cache() as apt_cache:
        for pkg in unique_packages:
            try:
                if not apt_cache[pkg].installed:
                    new_packages.append(pkg)
            except KeyError as e:
                raise EnvironmentError(
                    "Could not find a required package in "
                    "'build-packages': {}".format(str(e)))
    if new_packages:
        logger.info(
            'Installing build dependencies: %s', ' '.join(new_packages))
        env = os.environ.copy()
        env.update({
            'DEBIAN_FRONTEND': 'noninteractive',
            'DEBCONF_NONINTERACTIVE_SEEN': 'true',
        })
        subprocess.check_call(['sudo', 'apt-get', '-o',
                               'Dpkg::Progress-Fancy=1',
                               '--no-install-recommends', '-y',
                               'install'] + new_packages, env=env)


class PackageNotFoundError(Exception):

    @property
    def message(self):
        return 'The Ubuntu package "{}" was not found'.format(
            self.package_name)

    def __init__(self, package_name):
        self.package_name = package_name


class UnpackError(Exception):

    @property
    def message(self):
        return 'Error while provisioning "{}"'.format(self.package_name)

    def __init__(self, package_name):
        self.package_name = package_name


class Ubuntu:

    def __init__(self, rootdir, recommends=False,
                 sources=None, project_options=None):
        self.downloaddir = os.path.join(rootdir, 'download')
        self.rootdir = rootdir
        self.recommends = recommends

        if not project_options:
            project_options = snapcraft.ProjectOptions()
        self.apt_cache, self.apt_progress = _setup_apt_cache(
            rootdir, sources, project_options)

    def get(self, package_names):
        # Create the 'partial' subdir too (LP: #1578007).
        os.makedirs(os.path.join(self.downloaddir, 'partial'), exist_ok=True)

        manifest_dep_names = self._manifest_dep_names()

        for name in package_names:
            try:
                self.apt_cache[name].mark_install()
            except KeyError:
                raise PackageNotFoundError(name)

        skipped_essential = []
        skipped_blacklisted = []

        # unmark some base packages here
        # note that this will break the consistency check inside apt_cache
        # (self.apt_cache.broken_count will be > 0)
        # but that is ok as it was consistent before we excluded
        # these base package
        for pkg in self.apt_cache:
            # those should be already on each system, it also prevents
            # diving into downloading libc6
            if (pkg.candidate.priority in 'essential' and
               pkg.name not in package_names):
                skipped_essential.append(pkg.name)
                pkg.mark_keep()
                continue
            if (pkg.name in manifest_dep_names and
                    pkg.name not in package_names):
                skipped_blacklisted.append(pkg.name)
                pkg.mark_keep()
                continue

        if skipped_essential:
            print('Skipping priority essential packages:', skipped_essential)
        if skipped_blacklisted:
            print('Skipping blacklisted from manifest packages:',
                  skipped_blacklisted)

        # download the remaining ones with proper progress
        apt.apt_pkg.config.set("Dir::Cache::Archives", self.downloaddir)
        self.apt_cache.fetch_archives(progress=self.apt_progress)

    def unpack(self, rootdir):
        pkgs_abs_path = glob.glob(os.path.join(self.downloaddir, '*.deb'))
        for pkg in pkgs_abs_path:
            # TODO needs elegance and error control
            try:
                subprocess.check_call(['dpkg-deb', '--extract', pkg, rootdir])
            except subprocess.CalledProcessError:
                raise UnpackError(pkg)

        _fix_artifacts(rootdir)
        _fix_xml_tools(rootdir)
        _fix_shebangs(rootdir)

    def _manifest_dep_names(self):
        manifest_dep_names = set()

        with open(os.path.abspath(os.path.join(__file__, '..',
                                               'manifest.txt'))) as f:
            for line in f:
                pkg = line.strip()
                if pkg in self.apt_cache:
                    manifest_dep_names.add(pkg)

        return manifest_dep_names


def _get_local_sources_list():
    sources_list = glob.glob('/etc/apt/sources.list.d/*.list')
    sources_list.append('/etc/apt/sources.list')

    sources = ''
    for source in sources_list:
        with open(source) as f:
            sources += f.read()

    return sources


def _get_geoip_country_code_prefix():
    try:
        with urllib.request.urlopen(_GEOIP_SERVER) as f:
            xml_data = f.read()
        et = ElementTree.fromstring(xml_data)
        cc = et.find("CountryCode")
        if cc is None:
            return ""
        return cc.text.lower()
    except (ElementTree.ParseError, urllib.error.URLError):
        pass
    return ''


def _format_sources_list(sources, project_options, release='xenial'):
    if not sources:
        sources = _DEFAULT_SOURCES

    if project_options.deb_arch in ('amd64', 'i386'):
        if project_options.use_geoip:
            geoip_prefix = _get_geoip_country_code_prefix()
            prefix = '{}.archive'.format(geoip_prefix)
        else:
            prefix = 'archive'
        suffix = 'ubuntu'
        security = 'security'
    else:
        prefix = 'ports'
        suffix = 'ubuntu-ports'
        security = 'ports'

    return string.Template(sources).substitute({
        'prefix': prefix,
        'release': release,
        'suffix': suffix,
        'security': security,
    })


def _setup_apt_cache(rootdir, sources, project_options):
    os.makedirs(os.path.join(rootdir, 'etc', 'apt'), exist_ok=True)
    srcfile = os.path.join(rootdir, 'etc', 'apt', 'sources.list')

    if project_options.use_geoip or sources:
        release = platform.linux_distribution()[2]
        sources = _format_sources_list(
            sources, project_options, release)
    else:
        sources = _get_local_sources_list()

    with open(srcfile, 'w') as f:
        f.write(sources)

    # Do not install recommends
    apt.apt_pkg.config.set('Apt::Install-Recommends', 'False')

    # Make sure we always use the system GPG configuration, even with
    # apt.Cache(rootdir).
    for key in 'Dir::Etc::Trusted', 'Dir::Etc::TrustedParts':
        apt.apt_pkg.config.set(key, apt.apt_pkg.config.find_file(key))

    progress = apt.progress.text.AcquireProgress()
    if not os.isatty(1):
        # Make output more suitable for logging.
        progress.pulse = lambda owner: True
        progress._width = 0

    apt_cache = apt.Cache(rootdir=rootdir, memonly=True)
    apt.apt_pkg.config.clear("APT::Update::Post-Invoke-Success")
    apt_cache.update(fetch_progress=progress, sources_list=srcfile)
    apt_cache.open()

    return apt_cache, progress


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


def _fix_artifacts(debdir):
    '''
    Sometimes debs will contain absolute symlinks (e.g. if the relative
    path would go all the way to root, they just do absolute).  We can't
    have that, so instead clean those absolute symlinks.

    Some unpacked items will also contain suid binaries which we do not want in
    the resulting snap.
    '''
    for root, dirs, files in os.walk(debdir):
        # Symlinks to directories will be in dirs, while symlinks to
        # non-directories will be in files.
        for entry in itertools.chain(files, dirs):
            path = os.path.join(root, entry)
            if os.path.islink(path) and os.path.isabs(os.readlink(path)):
                _fix_symlink(path, debdir, root)
            elif os.path.exists(path):
                _fix_filemode(path)

            if path.endswith('.pc') and not os.path.islink(path):
                fix_pkg_config(debdir, path)


def _fix_xml_tools(root):
    xml2_config_path = os.path.join(root, 'usr', 'bin', 'xml2-config')
    if os.path.isfile(xml2_config_path):
        common.run(
            ['sed', '-i', '-e', 's|prefix=/usr|prefix={}/usr|'.
                format(root), xml2_config_path])

    xslt_config_path = os.path.join(root, 'usr', 'bin', 'xslt-config')
    if os.path.isfile(xslt_config_path):
        common.run(
            ['sed', '-i', '-e', 's|prefix=/usr|prefix={}/usr|'.
                format(root), xslt_config_path])


def _fix_symlink(path, debdir, root):
    target = os.path.join(debdir, os.readlink(path)[1:])
    if _skip_link(os.readlink(path)):
        logger.debug('Skipping {}'.format(target))
        return
    if not os.path.exists(target):
        if not _try_copy_local(path, target):
            return
    os.remove(path)
    os.symlink(os.path.relpath(target, root), path)


def _fix_filemode(path):
    mode = stat.S_IMODE(os.stat(path, follow_symlinks=False).st_mode)
    if mode & 0o4000 or mode & 0o2000:
        logger.warning('Removing suid/guid from {}'.format(path))
        os.chmod(path, mode & 0o1777)


def _fix_shebangs(path):
    """Changes hard coded shebangs for files in _BIN_PATHS to use env."""
    paths = [p for p in _BIN_PATHS if os.path.exists(os.path.join(path, p))]
    for p in [os.path.join(path, p) for p in paths]:
        common.replace_in_file(p, re.compile(r''),
                               re.compile(r'#!.*python\n'),
                               r'#!/usr/bin/env python\n')


_skip_list = None


def _skip_link(target):
    global _skip_list
    if not _skip_list:
        output = common.run_output(['dpkg', '-L', 'libc6']).split()
        _skip_list = [i for i in output if 'lib' in i]

    return target in _skip_list


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
