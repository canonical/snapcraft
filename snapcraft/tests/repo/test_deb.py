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

import os
from subprocess import CalledProcessError
from unittest.mock import ANY, call, patch, MagicMock

from testtools.matchers import (
    Contains,
    FileExists,
)

import snapcraft
from snapcraft.internal import repo
from snapcraft.internal.repo import errors
from snapcraft import tests
from . import RepoBaseTestCase


class UbuntuTestCase(RepoBaseTestCase):

    def setUp(self):
        super().setUp()
        patcher = patch('snapcraft.repo._deb.apt.Cache')
        self.mock_cache = patcher.start()
        self.addCleanup(patcher.stop)

        def _fetch_binary(download_dir, **kwargs):
            path = os.path.join(download_dir, 'fake-package.deb')
            open(path, 'w').close()
            return path

        self.mock_package = MagicMock()
        self.mock_package.candidate.fetch_binary.side_effect = _fetch_binary
        self.mock_cache.return_value.get_changes.return_value = [
            self.mock_package]

    def test_get_pkg_name_parts_name_only(self):
        name, version = repo.get_pkg_name_parts('hello')
        self.assertEqual('hello', name)
        self.assertEqual(None, version)

    def test_get_pkg_name_parts_all(self):
        name, version = repo.get_pkg_name_parts('hello:i386=2.10-1')
        self.assertEqual('hello:i386', name)
        self.assertEqual('2.10-1', version)

    def test_get_pkg_name_parts_no_arch(self):
        name, version = repo.get_pkg_name_parts('hello=2.10-1')
        self.assertEqual('hello', name)
        self.assertEqual('2.10-1', version)

    @patch('snapcraft.internal.repo._deb.apt.apt_pkg')
    def test_get_package(self, mock_apt_pkg):
        project_options = snapcraft.ProjectOptions(
            use_geoip=False)
        ubuntu = repo.Ubuntu(self.tempdir, project_options=project_options)
        ubuntu.get(['fake-package'])

        mock_apt_pkg.assert_has_calls([
            call.config.set('Apt::Install-Recommends', 'False'),
            call.config.find_file('Dir::Etc::Trusted'),
            call.config.set('Dir::Etc::Trusted', ANY),
            call.config.find_file('Dir::Etc::TrustedParts'),
            call.config.set('Dir::Etc::TrustedParts', ANY),
            call.config.clear('APT::Update::Post-Invoke-Success'),
        ])

        self.mock_cache.assert_has_calls([
            call(memonly=True, rootdir=ANY),
            call().update(fetch_progress=ANY, sources_list=ANY),
            call().open(),
        ])

        # __getitem__ is tricky
        self.assertThat(
            self.mock_cache.return_value.__getitem__.call_args_list,
            Contains(call('fake-package')))

        self.mock_package.assert_has_calls([
            call.candidate.fetch_binary(ANY, progress=ANY)
        ])

        # Verify that the package was actually fetched and copied into the
        # requested location.
        self.assertThat(
            os.path.join(self.tempdir, 'download', 'fake-package.deb'),
            FileExists())

    @patch('snapcraft.repo._deb.apt.apt_pkg')
    def test_get_multiarch_package(self, mock_apt_pkg):
        project_options = snapcraft.ProjectOptions(
            use_geoip=False)
        ubuntu = repo.Ubuntu(self.tempdir, project_options=project_options)
        ubuntu.get(['fake-package:arch'])

        mock_apt_pkg.assert_has_calls([
            call.config.set('Apt::Install-Recommends', 'False'),
            call.config.find_file('Dir::Etc::Trusted'),
            call.config.set('Dir::Etc::Trusted', ANY),
            call.config.find_file('Dir::Etc::TrustedParts'),
            call.config.set('Dir::Etc::TrustedParts', ANY),
            call.config.clear('APT::Update::Post-Invoke-Success'),
        ])
        self.mock_cache.assert_has_calls([
            call(memonly=True, rootdir=ANY),
            call().update(fetch_progress=ANY, sources_list=ANY),
            call().open(),
        ])

        # __getitem__ is tricky
        self.assertThat(
            self.mock_cache.return_value.__getitem__.call_args_list,
            Contains(call('fake-package:arch')))

        self.mock_package.assert_has_calls([
            call.candidate.fetch_binary(ANY, progress=ANY)
        ])

        # Verify that the package was actually fetched and copied into the
        # requested location.
        self.assertThat(
            os.path.join(self.tempdir, 'download', 'fake-package.deb'),
            FileExists())

    @patch('snapcraft.repo._deb._get_geoip_country_code_prefix')
    def test_sources_is_none_uses_default(self, mock_cc):
        mock_cc.return_value = 'ar'

        self.maxDiff = None
        sources_list = repo._deb._format_sources_list(
            '', use_geoip=True, deb_arch='amd64')

        expected_sources_list = \
            '''deb http://ar.archive.ubuntu.com/ubuntu/ xenial main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ xenial universe
deb http://ar.archive.ubuntu.com/ubuntu/ xenial-updates universe
deb http://ar.archive.ubuntu.com/ubuntu/ xenial multiverse
deb http://ar.archive.ubuntu.com/ubuntu/ xenial-updates multiverse
deb http://security.ubuntu.com/ubuntu xenial-security main restricted
deb http://security.ubuntu.com/ubuntu xenial-security universe
deb http://security.ubuntu.com/ubuntu xenial-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)

    def test_no_geoip_uses_default_archive(self):
        sources_list = repo._deb._format_sources_list(
            repo._deb._DEFAULT_SOURCES, deb_arch='amd64', use_geoip=False)

        expected_sources_list = \
            '''deb http://archive.ubuntu.com/ubuntu/ xenial main restricted
deb http://archive.ubuntu.com/ubuntu/ xenial-updates main restricted
deb http://archive.ubuntu.com/ubuntu/ xenial universe
deb http://archive.ubuntu.com/ubuntu/ xenial-updates universe
deb http://archive.ubuntu.com/ubuntu/ xenial multiverse
deb http://archive.ubuntu.com/ubuntu/ xenial-updates multiverse
deb http://security.ubuntu.com/ubuntu xenial-security main restricted
deb http://security.ubuntu.com/ubuntu xenial-security universe
deb http://security.ubuntu.com/ubuntu xenial-security multiverse
'''

        self.assertEqual(sources_list, expected_sources_list)

    @patch('snapcraft.internal.repo._deb._get_geoip_country_code_prefix')
    def test_sources_amd64_vivid(self, mock_cc):
        self.maxDiff = None
        mock_cc.return_value = 'ar'

        sources_list = repo._deb._format_sources_list(
            repo._deb._DEFAULT_SOURCES, deb_arch='amd64',
            use_geoip=True, release='vivid')

        expected_sources_list = \
            '''deb http://ar.archive.ubuntu.com/ubuntu/ vivid main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ vivid universe
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates universe
deb http://ar.archive.ubuntu.com/ubuntu/ vivid multiverse
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates multiverse
deb http://security.ubuntu.com/ubuntu vivid-security main restricted
deb http://security.ubuntu.com/ubuntu vivid-security universe
deb http://security.ubuntu.com/ubuntu vivid-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)

    @patch('snapcraft.repo._deb._get_geoip_country_code_prefix')
    def test_sources_armhf_trusty(self, mock_cc):
        sources_list = repo._deb._format_sources_list(
            repo._deb._DEFAULT_SOURCES, deb_arch='armhf', release='trusty')

        expected_sources_list = \
            '''deb http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ trusty universe
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates universe
deb http://ports.ubuntu.com/ubuntu-ports/ trusty multiverse
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates multiverse
deb http://ports.ubuntu.com/ubuntu-ports trusty-security main restricted
deb http://ports.ubuntu.com/ubuntu-ports trusty-security universe
deb http://ports.ubuntu.com/ubuntu-ports trusty-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)
        self.assertFalse(mock_cc.called)


class BuildPackagesTestCase(tests.TestCase):

    test_packages = {'package-not-installed': MagicMock(installed=False),
                     'package-installed': MagicMock(installed=True),
                     'another-uninstalled': MagicMock(installed=False),
                     'another-installed': MagicMock(installed=True),
                     'repeated-package': MagicMock(installed=False),
                     'repeated-package': MagicMock(installed=False),
                     'versioned-package=0.2': MagicMock(installed=False),
                     'versioned-package': MagicMock(installed=True,
                                                    version='0.1')}

    def get_installable_packages(self, pkgs):
        return [p for p in pkgs if not pkgs[p].installed]

    @patch('os.environ')
    @patch('snapcraft.repo._deb.apt')
    def install_test_packages(self, test_pkgs, mock_apt, mock_env):
        mock_env.copy.return_value = {}
        mock_apt_cache = mock_apt.Cache.return_value
        mock_apt_cache_with = mock_apt_cache.__enter__.return_value
        mock_apt_cache_with.__getitem__.side_effect = lambda p: test_pkgs[p]

        repo.Ubuntu.install_build_packages(test_pkgs.keys())

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    @patch('subprocess.check_call')
    def test_install_build_package(
            self, mock_check_call, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = False
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        mock_check_call.assert_has_calls([
            call('sudo apt-get --no-install-recommends -y '
                 '-o Dpkg::Progress-Fancy=1 install'.split() +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    @patch('subprocess.check_call')
    def test_install_buid_package_in_dumb_terminal(
            self, mock_check_call, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = True
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        mock_check_call.assert_has_calls([
            call('sudo apt-get --no-install-recommends -y install'.split() +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    @patch('subprocess.check_call')
    def test_install_buid_package_marks_auto_installed(self, mock_check_call):
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        mock_check_call.assert_has_calls([
            call('sudo apt-mark auto'.split() +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    @patch('subprocess.check_call')
    def test_mark_installed_auto_error_is_not_fatal(self, mock_check_call):
        error = CalledProcessError(101, 'bad-cmd')
        mock_check_call.side_effect = \
            lambda c, env: error if 'apt-mark' in c else None
        self.install_test_packages(self.test_packages)

    def test_invalid_package_requested(self):
        raised = self.assertRaises(
            errors.BuildPackageNotFoundError,
            repo.Ubuntu.install_build_packages,
            ['package-does-not-exist'])

        self.assertEqual(
            "Could not find a required package in 'build-packages': "
            '"The cache has no package named \'package-does-not-exist\'"',
            str(raised))
