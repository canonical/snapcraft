# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
    Equals,
    FileExists,
)

import snapcraft
from snapcraft.internal import repo
from snapcraft import tests
from snapcraft.tests import fixture_setup
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
        self.assertThat(name, Equals('hello'))
        self.assertThat(version, Equals(None))

    def test_get_pkg_name_parts_all(self):
        name, version = repo.get_pkg_name_parts('hello:i386=2.10-1')
        self.assertThat(name, Equals('hello:i386'))
        self.assertThat(version, Equals('2.10-1'))

    def test_get_pkg_name_parts_no_arch(self):
        name, version = repo.get_pkg_name_parts('hello=2.10-1')
        self.assertThat(name, Equals('hello'))
        self.assertThat(version, Equals('2.10-1'))

    @patch('snapcraft.internal.repo._deb.apt.apt_pkg')
    def test_get_package(self, mock_apt_pkg):
        self.mock_cache().is_virtual_package.return_value = False

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
        self.mock_cache().is_virtual_package.return_value = False

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
        self.assertThat(sources_list, Equals(expected_sources_list))

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

        self.assertThat(sources_list, Equals(expected_sources_list))

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
        self.assertThat(sources_list, Equals(expected_sources_list))

    def test_custom_sources_invalid_placeholder(self):
        custom_sources = \
            '''deb http://mydomain.com/ubuntu/ $release $foo restricted
''' # noqa
        self.assertIn('unknown placeholder $foo in template',
                      str(self.assertRaises(
                          ValueError,
                          repo._deb._format_sources_list,
                          custom_sources, deb_arch='amd64')))

    def test_custom_sources_invalid_placeholder_braces(self):
        custom_sources = \
            '''deb http://mydomain.com/ubuntu/ ${release} ${foo} restricted
''' # noqa
        self.assertIn('unknown placeholder ${foo} in template',
                      str(self.assertRaises(
                          ValueError,
                          repo._deb._format_sources_list,
                          custom_sources, deb_arch='amd64')))

    def test_custom_sources_missing_placeholders(self):
        custom_sources = \
            '''deb http://mydomain.com/ubuntu/ ${release} main restricted
''' # noqa
        sources_list = repo._deb._format_sources_list(
            custom_sources, deb_arch='amd64')
        expected_sources_list = \
                '''deb http://mydomain.com/ubuntu/ xenial main restricted
''' # noqa
        self.assertThat(sources_list, Equals(expected_sources_list))

    def test_sources_armhf_foreign(self):
        sources_list = repo._deb._format_sources_list(
            None, deb_arch='armhf', foreign=True)

        expected_sources_list = \
            '''deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ xenial main restricted
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ xenial-updates main restricted
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ xenial universe
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ xenial-updates universe
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ xenial multiverse
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports/ xenial-updates multiverse
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports xenial-security main restricted
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports xenial-security universe
deb [arch=armhf] http://ports.ubuntu.com/ubuntu-ports xenial-security multiverse
''' # noqa
        self.assertThat(sources_list, Equals(expected_sources_list))

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
        self.assertThat(sources_list, Equals(expected_sources_list))
        self.assertFalse(mock_cc.called)


class BuildPackagesTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_apt_cache = fixture_setup.FakeAptCache()
        self.useFixture(self.fake_apt_cache)
        self.test_packages = (
            'package-not-installed', 'package-installed',
            'another-uninstalled', 'another-installed', 'repeated-package',
            'repeated-package', 'versioned-package=0.2', 'versioned-package')
        self.fake_apt_cache.add_packages(self.test_packages)
        self.fake_apt_cache.cache['package-installed'].installed = True
        self.fake_apt_cache.cache['another-installed'].installed = True
        self.fake_apt_cache.cache['versioned-package'].version = '0.1'

    def get_installable_packages(self, packages):
        return ['package-not-installed', 'another-uninstalled',
                'repeated-package', 'versioned-package=0.2']

    @patch('os.environ')
    def install_test_packages(self, test_pkgs, mock_env):
        mock_env.copy.return_value = {}

        repo.Ubuntu.install_build_packages(test_pkgs)

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_build_package(
            self, mock_is_dumb_terminal):
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(
            self.test_packages)
        self.useFixture(fake_apt)
        mock_is_dumb_terminal.return_value = False
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        fake_apt.check_call_mock.assert_has_calls([
            call('sudo apt-get --no-install-recommends -y '
                 '-o Dpkg::Progress-Fancy=1 install'.split() +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_build_package_for_arch(
            self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = False
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(
            self.test_packages)
        self.useFixture(fake_apt)
        self.fake_apt_cache.add_packages(
            ('some-package:amd64', 'some-package:armhf'))
        self.install_test_packages(
            ('package-installed', 'some-package:armhf'))

        installable = (('some-package:armhf', ))
        fake_apt.check_call_mock.assert_has_calls([
            call('sudo apt-get --no-install-recommends -y '
                 '-o Dpkg::Progress-Fancy=1 install'.split() +
                 sorted(set(installable)),
                 env=ANY)
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_build_package_for_invalid_arch(
            self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = False
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(
            self.test_packages)
        self.useFixture(fake_apt)
        self.fake_apt_cache.add_packages(
            ('some-package:amd64', 'some-package:armhf'))
        self.assertIn("'invalid' is not a valid architecture name",
                      str(self.assertRaises(
                          repo._deb.BuildPackageNotFoundError,
                          self.install_test_packages,
                          ('package-installed', 'some-package:invalid'))))
        fake_apt.check_output_mock.assert_has_calls([
            call(['dpkg-architecture', '-L']),
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_build_package_for_unregistered_arch(
            self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = False
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(
            self.test_packages)
        self.useFixture(fake_apt)
        self.fake_apt_cache.add_packages(
            ('some-package:amd64'))

        self.assertIn('target architecture needs to be registered',
                      str(self.assertRaises(
                          repo._deb.BuildPackageNotFoundError,
                          self.install_test_packages,
                          ('package-installed', 'some-package:armhf'))))
        fake_apt.check_output_mock.assert_has_calls([
            call(['dpkg-architecture', '-L']),
            call(['dpkg', '--print-architecture']),
            call(['dpkg', '--print-foreign-architectures']),
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_build_package_sources_missing(
            self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = False
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(
            self.test_packages)
        self.useFixture(fake_apt)
        self.fake_apt_cache.add_packages(
            ('some-package:amd64'))

        self.assertIn("Sources for 'armhf' need to be added",
                      str(self.assertRaises(
                          repo._deb.BuildPackageNotFoundError,
                          self.install_test_packages,
                          ('package-installed', 'some-package:armhf'))))
        fake_apt.open_mock.assert_has_calls([
            call('/etc/apt/sources.list'),
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_buid_package_in_dumb_terminal(
            self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = True
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(self.test_packages)
        self.useFixture(fake_apt)
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        fake_apt.check_call_mock.assert_has_calls([
            call('sudo apt-get --no-install-recommends -y install'.split() +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    def test_install_buid_package_marks_auto_installed(self):
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(self.test_packages)
        self.useFixture(fake_apt)
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        fake_apt.check_call_mock.assert_has_calls([
            call('sudo apt-mark auto'.split() +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    @patch('subprocess.check_call')
    def test_mark_installed_auto_error_is_not_fatal(self, mock_check_call):
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(self.test_packages)
        self.useFixture(fake_apt)
        error = CalledProcessError(101, 'bad-cmd')
        mock_check_call.side_effect = \
            lambda c, env: error if 'apt-mark' in c else None
        self.install_test_packages(['package-not-installed'])

    def test_invalid_package_requested(self):
        fake_apt = tests.fixture_setup.FakeDpkgArchitecture(
            ['package-does-not-exist'])
        self.useFixture(fake_apt)
        raised = self.assertRaises(
            repo._deb.BuildPackageNotFoundError,
            repo.Ubuntu.install_build_packages,
            ['package-does-not-exist'])

        self.assertEqual(
            "Could not find a required package in 'build-packages': "
            "package-does-not-exist",
            str(raised))
