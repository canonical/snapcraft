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
    FileExists,
)

import snapcraft
from snapcraft.internal import repo
from snapcraft.internal.repo import errors
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
        name, arch, version = repo.get_pkg_name_parts('hello')
        self.assertEqual('hello', name)
        self.assertEqual('', arch)
        self.assertEqual(None, version)

    def test_get_pkg_name_parts_name_arch(self):
        name, arch, version = repo.get_pkg_name_parts('hello:armhf')
        self.assertEqual('hello', name)
        self.assertEqual(':armhf', arch)
        self.assertEqual(None, version)

    def test_get_pkg_name_parts_all(self):
        name, arch, version = repo.get_pkg_name_parts(
            'libpkg2:i386=3:0.4-0ubu5')
        self.assertEqual('libpkg2', name)
        self.assertEqual(':i386', arch)
        self.assertEqual('3:0.4-0ubu5', version)

    def test_get_pkg_name_parts_no_arch(self):
        name, arch, version = repo.get_pkg_name_parts('libpkg2=3:0.4-0ubu5')
        self.assertEqual('libpkg2', name)
        self.assertEqual('', arch)
        self.assertEqual('3:0.4-0ubu5', version)

    def test_get_pkg_name_parts_any_arch(self):
        name, arch, version = repo.get_pkg_name_parts('hello:any=2.10-1')
        self.assertEqual('hello', name)
        self.assertEqual('', arch)
        self.assertEqual('2.10-1', version)

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

    def test_ensure_package_format(self):
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep([])
        self.useFixture(fake_apt)
        self.assertEqual(
            ['libfoo1:armhf=1:0.2-0ubu3', 'libpkg2:armhf=3:0.4-0ubu5'],
            repo._deb.Ubuntu._ensure_package_format(
                ['libfoo1:armhf=1:0.2-0ubu3', 'libpkg2=3:0.4-0ubu5:armhf']))

    def test_setup_multi_arch_sources_skipped(self):
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep([])
        self.useFixture(fake_apt)
        repo._deb.Ubuntu._setup_multi_arch_sources(self.mock_cache,
                                                   'libpkg2=3:0.4-0ubu5')
        fake_apt.open_mock.assert_not_called()

    def test_setup_multi_arch_sources(self):
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep([])
        self.useFixture(fake_apt)
        repo._deb.Ubuntu._setup_multi_arch_sources(self.mock_cache,
                                                   'libpkg2:armhf=3:0.4-0ubu5')
        fake_apt.open_mock.assert_has_calls([
            call('/etc/apt/sources.list')
        ])
        sources_list = '/etc/apt/sources.list.d/ubuntu-{}.list'.format('armhf')
        fake_apt.check_call_mock.assert_has_calls([
            call(['sudo', 'cp', fake_apt.filename, sources_list]),
            call(['sudo', 'chmod', '644', sources_list]),
        ])


class BuildPackagesTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_apt_cache = fixture_setup.FakeAptCache()
        self.useFixture(self.fake_apt_cache)
        self.test_packages = (
            'package-not-installed', 'package-installed', 'libpkg2',
            'another-uninstalled', 'another-installed', 'repeated-package',
            'repeated-package', 'versioned-package=0.2', 'versioned-package')
        self.fake_apt_cache.add_packages(self.test_packages)
        self.fake_apt_cache.cache['package-installed'].installed = True
        self.fake_apt_cache.cache['libpkg2'].version = '3:0.4-0ubu5'
        self.fake_apt_cache.cache['another-installed'].installed = True
        self.fake_apt_cache.cache['versioned-package'].version = '0.1'

    def get_installable_packages(self, packages, target_arch=''):
        installable = []
        for pkg in packages:
            if not packages[pkg].installed:
                name, arch, version = repo.get_pkg_name_parts(pkg)
                if not arch:
                    arch = target_arch
                name += arch
                if version:
                    name += '={}'.format(version)
                installable.append(name)
        return installable

    @patch('snapcraft.repo._deb.apt')
    @patch('os.environ')
    def install_test_packages(self, test_pkgs, mock_env, mock_apt):
        mock_env.copy.return_value = {}
        mock_apt_cache = mock_apt.Cache.return_value
        mock_apt_cache_with = mock_apt_cache.__enter__.return_value
        mock_apt_cache_with.__getitem__.side_effect = lambda p: test_pkgs[p]

        repo.Ubuntu.install_build_packages(test_pkgs, 'amd64')

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_build_package(
            self, mock_is_dumb_terminal):
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep(
            self.test_packages.keys())
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
        fake_apt.check_output_mock.assert_has_calls([
            call(['dpkg', '--print-architecture']),
            call(['apt-get', 'build-dep', '-q', '-s',
                  '-aamd64', fake_apt.filename],
                 env={}, stderr=-2),
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_buid_package_in_dumb_terminal(
            self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = True
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep(self.test_packages)
        self.useFixture(fake_apt)
        self.install_test_packages(self.test_packages)

        installable = self.get_installable_packages(self.test_packages)
        fake_apt.check_call_mock.assert_has_calls([
            call('sudo apt-get --no-install-recommends -y install'.split() +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    @patch('snapcraft.repo._deb.apt')
    @patch('os.environ')
    def test_install_build_package_with_arch(
            self, mock_env, mock_apt, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = True
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep(
            self.test_packages, 'armhf')
        self.useFixture(fake_apt)
        mock_env.copy.return_value = {}
        mock_apt_cache = mock_apt.Cache.return_value
        mock_apt_cache_with = mock_apt_cache.__enter__.return_value
        mock_apt_cache_with.__getitem__.side_effect = lambda p: \
            self.test_packages[p.replace(':armhf', '')]
        self.assertEqual(['amd64'], fake_apt.archs)

        repo.Ubuntu.install_build_packages(self.test_packages.keys(), 'armhf')

        self.assertEqual(['amd64', 'armhf'], fake_apt.archs)
        fake_apt.check_output_mock.assert_has_calls([
            call(['apt-get', 'build-dep', '-q', '-s',
                  '-aarmhf', fake_apt.filename],
                 env={}, stderr=-2),
        ])

        installable = self.get_installable_packages(self.test_packages,
                                                    ':armhf')
        fake_apt.check_call_mock.assert_has_calls([
            call(['sudo', 'apt-get', '--no-install-recommends',
                  '-y', 'install'] +
                 sorted(set(installable)),
                 env={'DEBIAN_FRONTEND': 'noninteractive',
                      'DEBCONF_NONINTERACTIVE_SEEN': 'true'})
        ])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    @patch('snapcraft.repo._deb.apt')
    def test_install_build_package_already_installed(
            self, mock_apt, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = True
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep([])
        self.useFixture(fake_apt)
        self.install_test_packages(
            {'package-installed': MagicMock(installed=True)})

        fake_apt.check_output_mock.assert_has_calls([
            call(['apt-get', 'build-dep', '-q', '-s',
                  '-aamd64', fake_apt.filename],
                 env={}, stderr=-2),
        ])

        fake_apt.check_call_mock.assert_has_calls([])

    @patch('snapcraft.repo._deb.is_dumb_terminal')
    def test_install_build_package_with_arch_update_failed(
            self, mock_is_dumb_terminal):
        mock_is_dumb_terminal.return_value = True
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep(
            self.test_packages, 'armhf', update_error=True)
        self.useFixture(fake_apt)
        self.assertEqual(['amd64'], fake_apt.archs)

        self.assertRaises(
            (errors.BuildPackageNotFoundError, CalledProcessError),
            repo.Ubuntu.install_build_packages,
            self.test_packages.keys(),
            'armhf')

        fake_apt.check_output_mock.assert_has_calls([
            call(['dpkg', '--print-foreign-architectures']),
            call(['sudo', 'dpkg', '--add-architecture', 'armhf']),
            call(['apt-get', 'build-dep', '-q', '-s',
                  '-aarmhf', fake_apt.filename],
                 env={}, stderr=-2),
            call(['sudo', 'apt-get', 'update'],
                 stderr=-2),
        ])

        sources_list = '/etc/apt/sources.list.d/ubuntu-{}.list'.format('armhf')
        self.assertEqual(['amd64', 'armhf'], fake_apt.archs)
        fake_apt.check_call_mock.assert_has_calls([
            call(['sudo', 'cp', fake_apt.filename, sources_list]),
            call(['sudo', 'chmod', '644', sources_list]),
        ])
        fake_apt.open_mock.assert_has_calls([
            call('/etc/apt/sources.list')
        ])

    def test_install_buid_package_marks_auto_installed(self):
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep(self.test_packages)
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
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep(self.test_packages)
        self.useFixture(fake_apt)
        error = CalledProcessError(101, 'bad-cmd')
        mock_check_call.side_effect = \
            lambda c, env: error if 'apt-mark' in c else None
        self.install_test_packages(['package-not-installed'])

    def test_invalid_package_requested(self):
        fake_apt = tests.fixture_setup.FakeAptGetBuildDep(
            ['package-does-not-exist'], not_available=True)
        self.useFixture(fake_apt)
        project_options = snapcraft.ProjectOptions()
        raised = self.assertRaises(
            errors.BuildPackageNotFoundError,
            repo.Ubuntu.install_build_packages,
            ['package-does-not-exist'], project_options.deb_arch)

        self.assertEqual(
            "Could not find a required package in 'build-packages': "
            "package-does-not-exist",
            str(raised))
        fake_apt.check_output_mock.assert_has_calls([
            call(['apt-get', 'build-dep', '-q', '-s',
                  '-a{}'.format(project_options.deb_arch),
                  fake_apt.filename], env={}, stderr=-2)
        ])
