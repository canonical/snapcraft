# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2020 Canonical Ltd
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
import os
import pathlib
from textwrap import dedent
from unittest.mock import call, patch, Mock

import fixtures
from testtools.matchers import Equals, EndsWith, DirExists, Not

from . import (
    BaseProviderBaseTest,
    MacBaseProviderWithBasesBaseTest,
    ProviderImpl,
    get_project,
)
from snapcraft.internal.build_providers import errors


class BaseProviderTest(BaseProviderBaseTest):
    def test_initialize(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        self.assertThat(provider.project, Equals(self.project))
        self.assertThat(provider.instance_name, Equals(self.instance_name))
        self.assertThat(
            provider.provider_project_dir,
            EndsWith(
                os.path.join("snapcraft", "projects", "project-name", "stub-provider")
            ),
        )
        self.assertThat(
            provider.snap_filename,
            Equals("project-name_1.0_{}.snap".format(self.project.deb_arch)),
        )

    def test_context(self):
        with ProviderImpl(project=self.project, echoer=self.echoer_mock) as provider:
            provider.shell()
            fake_provider = provider

        fake_provider.create_mock.assert_called_once_with("create")
        fake_provider.destroy_mock.assert_called_once_with("destroy")

    def test_context_fails_create(self):
        create_mock = Mock()
        destroy_mock = Mock()

        class BadProviderImpl(ProviderImpl):
            def create(self):
                super().create()
                create_mock("create bad")
                raise errors.ProviderBaseError()

            def destroy(self):
                super().destroy()
                destroy_mock("destroy bad")

        with contextlib.suppress(errors.ProviderBaseError):
            with BadProviderImpl(project=self.project, echoer=self.echoer_mock):
                pass

        create_mock.assert_called_once_with("create bad")
        destroy_mock.assert_called_once_with("destroy bad")

    def test_initialize_snap_filename_with_version(self):
        self.project._snap_meta.version = "test-version"

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        self.assertThat(
            provider.snap_filename,
            Equals("project-name_test-version_{}.snap".format(self.project.deb_arch)),
        )

    def test_launch_instance(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAP_VERSION", "4.0"))

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.start_mock.side_effect = errors.ProviderInstanceNotFoundError(
            instance_name=self.instance_name
        )
        provider.launch_instance()

        provider.launch_mock.assert_any_call()
        provider.start_mock.assert_any_call()
        provider.save_info_mock.assert_called_once_with(
            {"data": {"base": "core16", "created-by-snapcraft-version": "4.0"}}
        )

        self.assertThat(
            provider.run_mock.mock_calls,
            Equals(
                [
                    call(["mv", "/var/tmp/L3Jvb3QvLmJhc2hyYw==", "/root/.bashrc"]),
                    call(["chown", "root:root", "/root/.bashrc"]),
                    call(["chmod", "0600", "/root/.bashrc"]),
                    call(
                        [
                            "mv",
                            "/var/tmp/L2Jpbi9fc25hcGNyYWZ0X3Byb21wdA==",
                            "/bin/_snapcraft_prompt",
                        ]
                    ),
                    call(["chown", "root:root", "/bin/_snapcraft_prompt"]),
                    call(["chmod", "0755", "/bin/_snapcraft_prompt"]),
                    call(
                        [
                            "mv",
                            "/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0",
                            "/etc/apt/sources.list",
                        ]
                    ),
                    call(["chown", "root:root", "/etc/apt/sources.list"]),
                    call(["chmod", "0644", "/etc/apt/sources.list"]),
                    call(
                        [
                            "mv",
                            "/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0LmQvZGVmYXVsdC5zb3VyY2Vz",
                            "/etc/apt/sources.list.d/default.sources",
                        ]
                    ),
                    call(
                        [
                            "chown",
                            "root:root",
                            "/etc/apt/sources.list.d/default.sources",
                        ]
                    ),
                    call(["chmod", "0644", "/etc/apt/sources.list.d/default.sources"]),
                    call(
                        [
                            "mv",
                            "/var/tmp/L2V0Yy9hcHQvc291cmNlcy5saXN0LmQvZGVmYXVsdC1zZWN1cml0eS5zb3VyY2Vz",
                            "/etc/apt/sources.list.d/default-security.sources",
                        ]
                    ),
                    call(
                        [
                            "chown",
                            "root:root",
                            "/etc/apt/sources.list.d/default-security.sources",
                        ]
                    ),
                    call(
                        [
                            "chmod",
                            "0644",
                            "/etc/apt/sources.list.d/default-security.sources",
                        ]
                    ),
                    call(
                        [
                            "mv",
                            "/var/tmp/L2V0Yy9hcHQvYXB0LmNvbmYuZC8wMC1zbmFwY3JhZnQ=",
                            "/etc/apt/apt.conf.d/00-snapcraft",
                        ]
                    ),
                    call(["chown", "root:root", "/etc/apt/apt.conf.d/00-snapcraft"]),
                    call(["chmod", "0644", "/etc/apt/apt.conf.d/00-snapcraft"]),
                    call(["apt-get", "update"]),
                    call(["apt-get", "dist-upgrade", "--yes"]),
                    call(["apt-get", "install", "--yes", "apt-transport-https"]),
                ]
            ),
        )

        self.assertThat(provider.provider_project_dir, DirExists())

    def test_expose_prime(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.expose_prime()

        provider.mount_mock.assert_called_once_with(
            self.project.prime_dir, "/root/prime"
        )

    @patch("pathlib.Path.home", return_value=pathlib.Path("/home/user"))
    def test_bind_ssh(self, mock_home):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        # False.
        provider.build_provider_flags = dict(SNAPCRAFT_BIND_SSH=False)
        provider.mount_project()
        provider.mount_mock.assert_has_calls(
            [call(self.project._project_dir, "/root/project")]
        )

        # Not present.
        provider.build_provider_flags = dict()
        provider.mount_project()
        provider.mount_mock.assert_has_calls(
            [call(self.project._project_dir, "/root/project")]
        )

        # True.
        provider.build_provider_flags = dict(SNAPCRAFT_BIND_SSH=True)
        provider.mount_project()
        provider.mount_mock.assert_has_calls(
            [
                call(self.project._project_dir, "/root/project"),
                call("/home/user/.ssh", "/root/.ssh"),
            ]
        )

    def test_setup_environment_content_amd64(self):
        self.useFixture(fixtures.MockPatch("platform.machine", return_value="x86_64"))
        recorded_files = dict()

        @contextlib.contextmanager
        def fake_namedtempfile(*, suffix: str, **kwargs):
            # Usage hides the file basename in the suffix.
            tmp_path = os.path.join(self.path, "tmpfile")
            with open(tmp_path, "wb") as f_write:
                yield f_write
            with open(tmp_path, "r") as f_read:
                recorded_files[suffix] = f_read.read()

        self.useFixture(
            fixtures.MockPatch("tempfile.NamedTemporaryFile", new=fake_namedtempfile)
        )

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider._setup_environment()

        self.expectThat(
            recorded_files,
            Equals(
                {
                    ".bashrc": '#!/bin/bash\nexport PS1="\\h \\$(/bin/_snapcraft_prompt)# "\n',
                    "00-snapcraft": 'Apt::Install-Recommends "false";\n',
                    "_snapcraft_prompt": dedent(
                        """\
                        #!/bin/bash
                        if [[ "$PWD" =~ ^$HOME.* ]]; then
                            path="${PWD/#$HOME/\\ ..}"
                            if [[ "$path" == " .." ]]; then
                                ps1=""
                            else
                                ps1="$path"
                            fi
                        else
                            ps1="$PWD"
                        fi
                        echo -n $ps1
                        """
                    ),
                    "default.sources": dedent(
                        """\
                        Types: deb deb-src
                        URIs: http://archive.ubuntu.com/ubuntu
                        Suites: xenial xenial-updates
                        Components: main multiverse restricted universe
                    """
                    ),
                    "default-security.sources": dedent(
                        """\
                        Types: deb deb-src
                        URIs: http://security.ubuntu.com/ubuntu
                        Suites: xenial-security
                        Components: main multiverse restricted universe
                    """
                    ),
                    "sources.list": "",
                }
            ),
        )

    def test_setup_environment_content_arm64(self):
        self.useFixture(fixtures.MockPatch("platform.machine", return_value="aarch64"))
        recorded_files = dict()

        @contextlib.contextmanager
        def fake_namedtempfile(*, suffix: str, **kwargs):
            # Usage hides the file basename in the suffix.
            tmp_path = os.path.join(self.path, "tmpfile")
            with open(tmp_path, "wb") as f_write:
                yield f_write
            with open(tmp_path, "r") as f_read:
                recorded_files[suffix] = f_read.read()

        self.useFixture(
            fixtures.MockPatch("tempfile.NamedTemporaryFile", new=fake_namedtempfile)
        )

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider._setup_environment()

        self.expectThat(
            recorded_files,
            Equals(
                {
                    ".bashrc": '#!/bin/bash\nexport PS1="\\h \\$(/bin/_snapcraft_prompt)# "\n',
                    "00-snapcraft": 'Apt::Install-Recommends "false";\n',
                    "_snapcraft_prompt": dedent(
                        """\
                        #!/bin/bash
                        if [[ "$PWD" =~ ^$HOME.* ]]; then
                            path="${PWD/#$HOME/\\ ..}"
                            if [[ "$path" == " .." ]]; then
                                ps1=""
                            else
                                ps1="$path"
                            fi
                        else
                            ps1="$PWD"
                        fi
                        echo -n $ps1
                        """
                    ),
                    "default.sources": dedent(
                        """\
                        Types: deb deb-src
                        URIs: http://ports.ubuntu.com/ubuntu-ports
                        Suites: xenial xenial-updates
                        Components: main multiverse restricted universe
                    """
                    ),
                    "default-security.sources": dedent(
                        """\
                        Types: deb deb-src
                        URIs: http://ports.ubuntu.com/ubuntu-ports
                        Suites: xenial-security
                        Components: main multiverse restricted universe
                    """
                    ),
                    "sources.list": "",
                }
            ),
        )

    def test_start_instance(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        provider.launch_instance()

        provider.launch_mock.assert_not_called()
        provider.start_mock.assert_any_call()
        provider.run_mock.assert_not_called()

        # Given the way we constructe this test, this directory should not exist
        # TODO add robustness to start. (LP: #1792242)
        self.assertThat(provider.provider_project_dir, Not(DirExists()))

    def test_clean_part(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        provider.clean(part_names=("part1",))

        provider.run_mock.assert_called_once_with(["snapcraft", "clean", "part1"])

    def test_clean_multiple_parts(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        provider.clean(part_names=("part1", "part2"))

        provider.run_mock.assert_called_once_with(
            ["snapcraft", "clean", "part1", "part2"]
        )

    def test_passthrough_environment_flags_empty(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.build_provider_flags = dict()

        results = provider._get_env_command()

        self.assertThat(
            results,
            Equals(
                [
                    "env",
                    "SNAPCRAFT_BUILD_ENVIRONMENT=managed-host",
                    "HOME=/root",
                    "SNAPCRAFT_HAS_TTY=False",
                ]
            ),
        )

    def test_passthrough_environment_flags_non_string(self):
        # Set http_proxy to a bool even though it doesn't make sense...
        # This is to verify non-strings are treated as strings OK for environ.
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.build_provider_flags = dict(http_proxy=True)

        results = provider._get_env_command()

        self.assertThat(
            results,
            Equals(
                [
                    "env",
                    "SNAPCRAFT_BUILD_ENVIRONMENT=managed-host",
                    "HOME=/root",
                    "SNAPCRAFT_HAS_TTY=False",
                    "http_proxy=True",
                ]
            ),
        )

    def test_passthrough_environment_flags_all(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.build_provider_flags = dict(
            http_proxy="http://127.0.0.1:8080",
            https_proxy="http://127.0.0.1:8080",
            SNAPCRAFT_BUILD_INFO=True,
            SNAPCRAFT_IMAGE_INFO='{"build_url":"https://example.com"}',
        )

        results = provider._get_env_command()

        self.assertThat(
            results,
            Equals(
                [
                    "env",
                    "SNAPCRAFT_BUILD_ENVIRONMENT=managed-host",
                    "HOME=/root",
                    "SNAPCRAFT_HAS_TTY=False",
                    "http_proxy=http://127.0.0.1:8080",
                    "https_proxy=http://127.0.0.1:8080",
                    "SNAPCRAFT_BUILD_INFO=True",
                    'SNAPCRAFT_IMAGE_INFO={"build_url":"https://example.com"}',
                ]
            ),
        )


class BaseProviderProvisionSnapcraftTest(BaseProviderBaseTest):
    def test_setup_snapcraft(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider._setup_snapcraft()

        self.snap_injector_mock.assert_called_once_with(
            registry_filepath=os.path.join(
                provider.provider_project_dir, "snap-registry.yaml"
            ),
            snap_arch=self.project.deb_arch,
            runner=provider._run,
            file_pusher=provider._push_file,
            inject_from_host=True,
        )
        self.snap_injector_mock().add.assert_has_calls(
            [
                call(snap_name="snapd"),
                call(snap_name="core16"),
                call(snap_name="core18"),
                call(snap_name="snapcraft"),
            ]
        )
        self.assertThat(self.snap_injector_mock().add.call_count, Equals(4))
        self.snap_injector_mock().apply.assert_called_once_with()

    def test_ephemeral_setup_snapcraft(self):
        provider = ProviderImpl(
            project=self.project, echoer=self.echoer_mock, is_ephemeral=True
        )
        provider._setup_snapcraft()

        self.snap_injector_mock.assert_called_once_with(
            registry_filepath=os.path.join(
                provider.provider_project_dir, "snap-registry.yaml"
            ),
            snap_arch=self.project.deb_arch,
            runner=provider._run,
            file_pusher=provider._push_file,
            inject_from_host=True,
        )
        self.snap_injector_mock().add.assert_has_calls(
            [
                call(snap_name="snapd"),
                call(snap_name="core16"),
                call(snap_name="core18"),
                call(snap_name="snapcraft"),
            ]
        )
        self.assertThat(self.snap_injector_mock().add.call_count, Equals(4))
        self.snap_injector_mock().apply.assert_called_once_with()

    def test_setup_snapcraft_with_core(self):
        self.project._snap_meta.base = "core"
        self.project._snap_meta.confinement = "classic"

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider._setup_snapcraft()

        self.snap_injector_mock().add.assert_has_calls(
            [
                call(snap_name="core"),
                call(snap_name="core18"),
                call(snap_name="snapcraft"),
            ]
        )
        self.assertThat(self.snap_injector_mock().add.call_count, Equals(3))
        self.snap_injector_mock().apply.assert_called_once_with()

    def test_setup_snapcraft_for_classic_build(self):
        self.project._snap_meta.base = "core18"
        self.project._snap_meta.confinement = "classic"

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider._setup_snapcraft()

        self.snap_injector_mock().add.assert_has_calls(
            [
                call(snap_name="snapd"),
                call(snap_name="core18"),
                call(snap_name="snapcraft"),
            ]
        )
        self.assertThat(self.snap_injector_mock().add.call_count, Equals(3))
        self.snap_injector_mock().apply.assert_called_once_with()


class MacProviderProvisionSnapcraftTest(MacBaseProviderWithBasesBaseTest):
    def test_setup_snapcraft_with_base(self):
        self.project._snap_meta.base = "core18"

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider._setup_snapcraft()

        self.snap_injector_mock.assert_called_once_with(
            registry_filepath=os.path.join(
                provider.provider_project_dir, "snap-registry.yaml"
            ),
            snap_arch=self.project.deb_arch,
            runner=provider._run,
            file_pusher=provider._push_file,
            inject_from_host=False,
        )
        self.snap_injector_mock().add.assert_has_calls(
            [
                call(snap_name="snapd"),
                call(snap_name="core18"),
                call(snap_name="snapcraft"),
            ]
        )
        self.assertThat(self.snap_injector_mock().add.call_count, Equals(3))
        self.snap_injector_mock().apply.assert_called_once_with()

    def test_setup_snapcraft_for_classic_build(self):
        self.project._snap_meta.base = "core18"
        self.project._snap_meta.confinement = "classic"

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider._setup_snapcraft()

        self.snap_injector_mock.assert_called_once_with(
            registry_filepath=os.path.join(
                provider.provider_project_dir, "snap-registry.yaml"
            ),
            snap_arch=self.project.deb_arch,
            runner=provider._run,
            file_pusher=provider._push_file,
            inject_from_host=False,
        )
        self.snap_injector_mock().add.assert_has_calls(
            [
                call(snap_name="snapd"),
                call(snap_name="core18"),
                call(snap_name="snapcraft"),
            ]
        )
        self.assertThat(self.snap_injector_mock().add.call_count, Equals(3))
        self.snap_injector_mock().apply.assert_called_once_with()


class TestCompatibilityClean:
    scenarios = [
        (
            "same-base-no-clean",
            dict(
                base="core16",
                loaded_info={"base": "core16", "created-by-snapcraft-version": "1.0"},
                version="1.0",
                expect_clean=False,
            ),
        ),
        (
            "different-base-clean",
            dict(
                base="core16",
                loaded_info={"base": "core18", "created-by-snapcraft-version": "1.0"},
                version="1.0",
                expect_clean=True,
            ),
        ),
        (
            "unspecified-base-clean",
            dict(
                base="core20",
                loaded_info={"created-by-snapcraft-version": "1.0"},
                version="1.0",
                expect_clean=True,
            ),
        ),
        (
            "unspecified-created-version-clean",
            dict(
                base="core20",
                loaded_info={"base": "core20"},
                version="1.0",
                expect_clean=True,
            ),
        ),
        (
            "downgrade-version-clean",
            dict(
                base="core20",
                loaded_info={"base": "core20", "created-by-snapcraft-version": "2.0"},
                version="1.0",
                expect_clean=True,
            ),
        ),
        (
            "same-version-no-clean",
            dict(
                base="core20",
                loaded_info={"base": "core20", "created-by-snapcraft-version": "2.0"},
                version="2.0",
                expect_clean=False,
            ),
        ),
        (
            "upgrade-version-no-clean",
            dict(
                base="core20",
                loaded_info={"base": "core20", "created-by-snapcraft-version": "2.0"},
                version="3.0",
                expect_clean=False,
            ),
        ),
    ]

    def test_scenario(
        self, monkeypatch, in_snap, base, loaded_info, version, expect_clean
    ):
        monkeypatch.setenv("SNAP_VERSION", version)

        provider = ProviderImpl(project=get_project(), echoer=Mock())
        provider.project._snap_meta.base = base
        provider.loaded_info = loaded_info

        provider._ensure_compatible_build_environment()

        if expect_clean:
            provider.clean_project_mock.assert_called_once_with()
        else:
            provider.clean_project_mock.assert_not_called()
