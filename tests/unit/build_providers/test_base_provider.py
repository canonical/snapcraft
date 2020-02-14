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

from testtools.matchers import Equals, EndsWith, DirExists, FileContains, Not

from . import BaseProviderBaseTest, MacBaseProviderWithBasesBaseTest, ProviderImpl
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
            Equals("project-name_{}.snap".format(self.project.deb_arch)),
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
        self.project.info.version = "test-version"

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        self.assertThat(
            provider.snap_filename,
            Equals("project-name_test-version_{}.snap".format(self.project.deb_arch)),
        )

    def test_launch_instance(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.start_mock.side_effect = errors.ProviderInstanceNotFoundError(
            instance_name=self.instance_name
        )
        provider.launch_instance()

        provider.launch_mock.assert_any_call()
        provider.start_mock.assert_any_call()
        provider.save_info_mock.assert_called_once_with({"base": "core16"})
        provider.run_mock.assert_called_once_with(["snapcraft", "refresh"])

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
        provider.build_provider_flags = dict(bind_ssh=False)
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
        provider.build_provider_flags = dict(bind_ssh=True)
        provider.mount_project()
        provider.mount_mock.assert_has_calls(
            [
                call(self.project._project_dir, "/root/project"),
                call("/home/user/.ssh", "/root/.ssh"),
            ]
        )

    def test_ensure_base_same_base(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.project.info.base = "core16"

        # Provider and project have the same base
        patcher = patch(
            "snapcraft.internal.build_providers._base_provider.Provider._load_info",
            return_value={"base": "core16"},
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        provider._ensure_base()
        provider.clean_project_mock.assert_not_called()

    def test_ensure_base_new_base(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.project.info.base = "core16"

        # Provider and project have different bases
        patcher = patch(
            "snapcraft.internal.build_providers._base_provider.Provider._load_info",
            return_value={"base": "core18"},
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        provider._ensure_base()
        provider.clean_project_mock.assert_called_once_with()

    def test_ensure_base_no_base_clean(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.project.info.base = "core16"

        # Provider has no base, project has base that's not core18
        # (assume provider has core18 for backward compatibility)
        patcher = patch(
            "snapcraft.internal.build_providers._base_provider.Provider._load_info",
            return_value={},
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        provider._ensure_base()
        provider.clean_project_mock.assert_called_once_with()

    def test_ensure_base_no_base_keep(self):

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.project.info.base = "core18"

        # Provider has no base, project has base core18
        # (assume provider has core18 for backward compatibility)
        patcher = patch(
            "snapcraft.internal.build_providers._base_provider.Provider._load_info",
            return_value={},
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        provider._ensure_base()
        provider.clean_project_mock.assert_not_called()

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

        self.assertThat(results, Equals(["env", "SNAPCRAFT_HAS_TTY=False"]))

    def test_passthrough_environment_flags_non_string(self):
        # Set http_proxy to a bool even though it doesn't make sense...
        # This is to verify non-strings are treated as strings OK for environ.
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.build_provider_flags = dict(http_proxy=True)

        results = provider._get_env_command()

        self.assertThat(
            results, Equals(["env", "SNAPCRAFT_HAS_TTY=False", "http_proxy=True"])
        )

    def test_passthrough_environment_flags_all(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.build_provider_flags = dict(
            http_proxy="http://127.0.0.1:8080", https_proxy="http://127.0.0.1:8080"
        )

        results = provider._get_env_command()

        self.assertThat(
            results,
            Equals(
                [
                    "env",
                    "SNAPCRAFT_HAS_TTY=False",
                    "http_proxy=http://127.0.0.1:8080",
                    "https_proxy=http://127.0.0.1:8080",
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
        self.project.info.base = "core"
        self.project.info.confinement = "classic"

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
        self.project.info.base = "core18"
        self.project.info.confinement = "classic"

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
        self.project.info.base = "core18"

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

    def test_setup_snapcraft_with_no_base(self):
        self.project.info.base = None

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
            [call(snap_name="core18"), call(snap_name="snapcraft")]
        )
        self.assertThat(self.snap_injector_mock().add.call_count, Equals(2))
        self.snap_injector_mock().apply.assert_called_once_with()

    def test_setup_snapcraft_for_classic_build(self):
        self.project.info.base = "core18"
        self.project.info.confinement = "classic"

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


class GetCloudUserDataTest(BaseProviderBaseTest):
    def test_get(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        os.makedirs(provider.provider_project_dir)

        cloud_data_filepath = provider._get_cloud_user_data()
        self.assertThat(
            cloud_data_filepath,
            FileContains(
                dedent(
                    """\
            #cloud-config
            manage_etc_hosts: true
            package_update: false
            growpart:
                mode: growpart
                devices: ["/"]
                ignore_growroot_disabled: false
            write_files:
                - path: /root/.bashrc
                  permissions: 0644
                  content: |
                    export SNAPCRAFT_BUILD_ENVIRONMENT=managed-host
                    export PS1="\h \$(/bin/_snapcraft_prompt)# "
                    export PATH=/snap/bin:$PATH
                - path: /bin/_snapcraft_prompt
                  permissions: 0755
                  content: |
                    #!/bin/bash
                    if [[ "$PWD" =~ ^$HOME.* ]]; then
                        path="${PWD/#$HOME/\ ..}"
                        if [[ "$path" == " .." ]]; then
                            ps1=""
                        else
                            ps1="$path"
                        fi
                    else
                        ps1="$PWD"
                    fi
                    echo -n $ps1
        """  # noqa: W605
                )
            ),
        )
