# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from unittest import mock

import fixtures
from testtools.matchers import Equals

from snapcraft.internal import repo
from snapcraft.internal.pluginhandler._dependencies import MissingDependencyResolver
from tests import unit


class MissingDependencyTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        packages = {"/bin/bash": "bash", "/bin/sh": "dash"}

        def fake_repo_query(*args, **kwargs):
            file_path = kwargs["file_path"]
            try:
                return packages[file_path]
            except KeyError:
                raise repo.errors.FileProviderNotFound(file_path=file_path)

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.repo.Repo.get_package_for_file",
                side_effect=fake_repo_query,
            )
        )

    def test_all_stage_packages(self):
        d = MissingDependencyResolver(elf_files=["/bin/bash", "/bin/sh"])

        self.assertThat(d._stage_packages_dependencies, Equals({"bash", "dash"}))
        self.assertThat(d._unhandled_dependencies, Equals(set()))

        echoer = mock.Mock()
        d.print_resolutions(
            part_name="fake-part", stage_packages_exist=True, echoer=echoer
        )
        echoer.warning.assert_called_once_with(
            "The 'fake-part' part is missing libraries that are not included "
            "in the snap or base. They can be satisfied by adding the "
            "following entries to the existing stage-packages for this part:\n- bash\n- dash"
        )

        echoer.reset_mock()
        d.print_resolutions(
            part_name="fake-part", stage_packages_exist=False, echoer=echoer
        )
        echoer.warning.assert_called_once_with(
            "The 'fake-part' part is missing libraries that are not included "
            "in the snap or base. They can be satisfied by adding the "
            "following entry for this part\nstage-packages:\n- bash\n- dash"
        )

    def test_unhandled_dependencies(self):
        d = MissingDependencyResolver(elf_files=["/foo", "/bar", "/foo/bar"])

        self.assertThat(d._stage_packages_dependencies, Equals(set()))
        self.assertThat(d._unhandled_dependencies, Equals({"/foo", "/bar", "/foo/bar"}))

        echoer = mock.Mock()
        d.print_resolutions(
            part_name="fake-part", stage_packages_exist=True, echoer=echoer
        )
        echoer.warning.assert_called_once_with(
            "This part is missing libraries that cannot be satisfied with "
            "any available stage-packages known to snapcraft:\n"
            "- /bar\n- /foo\n- /foo/bar\n"
            "These dependencies can be satisfied via additional parts or "
            "content sharing. "
            "Consider validating configured filesets if this dependency was built."
        )

    def test_stage_packages_and_unhandled_dependencies(self):
        d = MissingDependencyResolver(
            elf_files=["/bin/bash", "/bin/sh", "/foo", "/bar", "/foo/bar"]
        )

        self.assertThat(d._stage_packages_dependencies, Equals({"bash", "dash"}))
        self.assertThat(d._unhandled_dependencies, Equals({"/foo", "/bar", "/foo/bar"}))

        echoer = mock.Mock()
        d.print_resolutions(
            part_name="fake-part", stage_packages_exist=False, echoer=echoer
        )
        echoer.warning.assert_has_calls(
            [
                mock.call(
                    "The 'fake-part' part is missing libraries that are not included "
                    "in the snap or base. They can be satisfied by adding the "
                    "following entry for this part\nstage-packages:\n- bash\n- dash"
                ),
                mock.call(
                    "This part is missing libraries that cannot be satisfied with "
                    "any available stage-packages known to snapcraft:\n"
                    "- /bar\n- /foo\n- /foo/bar\n"
                    "These dependencies can be satisfied via additional parts or "
                    "content sharing. "
                    "Consider validating configured filesets if this dependency was built."
                ),
            ]
        )
