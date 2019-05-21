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

from textwrap import dedent

from testtools.matchers import Equals

from snapcraft.project import Project
from snapcraft.project._environment_checks import EnvironmentChecks

from tests import fixture_setup, unit


class SnapTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())
        self.useFixture(fixture_setup.FakeSnapcraftIsADeb(return_value=False))
        self.useFixture(
            fixture_setup.FakeSnapcraftIsInDockerInstance(return_value=False)
        )

    def test_no_base(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="xenial", name="Ubuntu", version_id="16.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This snapcraft project does not specify the base keyword, explicitly setting the base keyword enables the latest snapcraft features.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )

    def test_no_base_host_mismatch(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="bionic", name="Ubuntu", version_id="18.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This snapcraft project does not specify the base keyword, explicitly setting the base keyword enables the latest snapcraft features.
            This project is best built on 'Ubuntu 16.04', but is building on a 'Ubuntu 18.04' host.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )


class SnapInDockerTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())
        self.useFixture(fixture_setup.FakeSnapcraftIsADeb(return_value=False))
        self.useFixture(fixture_setup.FakeSnapcraftIsInDockerInstance())

    def test_no_base(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="xenial", name="Ubuntu", version_id="16.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(EnvironmentChecks(project).get_messages(), Equals(""))

    def test_no_base_host_mismatch(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="bionic", name="Ubuntu", version_id="18.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This project is best built on 'Ubuntu 16.04', but is building on a 'Ubuntu 18.04' host.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )


class DebTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeSnapcraftIsADeb())
        self.useFixture(
            fixture_setup.FakeSnapcraftIsInDockerInstance(return_value=False)
        )

    def test_no_base(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="xenial", name="Ubuntu", version_id="16.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            Upgrade snapraft https://docs.snapcraft.io/t/upgrading-snapcraft/11658 to take advantage of the latest features in snapcraft.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )

    def test_no_base_host_mismatch(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="bionic", name="Ubuntu", version_id="18.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            Upgrade snapraft https://docs.snapcraft.io/t/upgrading-snapcraft/11658 to take advantage of the latest features in snapcraft.
            This project is best built on 'Ubuntu 16.04', but is building on a 'Ubuntu 18.04' host.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )

    def test_with_base(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path, base="core18")
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="bionic", name="Ubuntu", version_id="18.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This version of snapcraft only has experimental support for the base keyword.
            Upgrade snapraft https://docs.snapcraft.io/t/upgrading-snapcraft/11658 to take advantage of the latest features in snapcraft.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )

    def test_with_base_host_mismatch(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path, base="core18")
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="xeanil", name="Ubuntu", version_id="16.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This version of snapcraft only has experimental support for the base keyword.
            Upgrade snapraft https://docs.snapcraft.io/t/upgrading-snapcraft/11658 to take advantage of the latest features in snapcraft.
            This project targets the 'core18' base, best built on 'Ubuntu 18.04', but is building on a 'Ubuntu 16.04' host.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )


class DebInDockerTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeSnapcraftIsADeb())
        self.useFixture(fixture_setup.FakeSnapcraftIsInDockerInstance())

    def test_no_base(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="xenial", name="Ubuntu", version_id="16.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(EnvironmentChecks(project).get_messages(), Equals(""))

    def test_no_base_host_mismatch(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path)
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="bionic", name="Ubuntu", version_id="18.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This project is best built on 'Ubuntu 16.04', but is building on a 'Ubuntu 18.04' host.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )

    def test_with_base(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path, base="core18")
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="bionic", name="Ubuntu", version_id="18.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This version of snapcraft only has experimental support for the base keyword.
            Upgrade snapraft https://docs.snapcraft.io/t/upgrading-snapcraft/11658 to take advantage of the latest features in snapcraft.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )

    def test_with_base_host_mismatch(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(path=self.path, base="core18")
        self.useFixture(snapcraft_yaml)
        self.useFixture(
            fixture_setup.FakeOsRelease(
                codename="xeanil", name="Ubuntu", version_id="16.04"
            )
        )

        project = Project(
            snapcraft_yaml_file_path=snapcraft_yaml.snapcraft_yaml_file_path
        )
        self.assertThat(
            EnvironmentChecks(project).get_messages(),
            Equals(
                dedent(
                    """\
            This version of snapcraft only has experimental support for the base keyword.
            Upgrade snapraft https://docs.snapcraft.io/t/upgrading-snapcraft/11658 to take advantage of the latest features in snapcraft.
            This project targets the 'core18' base, best built on 'Ubuntu 18.04', but is building on a 'Ubuntu 16.04' host.
            Read more about bases at https://docs.snapcraft.io/t/base-snaps/11198"""
                )
            ),
        )
