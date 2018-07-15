# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
from glob import glob
import subprocess
from textwrap import dedent

from testtools.matchers import DirExists, Equals, FileContains, FileExists

from tests import integration, fixture_setup


class PythonPluginTestCase(integration.TestCase):
    def test_use_staged_pip(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "test-part",
            {"plugin": "python", "source": ".", "stage-packages": ["python3-pip"]},
        )
        self.useFixture(snapcraft_yaml)

        self.run_snapcraft("pull")

    def test_pull_with_pip_requirements_file(self):
        self.run_snapcraft("build", "pip-requirements-file")
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python2",
                    "install",
                    "lib",
                    "python2*",
                    "site-packages",
                    "argparse.py",
                )
            )[0],
            FileExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python3",
                    "install",
                    "lib",
                    "python3*",
                    "site-packages",
                    "argparse.py",
                )
            )[0],
            FileExists(),
        )

    def test_pull_with_pip_requirements_list(self):
        self.run_snapcraft("build", "pip-requirements-list")
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python2",
                    "install",
                    "lib",
                    "python2*",
                    "site-packages",
                    "argparse.py",
                )
            )[0],
            FileExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python2",
                    "install",
                    "lib",
                    "python2*",
                    "site-packages",
                    "jsonschema",
                )
            )[0],
            DirExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python3",
                    "install",
                    "lib",
                    "python3*",
                    "site-packages",
                    "argparse.py",
                )
            )[0],
            FileExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python3",
                    "install",
                    "lib",
                    "python3*",
                    "site-packages",
                    "jsonschema",
                )
            )[0],
            DirExists(),
        )

    def test_build_rewrites_shebangs(self):
        """Verify that LP: #1597919 doesn't come back."""
        self.run_snapcraft("stage", "python-entry-point")
        python2_entry_point = os.path.join(self.stage_dir, "bin", "python2_test")
        python3_entry_point = os.path.join(self.stage_dir, "bin", "python3_test")
        python_entry_point = os.path.join(self.stage_dir, "bin", "python_test")

        with open(python2_entry_point) as f:
            python2_shebang = f.readline().strip()
        with open(python3_entry_point) as f:
            python3_shebang = f.readline().strip()
        with open(python_entry_point) as f:
            python_shebang = f.readline().strip()

        self.assertThat(python2_shebang, Equals("#!/usr/bin/env python2"))
        self.assertThat(python3_shebang, Equals("#!/usr/bin/env python3"))
        self.assertThat(python_shebang, Equals("#!/usr/bin/env python3"))

    def test_pbr_console_scripts(self):
        """Verify that LP: #1670852 doesn't come back."""
        # pbr needs git repos to work
        def pre_func():
            for python_dir in ["python2", "python3"]:
                d = os.path.join(self.path, python_dir)
                subprocess.check_output(["git", "init"], cwd=d)
                subprocess.check_output(
                    ["git", "config", "user.name", "Test User"], cwd=d
                )
                subprocess.check_output(
                    ["git", "config", "user.email", "<test.user@example.com"], cwd=d
                )
                subprocess.check_output(["git", "add", "."], cwd=d)
                subprocess.check_output(["git", "commit", "-m", "initial"], cwd=d)

        self.run_snapcraft("stage", "python-pbr", pre_func=pre_func)

        console_script_template = dedent(
            """\
            #!/usr/bin/env python{version}
            # PBR Generated from {console_scripts}

            import sys

            from python{version}_test_package.main import main


            if __name__ == "__main__":
                sys.exit(main())
        """
        )

        python2_entry_point = os.path.join(self.stage_dir, "bin", "python2_test")
        self.assertThat(python2_entry_point, FileExists())
        console_script = console_script_template.format(
            version="2", console_scripts="u'console_scripts'"
        )
        self.assertThat(python2_entry_point, FileContains(console_script))

        python3_entry_point = os.path.join(self.stage_dir, "bin", "python3_test")
        self.assertThat(python3_entry_point, FileExists())
        console_script = console_script_template.format(
            version="3", console_scripts="'console_scripts'"
        )
        self.assertThat(python3_entry_point, FileContains(console_script))

    def test_build_does_not_keep_pyc_or_pth_files_in_install(self):
        # .pyc and .pyc files collide between parts.
        # There is no way to tell pip or setup.py to disable generation of
        # .pyc
        # The .pth files are only relevant if found inside the pre compiled
        # site-packges directory so we don't want those either.
        self.run_snapcraft("stage", "pip-requirements-file")

        pyc_files = []
        pth_files = []
        for _, _, files in os.walk(self.stage_dir):
            pyc_files.extend([f for f in files if f.endswith("pyc")])
            pth_files.extend([f for f in files if f.endswith("pth")])

        self.assertThat(pyc_files, Equals([]))
        self.assertThat(pth_files, Equals([]))

    def test_build_doesnt_get_bad_install_directory_lp1586546(self):
        """Verify that LP: #1586546 doesn't come back."""
        self.run_snapcraft("stage", "python-pyyaml")
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python2",
                    "install",
                    "lib",
                    "python2*",
                    "site-packages",
                    "yaml",
                )
            )[0],
            DirExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python3",
                    "install",
                    "lib",
                    "python3*",
                    "site-packages",
                    "yaml",
                )
            )[0],
            DirExists(),
        )

    def test_pypi_package_dep_satisfied_by_stage_package(self):
        """yamllint depends on yaml which is a stage-package."""
        self.run_snapcraft("stage", "python-with-stage-packages")
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python2",
                    "install",
                    "lib",
                    "python2*",
                    "site-packages",
                    "yamllint",
                )
            )[0],
            DirExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python2",
                    "install",
                    "usr",
                    "lib",
                    "python2*",
                    "dist-packages",
                    "yaml",
                )
            )[0],
            DirExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python2",
                    "install",
                    "lib",
                    "python2*",
                    "site-packages",
                    "yaml",
                )
            ),
            Equals([]),
        )

        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python3",
                    "install",
                    "lib",
                    "python3*",
                    "site-packages",
                    "yamllint",
                )
            )[0],
            DirExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python3",
                    "install",
                    "usr",
                    "lib",
                    "python3*",
                    "dist-packages",
                    "yaml",
                )
            )[0],
            DirExists(),
        )
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "python3",
                    "install",
                    "lib",
                    "python3*",
                    "site-packages",
                    "yaml",
                )
            ),
            Equals([]),
        )

    def test_pull_a_package_from_bzr(self):
        self.run_snapcraft("pull", "pip-bzr")
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir, "pip-bzr", "python-packages", "bzrtest-*.zip"
                )
            )[0],
            FileExists(),
        )

    def test_build_with_data_files_with_root(self):
        self.run_snapcraft("build", "pip-root-data-files")
        self.assertThat(
            glob(
                os.path.join(
                    self.parts_dir,
                    "root",
                    "install",
                    "lib",
                    "python3*",
                    "site-packages",
                    "etc",
                    "broken.txt",
                )
            )[0],
            FileExists(),
        )

    def test_stage_with_two_python_parts(self):
        # Regression test for https://bugs.launchpad.net/snapcraft/+bug/1663739
        self.run_snapcraft("stage", "python-with-two-parts")
        # If there is a conflict, stage will raise an exception.

    def test_use_of_source_subdir_picks_up_setup_py(self):
        # Regression test for LP: #1752481
        self.run_snapcraft("stage", "python-with-source-subdir")

        self.assertThat(os.path.join(self.stage_dir, "bin", "hello"), FileExists())

    def test_plugin_plays_nice_with_stage_packages(self):
        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part(
            "python-from-archive",
            {"plugin": "nil", "source": ".", "stage-packages": ["python3-yaml"]},
        )
        snapcraft_yaml.update_part(
            "python-plugin",
            {"plugin": "python", "source": ".", "python-packages": ["bitstring"]},
        )
        self.useFixture(snapcraft_yaml)

        try:
            self.run_snapcraft("stage")
        except subprocess.CalledProcessError:
            self.fail("These parts should not have conflicted")

    def test_python_part_can_be_cleaned(self):
        # Regression test for LP: #1778716

        snapcraft_yaml = fixture_setup.SnapcraftYaml(self.path)
        snapcraft_yaml.update_part("python-part", {"plugin": "python", "source": "."})
        snapcraft_yaml.update_part("dummy-part", {"plugin": "nil"})
        self.useFixture(snapcraft_yaml)

        # First stage both parts
        self.run_snapcraft("stage")

        # Now clean the python one, which should succeed
        try:
            self.run_snapcraft(["clean", "python-part"])
        except subprocess.CalledProcessError:
            self.fail("Expected python part to clean successfully")
