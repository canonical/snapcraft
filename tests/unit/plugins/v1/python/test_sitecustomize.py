# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017,2020 Canonical Ltd
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
from textwrap import dedent

from testtools.matchers import Contains, FileContains

from snapcraft.plugins.v1 import _python

from ._basesuite import PythonBaseTestCase


def _create_site_py(base_dir):
    site_py = os.path.join(base_dir, "usr", "lib", "pythontest", "site.py")
    os.makedirs(os.path.dirname(site_py))
    open(site_py, "w").close()


def _create_user_site_packages(base_dir):
    user_site_dir = os.path.join(base_dir, "lib", "pythontest", "site-packages")
    os.makedirs(user_site_dir)


class SiteCustomizeTestCase(PythonBaseTestCase):
    def setUp(self):
        super().setUp()

        self.expected_sitecustomize = dedent(
            """\
            import site
            import os

            snap_dir = os.getenv("SNAP")
            snapcraft_stage_dir = os.getenv("SNAPCRAFT_STAGE")
            snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")

            # Do not include snap_dir during builds as this will include
            # snapcraft's in-snap site directory.
            if snapcraft_stage_dir is not None and snapcraft_part_install is not None:
                site_directories = [snapcraft_stage_dir, snapcraft_part_install]
            else:
                site_directories = [snap_dir]

            for d in site_directories:
                if d:
                    site_dir = os.path.join(d, "lib/pythontest/site-packages")
                    site.addsitedir(site_dir)

            if snap_dir:
                site.ENABLE_USER_SITE = False"""
        )

    def test_generate_sitecustomize_staged(self):
        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create the python binary in the staging area
        self._create_python_binary(stage_dir)

        # Create a site.py in both staging and install areas
        _create_site_py(stage_dir)
        _create_site_py(install_dir)

        # Create a user site dir in install area
        _create_user_site_packages(install_dir)

        _python.generate_sitecustomize(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )

        site_path = os.path.join(
            install_dir, "usr", "lib", "pythontest", "sitecustomize.py"
        )
        self.assertThat(site_path, FileContains(self.expected_sitecustomize))

    def test_generate_sitecustomize_installed(self):
        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create the python binary in the installed area
        self._create_python_binary(install_dir)

        # Create a site.py in both staging and install areas
        _create_site_py(stage_dir)
        _create_site_py(install_dir)

        # Create a user site dir in install area
        _create_user_site_packages(install_dir)

        _python.generate_sitecustomize(
            "test", stage_dir=stage_dir, install_dir=install_dir
        )

        site_path = os.path.join(
            install_dir, "usr", "lib", "pythontest", "sitecustomize.py"
        )
        self.assertThat(site_path, FileContains(self.expected_sitecustomize))

    def test_generate_sitecustomize_missing_user_site_raises(self):
        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create the python binary in the installed area
        self._create_python_binary(install_dir)

        # Create a site.py in both staging and install areas
        _create_site_py(stage_dir)
        _create_site_py(install_dir)

        # Do NOT create a user site dir, and attempt to generate sitecustomize.
        raised = self.assertRaises(
            _python.errors.MissingUserSitePackagesError,
            _python.generate_sitecustomize,
            "test",
            stage_dir=stage_dir,
            install_dir=install_dir,
        )
        self.assertThat(str(raised), Contains("Unable to find user site packages"))

    def test_generate_sitecustomize_missing_site_py_raises(self):
        stage_dir = "stage_dir"
        install_dir = "install_dir"

        # Create the python binary in the staging area
        self._create_python_binary(stage_dir)

        # Create a site.py, but only in install area (not staging area)
        _create_site_py(install_dir)

        # Create a user site dir in install area
        _create_user_site_packages(install_dir)

        raised = self.assertRaises(
            _python.errors.MissingSitePyError,
            _python.generate_sitecustomize,
            "test",
            stage_dir=stage_dir,
            install_dir=install_dir,
        )
        self.assertThat(str(raised), Contains("Unable to find site.py"))
