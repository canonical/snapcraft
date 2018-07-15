# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

from testtools.matchers import Contains, FileContains

from snapcraft.plugins import _python

from ._basesuite import PythonBaseTestCase


def _create_site_py(base_dir):
    site_py = os.path.join(base_dir, "usr", "lib", "pythontest", "site.py")
    os.makedirs(os.path.dirname(site_py))
    open(site_py, "w").close()


def _create_user_site_packages(base_dir):
    user_site_dir = os.path.join(base_dir, "lib", "pythontest", "site-packages")
    os.makedirs(user_site_dir)


class SiteCustomizeTestCase(PythonBaseTestCase):
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

        expected_sitecustomize = (
            "import site\n"
            "import os\n"
            "\n"
            'snap_dir = os.getenv("SNAP")\n'
            'snapcraft_stage_dir = os.getenv("SNAPCRAFT_STAGE")\n'
            'snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")\n'
            "\n"
            "for d in (snap_dir, snapcraft_stage_dir, "
            "snapcraft_part_install):\n"
            "    if d:\n"
            "        site_dir = os.path.join(d, "
            '"lib/pythontest/site-packages")\n'
            "        site.addsitedir(site_dir)\n"
            "\n"
            "if snap_dir:\n"
            "    site.ENABLE_USER_SITE = False"
        )

        site_path = os.path.join(
            install_dir, "usr", "lib", "pythontest", "sitecustomize.py"
        )
        self.assertThat(site_path, FileContains(expected_sitecustomize))

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

        expected_sitecustomize = (
            "import site\n"
            "import os\n"
            "\n"
            'snap_dir = os.getenv("SNAP")\n'
            'snapcraft_stage_dir = os.getenv("SNAPCRAFT_STAGE")\n'
            'snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")\n'
            "\n"
            "for d in (snap_dir, snapcraft_stage_dir, "
            "snapcraft_part_install):\n"
            "    if d:\n"
            "        site_dir = os.path.join(d, "
            '"lib/pythontest/site-packages")\n'
            "        site.addsitedir(site_dir)\n"
            "\n"
            "if snap_dir:\n"
            "    site.ENABLE_USER_SITE = False"
        )

        site_path = os.path.join(
            install_dir, "usr", "lib", "pythontest", "sitecustomize.py"
        )
        self.assertThat(site_path, FileContains(expected_sitecustomize))

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
