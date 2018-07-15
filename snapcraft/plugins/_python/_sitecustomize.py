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

import contextlib
import glob
import os
from textwrap import dedent

from ._python_finder import get_python_command
from . import errors

_SITECUSTOMIZE_TEMPLATE = dedent(
    """\
    import site
    import os

    snap_dir = os.getenv("SNAP")
    snapcraft_stage_dir = os.getenv("SNAPCRAFT_STAGE")
    snapcraft_part_install = os.getenv("SNAPCRAFT_PART_INSTALL")

    for d in (snap_dir, snapcraft_stage_dir, snapcraft_part_install):
        if d:
            site_dir = os.path.join(d, "{site_dir}")
            site.addsitedir(site_dir)

    if snap_dir:
        site.ENABLE_USER_SITE = False"""
)


def _get_user_site_dir(python_major_version, *, install_dir):
    path_glob = os.path.join(
        install_dir, "lib", "python{}*".format(python_major_version), "site-packages"
    )
    user_site_dirs = glob.glob(path_glob)
    if not user_site_dirs:
        raise errors.MissingUserSitePackagesError(path_glob)

    return user_site_dirs[0][len(install_dir) + 1 :]


def _get_sitecustomize_path(python_major_version, *, stage_dir, install_dir):
    # Use the part's install_dir unless there's a python in the staging area
    base_dir = install_dir
    with contextlib.suppress(errors.MissingPythonCommandError):
        python_command = get_python_command(
            python_major_version, stage_dir=stage_dir, install_dir=install_dir
        )
        if python_command.startswith(stage_dir):
            base_dir = stage_dir

    site_py_glob = os.path.join(
        base_dir, "usr", "lib", "python{}*".format(python_major_version), "site.py"
    )
    python_sites = glob.glob(site_py_glob)
    if not python_sites:
        raise errors.MissingSitePyError(site_py_glob)

    python_site_dir = os.path.dirname(python_sites[0])

    return os.path.join(
        install_dir, python_site_dir[len(base_dir) + 1 :], "sitecustomize.py"
    )


def generate_sitecustomize(python_major_version, *, stage_dir, install_dir):
    """Generate a sitecustomize.py to look in staging, part install, and snap.

    This is done by checking the values of the environment variables $SNAP,
    $SNAPCRAFT_STAGE, and $SNAPCRAFT_PART_INSTALL. As a result, the same
    sitecustomize.py works to find packages at both build- and run-time.

    :param str python_major_version: The python major version to use (2 or 3)
    :param str stage_dir: Path to the staging area
    :param str install_dir: Path to the part's install area

    :raises MissingUserSitePackagesError: If no user site packages are found in
                                          install_dir/lib/pythonX*
    :raises MissingSitePyError: If no site.py can be found in either the
                                staging area or the part install area.
    """
    sitecustomize_path = _get_sitecustomize_path(
        python_major_version, stage_dir=stage_dir, install_dir=install_dir
    )
    os.makedirs(os.path.dirname(sitecustomize_path), exist_ok=True)

    # There may very well already be a sitecustomize.py already there. If so,
    # get rid of it. Is may be a symlink to another sitecustomize.py, in which
    # case, we'll get rid of that one as well.
    if os.path.islink(sitecustomize_path):
        target_path = os.path.realpath(sitecustomize_path)

        # Only remove the target if it's contained within the install directory
        if target_path.startswith(os.path.abspath(install_dir) + os.sep):
            with contextlib.suppress(FileNotFoundError):
                os.remove(target_path)

    with contextlib.suppress(FileNotFoundError):
        os.remove(sitecustomize_path)

    # Create our sitecustomize. Python from the archives already has one
    # which is distro-specific and not needed here, so we truncate it if it's
    # already there.
    with open(sitecustomize_path, "w") as f:
        f.write(
            _SITECUSTOMIZE_TEMPLATE.format(
                site_dir=_get_user_site_dir(
                    python_major_version, install_dir=install_dir
                )
            )
        )
