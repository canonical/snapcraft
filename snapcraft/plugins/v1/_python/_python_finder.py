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
import glob

from . import errors


def get_python_command(python_major_version, *, stage_dir, install_dir):
    """Find the python command to use, preferring staged over the part.

    We prefer the staged python as opposed to the in-part python in order to
    support one part that supplies python, with another part built `after` it
    wanting to use its python.

    :param str python_major_version: The python major version to find (2 or 3)
    :param str stage_dir: Path to the staging area
    :param str install_dir: Path to the part's install area

    :return: Path to the python command that was found
    :rtype: str

    :raises MissingPythonCommandError: If no python could be found in the
                                       staging or part's install area.
    """
    python_command_name = "python{}".format(python_major_version)
    python_command = os.path.join("usr", "bin", python_command_name)
    staged_python = os.path.join(stage_dir, python_command)
    part_python = os.path.join(install_dir, python_command)

    if os.path.exists(staged_python):
        return staged_python
    elif os.path.exists(part_python):
        return part_python
    else:
        raise errors.MissingPythonCommandError(
            python_command_name, [stage_dir, install_dir]
        )


def get_python_headers(python_major_version, *, stage_dir):
    """Find the python headers to use, if any, preferring staged over the host.

    We want to make sure we use the headers from the staging area if available,
    or we may end up building for an older python version than the one we
    actually want to use.

    :param str python_major_version: The python version to find (2 or 3)
    :param str stage_dir: Path to the staging area

    :return: Path to the python headers that were found ('' if none)
    :rtype: str
    """
    python_command_name = "python{}".format(python_major_version)
    base_match = os.path.join("usr", "include", "{}*".format(python_command_name))
    staged_python = glob.glob(os.path.join(stage_dir, base_match))
    host_python = glob.glob(os.path.join(os.path.sep, base_match))

    if staged_python:
        return staged_python[0]
    elif host_python:
        return host_python[0]
    else:
        return ""


def get_python_home(python_major_version, *, stage_dir, install_dir):
    """Find the correct PYTHONHOME, preferring staged over the part.

    We prefer the staged python as opposed to the in-part python in order to
    support one part that supplies python, with another part built `after` it
    wanting to use its python.

    :param str python_major_version: The python major version to find (2 or 3)
    :param str stage_dir: Path to the staging area
    :param str install_dir: Path to the part's install area

    :return: Path to the PYTHONHOME that was found
    :rtype: str

    :raises MissingPythonCommandError: If no python could be found in the
                                       staging or part's install area.
    """
    python_command = get_python_command(
        python_major_version, stage_dir=stage_dir, install_dir=install_dir
    )
    if python_command.startswith(stage_dir):
        return os.path.join(stage_dir, "usr")
    else:
        return os.path.join(install_dir, "usr")
