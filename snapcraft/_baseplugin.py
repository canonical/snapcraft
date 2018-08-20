# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

import collections
import contextlib
import logging
import os
from subprocess import CalledProcessError
from typing import List

from snapcraft.internal import common, errors


logger = logging.getLogger(__name__)


class BasePlugin:
    @classmethod
    def schema(cls):
        """Return a json-schema for the plugin's properties as a dictionary.
        Of importance to plugin authors is the 'properties' keyword and
        optionally the 'requires' keyword with a list of required
        'properties'.

        By default the properties will be that of a standard VCS,
        override in custom implementations if required.
        """
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {},
            "pull-properties": [],
            "build-properties": [],
            "required": [],
        }

    @classmethod
    def get_pull_properties(cls):
        schema_pull_properties = cls.schema().get("pull-properties", [])
        if schema_pull_properties:
            logger.warning(
                "Use of pull-properties in the schema is deprecated.\n"
                "Plugins should now implement get_pull_properties"
            )
            return schema_pull_properties

        return []

    @classmethod
    def get_build_properties(cls):
        schema_build_properties = cls.schema().get("build-properties", [])
        if schema_build_properties:
            logger.warning(
                "Use of build-properties in the schema is deprecated.\n"
                "Plugins should now implement get_build_properties"
            )
            return schema_build_properties

        return []

    @property
    def PLUGIN_STAGE_SOURCES(self):
        """Define alternative sources.list."""
        return getattr(self, "_PLUGIN_STAGE_SOURCES", [])

    @property
    def stage_packages(self):
        return self._stage_packages

    @stage_packages.setter
    def stage_packages(self, value):
        self._stage_packages = value

    def __init__(self, name, options, project=None):
        self.name = name
        self.build_snaps = []
        self.build_packages = []
        self._stage_packages = []
        self.__dependencies = []  # type: List[BasePlugin]

        with contextlib.suppress(AttributeError):
            self._stage_packages = options.stage_packages.copy()
        with contextlib.suppress(AttributeError):
            self.build_packages = options.build_packages.copy()
        with contextlib.suppress(AttributeError):
            self.build_snaps = options.build_snaps.copy()

        self.project = project
        self.options = options

        # The remote parts can have a '/' in them to separate the main project
        # part with the subparts. This is rather unfortunate as it affects the
        # the layout of parts inside the parts directory causing collisions
        # between the main project part and its subparts.
        part_dir = name.replace("/", "\N{BIG SOLIDUS}")
        if project:
            self.partdir = os.path.join(project.parts_dir, part_dir)
        else:
            self.partdir = os.path.join(os.getcwd(), "parts", part_dir)

        self.sourcedir = os.path.join(self.partdir, "src")
        self.installdir = os.path.join(self.partdir, "install")
        self.statedir = os.path.join(self.partdir, "state")
        self.osrepodir = os.path.join(self.partdir, "ubuntu")

        self.build_basedir = os.path.join(self.partdir, "build")
        source_subdir = getattr(self.options, "source_subdir", None)
        if source_subdir:
            self.builddir = os.path.join(self.build_basedir, source_subdir)
        else:
            self.builddir = self.build_basedir

        # By default, snapcraft does an in-source build. Set this property to
        # True if that's not desired.
        self.out_of_source_build = False

    def add_dependency(self, plugin: "BasePlugin") -> None:
        self.__dependencies.append(plugin)

    def get_dependencies(self) -> List["BasePlugin"]:
        return self.__dependencies.copy()

    # The API
    def pull(self):
        """Pull the source code and/or internal prereqs to build the part."""
        pass

    def clean_pull(self):
        """Clean the pulled source for this part."""
        pass

    def build(self):
        """Build the source code retrieved from the pull phase."""
        pass

    def clean_build(self):
        """Clean the artifacts that resulted from building this part."""
        pass

    def get_manifest(self):
        """Return the information to record after the build of this part.

        :rtype: dict
        """
        pass

    def snap_fileset(self):
        """Return a list of files to include or exclude in the resulting snap

        The staging phase of a plugin's lifecycle may populate many things
        into the staging directory in order to succeed in building a
        project.
        During the stripping phase and in order to have a clean snap, the
        plugin can provide additional logic for stripping build components
        from the final snap and alleviate the part author from doing so for
        repetetive filesets.

        These are the rules to honor when creating such list:

            - includes can be just listed
            - excludes must be preceded by -

        For example::
            (['bin', 'lib', '-include'])
        """
        return []

    def env(self, root: str) -> List[str]:
        """Return a list with the execution environment for building.

        Plugins often need special environment variables exported to the system for some
        builds to take place. This is a list of strings of the form key=value. The
        parameter root is the path to this part. More fine-grained control can be had
        by implementing get_pull_env(), get_pull_command_chain(), and their family of
        functions.

        :param str root: The root for the part
        """
        return []

    def get_pull_env(self) -> "collections.OrderedDict[str, str]":
        """Return a dict of environment variable names and values for pulling.

        By default this function simply calls env() with the install directory as the
        root.
        """
        # Maintain backward compatibility with the old-style env() function
        return _env_list_to_ordered_dict(self.env(self.installdir))

    def get_pull_command_chain(self) -> List[str]:
        """Return a list of commands to be prepended to the commands when pulling.

        Each item must either be in the PATH, or be an absolute path to an executable."""
        return list()

    def get_build_env(self) -> "collections.OrderedDict[str, str]":
        """Return a dict of environment variable names and values for building.

        By default this function simply calls env() with the install directory as the
        root."""
        # Maintain backward compatibility with the old-style env() function
        return _env_list_to_ordered_dict(self.env(self.installdir))

    def get_build_command_chain(self) -> List[str]:
        """Return a list of commands to be prepended to the commands when building.

        Each item must either be in the PATH, or be an absolute path to an executable."""
        return list()

    def get_stage_env(self) -> "collections.OrderedDict[str, str]":
        """Return a dict of environment variable names and values for staging.

        By default this function simply calls env() with the stage directory as the
        root."""
        # Maintain backward compatibility with the old-style env() function
        return _env_list_to_ordered_dict(self.env(self.project.stage_dir))

    def get_stage_command_chain(self) -> List[str]:
        """Return a list of commands to be prepended to the commands when staging.

        Each item must either be in the PATH, or be an absolute path to an executable."""
        return list()

    def get_prime_env(self) -> "collections.OrderedDict[str, str]":
        """Return a dict of environment variable names and values for priming.

        By default this function simply calls env() with the prime directory as the
        root."""
        # Maintain backward compatibility with the old-style env() function
        return _env_list_to_ordered_dict(self.env(self.project.prime_dir))

    def get_prime_command_chain(self) -> List[str]:
        """Return a list of commands to be prepended to the commands when priming.

        Each item must either be in the PATH, or be an absolute path to an executable."""
        return list()

    def get_dependency_env(self) -> "collections.OrderedDict[str, str]":
        """Return a dict of environment variable names and values for use by dependents."""
        return self.get_stage_env()

    def get_dependency_command_chain(self) -> List[str]:
        """Return a list of commands to be prepended to the actual commands used by dependents.

        Each item must either be in the PATH, or be an absolute path to an executable.
        """
        return list()

    def get_snap_env(self) -> "collections.OrderedDict[str, str]":
        """Return a dict of environment variable names and values for apps in the final snap.

        By default this function simply calls get_prime_env() and replaces all
        occurrences of the prime directory with $SNAP."""
        env = self.get_prime_env()
        for variable_name, value in env.items():
            env[variable_name] = value.replace(self.project.prime_dir, "$SNAP")

        return env

    def get_snap_command_chain(self) -> List[str]:
        """Return a list of commands to be prepended to the apps in the final snap.

        The paths returned should all be relative paths to the root of the snap, and
        should not include any environment variables.
        """
        return list()

    def enable_cross_compilation(self):
        """Enable cross compilation for the plugin."""
        raise NotImplementedError(
            "The plugin used by {!r} does not support cross-compiling "
            "to a different target architecture".format(self.name)
        )

    @property
    def parallel_build_count(self):
        """Number of CPU's to use for building.

        Number comes from `project.parallel_build_count` unless the part
        has defined `disable-parallel` as `True`.
        """
        if getattr(self.options, "disable_parallel", False):
            return 1
        else:
            return self.project.parallel_build_count

    # Helpers
    def run(self, cmd, cwd=None, **kwargs):
        if not cwd:
            cwd = self.builddir
        print(" ".join(cmd))
        os.makedirs(cwd, exist_ok=True)
        try:
            return common.run(cmd, cwd=cwd, **kwargs)
        except CalledProcessError as process_error:
            raise errors.SnapcraftPluginCommandError(
                command=cmd, part_name=self.name, exit_code=process_error.returncode
            ) from process_error

    def run_output(self, cmd, cwd=None, **kwargs):
        if not cwd:
            cwd = self.builddir
        os.makedirs(cwd, exist_ok=True)
        try:
            return common.run_output(cmd, cwd=cwd, **kwargs)
        except CalledProcessError as process_error:
            raise errors.SnapcraftPluginCommandError(
                command=cmd, part_name=self.name, exit_code=process_error.returncode
            ) from process_error


def _env_list_to_ordered_dict(
    env_list: List[str]
) -> "collections.OrderedDict[str, str]":
    env = collections.OrderedDict()  # type: collections.OrderedDict[str, str]
    for item in env_list:
        variable_name, value = item.split("=", maxsplit=1)
        env[variable_name] = value
    return env
