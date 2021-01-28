# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

import logging
from collections import ChainMap
from os import path
from typing import Set  # noqa: F401
from typing import List

import snapcraft
from snapcraft.internal import elf, pluginhandler, repo
from snapcraft.internal.pluginhandler._part_environment import (
    get_snapcraft_global_environment,
    get_snapcraft_part_directory_environment,
)

from . import errors, grammar_processing
from ._env import build_env, build_env_for_stage, runtime_env

logger = logging.getLogger(__name__)


class PartsConfig:
    def __init__(self, *, parts, project, validator):
        self._soname_cache = elf.SonameCache()
        self._parts_data = parts.get("parts", {})
        self._snap_type = parts.get("type", "app")
        self._project = project
        self._validator = validator

        self.all_parts = []
        self._part_names = []
        self.after_requests = {}

        self._process_parts()

    @property
    def part_names(self):
        return self._part_names

    def _process_parts(self):
        for part_name in self._parts_data:
            self._part_names.append(part_name)
            properties = self._parts_data[part_name] or {}

            plugin_name = properties.get("plugin")

            if "after" in properties:
                self.after_requests[part_name] = properties.pop("after")

            if "filesets" in properties:
                del properties["filesets"]

            self.load_part(part_name, plugin_name, properties)

        self._compute_dependencies()
        self.all_parts = self._sort_parts()

    def _compute_dependencies(self):
        """Gather the lists of dependencies and adds to all_parts."""

        for part in self.all_parts:
            dep_names = self.after_requests.get(part.name, [])
            for dep_name in dep_names:
                dep = self.get_part(dep_name)
                if not dep:
                    raise errors.SnapcraftAfterPartMissingError(part.name, dep_name)

                part.deps.append(dep)

    def _sort_parts(self):
        """Performs an inneficient but easy to follow sorting of parts."""
        sorted_parts = []

        # We want to process parts in a consistent order between runs. The
        # simplest way to do this is to sort them by name.
        self.all_parts = sorted(
            self.all_parts, key=lambda part: part.name, reverse=True
        )

        while self.all_parts:
            top_part = None
            for part in self.all_parts:
                mentioned = False
                for other in self.all_parts:
                    if part in other.deps:
                        mentioned = True
                        break
                if not mentioned:
                    top_part = part
                    break
            if not top_part:
                raise errors.SnapcraftLogicError(
                    "circular dependency chain found in parts definition"
                )
            sorted_parts = [top_part] + sorted_parts
            self.all_parts.remove(top_part)

        return sorted_parts

    def get_dependencies(
        self, part_name: str, *, recursive: bool = False
    ) -> Set[pluginhandler.PluginHandler]:
        """Returns a set of all the parts upon which part_name depends."""

        dependency_names = set(self.after_requests.get(part_name, []))
        dependencies = {p for p in self.all_parts if p.name in dependency_names}

        if recursive:
            # No need to worry about infinite recursion due to circular
            # dependencies since the YAML validation won't allow it.
            for dependency_name in dependency_names:
                dependencies |= self.get_dependencies(
                    dependency_name, recursive=recursive
                )

        return dependencies

    def get_reverse_dependencies(
        self, part_name: str, *, recursive: bool = False
    ) -> Set[pluginhandler.PluginHandler]:
        """Returns a set of all the parts that depend upon part_name."""

        reverse_dependency_names = set()
        for part, dependencies in self.after_requests.items():
            if part_name in dependencies:
                reverse_dependency_names.add(part)

        reverse_dependencies = {
            p for p in self.all_parts if p.name in reverse_dependency_names
        }

        if recursive:
            # No need to worry about infinite recursion due to circular
            # dependencies since the YAML validation won't allow it.
            for reverse_dependency_name in reverse_dependency_names:
                reverse_dependencies |= self.get_reverse_dependencies(
                    reverse_dependency_name, recursive=recursive
                )

        return reverse_dependencies

    def get_part(self, part_name):
        for part in self.all_parts:
            if part.name == part_name:
                return part

        return None

    def clean_part(self, part_name, staged_state, primed_state, step):
        part = self.get_part(part_name)
        part.clean(staged_state, primed_state, step)

    def validate(self, part_names):
        for part_name in part_names:
            if part_name not in self._part_names:
                raise snapcraft.internal.errors.SnapcraftEnvironmentError(
                    "The part named {!r} is not defined in "
                    "{!r}".format(
                        part_name, self._project.info.snapcraft_yaml_file_path
                    )
                )

    def load_part(self, part_name, plugin_name, part_properties):
        plugin = pluginhandler.load_plugin(
            plugin_name=plugin_name,
            part_name=part_name,
            properties=part_properties,
            project=self._project,
            part_schema=self._validator.part_schema,
            definitions_schema=self._validator.definitions_schema,
        )

        logger.debug(
            "Setting up part {!r} with plugin {!r} and "
            "properties {!r}.".format(part_name, plugin_name, part_properties)
        )

        stage_packages_repo = repo.Repo

        grammar_processor = grammar_processing.PartGrammarProcessor(
            plugin=plugin,
            properties=part_properties,
            project=self._project,
            repo=stage_packages_repo,
        )

        part = pluginhandler.PluginHandler(
            plugin=plugin,
            part_properties=part_properties,
            project=self._project,
            part_schema=self._validator.part_schema,
            definitions_schema=self._validator.definitions_schema,
            stage_packages_repo=stage_packages_repo,
            grammar_processor=grammar_processor,
            snap_base_path=path.join("/", "snap", self._project.info.name, "current"),
            soname_cache=self._soname_cache,
        )

        self.all_parts.append(part)

        return part

    def build_env_for_part(self, part, root_part=True) -> List[str]:
        """Return a build env of all the part's dependencies."""

        env = []  # type: List[str]
        stagedir = self._project.stage_dir

        if root_part:
            # this has to come before any {}/usr/bin
            env += part.env(part.part_install_dir)
            env += runtime_env(part.part_install_dir, self._project.arch_triplet)
            env += runtime_env(stagedir, self._project.arch_triplet)
            env += build_env(
                part.part_install_dir,
                self._project.info.name,
                self._project.arch_triplet,
            )
            env += build_env_for_stage(
                stagedir, self._project.info.name, self._project.arch_triplet
            )

            global_env = get_snapcraft_global_environment(self._project)
            part_env = get_snapcraft_part_directory_environment(part)

            for variable, value in ChainMap(part_env, global_env).items():
                env.append('{}="{}"'.format(variable, value))

            # Finally, add the declared environment from the part.
            # This is done only for the "root" part.
            for be in part.build_environment:
                env.extend([f'{k}="{v}"' for k, v in be.items()])
        else:
            env += part.env(stagedir)
            env += runtime_env(stagedir, self._project.arch_triplet)

        for dep_part in part.deps:
            env += dep_part.env(stagedir)
            env += self.build_env_for_part(dep_part, root_part=False)

        # LP: #1767625
        # Remove duplicates from using the same plugin in dependent parts.
        seen = set()  # type: Set[str]
        deduped_env = list()  # type: List[str]
        for e in env:
            if e not in seen:
                deduped_env.append(e)
                seen.add(e)

        return deduped_env
