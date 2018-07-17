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

from collections import ChainMap
import logging
from os import path
from typing import List
from typing import Set  # noqa: F401

import snapcraft
from snapcraft.internal import deprecations, elf, pluginhandler, repo
from ._env import (
    env_for_classic,
    build_env,
    build_env_for_stage,
    runtime_env,
    snapcraft_global_environment,
    snapcraft_part_environment,
)
from . import errors, grammar_processing

logger = logging.getLogger(__name__)


class PartsConfig:
    def __init__(self, *, parts, project, validator, build_snaps, build_tools):
        self._snap_name = parts["name"]
        self._base = parts.get("base", "core")
        self._confinement = parts.get("confinement")
        self._soname_cache = elf.SonameCache()
        self._parts_data = parts.get("parts", {})
        self._snap_type = parts.get("type", "app")
        self._project = project
        self._validator = validator
        self.build_snaps = build_snaps
        self.build_tools = build_tools

        self.all_parts = []
        self._part_names = []
        self.after_requests = {}

        self._process_parts()

    @property
    def part_names(self):
        return self._part_names

    def _process_parts(self):
        for part_name in self._parts_data:
            if "/" in part_name:
                logger.warning(
                    'DEPRECATED: Found a "/" '
                    "in the name of the {!r} part".format(part_name)
                )
            self._part_names.append(part_name)
            properties = self._parts_data[part_name] or {}

            plugin_name = properties.get("plugin")

            if "after" in properties:
                self.after_requests[part_name] = properties.pop("after")

            if "filesets" in properties:
                del properties["filesets"]

            # Handle the deprecated snap keyword.
            if "snap" in properties:
                snap = properties.pop("snap")
                if snap:
                    deprecations.handle_deprecation_notice("dn1")
                    properties["prime"] = snap

            self.load_part(part_name, plugin_name, properties)

        self._compute_dependencies()
        self.all_parts = self._sort_parts()

    def _compute_dependencies(self):
        """Gather the lists of dependencies and adds to all_parts."""

        for part in self.all_parts:
            dep_names = self.after_requests.get(part.name, [])
            for dep in dep_names:
                for i in range(len(self.all_parts)):
                    if dep == self.all_parts[i].name:
                        part.deps.append(self.all_parts[i])
                        break

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

    def get_dependencies(self, part_name, *, recursive=False):
        # type: (str, bool) -> Set[pluginhandler.PluginHandler]
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

    def get_reverse_dependencies(self, part_name, *, recursive=False):
        # type: (str, bool) -> Set[pluginhandler.PluginHandler]
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
        # Some legacy parts can have a '/' in them to separate the main project
        # part with the subparts. This is rather unfortunate as it affects the
        # the layout of parts inside the parts directory causing collisions
        # between the main project part and its subparts.
        part_name = part_name.replace("/", "\N{BIG SOLIDUS}")

        plugin = pluginhandler.load_plugin(
            plugin_name=plugin_name,
            part_name=part_name,
            properties=part_properties,
            project_options=self._project,
            part_schema=self._validator.part_schema,
            definitions_schema=self._validator.definitions_schema,
        )

        logger.debug(
            "Setting up part {!r} with plugin {!r} and "
            "properties {!r}.".format(part_name, plugin_name, part_properties)
        )

        sources = getattr(plugin, "PLUGIN_STAGE_SOURCES", None)
        stage_packages_repo = repo.Repo(
            plugin.osrepodir, sources=sources, project_options=self._project
        )

        grammar_processor = grammar_processing.PartGrammarProcessor(
            plugin=plugin,
            properties=part_properties,
            project=self._project,
            repo=stage_packages_repo,
        )

        part = pluginhandler.PluginHandler(
            plugin=plugin,
            part_properties=part_properties,
            project_options=self._project,
            part_schema=self._validator.part_schema,
            definitions_schema=self._validator.definitions_schema,
            stage_packages_repo=stage_packages_repo,
            grammar_processor=grammar_processor,
            snap_base_path=path.join("/", "snap", self._snap_name, "current"),
            base=self._base,
            confinement=self._confinement,
            snap_type=self._snap_type,
            soname_cache=self._soname_cache,
        )

        self.build_snaps |= grammar_processor.get_build_snaps()
        self.build_tools |= grammar_processor.get_build_packages()

        # TODO: this should not pass in command but the required package,
        #       where the required package is to be determined by the
        #       source handler.
        if part.source_handler and part.source_handler.command:
            # TODO get_packages_for_source_type should not be a thing.
            self.build_tools |= repo.Repo.get_packages_for_source_type(
                part.source_handler.command
            )
        self.all_parts.append(part)

        return part

    def build_env_for_part(self, part, root_part=True) -> List[str]:
        """Return a build env of all the part's dependencies."""

        env = []  # type: List[str]
        stagedir = self._project.stage_dir
        is_host_compat = self._project.is_host_compatible_with_base(self._base)

        if root_part:
            # this has to come before any {}/usr/bin
            env += part.env(part.plugin.installdir)
            env += runtime_env(part.plugin.installdir, self._project.arch_triplet)
            env += runtime_env(stagedir, self._project.arch_triplet)
            env += build_env(
                part.plugin.installdir, self._snap_name, self._project.arch_triplet
            )
            env += build_env_for_stage(
                stagedir, self._snap_name, self._project.arch_triplet
            )
            # Only set the paths to the base snap if we are building on the
            # same host. Failing to do so will cause Segmentation Faults.
            if self._confinement == "classic" and is_host_compat:
                env += env_for_classic(self._base, self._project.arch_triplet)

            global_env = snapcraft_global_environment(self._project)
            part_env = snapcraft_part_environment(part)
            for variable, value in ChainMap(part_env, global_env).items():
                env.append('{}="{}"'.format(variable, value))
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
