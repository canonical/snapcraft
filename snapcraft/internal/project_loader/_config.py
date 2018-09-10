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

import collections
import logging
import os
import os.path
import re

import jsonschema
from typing import Set  # noqa: F401

from snapcraft import project, formatting_utils
from snapcraft.internal import deprecations, remote_parts, states, steps

from ._schema import Validator
from ._parts_config import PartsConfig
from ._extensions import apply_extensions
from ._env import (
    build_env_for_stage,
    runtime_env,
    snapcraft_global_environment,
    environment_to_replacements,
)
from . import errors, grammar_processing, replace_attr


logger = logging.getLogger(__name__)


@jsonschema.FormatChecker.cls_checks("icon-path")
def _validate_icon(instance):
    allowed_extensions = [".png", ".svg"]
    extension = os.path.splitext(instance.lower())[1]
    if extension not in allowed_extensions:
        raise jsonschema.exceptions.ValidationError(
            "'icon' must be either a .png or a .svg"
        )

    if not os.path.exists(instance):
        raise jsonschema.exceptions.ValidationError(
            "Specified icon '{}' does not exist".format(instance)
        )

    return True


@jsonschema.FormatChecker.cls_checks("epoch", raises=errors.InvalidEpochError)
def _validate_epoch(instance):
    str_instance = str(instance)
    pattern = re.compile("^(?:0|[1-9][0-9]*[*]?)$")
    if not pattern.match(str_instance):
        raise errors.InvalidEpochError()

    return True


@jsonschema.FormatChecker.cls_checks("architectures")
def _validate_architectures(instance):
    standalone_build_ons = collections.Counter()
    build_ons = collections.Counter()
    run_ons = collections.Counter()

    saw_strings = False
    saw_dicts = False

    for item in instance:
        # This could either be a dict or a string. In the latter case, the
        # schema will take care of it. We just need to further validate the
        # dict.
        if isinstance(item, str):
            saw_strings = True
        elif isinstance(item, dict):
            saw_dicts = True
            build_on = _get_architectures_set(item, "build-on")
            build_ons.update(build_on)

            # Add to the list of run-ons. However, if no run-on is specified,
            # we know it's implicitly the value of build-on, so use that
            # for validation instead.
            run_on = _get_architectures_set(item, "run-on")
            if run_on:
                run_ons.update(run_on)
            else:
                standalone_build_ons.update(build_on)

    # Architectures can either be a list of strings, or a list of objects.
    # Mixing the two forms is unsupported.
    if saw_strings and saw_dicts:
        raise jsonschema.exceptions.ValidationError(
            "every item must either be a string or an object",
            path=["architectures"],
            instance=instance,
        )

    # At this point, individual build-ons and run-ons have been validated,
    # we just need to validate them across each other.

    # First of all, if we have a `run-on: [all]` (or a standalone
    # `build-on: [all]`) then we should only have one item in the instance,
    # otherwise we know we'll have multiple snaps claiming they run on the same
    # architectures (i.e. all and something else).
    number_of_snaps = len(instance)
    if "all" in run_ons and number_of_snaps > 1:
        raise jsonschema.exceptions.ValidationError(
            "one of the items has 'all' in 'run-on', but there are {} "
            "items: upon release they will conflict. 'all' should only be "
            "used if there is a single item".format(number_of_snaps),
            path=["architectures"],
            instance=instance,
        )
    if "all" in build_ons and number_of_snaps > 1:
        raise jsonschema.exceptions.ValidationError(
            "one of the items has 'all' in 'build-on', but there are {} "
            "items: snapcraft doesn't know which one to use. 'all' should "
            "only be used if there is a single item".format(number_of_snaps),
            path=["architectures"],
            instance=instance,
        )

    # We want to ensure that multiple `run-on`s (or standalone `build-on`s)
    # don't include the same arch, or they'll clash with each other when
    # releasing.
    all_run_ons = run_ons + standalone_build_ons
    duplicates = {arch for (arch, count) in all_run_ons.items() if count > 1}
    if duplicates:
        raise jsonschema.exceptions.ValidationError(
            "multiple items will build snaps that claim to run on {}".format(
                formatting_utils.humanize_list(duplicates, "and")
            ),
            path=["architectures"],
            instance=instance,
        )

    # Finally, ensure that multiple `build-on`s don't include the same arch
    # or Snapcraft has no way of knowing which one to use.
    duplicates = {arch for (arch, count) in build_ons.items() if count > 1}
    if duplicates:
        raise jsonschema.exceptions.ValidationError(
            "{} {} present in the 'build-on' of multiple items, which means "
            "snapcraft doesn't know which 'run-on' to use when building on "
            "{} {}".format(
                formatting_utils.humanize_list(duplicates, "and"),
                formatting_utils.pluralize(duplicates, "is", "are"),
                formatting_utils.pluralize(duplicates, "that", "those"),
                formatting_utils.pluralize(duplicates, "architecture", "architectures"),
            ),
            path=["architectures"],
            instance=instance,
        )

    return True


def _get_architectures_set(item, name):
    value = item.get(name, set())
    if isinstance(value, str):
        value_set = {value}
    else:
        value_set = set(value)

    _validate_architectures_set(value_set, name)

    return value_set


def _validate_architectures_set(architectures_set, name):
    if "all" in architectures_set and len(architectures_set) > 1:
        raise jsonschema.exceptions.ValidationError(
            "'all' can only be used within {!r} by itself, "
            "not with other architectures".format(name),
            path=["architectures"],
            instance=architectures_set,
        )


class Config:
    @property
    def part_names(self):
        return self.parts.part_names

    @property
    def all_parts(self):
        return self.parts.all_parts

    @property
    def _remote_parts(self):
        if getattr(self, "_remote_parts_attr", None) is None:
            self._remote_parts_attr = remote_parts.get_remote_parts()
        return self._remote_parts_attr

    def __init__(self, project: project.Project) -> None:
        self.build_snaps = set()  # type: Set[str]
        self.project = project

        # raw_snapcraft_yaml is read only, create a new copy
        snapcraft_yaml = apply_extensions(project.info.get_raw_snapcraft())

        self.validator = Validator(snapcraft_yaml)
        self.validator.validate()

        snapcraft_yaml = self._process_remote_parts(snapcraft_yaml)
        snapcraft_yaml = self._expand_filesets(snapcraft_yaml)

        self.data = self._expand_env(snapcraft_yaml)
        self._ensure_no_duplicate_app_aliases()

        grammar_processor = grammar_processing.GlobalGrammarProcessor(
            properties=self.data, project=project
        )

        self.build_tools = grammar_processor.get_build_packages()
        self.build_tools |= set(project.additional_build_packages)

        self.parts = PartsConfig(
            parts=self.data,
            project=project,
            validator=self.validator,
            build_snaps=self.build_snaps,
            build_tools=self.build_tools,
        )

        self.data["architectures"] = _process_architectures(
            self.data.get("architectures"), project.deb_arch
        )

    def _ensure_no_duplicate_app_aliases(self):
        # Prevent multiple apps within a snap from having duplicate alias names
        aliases = []
        for app_name, app in self.data.get("apps", {}).items():
            aliases.extend(app.get("aliases", []))

        # The aliases property is actually deprecated:
        if aliases:
            deprecations.handle_deprecation_notice("dn5")
        seen = set()
        duplicates = set()
        for alias in aliases:
            if alias in seen:
                duplicates.add(alias)
            else:
                seen.add(alias)
        if duplicates:
            raise errors.DuplicateAliasError(aliases=duplicates)

    def get_project_state(self, step: steps.Step):
        """Returns a dict of states for the given step of each part."""

        state = {}
        for part in self.parts.all_parts:
            state[part.name] = states.get_state(part.plugin.statedir, step)

        return state

    def stage_env(self):
        stage_dir = self.project.stage_dir
        env = []

        env += runtime_env(stage_dir, self.project.arch_triplet)
        env += build_env_for_stage(
            stage_dir, self.data["name"], self.project.arch_triplet
        )
        for part in self.parts.all_parts:
            env += part.env(stage_dir)

        return env

    def snap_env(self):
        prime_dir = self.project.prime_dir
        env = []

        env += runtime_env(prime_dir, self.project.arch_triplet)
        dependency_paths = set()
        for part in self.parts.all_parts:
            env += part.env(prime_dir)
            dependency_paths |= part.get_primed_dependency_paths()

        # Dependency paths are only valid if they actually exist. Sorting them
        # here as well so the LD_LIBRARY_PATH is consistent between runs.
        dependency_paths = sorted(
            {path for path in dependency_paths if os.path.isdir(path)}
        )

        if dependency_paths:
            # Add more specific LD_LIBRARY_PATH from the dependencies.
            env.append(
                'LD_LIBRARY_PATH="' + ":".join(dependency_paths) + ':$LD_LIBRARY_PATH"'
            )

        return env

    def project_env(self):
        return [
            '{}="{}"'.format(variable, value)
            for variable, value in snapcraft_global_environment(self.project).items()
        ]

    def _expand_env(self, snapcraft_yaml):
        environment_keys = ["name", "version"]
        for key in snapcraft_yaml:
            if any((key == env_key for env_key in environment_keys)):
                continue

            replacements = environment_to_replacements(
                snapcraft_global_environment(self.project)
            )

            snapcraft_yaml[key] = replace_attr(snapcraft_yaml[key], replacements)
        return snapcraft_yaml

    def _expand_filesets(self, snapcraft_yaml):
        parts = snapcraft_yaml.get("parts", {})

        for part_name in parts:
            # FIXME: Remove `snap` from here; it's deprecated
            for step in ("stage", "snap", "prime"):
                step_fileset = _expand_filesets_for(step, parts[part_name])
                parts[part_name][step] = step_fileset

        return snapcraft_yaml

    def _process_remote_parts(self, snapcraft_yaml):
        parts = snapcraft_yaml.get("parts", {})
        new_parts = {}

        for part_name in parts:
            if not parts[part_name]:
                parts[part_name] = dict()

            if "plugin" not in parts[part_name]:
                properties = self._remote_parts.compose(part_name, parts[part_name])
                new_parts[part_name] = properties
            else:
                new_parts[part_name] = parts[part_name].copy()

            after_parts = parts[part_name].get("after", [])
            after_remote_parts = [p for p in after_parts if p not in parts]

            for after_part in after_remote_parts:
                properties = self._remote_parts.get_part(after_part)
                new_parts[after_part] = properties

        snapcraft_yaml["parts"] = new_parts
        return snapcraft_yaml


def _expand_filesets_for(step, properties):
    filesets = properties.get("filesets", {})
    fileset_for_step = properties.get(step, {})
    new_step_set = []

    for item in fileset_for_step:
        if item.startswith("$"):
            try:
                new_step_set.extend(filesets[item[1:]])
            except KeyError:
                raise errors.SnapcraftLogicError(
                    "'{}' referred to in the '{}' fileset but it is not "
                    "in filesets".format(item, step)
                )
        else:
            new_step_set.append(item)

    return new_step_set


class _Architecture:
    def __init__(self, *, build_on, run_on=None):
        if isinstance(build_on, str):
            self.build_on = [build_on]
        else:
            self.build_on = build_on

        # If there is no run_on, it defaults to the value of build_on
        if not run_on:
            self.run_on = self.build_on
        elif isinstance(run_on, str):
            self.run_on = [run_on]
        else:
            self.run_on = run_on


def _create_architecture_list(architectures, current_arch):
    if not architectures:
        return [_Architecture(build_on=[current_arch])]

    build_architectures = []  # type: List[str]
    architecture_list = []  # type: List[_Architecture]
    for item in architectures:
        if isinstance(item, str):
            build_architectures.append(item)
        if isinstance(item, dict):
            architecture_list.append(
                _Architecture(build_on=item.get("build-on"), run_on=item.get("run-on"))
            )

    if build_architectures:
        architecture_list.append(_Architecture(build_on=build_architectures))

    return architecture_list


def _process_architectures(architectures, current_arch):
    architecture_list = _create_architecture_list(architectures, current_arch)

    for architecture in architecture_list:
        if current_arch in architecture.build_on or "all" in architecture.build_on:
            return architecture.run_on

    return [current_arch]
