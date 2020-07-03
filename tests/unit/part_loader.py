# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

from unittest import mock

from snapcraft.internal import elf, pluginhandler
from snapcraft.internal.project_loader import grammar_processing
from snapcraft.project import Project, _schema


def load_part(
    part_name,
    plugin_name=None,
    part_properties=None,
    project=None,
    stage_packages_repo=None,
    snap_name="test-snap",
    base="core18",
    build_base=None,
    confinement="strict",
    snap_type="app",
):
    if not plugin_name:
        plugin_name = "nil"
    properties = {"plugin": plugin_name}
    if part_properties:
        properties.update(part_properties)
    if "build-environment" not in properties:
        properties["build-environment"] = list()
    if not project:
        project = Project()

    project._snap_meta.name = snap_name
    project._snap_meta.version = "1.0"
    project._snap_meta.grade = "devel"
    project._snap_meta.type = snap_type
    project._snap_meta.confinement = confinement
    project._snap_meta.base = base
    if build_base is not None:
        project._snap_meta.build_base = build_base

    validator = _schema.Validator()
    schema = validator.part_schema
    definitions_schema = validator.definitions_schema
    plugin = pluginhandler.load_plugin(
        part_name=part_name,
        plugin_name=plugin_name,
        properties=properties,
        project=project,
        part_schema=schema,
        definitions_schema=definitions_schema,
    )

    if not stage_packages_repo:
        stage_packages_repo = mock.Mock()
    grammar_processor = grammar_processing.PartGrammarProcessor(
        plugin=plugin, properties=properties, project=project, repo=stage_packages_repo
    )

    return pluginhandler.PluginHandler(
        plugin=plugin,
        part_properties=properties,
        project=project,
        part_schema=schema,
        definitions_schema=definitions_schema,
        grammar_processor=grammar_processor,
        stage_packages_repo=stage_packages_repo,
        snap_base_path="/snap/fake-name/current",
        soname_cache=elf.SonameCache(),
    )
