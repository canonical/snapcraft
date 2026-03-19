# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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
"""Data models for snapcraft."""

from .assertions import (
    Assertion,
    EditableAssertion,
    EditableConfdbSchemaAssertion,
    EditableValidationSetAssertion,
    ConfdbSchema,
    ConfdbSchemaAssertion,
    ValidationSetAssertion,
)
from .manifest import Manifest
from .project import (
    MANDATORY_ADOPTABLE_FIELDS,
    App,
    Architecture,
    ArchitectureProject,
    BareCore22Project,
    BareCore24Project,
    Core22Project,
    Core24Project,
    Component,
    ComponentProject,
    ContentPlug,
    GrammarAwareProject,
    Hook,
    Lint,
    Platform,
    Project,
    Socket,
)

__all__ = [
    "MANDATORY_ADOPTABLE_FIELDS",
    "Assertion",
    "App",
    "Architecture",
    "ArchitectureProject",
    "BareCore22Project",
    "BareCore24Project",
    "Core22Project",
    "Core24Project",
    "Component",
    "ComponentProject",
    "ContentPlug",
    "EditableAssertion",
    "EditableConfdbSchemaAssertion",
    "EditableValidationSetAssertion",
    "GrammarAwareProject",
    "Hook",
    "Lint",
    "Manifest",
    "Platform",
    "Project",
    "ConfdbSchema",
    "ConfdbSchemaAssertion",
    "Socket",
    "ValidationSetAssertion",
]
