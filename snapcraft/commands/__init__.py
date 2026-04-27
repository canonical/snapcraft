# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Snapcraft commands."""

from . import core22, legacy
from .account import (
    StoreExportLoginCommand,
    StoreLoginCommand,
    StoreLogoutCommand,
    StoreWhoAmICommand,
)
from .extensions import (
    ExpandExtensionsCommand,
    ExtensionsCommand,
    ListExtensionsCommand,
)
from .keys import (
    StoreCreateKeyCommand,
    StoreListKeysCommand,
    StoreKeysCommand,
    StoreRegisterKeyCommand,
)
from .legacy import (
    StoreLegacyPromoteCommand,
    StoreLegacySignBuildCommand,
)
from .lifecycle import PackCommand, SnapCommand, TryCommand
from .lint import LintCommand
from .manage import (
    StoreCloseCommand,
    StoreReleaseCommand,
    StoreSetDefaultTrackCommand,
)
from .names import (
    StoreLegacyListCommand,
    StoreLegacyListRegisteredCommand,
    StoreNamesCommand,
    StoreRegisterCommand,
)
from .plugins import ListPluginsCommand, PluginsCommand
from .confdb_schemas import (
    StoreEditConfdbSchemaCommand,
    StoreListConfdbSchemasCommand,
    StoreConfdbSchemasCommand,
)
from .remote import RemoteBuildCommand
from .status import (
    StoreListRevisionsCommand,
    StoreListTracksCommand,
    StoreRevisionsCommand,
    StoreStatusCommand,
    StoreTracksCommand,
)
from .upload import (
    StoreLegacyPushCommand,
    StoreUploadCommand,
    StoreUploadMetadataCommand,
)
from .validation_sets import (
    StoreEditValidationSetsCommand,
    StoreListValidationSetsCommand,
    StoreValidationSetsCommand,
)
from .validations import (
    StoreGatedCommand,
    StoreValidateCommand,
)
from .metrics import StoreMetricsCommand

__all__ = [
    "ExpandExtensionsCommand",
    "ExtensionsCommand",
    "LintCommand",
    "ListExtensionsCommand",
    "ListPluginsCommand",
    "RemoteBuildCommand",
    "PackCommand",
    "PluginsCommand",
    "SnapCommand",
    "StoreCloseCommand",
    "StoreConfdbSchemasCommand",
    "StoreCreateKeyCommand",
    "StoreEditValidationSetsCommand",
    "StoreEditConfdbSchemaCommand",
    "StoreExportLoginCommand",
    "StoreKeysCommand",
    "StoreGatedCommand",
    "StoreLegacyListCommand",
    "StoreLegacyListRegisteredCommand",
    "StoreLegacyPromoteCommand",
    "StoreLegacyPushCommand",
    "StoreLegacySignBuildCommand",
    "StoreListRevisionsCommand",
    "StoreListConfdbSchemasCommand",
    "StoreListKeysCommand",
    "StoreListTracksCommand",
    "StoreListValidationSetsCommand",
    "StoreLoginCommand",
    "StoreLogoutCommand",
    "StoreMetricsCommand",
    "StoreNamesCommand",
    "StoreRegisterCommand",
    "StoreRegisterKeyCommand",
    "StoreReleaseCommand",
    "StoreRevisionsCommand",
    "StoreSetDefaultTrackCommand",
    "StoreStatusCommand",
    "StoreTracksCommand",
    "StoreUploadCommand",
    "StoreUploadMetadataCommand",
    "StoreValidateCommand",
    "StoreValidationSetsCommand",
    "StoreWhoAmICommand",
    "TryCommand",
    "core22",
    "legacy",
]
