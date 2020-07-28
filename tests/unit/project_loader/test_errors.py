# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from snapcraft.internal.project_loader import errors


def test_SnapcraftAfterPartMissingError():
    exception = errors.SnapcraftAfterPartMissingError(
        part_name="some-part", after_part_name="after-part"
    )
    assert (
        exception.get_brief()
        == "Part 'some-part' after configuration refers to unknown part 'after-part'."
    )
    assert exception.get_details() is None
    assert (
        exception.get_resolution()
        == "Ensure 'after' configuration and referenced part name is correct."
    )
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftDuplicateAliasError_one_alias():
    exception = errors.SnapcraftDuplicateAliasError(aliases={"alias-a"})
    assert (
        exception.get_brief() == "Multiple parts have the same alias defined: alias-a"
    )
    assert exception.get_details() is None
    assert exception.get_resolution() == "Use each alias only once."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftDuplicateAliasError_multiple_alias():
    exception = errors.SnapcraftDuplicateAliasError(aliases={"alias-a", "alias-b"})
    assert (
        exception.get_brief()
        == "Multiple parts have the same alias defined: alias-a, alias-b"
    )
    assert exception.get_details() is None
    assert exception.get_resolution() == "Use each alias only once."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftExtensionBaseRequiredError():
    exception = errors.SnapcraftExtensionBaseRequiredError()
    assert (
        exception.get_brief()
        == "Extensions can only be used if the snapcraft.yaml specifies a 'base'."
    )
    assert exception.get_details() is None
    assert exception.get_resolution() == "Ensure your base configuration is correct."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftExtensionNotFoundError():
    exception = errors.SnapcraftExtensionNotFoundError(extension_name="extension-name")
    assert exception.get_brief() == "Failed to find extension 'extension-name'."
    assert exception.get_details() is None
    assert exception.get_resolution() == "Ensure the extension name is correct."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftExtensionPartConflictError():
    exception = errors.SnapcraftExtensionPartConflictError(
        extension_name="extension-name", part_name="part-name"
    )
    assert exception.get_brief() == "Failed to apply extension 'extension-name'."
    assert (
        exception.get_details()
        == "This extension adds a part named 'part-name', but a part by that name already exists."
    )
    assert exception.get_resolution() == "Rename the 'part-name' part."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftExtensionUnsupportedBaseError():
    exception = errors.SnapcraftExtensionUnsupportedBaseError(
        extension_name="extension-name", base="some-base"
    )
    assert exception.get_brief() == "Failed to load extension 'extension-name'."
    assert (
        exception.get_details()
        == "This extension does not support the 'some-base' base."
    )
    assert (
        exception.get_resolution()
        == "Either use a different extension, or use a base supported by this extension."
    )
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftExtensionUnsupportedConfinementError():
    exception = errors.SnapcraftExtensionUnsupportedConfinementError(
        extension_name="extension-name", confinement="some-confinement"
    )
    assert exception.get_brief() == "Failed to load extension 'extension-name'."
    assert (
        exception.get_details()
        == "This extension does not support 'some-confinement' confinement."
    )
    assert (
        exception.get_resolution()
        == "Either use a different extension, or use a confinement supported by this extension."
    )
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftFilesetReferenceError():
    exception = errors.SnapcraftFilesetReferenceError(
        item="some-item", step="some-step"
    )
    assert (
        exception.get_brief()
        == "Fileset 'some-item' referred to in the 'some-step' step was not found."
    )
    assert exception.get_details() is None
    assert exception.get_resolution() == "Ensure the fileset configuration is correct."
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False


def test_SnapcraftInvalidEpochError():
    exception = errors.SnapcraftInvalidEpochError(epoch="-1")
    assert exception.get_brief() == "Invalid epoch format for '-1'."
    assert exception.get_details() is None
    assert (
        exception.get_resolution()
        == "Valid epochs are positive integers followed by an optional asterisk."
    )
    assert exception.get_docs_url() is None
    assert exception.get_reportable() is False
