# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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


"""Tests for Assertion models."""


def test_assertion_defaults(fake_registry_assertion_data, check):
    """Test default values of the RegistryAssertion model."""
    check.equal(fake_registry_assertion_data.body, None)
    check.equal(fake_registry_assertion_data.body_length, None)
    check.equal(fake_registry_assertion_data.sign_key_sha3_384, None)
    check.equal(fake_registry_assertion_data.summary, None)
    check.equal(fake_registry_assertion_data.revision, 0)
