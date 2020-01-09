# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from collections import OrderedDict

from testtools.matchers import Equals

from snapcraft.internal.meta import errors
from snapcraft.internal.meta.hooks import Hook
from tests import unit


class GenericHookTests(unit.TestCase):
    def test_empty(self):
        hook_dict = OrderedDict({})
        hook_name = "hook-test"

        hook = Hook(hook_name=hook_name)
        hook.validate()

        self.assertEqual(hook.to_dict(), hook_dict)
        self.assertEqual(hook.hook_name, hook_name)

    def test_empty_from_dict(self):
        hook_dict = OrderedDict({})
        hook_name = "hook-test"

        hook = Hook.from_dict(hook_dict=hook_dict, hook_name=hook_name)
        hook.validate()

        self.assertEqual(hook.to_dict(), hook_dict)
        self.assertEqual(hook.hook_name, hook_name)

    def test_passthrough(self):
        hook_dict = OrderedDict({"passthrough": {"otherkey": "othervalue"}})
        hook_name = "hook-test"

        hook = Hook.from_dict(hook_dict=hook_dict, hook_name=hook_name)
        hook.validate()

        transformed_dict = OrderedDict({"otherkey": "othervalue"})

        self.assertEqual(hook.to_dict(), transformed_dict)
        self.assertEqual(hook.hook_name, hook_name)
        self.assertEqual(hook.passthrough, hook_dict["passthrough"])

    def test_plugs(self):
        hook_dict = OrderedDict({"plugs": ["plug1", "plug2"]})
        hook_name = "hook-test"

        hook = Hook.from_dict(hook_dict=hook_dict, hook_name=hook_name)
        hook.validate()

        self.assertEqual(hook.to_dict(), hook_dict)
        self.assertEqual(hook.hook_name, hook_name)

    def test_command_chain(self):
        hook_dict = OrderedDict({"command-chain": ["cmd1", "cmd2"]})
        hook_name = "hook-test"

        hook = Hook.from_dict(hook_dict=hook_dict, hook_name=hook_name)
        hook.validate()

        self.assertEqual(hook.to_dict(), hook_dict)
        self.assertEqual(hook.hook_name, hook_name)

    def test_invalid_command_chain(self):
        hook_dict = OrderedDict({"command-chain": ["&/foo/bar"]})
        hook_name = "hook-test"

        hook = Hook.from_dict(hook_dict=hook_dict, hook_name=hook_name)

        error = self.assertRaises(errors.HookValidationError, hook.validate)
        self.assertThat(
            str(error),
            Equals(
                "failed to validate hook=hook-test: '&/foo/bar' is not a valid command-chain command."
            ),
        )
