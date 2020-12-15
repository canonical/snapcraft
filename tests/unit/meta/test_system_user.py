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

from testtools.matchers import Equals

from snapcraft.internal.meta import errors
from snapcraft.internal.meta.system_user import SystemUser, SystemUserScope
from tests import unit


class SystemUserTests(unit.TestCase):
    def test_valid_user(self):
        user = SystemUser(name="name", scope=SystemUserScope.SHARED)
        user.validate()

        user_dict = {"scope": "shared"}
        user = SystemUser.from_dict(user_name="name", user_dict=user_dict)

    def test_invalid_user_empty_scope(self):
        user = SystemUser(name="name", scope=None)

        error = self.assertRaises(errors.SystemUsernamesValidationError, user.validate)

        self.assertThat(error._name, Equals("name"))
        self.assertThat(error._message, Equals("scope None is invalid"))

    def test_invalid_user_empty_scope_from_dict(self):
        user_dict = {"scope": None}

        error = self.assertRaises(
            errors.SystemUsernamesValidationError,
            SystemUser.from_dict,
            user_name="name",
            user_dict=user_dict,
        )

        self.assertThat(error._name, Equals("name"))
        self.assertThat(error._message, Equals("'scope' is undefined"))

    def test_invalid_user_non_shared_scope(self):
        user = SystemUser(name="name", scope="none")

        error = self.assertRaises(errors.SystemUsernamesValidationError, user.validate)

        self.assertThat(error._name, Equals("name"))
        self.assertThat(error._message, Equals("scope 'none' is invalid"))

    def test_invalid_user_non_shared_scope_from_dict(self):
        user_dict = {"scope": "ghost"}

        error = self.assertRaises(
            errors.SystemUsernamesValidationError,
            SystemUser.from_dict,
            user_name="name",
            user_dict=user_dict,
        )

        self.assertThat(error._name, Equals("name"))
        self.assertThat(error._message, Equals("scope 'ghost' is invalid"))

    def test_from_object_dict(self):
        user_name = "name"
        user_object = {"scope": "shared"}

        user = SystemUser.from_object(user_name=user_name, user_object=user_object)

        self.assertThat(user.name, Equals(user_name))
        self.assertThat(user.scope, Equals(SystemUserScope.SHARED))

    def test_from_object_dict_invalid(self):
        user_name = "name"
        user_object = {"scope": "invalid_scope"}

        error = self.assertRaises(
            errors.SystemUsernamesValidationError,
            SystemUser.from_object,
            user_name=user_name,
            user_object=user_object,
        )

        self.assertThat(error._name, Equals("name"))
        self.assertThat(error._message, Equals("scope 'invalid_scope' is invalid"))

    def test_from_object_none(self):
        user_name = "name"
        user_object = None

        error = self.assertRaises(
            errors.SystemUsernamesValidationError,
            SystemUser.from_object,
            user_name=user_name,
            user_object=user_object,
        )

        self.assertThat(error._name, Equals("name"))
        self.assertThat(error._message, Equals("undefined user"))

    def test_from_object_string(self):
        user_name = "name"
        user_object = "shared"

        user = SystemUser.from_object(user_name=user_name, user_object=user_object)

        self.assertThat(user.name, Equals(user_name))
        self.assertThat(user.scope, Equals(SystemUserScope.SHARED))

    def test_from_object_string_invalid(self):
        user_name = "name"
        user_object = "invalid_scope"

        error = self.assertRaises(
            errors.SystemUsernamesValidationError,
            SystemUser.from_object,
            user_name=user_name,
            user_object=user_object,
        )

        self.assertThat(error._name, Equals("name"))
        self.assertThat(error._message, Equals("scope 'invalid_scope' is invalid"))
