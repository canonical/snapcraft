# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016, 2017-2018 Canonical Ltd
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

import json
import logging
import os
import re
import urllib.parse
import uuid
from datetime import datetime
from typing import Any, Dict

import pymacaroons
from pyramid import response

from tests.fake_servers import base

logger = logging.getLogger(__name__)


class FakeStoreAPIServer(base.BaseFakeServer):

    _DEV_API_PATH = "/dev/api/"
    _V2_DEV_API_PATH = "/api/v2/snaps/"

    def __init__(self, fake_store, server_address):
        super().__init__(server_address)
        self.fake_store = fake_store
        self.account_keys = []
        self.registered_names = {}
        self.pushed_snaps = set()
        self.validation_set_revisions = dict()
        self.validation_set_assertions = dict()

    def configure(self, configurator):
        # POST
        configurator.add_route(
            "acl",
            urllib.parse.urljoin(self._DEV_API_PATH, "acl/"),
            request_method="POST",
        )
        configurator.add_view(self.acl, route_name="acl")

        configurator.add_route(
            "verify_acl",
            urllib.parse.urljoin(self._DEV_API_PATH, "acl/verify/"),
            request_method="POST",
        )
        configurator.add_view(self.verify_acl, route_name="verify_acl")

        configurator.add_route(
            "account_key",
            urllib.parse.urljoin(self._DEV_API_PATH, "account/account-key"),
            request_method="POST",
        )
        configurator.add_view(self.account_key, route_name="account_key")

        configurator.add_route(
            "snap_builds",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{dummy}/builds"),
            request_method="POST",
        )
        configurator.add_view(self.snap_builds, route_name="snap_builds")

        configurator.add_route(
            "snap_close",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{dummy}/close"),
            request_method="POST",
        )
        configurator.add_view(self.snap_close, route_name="snap_close")

        configurator.add_route(
            "snap_push",
            urllib.parse.urljoin(self._DEV_API_PATH, "snap-push/"),
            request_method="POST",
        )
        configurator.add_view(self.snap_push, route_name="snap_push")

        configurator.add_route(
            "snap_release",
            urllib.parse.urljoin(self._DEV_API_PATH, "snap-release/"),
            request_method="POST",
        )
        configurator.add_view(self.snap_release, route_name="snap_release")

        configurator.add_route(
            "register_name",
            urllib.parse.urljoin(self._DEV_API_PATH, "register-name/"),
            request_method="POST",
        )
        configurator.add_view(self.register_name, route_name="register_name")

        configurator.add_route(
            "agreement",
            urllib.parse.urljoin(self._DEV_API_PATH, "agreement/"),
            request_method="POST",
        )
        configurator.add_view(self.agreement, route_name="agreement")

        configurator.add_route(
            "snap_metadata_post",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/metadata"),
            request_method="POST",
        )
        configurator.add_view(self.snap_metadata, route_name="snap_metadata_post")

        configurator.add_route(
            "snap_binary_metadata_post",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/binary-metadata"),
            request_method="POST",
        )
        configurator.add_view(
            self.snap_binary_metadata, route_name="snap_binary_metadata_post"
        )

        configurator.add_route(
            "post_validation_sets_build_assertion",
            "/api/v2/validation-sets/build-assertion",
            request_method="POST",
        )
        configurator.add_view(
            self.post_validation_sets_build_assertion,
            route_name="post_validation_sets_build_assertion",
        )

        configurator.add_route(
            "post_validation_sets", "/api/v2/validation-sets", request_method="POST",
        )
        configurator.add_view(
            self.post_validation_sets, route_name="post_validation_sets",
        )

        # GET
        configurator.add_route(
            "get_validation_sets", "/api/v2/validation-sets", request_method="GET",
        )
        configurator.add_view(
            self.get_validation_sets, route_name="get_validation_sets",
        )

        configurator.add_route(
            "get_validation_sets_name",
            "/api/v2/validation-sets/{name}",
            request_method="GET",
        )
        configurator.add_view(
            self.get_validation_sets, route_name="get_validation_sets_name",
        )

        configurator.add_route(
            "details",
            urllib.parse.urljoin(self._DEV_API_PATH, "/details/upload-id/{snap}"),
            request_method="GET",
        )
        configurator.add_view(self.details, route_name="details")

        configurator.add_route(
            "account",
            urllib.parse.urljoin(self._DEV_API_PATH, "account"),
            request_method="GET",
        )
        configurator.add_view(self.account, route_name="account")

        configurator.add_route(
            "snap_state",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{dummy}/state"),
            request_method="GET",
        )
        configurator.add_view(self.snap_state, route_name="snap_state")

        configurator.add_route(
            "snap_channel_map",
            urllib.parse.urljoin(self._V2_DEV_API_PATH, "{snap}/channel-map"),
            request_method="GET",
        )
        configurator.add_view(self.snap_channel_map, route_name="snap_channel_map")

        configurator.add_route(
            "snap_releases",
            urllib.parse.urljoin(self._V2_DEV_API_PATH, "{snap}/releases"),
            request_method="GET",
        )
        configurator.add_view(self.snap_releases, route_name="snap_releases")

        configurator.add_route(
            "whoami", "/api/v2/tokens/whoami", request_method="GET",
        )

        configurator.add_view(self.whoami, route_name="whoami")

        configurator.add_route(
            "snap_validations",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/validations"),
            request_method="GET",
        )
        configurator.add_view(self.snap_validations, route_name="snap_validations")

        configurator.add_route(
            "snap_developers",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/developers"),
            request_method="GET",
        )
        configurator.add_view(self.snap_developers, route_name="snap_developers")

        configurator.add_route(
            "snap_binary_metadata_get",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/binary-metadata"),
            request_method="GET",
        )
        configurator.add_view(
            self.snap_binary_metadata, route_name="snap_binary_metadata_get"
        )

        # PUT
        configurator.add_route(
            "put_snap_validations",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/validations"),
            request_method="PUT",
        )
        configurator.add_view(
            self.put_snap_validations, route_name="put_snap_validations"
        )

        configurator.add_route(
            "put_snap_developers",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/developers"),
            request_method="PUT",
        )
        configurator.add_view(
            self.put_snap_developers, route_name="put_snap_developers"
        )

        configurator.add_route(
            "snap_metadata_put",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/metadata"),
            request_method="PUT",
        )
        configurator.add_view(self.snap_metadata, route_name="snap_metadata_put")

        configurator.add_route(
            "snap_binary_metadata_put",
            urllib.parse.urljoin(self._DEV_API_PATH, "snaps/{snap_id}/binary-metadata"),
            request_method="PUT",
        )
        configurator.add_view(
            self.snap_binary_metadata, route_name="snap_binary_metadata_put"
        )

    def _refresh_error(self):
        error = {
            "code": "macaroon-permission-required",
            "message": "Authorization Required",
        }
        payload = json.dumps({"error_list": [error]}).encode()
        response_code = 401
        content_type = "application/json"
        headers = [
            ("Content-Type", content_type),
            ("WWW-Authenticate", "Macaroon needs_refresh=1"),
        ]
        return response.Response(payload, response_code, headers)

    # POST

    def acl(self, request):
        content_type = "application/json"

        if "packages" in request.json_body:
            packages = request.json_body["packages"]
            unregistered_package = {"name": "unregistered-snap-name", "series": "16"}
            if unregistered_package in packages:
                return response.Response(
                    json.dumps(
                        {
                            "error_message": "Snap not found for the given snap "
                            "name: 'unregistered-snap-name' and "
                            "series: '16'"
                        }
                    ).encode(),
                    404,
                    [("Content-Type", content_type)],
                )

        permission = request.path.split("/")[-1]
        logger.debug("Handling ACL request for {}".format(permission))
        sso_host = urllib.parse.urlparse(
            os.environ.get("UBUNTU_ONE_SSO_URL", "http://localhost")
        ).netloc
        macaroon = pymacaroons.Macaroon(
            caveats=[
                pymacaroons.Caveat(
                    caveat_id="test caveat",
                    location=sso_host,
                    verification_key_id="test verifiacion",
                )
            ]
        )
        payload = json.dumps({"macaroon": macaroon.serialize()}).encode()
        response_code = 200
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def verify_acl(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()

        print(request.json_body)

        return self._verify_acl_wide_open()

    def _verify_acl_wide_open(self):
        acl = {
            "snap_ids": None,
            "channels": None,
            "permissions": ["package_upload", "package_access", "package_manage"],
        }

        payload = json.dumps(acl).encode()
        response_code = 200
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def account_key(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        data = request.json_body
        logger.debug("Handling account-key request with content {}".format(data))
        account_key_request = data["account_key_request"]

        if account_key_request == "test-not-implemented":
            return self._account_key_not_implemented()
        elif account_key_request == "test-invalid-data":
            return self._account_key_invalid_field()
        else:
            return self._account_key_successful(account_key_request)

    def _account_key_not_implemented(self):
        payload = b"Not Implemented"
        response_code = 501
        content_type = "text/plain"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _account_key_invalid_field(self):
        error = {
            "error_list": [
                {
                    "code": "invalid-field",
                    "message": "The account-key-request assertion is not valid.",
                }
            ]
        }
        payload = json.dumps(error).encode()
        response_code = 400
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _account_key_successful(self, account_key_request):
        # Extremely basic assertion parsing, just enough to make tests work.
        # Don't copy this.
        key_name = re.search(
            "^name: (.*)$", account_key_request, flags=re.MULTILINE
        ).group(1)
        key_id = re.search(
            "^public-key-sha3-384: (.*)$", account_key_request, flags=re.MULTILINE
        ).group(1)
        self.account_keys.append({"name": key_name, "public-key-sha3-384": key_id})
        account_key = {
            "account_key": {
                "account-id": "abcd",
                "name": key_name,
                "public-key-sha3-384": key_id,
            }
        }
        payload = json.dumps(account_key).encode()
        response_code = 200
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_builds(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug("Handling sign-build request")
        snap_build = request.json_body["assertion"]
        if snap_build == "test-not-implemented":
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "code": "feature-disabled",
                            "message": (
                                "The snap-build assertions are currently disabled."
                            ),
                        }
                    ]
                }
            ).encode()
            response_code = 501
            content_type = "application/json"
        elif snap_build == "test-invalid-data":
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "code": "invalid-field",
                            "message": "The snap-build assertion is not valid.",
                        }
                    ]
                }
            ).encode()
            response_code = 400
            content_type = "application/json"
        elif snap_build == "test-unexpected-data":
            payload = b"unexpected chunk of data"
            response_code = 500
            content_type = "text/plain"
        else:
            payload = json.dumps({"type": "snap-build", "foo": "bar"}).encode()
            response_code = 200
            content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_close(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug("Handling close request")
        channels = request.json_body["channels"]

        if channels == ["invalid"]:
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "code": "invalid-field",
                            "message": "The 'channels' field content is not valid.",
                        }
                    ]
                }
            ).encode()
            response_code = 400
            content_type = "application/json"
        elif channels == ["unexpected"]:
            payload = b"unexpected chunk of data"
            response_code = 500
            content_type = "text/plain"
        elif channels == ["broken-plain"]:
            payload = b"plain data"
            response_code = 200
            content_type = "text/plain"
        elif channels == ["broken-json"]:
            payload = json.dumps({"closed_channels": channels}).encode()
            response_code = 200
            content_type = "application/json"
        else:
            payload = json.dumps(
                {
                    "closed_channels": channels,
                    "channel_map_tree": {
                        "latest": {
                            "16": {
                                "amd64": [
                                    {"channel": "stable", "info": "none"},
                                    {"channel": "candidate", "info": "none"},
                                    {
                                        "channel": "beta",
                                        "info": "specific",
                                        "version": "1.1",
                                        "revision": 42,
                                    },
                                    {"channel": "edge", "info": "tracking"},
                                ]
                            }
                        }
                    },
                }
            ).encode()
            response_code = 200
            content_type = "application/json"

        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_push(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug(
            "Handling upload request with content {}".format(request.json_body)
        )

        name = request.json_body["name"]
        if name == "test-snap-unregistered":
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "code": "resource-not-found",
                            "message": "the snap name does not exist",
                        }
                    ]
                }
            ).encode()
            response_code = 404
            content_type = "application/json"
        elif name == "test-snap-forbidden":
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "code": "resource-forbidden",
                            "message": "not allowed to publish this snap name",
                        }
                    ]
                }
            ).encode()
            response_code = 403
            content_type = "application/json"
        else:
            response_code = 202
            content_type = "application/json"
            if name == "test-review-snap":
                details_path = "details/upload-id/review-snap"
            elif name == "test-duplicate-snap":
                details_path = "details/upload-id/duplicate-snap"
            elif name == "test-scan-error-with-braces":
                details_path = "details/upload-id/scan-error-with-braces"
            else:
                details_path = "details/upload-id/good-snap"
            if not request.json_body.get("dry_run", False):
                snap_id = self.registered_names[name]["snap_id"]
                self.pushed_snaps.add(snap_id)
            payload = json.dumps(
                {
                    "status_details_url": urllib.parse.urljoin(
                        "http://localhost:{}/".format(self.server_port), details_path
                    )
                }
            ).encode()
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_release(self, request):  # noqa: C901
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug(
            "Handling release request with content {}".format(request.json_body)
        )
        response_code = 200
        content_type = "application/json"
        name = request.json_body["name"]
        channels = request.json_body["channels"]
        revision = request.json_body["revision"]
        if name == "test-snap-unregistered":
            response_code = 404
            content_type = "text/plain"
            payload = b""
        elif "alpha" in channels:
            response_code = 400
            payload = json.dumps({"errors": "Not a valid channel: alpha"}).encode()
        elif "edge/{curly}" in channels:
            response_code = 400
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "message": (
                                "Invalid branch name: {curly}. Enter a value consisting of letters, numbers or hyphens. "
                                "Hyphens cannot occur at the start or end of the chosen value."
                            ),
                            "code": "invalid-field",
                        }
                    ]
                }
            ).encode()
        elif "no-permission" in channels:
            response_code = 403
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "code": "macaroon-permission-required",
                            "message": "Permission is required: channel",
                        }
                    ],
                    "permission": "channel",
                    "channels": ["no-permission"],
                }
            ).encode()
        elif "bad-channel" in channels:
            response_code = 500
            payload = json.dumps({}).encode()
        elif name == "test-snap" or name.startswith("test-snapcraft"):
            payload = json.dumps(
                {
                    "opened_channels": channels,
                    "channel_map": [
                        {"channel": "stable", "info": "none"},
                        {"channel": "candidate", "info": "none"},
                        {
                            "revision": int(revision),
                            "channel": "beta",
                            "version": "0",
                            "info": "specific",
                        },
                        {"channel": "edge", "info": "tracking"},
                    ],
                }
            ).encode()
        elif name.startswith("arm-"):
            payload = json.dumps(
                {
                    "opened_channels": channels,
                    "channel_map_tree": {
                        "0.1": {
                            "16": {
                                "armhf": [
                                    {"channel": "stable", "info": "none"},
                                    {"channel": "candidate", "info": "none"},
                                    {
                                        "revision": int(revision),
                                        "channel": "beta",
                                        "version": "0",
                                        "info": "specific",
                                    },
                                    {"channel": "edge", "info": "tracking"},
                                ]
                            }
                        }
                    },
                }
            ).encode()
        elif name.startswith("multiarch-"):
            payload = json.dumps(
                {
                    "opened_channels": channels,
                    "channel_map_tree": {
                        "0.1": {
                            "16": {
                                "amd64": [
                                    {"channel": "stable", "info": "none"},
                                    {"channel": "candidate", "info": "none"},
                                    {
                                        "revision": int(revision),
                                        "channel": "beta",
                                        "version": "0",
                                        "info": "specific",
                                    },
                                    {"channel": "edge", "info": "tracking"},
                                ],
                                "armhf": [
                                    {"channel": "stable", "info": "none"},
                                    {"channel": "candidate", "info": "none"},
                                    {
                                        "revision": int(revision),
                                        "channel": "beta",
                                        "version": "0",
                                        "info": "specific",
                                    },
                                    {"channel": "edge", "info": "tracking"},
                                ],
                            }
                        }
                    },
                }
            ).encode()
        elif "notanumber" in revision:
            response_code = 400
            payload = json.dumps(
                {
                    "success": False,
                    "error_list": [
                        {
                            "code": "invalid-field",
                            "message": "The 'revision' field must be an integer",
                        }
                    ],
                    "errors": {"revision": ["This field must be an integer."]},
                }
            ).encode()
        else:
            raise NotImplementedError(
                "Cannot handle release request for {!r}".format(name)
            )
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def register_name(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug(
            "Handling registration request with content {}".format(request.json_body)
        )
        snap_name = request.json_body["snap_name"]
        store = request.json_body.get("store")

        if store == "snapcraft-test" and not snap_name.startswith(
            "test-snapcraft-in-store"
        ):
            return self._register_name_409_error("reserved_name")
        elif snap_name == "test-snap-name-already-registered":
            return self._register_name_409_error("already_registered")
        elif snap_name == "test-reserved-snap-name":
            return self._register_name_409_error("reserved_name")
        elif snap_name == "test-already-owned-snap-name":
            return self._register_name_409_error("already_owned")
        elif snap_name.startswith("test-snapcraft-fast"):
            return self._register_name_429_error("register_window")
        elif snap_name.startswith("test_invalid") or len(snap_name) > 40:
            return self._register_name_invalid(snap_name)
        elif snap_name == "snap-name-no-clear-error":
            return self._register_name_unclear_error()
        else:
            return self._register_name_successful(
                snap_name, request.json_body["is_private"]
            )

    def _register_name_409_error(self, error_code):
        payload = json.dumps(
            {
                "status": 409,
                "code": error_code,
                "register_name_url": "https://myapps.com/register-name/",
            }
        ).encode()
        response_code = 409
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _register_name_429_error(self, error_code):
        error = {"status": 429, "code": error_code}
        if error_code == "register_window":
            error["retry_after"] = 177
        payload = json.dumps(error).encode()
        response_code = 429
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _register_name_invalid(self, snap_name):
        # Emulates the current Store behaviour and never combines errors.
        if len(snap_name) > 40:
            msg = (
                "The name '{}' is not valid: it should be no longer than"
                " 40 characters."
            ).format(snap_name)
        elif snap_name.startswith("-") or snap_name.endswith("-"):
            msg = (
                "The name '{}' is not valid: it should not start"
                " nor end with a hyphen."
            ).format(snap_name)
        elif "--" in snap_name:
            msg = (
                "The name '{}' is not valid: it should not have"
                " two hyphens in a row."
            ).format(snap_name)
        else:
            msg = (
                "The name '{}' is not valid: it should only have"
                " ASCII lowercase letters, numbers, and hyphens,"
                " and must have at least one letter."
            ).format(snap_name)

        payload = json.dumps(
            {"error_list": [{"code": "invalid", "message": msg}]}
        ).encode()
        response_code = 400
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _register_name_unclear_error(self):
        payload = json.dumps({"status": 409, "code": "unexistent_error_code"}).encode()
        response_code = 409
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def _register_name_successful(self, name, is_private):
        snap_id = uuid.uuid4().hex
        self.registered_names[name] = dict(private=is_private, snap_id=snap_id)
        payload = json.dumps({"snap_id": snap_id}).encode()
        response_code = 201
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def agreement(self, request):
        if "STORE_DOWN" in os.environ:
            response_code = 500
            content_type = "text/plain"
            payload = b"Broken"
        else:
            if request.json_body["latest_tos_accepted"] is not True:
                response_code = 400
                content_type = "application/json"
                payload = json.dumps(
                    {
                        "error_list": [
                            {
                                "message": "`latest_tos_accepted` must be `true`",
                                "code": "bad-request",
                                "extra": {"latest_tos_accepted": "true"},
                            }
                        ]
                    }
                ).encode()
            else:
                response_code = 200
                content_type = "application/json"
                payload = json.dumps(
                    {
                        "content": {
                            "latest_tos_accepted": True,
                            "tos_url": "http://fake-url.com",
                            "latest_tos_date": "2000-01-01",
                            "accepted_tos_date": "2010-10-10",
                        }
                    }
                ).encode()

        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_metadata(self, request):
        logger.debug("Handling metadata request")

        if "invalid" in request.json_body:
            err = {
                "error_list": [
                    {"message": "Invalid field: invalid", "code": "invalid-request"}
                ]
            }
            payload = json.dumps(err).encode("utf8")
            response_code = 400
            content_type = "application/json"
        elif any("conflict" in field_name for field_name in request.json_body):
            # conflicts!
            if request.method == "PUT":
                # update anyway
                payload = b""
                response_code = 200
                content_type = "text/plain"
            else:
                # POST, return error
                error_list = []
                for name, value in request.json_body.items():
                    if name == "test-conflict-with-braces":
                        conflict_value = "value with {braces}"
                    else:
                        conflict_value = value + "-changed"
                    error_list.append(
                        {
                            "message": "Conflict on {}".format(name),
                            "code": "conflict",
                            "extra": {
                                "name": name,
                                "field": name,
                                "current": conflict_value,
                                "value": value,
                            },
                        }
                    )
                payload = json.dumps({"error_list": error_list}).encode("utf8")
                response_code = 409
                content_type = "application/json"
        else:
            # all fine by default
            payload = b""
            response_code = 200
            content_type = "text/plain"

        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_binary_metadata(self, request):
        logger.debug("Handling binary metadata request")
        if request.method == "GET":
            current = [
                {"type": "icon", "hash": "1234567890", "filename": "icon.png"},
                {"type": "screenshot", "hash": "0987654321", "filename": "ss1.png"},
                {"type": "screenshot", "hash": "1122334455", "filename": "ss2.png"},
            ]
            return response.Response(
                json.dumps(current).encode("utf-8"),
                200,
                [("Content-Type", "application/json")],
            )
        else:
            # POST/PUT
            # snapcraft.storeapi._metadata._build_binary_request_data
            if type(request.params["info"]) == bytes:
                info = json.loads(request.params["info"].decode())
            else:
                info = json.loads(request.params["info"])
            invalid = any([e.get("filename", "").endswith("invalid") for e in info])
            conflict = any([e.get("filename", "").endswith("conflict") for e in info])
            conflict_with_braces = any(
                [e.get("filename", "").endswith("conflict-with-braces") for e in info]
            )
            if invalid:
                err = {
                    "error_list": [
                        {"message": "Invalid field: icon", "code": "invalid-request"}
                    ]
                }
                payload = json.dumps(err).encode("utf8")
                response_code = 400
            elif conflict and request.method == "POST":
                # POST, return error
                error_list = [
                    {
                        "message": "Conflict on icon",
                        "code": "conflict",
                        "extra": {
                            "name": "icon",
                            "field": "icon",
                            "current": "original-icon",
                            "value": "<posted-icon>",
                        },
                    }
                ]
                payload = json.dumps({"error_list": error_list}).encode("utf8")
                response_code = 409
            elif conflict_with_braces and request.method == "POST":
                # POST, return error
                error_list = [
                    {
                        "message": "Conflict on icon",
                        "code": "conflict",
                        "extra": {
                            "name": "icon",
                            "field": "icon",
                            "current": "original icon with {braces}",
                            "value": "<posted-icon>",
                        },
                    }
                ]
                payload = json.dumps({"error_list": error_list}).encode("utf8")
                response_code = 409
            else:
                updated_info = []
                for entry in info:
                    entry.pop("key", None)
                    updated_info.append(entry)
                payload = json.dumps(updated_info).encode("utf-8")
                response_code = 200

        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    # GET

    def details(self, request):
        snap = request.matchdict["snap"]
        if snap == "duplicate-snap":
            logger.debug("Handling duplicate snap request")
            payload = json.dumps(
                {
                    "code": "processing_error",
                    "url": "/dev/click-apps/5349/rev/1",
                    "can_release": False,
                    "revision": "1",
                    "processed": True,
                    "errors": [{"message": "Duplicate snap already uploaded"}],
                }
            ).encode()
        elif snap == "scan-error-with-braces":
            logger.debug("Handling request for scan error with braces")
            payload = json.dumps(
                {
                    "code": "processing_error",
                    "url": "/dev/click-apps/5349/rev/1",
                    "can_release": False,
                    "revision": "1",
                    "processed": True,
                    "errors": [{"message": "Error message with {braces}"}],
                }
            ).encode()
        else:
            logger.debug("Handling scan complete request")
            if snap == "good-snap":
                can_release = True
                code = "ready_to_release"
            elif snap == "review-snap":
                can_release = False
                code = "need_manual_review"
            payload = json.dumps(
                {
                    "code": code,
                    "url": "/dev/click-apps/5349/rev/1",
                    "can_release": can_release,
                    "revision": "1",
                    "processed": True,
                }
            ).encode()

        response_code = 200
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def account(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug("Handling account request")
        snaps = {
            "basic": {
                "snap-id": "snap-id",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "test-snap-with-no-validations": {
                "snap-id": "test-snap-id-with-no-validations",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "test-snap-with-dev": {
                "snap-id": "test-snap-id-with-dev",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "core": {
                "snap-id": "good",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "core-no-dev": {
                "snap-id": "no-dev",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "badrequest": {
                "snap-id": "badrequest",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "revoked": {
                "snap-id": "revoked",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "no-revoked": {
                "snap-id": "no-revoked",
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
            "no-id": {
                "snap-id": None,
                "status": "Approved",
                "private": False,
                "price": None,
                "since": "2016-12-12T01:01:01Z",
            },
        }
        snaps.update(
            {
                name: {
                    "snap-id": snap_data["snap_id"],
                    "status": "Approved",
                    "private": snap_data["private"],
                    "price": None,
                    "since": "2016-12-12T01:01:01Z",
                }
                for name, snap_data in self.registered_names.items()
            }
        )
        payload = json.dumps(
            {
                "account_id": "abcd",
                "account_keys": self.account_keys,
                "snaps": {"16": snaps},
            }
        ).encode()
        response_code = 200
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_channel_map(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug("Handling snap channel_map request")
        snap = request.matchdict["snap"]
        snap_channel_map = {
            "channel-map": [
                {
                    "architecture": "all",
                    "channel": "2.1/beta",
                    "expiration-date": None,
                    "revision": 1,
                    "progressive": {
                        "paused": None,
                        "percentage": None,
                        "current-percentage": None,
                    },
                    "when": "2020-02-03T20:58:37Z",
                }
            ],
            "revisions": [
                {
                    "architectures": [
                        "amd64",
                        "arm64",
                        "armhf",
                        "i386",
                        "s390x",
                        "ppc64el",
                    ],
                    "revision": 1,
                    "version": "10",
                }
            ],
            "snap": {
                "name": snap,
                "channels": [
                    {
                        "branch": None,
                        "fallback": None,
                        "name": "2.1/stable",
                        "risk": "stable",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/stable",
                        "name": "2.1/candidate",
                        "risk": "candidate",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/candidate",
                        "name": "2.1/beta",
                        "risk": "beta",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/beta",
                        "name": "2.1/edge",
                        "risk": "edge",
                        "track": "2.1",
                    },
                ],
                "tracks": [
                    {
                        "name": "latest",
                        "status": "active",
                        "creation-date": None,
                        "version-pattern": None,
                    },
                    {
                        "name": "1.0",
                        "status": "default",
                        "creation-date": "2019-10-17T14:11:59Z",
                        "version-pattern": "1.*",
                    },
                ],
                "default-track": "2.1",
            },
        }

        response_code = 200
        content_type = "application/json"
        return response.Response(
            json.dumps(snap_channel_map).encode(),
            response_code,
            [("Content-Type", content_type)],
        )

    def snap_releases(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug("Handling snap releases request")
        snap_releases = {
            "revisions": [
                {
                    "architectures": ["arm64"],
                    "base": "core20",
                    "build_url": None,
                    "confinement": "strict",
                    "created_at": "2020-02-11T17:51:40.891996Z",
                    "grade": "stable",
                    "revision": 1,
                    "sha3-384": "a9060ef4872ccacbfa440617a76fcd84967896b28d0d1eb7571f00a1098d766e7e93353b084ba6ad841d7b14b95ede48",
                    "size": 20,
                    "status": "Published",
                    "version": "1.0",
                },
            ],
            "releases": [
                {
                    "architecture": "amd64",
                    "branch": None,
                    "channel": "latest/edge",
                    "expiration-date": None,
                    "revision": 1,
                    "risk": "stable",
                    "track": "latest",
                    "when": "2020-01-12T17:51:40.891996Z",
                },
            ],
        }

        response_code = 200
        content_type = "application/json"
        return response.Response(
            json.dumps(snap_releases).encode(),
            response_code,
            [("Content-Type", content_type)],
        )

    def snap_state(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug("Handling snap state request")
        channel_map = {
            "channel_map_tree": {
                "latest": {
                    "16": {
                        "all": [
                            {"info": "none", "channel": "stable"},
                            {"info": "none", "channel": "candidate"},
                            {
                                "info": "specific",
                                "version": "1.1-amd64",
                                "channel": "beta",
                                "revision": 6,
                            },
                            {
                                "info": "specific",
                                "version": "1.0-i386",
                                "channel": "edge",
                                "revision": 3,
                            },
                            {
                                "info": "branch",
                                "version": "1.1-i386",
                                "channel": "edge/test",
                                "revision": 9,
                                "expires_at": "2019-05-30T01:17:06.465504",
                            },
                        ],
                        "i386": [
                            {"info": "none", "channel": "stable"},
                            {"info": "none", "channel": "candidate"},
                            {
                                "info": "specific",
                                "version": "1.1-amd64",
                                "channel": "beta",
                                "revision": 6,
                            },
                            {
                                "info": "specific",
                                "version": "1.0-i386",
                                "channel": "edge",
                                "revision": 3,
                            },
                            {
                                "info": "branch",
                                "version": "1.1-i386",
                                "channel": "edge/test",
                                "revision": 9,
                                "expires_at": "2019-05-30T01:17:06.465504",
                            },
                        ],
                        "amd64": [
                            {
                                "info": "specific",
                                "version": "1.0-amd64",
                                "channel": "stable",
                                "revision": 2,
                            },
                            {"info": "none", "channel": "candidate"},
                            {
                                "info": "specific",
                                "version": "1.1-amd64",
                                "channel": "beta",
                                "revision": 4,
                            },
                            {"info": "tracking", "channel": "edge"},
                            {
                                "info": "branch",
                                "version": "1.1-amd64",
                                "channel": "edge/test",
                                "revision": 10,
                                "expires_at": "2019-05-30T01:17:06.465504",
                            },
                        ],
                    }
                }
            }
        }

        parsed_qs = urllib.parse.parse_qs(urllib.parse.urlparse(request.url).query)
        if "architecture" in parsed_qs:
            arch = parsed_qs["architecture"][0]
            series = channel_map["channel_map_tree"]["latest"]["16"]
            if arch in series:
                output = {"channel_map_tree": {"latest": {"16": {arch: series[arch]}}}}
            else:
                output = {}
        else:
            output = channel_map
        response_code = 200
        content_type = "application/json"
        return response.Response(
            json.dumps(output).encode(), response_code, [("Content-Type", content_type)]
        )

    def snap_validations(self, request):
        logger.debug("Handling validation request")
        snap_id = request.matchdict["snap_id"]
        if snap_id == "good":
            validation = [
                {
                    "approved-snap-id": "snap-id-1",
                    "approved-snap-revision": "3",
                    "approved-snap-name": "snap-1",
                    "authority-id": "dev-1",
                    "series": "16",
                    "sign-key-sha3-384": "1234567890",
                    "snap-id": "snap-id-gating",
                    "timestamp": "2016-09-19T21:07:27.756001Z",
                    "type": "validation",
                    "revoked": "false",
                    "required": True,
                },
                {
                    "approved-snap-id": "snap-id-2",
                    "approved-snap-revision": "5",
                    "approved-snap-name": "snap-2",
                    "authority-id": "dev-1",
                    "series": "16",
                    "sign-key-sha3-384": "1234567890",
                    "snap-id": "snap-id-gating",
                    "timestamp": "2016-09-19T21:07:27.756001Z",
                    "type": "validation",
                    "revoked": "false",
                    "required": False,
                },
                {
                    "approved-snap-id": "snap-id-3",
                    "approved-snap-revision": "-",
                    "approved-snap-name": "snap-3",
                    "authority-id": "dev-1",
                    "series": "16",
                    "sign-key-sha3-384": "1234567890",
                    "snap-id": "snap-id-gating",
                    "timestamp": "2016-09-19T21:07:27.756001Z",
                    "type": "validation",
                    "revoked": "false",
                    "required": True,
                },
            ]
            payload = json.dumps(validation).encode()
            response_code = 200
        elif snap_id == "bad":
            payload = "foo".encode()
            response_code = 200
        elif snap_id == "test-snap-id-with-no-validations":
            payload = json.dumps([]).encode()
            response_code = 200
        elif snap_id == "err":
            payload = json.dumps(
                {"error_list": [{"code": "test-code", "message": "test-error"}]}
            ).encode()
            response_code = 503
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def snap_developers(self, request):
        logger.debug("Handling snap developers request")
        snap_id = request.matchdict["snap_id"]
        if snap_id == "good":
            payload = json.dumps({"snap_developer": {}}).encode()
            response_code = 200
        elif snap_id in ("test-snap-id-with-dev", "revoked", "no-revoked"):
            payload = json.dumps(
                {
                    "snap_developer": {
                        "type": "snap-developer",
                        "authority-id": "dummy",
                        "publisher-id": "dummy",
                        "snap-id": snap_id,
                        "developers": [
                            {
                                "developer-id": "test-dev-id",
                                "since": "2017-02-10T08:35:00.390258Z",
                                "until": "2018-02-10T08:35:00.390258Z",
                            }
                        ],
                    }
                }
            ).encode()
            response_code = 200
        elif snap_id == "no-dev":
            payload = json.dumps(
                {
                    "error_list": [
                        {"message": "error", "code": "snap-developer-not-found"}
                    ]
                }
            ).encode()
            response_code = 403
        elif snap_id == "badrequest":
            payload = json.dumps({"snap_developer": {}}).encode()
            response_code = 200
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    # PUT

    def put_snap_validations(self, request):
        snap_id = request.matchdict["snap_id"]
        if snap_id == "good":
            payload = request.body
            response_code = 200
        elif snap_id == "err":
            payload = json.dumps(
                {"error_list": [{"code": "test-code", "message": "test-error"}]}
            ).encode()
            response_code = 501
        elif snap_id == "bad":
            payload = "foo".encode()
            response_code = 200
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def put_snap_developers(self, request):
        snap_id = request.matchdict["snap_id"]
        if snap_id in ("good", "test-snap-id-with-dev"):
            payload = request.body
            response_code = 200
        elif snap_id == "no-dev":
            payload = request.body
            response_code = 200
        elif snap_id == "badrequest":
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "message": "The given `snap-id` does not match the "
                            "assertion.",
                            "code": "invalid-request",
                        }
                    ]
                }
            ).encode()
            response_code = 400
        elif snap_id == "revoked":
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "message": "The assertion's `developers` would revoke "
                            "existing uploads.",
                            "code": "revoked-uploads",
                            "extra": ["this"],
                        }
                    ]
                }
            ).encode()
            response_code = 409
        elif snap_id == "no-revoked":
            payload = json.dumps(
                {
                    "error_list": [
                        {
                            "message": "The collaborators for this snap haven't been "
                            "altered. Exiting... ",
                            "code": "revoked-uploads",
                            "extra": ["this"],
                        }
                    ]
                }
            ).encode()
            response_code = 409
        content_type = "application/json"
        return response.Response(
            payload, response_code, [("Content-Type", content_type)]
        )

    def whoami(self, request):
        if self.fake_store.needs_refresh:
            return self._refresh_error()
        logger.debug("Handling whoami request")
        payload = {
            "account": {
                "email": "foo@bar.baz",
                "id": "1234567890",
                "name": "Foo from Bar",
                "username": "baz",
            },
            "channels": None,
            "packages": None,
            "permissions": None,
        }

        response_code = 200
        content_type = "application/json"
        return response.Response(
            json.dumps(payload).encode(),
            response_code,
            [("Content-Type", content_type)],
        )

    def _get_validation_sets_revision(self, name: str) -> str:
        if name not in self.validation_set_revisions:
            self.validation_set_revisions[name] = 0
        else:
            self.validation_set_revisions[name] += 1

        return str(self.validation_set_revisions[name])

    def post_validation_sets_build_assertion(self, request):
        if request.json_body["sequence"] > 0:
            payload = request.json_body
            payload["sequence"] = str(payload["sequence"])
            # Get a revision for this assertion.
            payload["revision"] = self._get_validation_sets_revision(payload["name"])
            # Convert revisions for snaps into strings and add ids if not there.
            for snap in payload["snaps"]:
                if "revision" in snap:
                    snap["revision"] = str(snap["revision"])
                if "id" not in snap:
                    snap["id"] = "snap-id" + snap["name"]
            # Send and authority-id for this assertion.
            payload["authority-id"] = payload["account-id"]
            # And add all the other required fields for this response.
            payload["type"] = "validation-set"
            payload["timestamp"] = datetime.strftime(
                datetime.now(), "%Y-%m-%dT%H:%M:%S.%fZ"
            )
            payload["series"] = "16"
            response_code = 200
        else:
            payload = {
                "error-list": [
                    {
                        "message": "0 is less than the minimum of 1 at /sequence",
                        "code": "invalid-request",
                    }
                ]
            }
            response_code = 400

        return response.Response(
            json.dumps(payload).encode(),
            response_code,
            [("Content-Type", "application/json")],
        )

    def _save_validation_sets_assertion(self, assertion: Dict[str, Any]) -> None:
        self.validation_set_assertions[assertion["name"]] = assertion

    def post_validation_sets(self, request):
        assertion_data = json.loads(request.body.decode().splitlines()[0])

        if assertion_data["authority-id"] == assertion_data["account-id"]:
            response_code = 200
            # API has a response with the assertion as json, but we don't use it.
            payload = {"assertions": [{"headers": assertion_data}]}
            self._save_validation_sets_assertion(assertion_data)
        else:
            payload = {
                "error-list": [
                    {
                        "message": "account-id and authority-id must match the requesting user.",
                        "code": "invalid-request",
                    }
                ]
            }
            response_code = 400

        return response.Response(
            json.dumps(payload).encode(),
            response_code,
            [("Content-Type", "application/json")],
        )

    def get_validation_sets(self, request):
        # This fake does not parse the sequence, it just verifies it is used
        # in some capacity.
        # TODO: support specific name.
        parsed_qs = urllib.parse.parse_qs(urllib.parse.urlparse(request.url).query)

        try:
            name = request.matchdict["name"]
        except KeyError:
            name = None

        if name is not None and name in self.validation_set_assertions:
            assertions = [self.validation_set_assertions[name]]
        else:
            assertions = self.validation_set_assertions.values()

        if "sequence" not in parsed_qs or ["sequence"] == "latest":
            payload = {"assertions": [{"headers": a} for a in assertions]}
            response_code = 200
        else:
            payload = {
                "error-list": [
                    {
                        "message": "sequence must be one of ['all', 'latest'] or a positive integer higher than 0.",
                        "code": "invalid-request",
                    }
                ]
            }
            response_code = 400

        return response.Response(
            json.dumps(payload).encode(),
            response_code,
            [("Content-Type", "application/json")],
        )
