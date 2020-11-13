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

import pytest

from snapcraft.storeapi.v2 import releases


@pytest.mark.parametrize(
    "branch", ((None, None), ("test-branch", "2020-02-11T17:51:40.891996Z"))
)
@pytest.mark.parametrize("revision", (42, None))
def test_release(branch, revision):
    payload = {
        "architecture": "amd64",
        "branch": branch[0],
        "channel": "latest/stable",
        "expiration-date": branch[1],
        "revision": revision,
        "risk": "stable",
        "track": "latest",
        "when": "2020-02-11T17:51:40.891996Z",
    }

    r = releases.Release.unmarshal(payload)

    assert repr(r) == "<Release: 'latest/stable'>"
    assert r.architecture == payload["architecture"]
    assert r.branch == payload["branch"]
    assert r.channel == payload["channel"]
    assert r.expiration_date == payload["expiration-date"]
    assert r.revision == payload["revision"]
    assert r.risk == payload["risk"]
    assert r.track == payload["track"]
    assert r.when == payload["when"]
    assert r.marshal() == payload


@pytest.mark.parametrize("architectures", (["amd64"], ["amd64", "arm64"]))
@pytest.mark.parametrize("build_url", (None, "https://launchpad.net/foo"))
@pytest.mark.parametrize("base", (None, "core20"))
def test_revision(architectures, build_url, base):
    payload = {
        "architectures": architectures,
        "build_url": build_url,
        "confinement": "strict",
        "created_at": "2020-02-11T17:51:40.891996Z",
        "grade": "stable",
        "revision": 42,
        "sha3-384": "a9060ef4872ccacbfa440617a76fcd84967896b28d0d1eb7571f00a1098d766e7e93353b084ba6ad841d7b14b95ede48",
        "size": 20,
        "status": "Published",
        "version": "1.0",
    }
    if base is not None:
        payload["base"] = base

    r = releases.Revision.unmarshal(payload)

    assert repr(r) == "<Revision: 42>"
    assert r.architectures == payload["architectures"]
    assert r.base == base
    assert r.build_url == payload["build_url"]
    assert r.confinement == payload["confinement"]
    assert r.created_at == payload["created_at"]
    assert r.revision == payload["revision"]
    assert r.sha3_384 == payload["sha3-384"]
    assert r.size == payload["size"]
    assert r.status == payload["status"]
    assert r.version == payload["version"]
    assert r.marshal() == payload


def test_releases():
    payload = {
        "revisions": [
            {
                "architectures": ["amd64"],
                "build_url": None,
                "confinement": "strict",
                "created_at": "2020-02-11T17:51:40.891996Z",
                "grade": "stable",
                "revision": 2,
                "sha3-384": "a9060ef4872ccacbfa440617a76fcd84967896b28d0d1eb7571f00a1098d766e7e93353b084ba6ad841d7b14b95ede48",
                "size": 20,
                "status": "Published",
                "version": "1.0",
            },
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
                "channel": "latest/stable",
                "expiration-date": None,
                "revision": 1,
                "risk": "stable",
                "track": "latest",
                "when": "2020-02-12T17:51:40.891996Z",
            },
            {
                "architecture": "amd64",
                "branch": None,
                "channel": "latest/stable",
                "expiration-date": None,
                "revision": 2,
                "risk": "stable",
                "track": "latest",
                "when": "2020-02-11T17:51:40.891996Z",
            },
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

    r = releases.Releases.unmarshal(payload)

    assert repr(r) == "<Releases>"
    assert r.marshal() == payload
