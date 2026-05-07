# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd
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

"""Tests for Releases models."""

import pytest

from snapcraft.models.releases import Epoch, Progressive, Release, Releases, Revision


@pytest.fixture
def fake_release_payload():
    return {
        "architecture": "amd64",
        "branch": None,
        "channel": "latest/stable",
        "expiration-date": None,
        "revision": 1,
        "risk": "stable",
        "track": "latest",
        "when": "2020-02-12T17:51:40.891996Z",
    }


@pytest.fixture
def fake_revision_payload():
    return {
        "architectures": ["amd64"],
        "base": "core20",
        "build-url": None,
        "confinement": "strict",
        "created-at": "2016-09-27T18:38:43Z",
        "grade": "stable",
        "revision": 1,
        "sha3-384": "fake-a9060ef4872ccacbfa44",
        "size": 20,
        "status": "Published",
        "version": "2.0.2",
    }


@pytest.fixture
def fake_releases_payload(fake_release_payload, fake_revision_payload):
    return {"releases": [fake_release_payload], "revisions": [fake_revision_payload]}


class TestRelease:
    @pytest.mark.parametrize(
        "progressive",
        [None, {"paused": None, "percentage": 50.0, "current-percentage": 25.0}],
    )
    @pytest.mark.parametrize("branch", [None, "test-branch"])
    @pytest.mark.parametrize("revision", [None, 123])
    @pytest.mark.parametrize("expiration_date", [None, "2026-01-01T12:34:56Z"])
    def test_unmarshal(
        self, fake_release_payload, branch, revision, expiration_date, progressive
    ):
        fake_release_payload["branch"] = branch
        fake_release_payload["revision"] = revision
        fake_release_payload["expiration-date"] = expiration_date
        fake_release_payload["progressive"] = progressive

        release = Release.unmarshal(fake_release_payload)

        assert release.architecture == "amd64"
        assert release.branch == branch
        assert release.channel == "latest/stable"
        assert release.expiration_date == expiration_date
        assert release.progressive == (
            Progressive.unmarshal(progressive) if progressive is not None else None
        )
        assert release.revision == revision
        assert release.risk == "stable"
        assert release.track == "latest"
        assert release.when == "2020-02-12T17:51:40.891996Z"

    @pytest.mark.parametrize(
        "progressive",
        [None, {"paused": None, "percentage": 50.0, "current-percentage": 25.0}],
    )
    @pytest.mark.parametrize("branch", [None, "test-branch"])
    @pytest.mark.parametrize("revision", [None, 123])
    @pytest.mark.parametrize("expiration_date", [None, "2026-01-01T12:34:56Z"])
    def test_marshal(
        self, fake_release_payload, branch, revision, expiration_date, progressive
    ):
        fake_release_payload["branch"] = branch
        fake_release_payload["revision"] = revision
        fake_release_payload["expiration-date"] = expiration_date
        fake_release_payload["progressive"] = progressive

        data = Release.unmarshal(fake_release_payload).marshal()

        assert data["architecture"] == "amd64"
        assert data["branch"] is branch
        assert data["channel"] == "latest/stable"
        assert data["expiration-date"] is expiration_date
        assert data["progressive"] == progressive
        assert data["revision"] == revision
        assert data["risk"] == "stable"
        assert data["track"] == "latest"
        assert data["when"] == "2020-02-12T17:51:40.891996Z"


class TestRevision:
    @pytest.mark.parametrize("epoch", [None, {"read": [0], "write": [0]}])
    @pytest.mark.parametrize("attributes", [{}, {"key": "value"}])
    @pytest.mark.parametrize("base", [None, "core26"])
    @pytest.mark.parametrize("build_url", [None, "test-url"])
    def test_unmarshal_hyphenated_keys(
        self, fake_revision_payload, base, build_url, attributes, epoch
    ):
        fake_revision_payload["base"] = base
        fake_revision_payload["build-url"] = build_url
        fake_revision_payload["attributes"] = attributes
        fake_revision_payload["epoch"] = epoch

        rev = Revision.unmarshal(fake_revision_payload)

        assert rev.architectures == ["amd64"]
        assert rev.attributes == attributes
        assert rev.base == base
        assert rev.build_url == build_url
        assert rev.confinement == "strict"
        assert rev.created_at == "2016-09-27T18:38:43Z"
        assert rev.epoch == (Epoch.unmarshal(epoch) if epoch is not None else None)
        assert rev.grade == "stable"
        assert rev.revision == 1
        assert rev.sha3_384 == "fake-a9060ef4872ccacbfa44"
        assert rev.size == 20
        assert rev.status == "Published"
        assert rev.version == "2.0.2"

    def test_unmarshal_underscored_keys(self, fake_revision_payload):
        """Unmarshal underscored keys.

        The public store uses underscores and the on-prem store uses hyphenated keys.
        The pydantic model should handle both with the `populate_by_name` param.
        """
        fake_revision_payload["build_url"] = fake_revision_payload.pop("build-url")
        fake_revision_payload["created_at"] = fake_revision_payload.pop("created-at")

        rev = Revision.unmarshal(fake_revision_payload)

        assert rev.build_url is None
        assert rev.created_at == "2016-09-27T18:38:43Z"

    @pytest.mark.parametrize("epoch", [None, {"read": [0], "write": [0]}])
    @pytest.mark.parametrize("attributes", [{}, {"key": "value"}])
    @pytest.mark.parametrize("base", [None, "core26"])
    @pytest.mark.parametrize("build_url", [None, "test-url"])
    def test_marshal(self, fake_revision_payload, base, build_url, attributes, epoch):
        fake_revision_payload["base"] = base
        fake_revision_payload["build-url"] = build_url
        fake_revision_payload["attributes"] = attributes
        fake_revision_payload["epoch"] = epoch

        data = Revision.unmarshal(fake_revision_payload).marshal()

        assert data["architectures"] == ["amd64"]
        assert data["attributes"] == attributes
        assert data["base"] == base
        assert data["build-url"] is build_url
        assert data["confinement"] == "strict"
        assert data["created-at"] == "2016-09-27T18:38:43Z"
        assert data["epoch"] == epoch
        assert data["grade"] == "stable"
        assert data["revision"] == 1
        assert data["sha3-384"] == "fake-a9060ef4872ccacbfa44"
        assert data["size"] == 20
        assert data["status"] == "Published"
        assert data["version"] == "2.0.2"


class TestReleases:
    def test_unmarshal(
        self, fake_releases_payload, fake_revision_payload, fake_release_payload
    ):
        releases = Releases.unmarshal(fake_releases_payload)

        assert releases.revisions == [Revision.unmarshal(fake_revision_payload)]
        assert releases.releases == [Release.unmarshal(fake_release_payload)]

    def test_unmarshal_missing_releases_key(self, fake_revision_payload):
        """Default to an empty list if there are no releases."""
        payload = {"revisions": [fake_revision_payload]}
        releases = Releases.unmarshal(payload)

        assert releases.releases == []

    def test_unmarshal_ignores_extra_fields(self, fake_releases_payload):
        """Extra fields returned by the Store API are silently ignored."""
        fake_releases_payload["_links"] = {
            "self": "https://dashboard.snapcraft.io/api/v2/snaps/123/releases?page=1"
        }
        fake_releases_payload["snap"] = {"id": "abc123", "default-track": "latest"}

        releases = Releases.unmarshal(fake_releases_payload)

        assert len(releases.releases) == 1
        assert len(releases.revisions) == 1

    def test_marshal(
        self, fake_releases_payload, fake_revision_payload, fake_release_payload
    ):
        data = Releases.unmarshal(fake_releases_payload).marshal()

        assert data["revisions"] == [fake_revision_payload]
        assert data["releases"] == [fake_release_payload]
