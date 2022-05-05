# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import pydantic
import pytest

from snapcraft.repo.projects import AptDeb, AptPPA


class TestAptPPAValidation:
    """AptPPA field validation."""

    def test_apt_ppa_valid(self):
        repo = {
            "type": "apt",
            "ppa": "test/somerepo",
        }
        apt_ppa = AptPPA.unmarshal(repo)
        assert apt_ppa.type == "apt"
        assert apt_ppa.ppa == "test/somerepo"

    def test_apt_ppa_repository_invalid(self):
        repo = {
            "ppa": "test/somerepo",
        }
        error = r"type\s+field required"
        with pytest.raises(pydantic.ValidationError, match=error):
            AptPPA.unmarshal(repo)

    def test_project_package_ppa_repository_bad_type(self):
        repo = {
            "type": "invalid",
            "ppa": "test/somerepo",
        }
        error = "unexpected value; permitted: 'apt'"
        with pytest.raises(pydantic.ValidationError, match=error):
            AptPPA.unmarshal(repo)


class TestAptDebValidation:
    """AptDeb field validation."""

    @pytest.mark.parametrize(
        "repo",
        [
            {
                "type": "apt",
                "url": "https://some/url",
                "key-id": "BCDEF12345" * 4,
            },
            {
                "type": "apt",
                "url": "https://some/url",
                "key-id": "BCDEF12345" * 4,
                "formats": ["deb"],
                "components": ["some", "components"],
                "key-server": "my-key-server",
                "path": "my/path",
                "suites": ["some", "suites"],
            },
        ],
    )
    def test_apt_deb_valid(self, repo):
        apt_deb = AptDeb.unmarshal(repo)
        assert apt_deb.type == "apt"
        assert apt_deb.url == "https://some/url"
        assert apt_deb.key_id == "BCDEF12345" * 4
        assert apt_deb.formats == (["deb"] if "formats" in repo else None)
        assert apt_deb.components == (
            ["some", "components"] if "components" in repo else None
        )
        assert apt_deb.key_server == ("my-key-server" if "key-server" in repo else None)
        assert apt_deb.path == ("my/path" if "path" in repo else None)
        assert apt_deb.suites == (["some", "suites"] if "suites" in repo else None)

    @pytest.mark.parametrize(
        "key_id,error",
        [
            ("ABCDE12345" * 4, None),
            ("KEYID12345" * 4, "string does not match regex"),
            ("abcde12345" * 4, "string does not match regex"),
        ],
    )
    def test_apt_deb_key_id(self, key_id, error):
        repo = {
            "type": "apt",
            "url": "https://some/url",
            "key-id": key_id,
        }

        if not error:
            apt_deb = AptDeb.unmarshal(repo)
            assert apt_deb.key_id == key_id
        else:
            with pytest.raises(pydantic.ValidationError, match=error):
                AptDeb.unmarshal(repo)

    @pytest.mark.parametrize(
        "formats",
        [
            ["deb"],
            ["deb-src"],
            ["deb", "deb-src"],
            ["_invalid"],
        ],
    )
    def test_apt_deb_formats(self, formats):
        repo = {
            "type": "apt",
            "url": "https://some/url",
            "key-id": "ABCDE12345" * 4,
            "formats": formats,
        }

        if formats != ["_invalid"]:
            apt_deb = AptDeb.unmarshal(repo)
            assert apt_deb.formats == formats
        else:
            error = ".*unexpected value; permitted: 'deb', 'deb-src'"
            with pytest.raises(pydantic.ValidationError, match=error):
                AptDeb.unmarshal(repo)
