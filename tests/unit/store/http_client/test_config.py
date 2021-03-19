# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd
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

import pathlib

import pytest

from snapcraft.storeapi.http_clients import errors
from snapcraft.storeapi.http_clients._config import Config


class ConfigImpl(Config):
    def _get_section_name(self) -> str:
        return "test-section"

    def _get_config_path(self) -> pathlib.Path:
        return pathlib.Path("config.cfg")


@pytest.fixture
def conf(tmp_work_path):
    conf = ConfigImpl()
    yield conf


def test_non_existing_file_succeeds(conf):
    assert conf.parser.sections() == []
    assert conf.is_section_empty() is True


def test_existing_file(conf):
    conf.set("foo", "bar")
    conf.save()

    conf.load()

    assert conf.get("foo") == "bar"
    assert conf.is_section_empty() is False


def test_irrelevant_sections_are_ignored(conf):
    with conf._get_config_path().open("w") as config_file:
        print("[example.com]", file=config_file)
        print("foo=bar", file=config_file)

    conf.load()

    assert conf.get("foo") is None


def test_clear_preserver_other_sections(conf):
    with conf._get_config_path().open("w") as config_file:
        print("[keep_me]", file=config_file)
        print("foo=bar", file=config_file)

    conf.load()
    conf.set("bar", "baz")

    assert conf.get("bar") == "baz"

    conf.clear()
    conf.save()
    conf.load()

    assert conf.get("bar") is None
    assert conf.get("foo", "keep_me") == "bar"
    assert conf.is_section_empty() is True


def test_save_encoded(conf):
    conf.set("bar", "baz")
    conf.save(encode=True)
    conf.load()

    assert conf.get("bar") == "baz"
    with conf._get_config_path().open() as config_file:
        assert config_file.read() == "W3Rlc3Qtc2VjdGlvbl0KYmFyID0gYmF6Cgo=\n"


def test_save_encoded_other_config_file(conf):
    conf.set("bar", "baz")
    test_config_file = pathlib.Path("test-config")
    with test_config_file.open("w") as config_fd:
        conf.save(config_fd=config_fd, encode=True)
        config_fd.flush()

    with test_config_file.open() as config_file:
        assert config_file.read() == "W3Rlc3Qtc2VjdGlvbl0KYmFyID0gYmF6Cgo=\n"


def test_load_invalid_config(conf):
    test_config_file = pathlib.Path("test-config")
    with test_config_file.open("w") as config_fd:
        print("invalid config", file=config_fd)
        config_fd.flush()

    with test_config_file.open() as config_fd:
        with pytest.raises(errors.InvalidLoginConfig):
            conf.load(config_fd=config_fd)
