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

import io
import pytest


from snapcraft import yaml_utils
from snapcraft.yaml_utils import YamlValidationError


def test_load_yaml_file(caplog, tmp_path):
    yaml_path = tmp_path / "test.yaml"
    yaml_path.write_text("foo: bar")

    cfg = yaml_utils.load_yaml_file(str(yaml_path))

    assert cfg == {"foo": "bar"}
    assert [r.message for r in caplog.records] == []


def test_load_yaml_file_bad_yaml(caplog, tmp_path):
    yaml_path = tmp_path / "test.yaml"
    yaml_path.write_text("foo: bara-b-c: xyz\n")

    with pytest.raises(YamlValidationError):
        yaml_utils.load_yaml_file(str(yaml_path))

    assert [r.message for r in caplog.records] == []


def test_load_yaml_file_with_duplicate_warnings(caplog, tmp_path):
    yaml_path = tmp_path / "test.yaml"
    yaml_path.write_text("foo: bar\nfoo: bar2")

    cfg = yaml_utils.load_yaml_file(str(yaml_path), warn_duplicate_keys=True)

    assert cfg == {"foo": "bar2"}
    assert [r.message for r in caplog.records] == [
        "Duplicate key in YAML detected: 'foo'"
    ]


def test_dump_octint(tmp_path):
    output = io.StringIO()
    yaml_utils.dump(dict(number=yaml_utils.OctInt(8)), stream=output)
    output.seek(0)

    assert output.read().strip() == "number: 0010"
