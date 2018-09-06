# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
import contextlib
import os
from textwrap import dedent
import yaml

from snapcraft.internal import errors


_TEMPLATE_YAML = dedent(
    """\
    name: my-snap-name # you probably want to 'snapcraft register <name>'
    version: '0.1' # just for humans, typically '1.2+git' or '1.3.2'
    summary: Single-line elevator pitch for your amazing snap # 79 char long summary
    description: |
      This is my-snap's description. You have a paragraph or two to tell the
      most important story about your snap. Keep it under 100 words though,
      we live in tweetspace and your description wants to look good in the snap
      store.

    grade: devel # must be 'stable' to release into candidate/stable channels
    confinement: devmode # use 'strict' once you have the right plugs and slots

    parts:
      my-part:
        # See 'snapcraft plugins'
        plugin: nil
    """
)  # noqa, lines too long.


def init():
    """Initialize a snapcraft project."""
    snapcraft_yaml_path = os.path.join("snap", "snapcraft.yaml")

    if os.path.exists(snapcraft_yaml_path):
        raise errors.SnapcraftEnvironmentError(
            "{} already exists!".format(snapcraft_yaml_path)
        )
    elif os.path.exists("snapcraft.yaml"):
        raise errors.SnapcraftEnvironmentError("snapcraft.yaml already exists!")
    elif os.path.exists(".snapcraft.yaml"):
        raise errors.SnapcraftEnvironmentError(".snapcraft.yaml already exists!")
    text = _TEMPLATE_YAML
    with contextlib.suppress(FileExistsError):
        os.mkdir(os.path.dirname(snapcraft_yaml_path))
    with open(snapcraft_yaml_path, mode="w") as f:
        f.write(text)

    return snapcraft_yaml_path


def get_init_data():
    return yaml.load(_TEMPLATE_YAML)
