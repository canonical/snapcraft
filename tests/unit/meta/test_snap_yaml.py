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

import textwrap
from pathlib import Path

import pytest
import yaml

from snapcraft.meta import snap_yaml
from snapcraft.projects import Project


@pytest.fixture
def simple_project():
    snapcraft_yaml = textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        base: core22
        summary: Single-line elevator pitch for your amazing snap
        description: |
          This is my-snap's description. You have a paragraph or two to tell the
          most important story about your snap. Keep it under 100 words though,
          we live in tweetspace and your description wants to look good in the snap
          store.

        grade: stable
        confinement: strict

        parts:
          part1:
            plugin: nil

        apps:
          app1:
            command: bin/mytest
        """
    )
    data = yaml.safe_load(snapcraft_yaml)
    yield Project.unmarshal(data)


def test_simple_snap_yaml(simple_project, new_dir):
    snap_yaml.write(simple_project, prime_dir=Path(new_dir), arch="arch")
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: |
          This is my-snap's description. You have a paragraph or two to tell the
          most important story about your snap. Keep it under 100 words though,
          we live in tweetspace and your description wants to look good in the snap
          store.
        type: app
        architectures:
        - arch
        base: core22
        assumes:
        - command-chain
        apps:
          app1:
            command: bin/mytest
            command_chain:
            - snap/command-chain/snapcraft-runner
        confinement: strict
        grade: stable
        """
    )


@pytest.fixture
def complex_project():
    snapcraft_yaml = textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        base: core22
        summary: Single-line elevator pitch for your amazing snap
        description: |
          This is my-snap's description. You have a paragraph or two to tell the
          most important story about your snap. Keep it under 100 words though,
          we live in tweetspace and your description wants to look good in the snap
          store.

        grade: stable
        confinement: strict

        environment:
          GLOBAL_VARIABLE: test-global-variable

        parts:
          part1:
            plugin: nil

        apps:
          app1:
            command: bin/mytest
            autostart: test-app.desktop
            common_id: test-common-id
            bus_name: test-bus-name
            completer: test-completer
            stop_command: test-stop-command
            post_stop_command: test-post-stop-command
            start_timeout: 1s
            stop_timeout: 2s
            watchdog_timeout: 3s
            reload_command: test-reload-command
            restart_delay: 4s
            timer: test-timer
            daemon: simple
            after: [test-after-1, test-after-2]
            before: [test-before-1, test-before-2]
            refresh_mode: endure
            stop_mode: sigterm
            restart_condition: on-success
            install_mode: enable
            aliases: [test-alias-1, test-alias-2]
            environment:
              APP_VARIABLE: test-app-variable
            adapter: none
            command_chain:
            - snap/command-chain/snapcraft-runner
            sockets:
              test-socket-1:
                listen_stream: /tmp/test-socket.sock
                socket_mode: 0
              test-socket-2:
                listen_stream: 100
                socket_mode: 1
        """
    )
    data = yaml.safe_load(snapcraft_yaml)
    yield Project.unmarshal(data)


def test_complex_snap_yaml(complex_project, new_dir):
    snap_yaml.write(complex_project, prime_dir=Path(new_dir), arch="arch")
    yaml_file = Path("meta/snap.yaml")
    assert yaml_file.is_file()

    content = yaml_file.read_text()
    assert content == textwrap.dedent(
        """\
        name: mytest
        version: 1.29.3
        summary: Single-line elevator pitch for your amazing snap
        description: |
          This is my-snap's description. You have a paragraph or two to tell the
          most important story about your snap. Keep it under 100 words though,
          we live in tweetspace and your description wants to look good in the snap
          store.
        type: app
        architectures:
        - arch
        base: core22
        assumes:
        - command-chain
        apps:
          app1:
            command: bin/mytest
            autostart: test-app.desktop
            common_id: test-common-id
            bus_name: test-bus-name
            completer: test-completer
            stop_command: test-stop-command
            post_stop_command: test-post-stop-command
            start_timeout: 1s
            stop_timeout: 2s
            watchdog_timeout: 3s
            reload_command: test-reload-command
            restart_delay: 4s
            timer: test-timer
            daemon: simple
            after:
            - test-after-1
            - test-after-2
            before:
            - test-before-1
            - test-before-2
            refresh_mode: endure
            stop_mode: sigterm
            restart_condition: on-success
            install_mode: enable
            aliases:
            - test-alias-1
            - test-alias-2
            environment:
              APP_VARIABLE: test-app-variable
            adapter: none
            command_chain:
            - snap/command-chain/snapcraft-runner
            sockets:
              test-socket-1:
                listen_stream: /tmp/test-socket.sock
                socket_mode: 0
              test-socket-2:
                listen_stream: 100
                socket_mode: 1
        confinement: strict
        grade: stable
        environment:
          GLOBAL_VARIABLE: test-global-variable
        """
    )
